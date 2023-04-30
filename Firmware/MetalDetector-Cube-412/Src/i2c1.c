/*
 * i2c1.c
 *
 *  Created on: Jun 16, 2019
 *      Author: stimey
 */

#include "i2c1.h"

#ifdef STM32F303X8

//#include "stm32f3x.h"
#include "stm32f303x8.h"

#else

#include "stm32l4xx.h"

#endif

#include "uart.h"
#include "sys_tick.h"

#ifdef I2C1_DEBUG_MSGS

#define	I2C_DBG_puts(str) 		uart_puts(str)
#define I2C_DBG_puthexword(val) uart_print_hex_word(val)
#define I2C_DBG_puthexbyte(val) uart_putch(' '); uart_print_hex_byte(val);

#else

#define I2C_DBG_puts(str)
#define I2C_DBG_puthexword(val)
#define I2C_DBG_puthexbyte(val)

#endif

#ifndef I2C_USE_BITBANG
/*
 *
 * Internal to library routines routines
 *
 */

void I2C_DumpRegs()
{
#ifdef I2C1_DEBUG_MSGS
#ifdef I2C1_DUMP_REGS_ENABLED
    uart_puts("\r\nCR1     ="); uart_print_hex_word(I2C1->CR1);
    uart_puts("\r\nCR2     ="); uart_print_hex_word(I2C1->CR2);
    uart_puts("\r\nTIMINGR ="); uart_print_hex_word(I2C1->TIMINGR);
    uart_puts("\r\nTIMEOUTR="); uart_print_hex_word(I2C1->TIMEOUTR);
    uart_puts("\r\nISR     ="); uart_print_hex_word(I2C1->ISR);
    uart_puts("\r\nICR     ="); uart_print_hex_word(I2C1->ICR);
    uart_puts("\r\nAPB1ENR ="); uart_print_hex_word(RCC->APB1ENR);
#endif
#endif
};

int I2C1_ReadOne()
{
    volatile uint32_t timeout=10000000;
    int res = -1;

    while((~I2C1->ISR & I2C_ISR_RXNE) && (--timeout)) /* WAIT */ ;

    if (timeout > 0)
    {
    	res = I2C1->RXDR;
    }
    else
    {
        I2C_DumpRegs();
    }

    return(res);
}


bool I2C1_SendOne(uint8_t byte_to_send)
{
    volatile uint32_t timeout=10000000;

    while((!(I2C1->ISR & I2C_ISR_TXE)) & (--timeout > 0)) /* NOTHING */;

    if (I2C1->ISR & I2C_ISR_TXE)
        I2C1->TXDR = byte_to_send;

    if (timeout==0)
    {
    	I2C_DBG_puts("I2C1_SendOne Timed Out\r\n");
        I2C_DumpRegs();
    };

    return(timeout>0);
}

/*
 *
 * External routines
 *
 */

int I2C1_Initialized;
int I2C1_DMA_Active;

bool I2C1_InitMaster(void)
{
#ifdef STM32F303X8
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN; // ensure the clock is turned on
#else
   // CPU startup and clock initialization sets I2C to use HSI16 as clock source

    RCC->APB1ENR1 |= RCC_APB1ENR1_I2C1EN; // ensure the clock is turned on
#endif

    // disable the I2C for configuration
    I2C1->CR1 = (I2C1->CR1 & ~I2C_CR1_PE) ;

    // configure the I2C
    I2C1->CR1 |= I2C_CR1_ANFOFF; // configure digital and analog filtering

#ifdef I2C_USE_400KHZ
    // configure timing for FM (400 kHz) - entry values taken from data sheet
    I2C1->TIMINGR = (0 << 28) |	// PRESC
    				(3 << 20) | // SCLDEL
                    (1 << 16) | // SDADEL
                    (3 << 8 ) | // SCLH
                    (9 << 0 ); // SCLL
#elif I2C_USE_10KHZ
    // configure timing for SM (10 kHz) - entry values taken from data sheet
    I2C1->TIMINGR = (1 << 28) |	// PRESC
    				(3 << 20) | // SCLDEL
                    (2 << 16) | // SDADEL
                    (195 << 8) | // SCLH
                    (198 << 0 ); // SCLL
#else
    // configure timing for SM (100 kHz) - entry values taken from data sheet
    I2C1->TIMINGR = (1 << 28) |	// PRESC - SHOULD BE 1
    				(4 << 20) | // SCLDEL = 3
                    (5 << 16) | // SDADEL =
                    (19 << 8) | // SCLH
                    (23 << 0 ); // SCLL
#endif

#ifndef STM32F303X8
    // STM32L412 HSI is 16 MHz vs 8 Mhz, so double the prescaler value
    I2C1->TIMINGR = ((I2C1->TIMINGR >> 28) + 1) | (I2C1->TIMINGR & 0x0FFFFFFF);
#endif

    // enable the I2C
    I2C1->CR1 |= I2C_CR1_PE;

    I2C1_Initialized = true;
    I2C1_DMA_Active = false;

    return(true);
}

bool I2C1_ShutdownMaster(void)
{
	I2C1->CR1 &= ~I2C_CR1_PE;
	I2C1_Initialized = false;
	return(true);
}

bool I2C1_WriteMaster(uint16_t DevAddr, uint16_t MemAddr, uint16_t MemAddrSize, uint8_t *buffer, uint8_t buff_len)
{
	volatile unsigned int timeout = 1000000;
    int8_t last_res, Success=0;

    I2C_DBG_puts("\r\nI2C1_WriteMaster...");

    I2C1->CR1 &= ~I2C_CR1_PE;

    // set NBYTE = buff_len, AUTOEND=1, set SLAVE_ADDRESS
    I2C1->CR2=  (((buff_len + MemAddrSize) & 0xFF) << 16) |
                ((DevAddr & 0x7F) << 1);

    I2C1->CR1 |= I2C_CR1_PE;

    // set START
    I2C1->CR2 |= I2C_CR2_START;
    while((I2C1->CR2 & I2C_CR2_START)) /* NOTHING */ ;

    last_res = true;
    Success = false;

	if (MemAddrSize>1)	// we only support 16 bit addressing on I2C slaves
		last_res = I2C1_SendOne(MemAddr>>8);

    if ((last_res) && (I2C1_SendOne(MemAddr & 0xFF))) // I2C hardware sends address byte, we send command
    {
        // send data
        while((buff_len--) && (last_res))
        {
            last_res = I2C1_SendOne(*buffer);
            buffer++;
        }
        Success = last_res;
    };

    if (Success)
    {
		while((!(I2C1->ISR & I2C_ISR_TC)) & (--timeout > 0)) /* NOTHING */;

		I2C1->CR2 |= I2C_CR2_STOP;
		while(I2C1->CR2 & I2C_CR2_STOP) /* NOTHING */ ;
    }
    else
    {
    	// pause a short bit, then reset the I2C1
    	delay_ms(1);
    	I2C1_ShutdownMaster();
    	delay_ms(1);
    	I2C1_InitMaster();
    }
    // while(~I2C1->ISR & I2C_ISR_TC) /* WAIT */ ;

    if (Success)
    	I2C_DBG_puts("I2C1_WriteMaster Done\r\n");
    else
    	I2C_DBG_puts("I2C1_WriteMaster Failed\r\n");

    return(Success);
}

bool I2C1_ReadMaster(uint16_t DevAddr, uint16_t MemAddr, uint16_t MemAddrSize, uint8_t *buffer, uint8_t buff_len)
{
    int8_t last_res, Success=0;
    int ch;
	volatile unsigned int timeout = 1000000;

    I2C_DBG_puts("\r\nI2C1_ReadMaster...");

    I2C1->CR1 &= ~I2C_CR1_PE;

    // set NBYTE = memory length, AUTOEND=0, set SLAVE_ADDRESS
    I2C1->CR2=  ((MemAddrSize & 0xFF) << 16) |
                ((DevAddr & 0x7F) << 1);

    I2C1->CR1 |= I2C_CR1_PE;

    // set START
    I2C1->CR2 |= I2C_CR2_START;
    while((I2C1->CR2 & I2C_CR2_START)) /* NOTHING */ ;

    last_res = true;
    Success = false;

	if (MemAddrSize>1)	// we only support 16 bit addressing on I2C slaves
		last_res = I2C1_SendOne(MemAddr>>8);
	if (last_res)
		last_res = I2C1_SendOne(MemAddr);

    while((!(I2C1->ISR & I2C_ISR_TC)) && (--timeout > 0)) /* WAIT */ ;

    if (last_res) // I2C hardware sends address byte, we send command
    {
    	// set AUTO END true, NBYTES = # of bytes to receive, re-send device address
        I2C1->CR2= I2C_CR2_RD_WRN |
        			((buff_len& 0xFF) << 16) |
                    ((DevAddr & 0x7F) << 1);

        // set START
        I2C1->CR2 |= I2C_CR2_START;
        while((I2C1->CR2 & I2C_CR2_START)) /* NOTHING */ ;

        // read data
        while((buff_len--) && (last_res))
        {
            ch = I2C1_ReadOne();
            if (ch<0) last_res = false;
            else      *buffer = ch;
            buffer++;
        }
        I2C1->CR2 |= I2C_CR2_STOP;
        while(I2C1->CR2 & I2C_CR2_STOP) /* NOTHING */ ;

        Success = last_res;
    };

    if (Success)
    	I2C_DBG_puts("I2C1_WriteMaster Done\r\n");
    else
    	I2C_DBG_puts("I2C1_WriteMaster Failed\r\n");

    return(Success);
}

bool I2C1_WriteMasterDMA(uint16_t DevAddr, uint16_t MemAddr, uint16_t MemAddrSize, uint8_t *buffer, uint8_t buff_len)
{
	return(false);
}

bool I2C1_ReadMasterDMA(uint16_t DevAddr, uint16_t MemAddr, uint16_t MemAddrSize, uint8_t *buffer, uint8_t buff_len)
{
	return(false);
}

bool I2C1_DMABusy(void)
{
	return(false);
}

#else //  I2C_USE_BITBANG

#include "sys_tick.h"

#define BB_I2C_SDA_PORT GPIOB
#define BB_I2C_SDA_BIT  7

#define BB_I2C_SCL_PORT GPIOB
#define BB_I2C_SCL_BIT  6

#define BB_I2C_DELAY(val)   delay_us(val*1)

#define BB_SDA_LOW() 	{BB_I2C_SDA_PORT->MODER = (BB_I2C_SDA_PORT->MODER & ~(3 << (BB_I2C_SDA_BIT << 1))) | (1 << (BB_I2C_SDA_BIT << 1)); BB_I2C_SDA_PORT->ODR &= ~(1 << BB_I2C_SDA_BIT);}
#define BB_SDA_HIGH()	{BB_I2C_SDA_PORT->MODER = (BB_I2C_SDA_PORT->MODER & ~(3 << (BB_I2C_SDA_BIT << 1))); BB_I2C_SDA_PORT->ODR |= (1 << BB_I2C_SDA_BIT);}

#define BB_SDA_INPUT()	((BB_I2C_SDA_PORT->IDR >> BB_I2C_SDA_BIT) & 0x01)

#define BB_SCL_LOW() 	{BB_I2C_SCL_PORT->MODER = (BB_I2C_SCL_PORT->MODER & ~(3 << (BB_I2C_SCL_BIT << 1))) | (1 << (BB_I2C_SCL_BIT << 1)); BB_I2C_SCL_PORT->ODR &= ~(1 << BB_I2C_SCL_BIT);}
#define BB_SCL_HIGH()	{BB_I2C_SCL_PORT->MODER = (BB_I2C_SCL_PORT->MODER & ~(3 << (BB_I2C_SCL_BIT << 1))); BB_I2C_SCL_PORT->ODR |= (1 << BB_I2C_SCL_BIT);}

bool I2C1_InitMaster(void)
{
	// configure PB6 an PB7 as GPIO open drain outputs with Pull-up resistors ON, set SDA and SCL to output floating
	// 0 = GP Input, 1 = GP Output, 2 = Spec Func, 3 = Analog Input
    GPIOB->MODER = (GPIOB->MODER & ~((3 << 14) | (3 << 12)));	// set PB6 &  PB7 as inputs

    GPIOB->OTYPER |= // 1 = Open Drain, 0 = Push / Pull
                (1 << 7) | // PB7 - I2C1 SDA
                (1 << 6) ; // PB6 - I2C1 SCL

    GPIOB->OSPEEDR |= // 0 = SLOW (up to 12 MHz, 125 ns Rise/Fall), 1 = MEDIUM (up to 41 MHz, 25 ns Rise/Fall), 3=Fas5 (up to 50 MHz, ~10 ns Rise/Fall)
                (3 << 14) | // PB7 - I2C1 SDA
                (3 << 12) ; // PB6 - I2C1 SCL

    GPIOB->PUPDR = (GPIOB->PUPDR & ~((3 << 14) | (3 << 12))) | // 0 = NO PU/PD, 1=PU, 2=PD, 3 = NOT USED
                (1 << 14) | // PB7 - I2C1 SDA
                (1 << 12) ; // PB6 - I2C1 SCL

                    // AFRH
    GPIOB->AFR[0] &= ~((0x0F << 24) | (0x0F << 28));	// set PB7 / PB6 AFR to 0

    // send STOP
	BB_SCL_LOW();	BB_I2C_DELAY(1);
	BB_SDA_LOW();	BB_I2C_DELAY(1);

	BB_SCL_HIGH();	BB_I2C_DELAY(1);
	BB_SDA_HIGH();	BB_I2C_DELAY(1);

	return(true);
}

bool I2C1_ShutdownMaster(void)
{

	return(true);
}

void I2C_SendStart(void)
{
	BB_SCL_HIGH();	BB_I2C_DELAY(1);
	BB_SDA_HIGH();	BB_I2C_DELAY(1);

	BB_SDA_LOW();	BB_I2C_DELAY(1);
	BB_SCL_LOW();	BB_I2C_DELAY(1);
}

void I2C_SendRestart(void)
{
	I2C_SendStart();
}

void I2C_SendStop(void)
{
	BB_SCL_LOW();	BB_I2C_DELAY(1);
	BB_SDA_LOW();	BB_I2C_DELAY(1);

	BB_SCL_HIGH();	BB_I2C_DELAY(1);
	BB_SDA_HIGH();	BB_I2C_DELAY(1);
}

bool I2C_SendByte(uint8_t ch)
{
	int i;
	/* SEND 8 BITS, WAIT FOR ACK */
	for(i=0; i < 8; i++)
	{
		if ((ch << i) & 0x80)
			BB_SDA_HIGH()
		else
			BB_SDA_LOW()

		BB_I2C_DELAY(1);

		BB_SCL_HIGH();		BB_I2C_DELAY(1);
		BB_SCL_LOW();		BB_I2C_DELAY(1);
	}

	// set SDA to open collector input
	BB_SDA_HIGH();			BB_I2C_DELAY(1);

	// send 9th clock for ACK bit
	BB_SCL_HIGH();			BB_I2C_DELAY(1);

	// TODO - Sample and verify ACK received
	BB_SCL_LOW();			BB_I2C_DELAY(1);

	// ACK should be on the IO pin
	return(false);
}

int I2C_ReadByte()
{
	/* READ 8 BITS, SEND ACK */
	int i, res=0;

	for(i=0; i < 8; i++)
	{
		BB_SCL_HIGH();	BB_I2C_DELAY(1);

		res = (res << 1) | BB_SDA_INPUT();

		BB_SCL_LOW();	BB_I2C_DELAY(1);
	}
	BB_SDA_LOW();	BB_I2C_DELAY(1);	// send ACK to device

	BB_SCL_HIGH();	BB_I2C_DELAY(1);
	BB_SCL_LOW();	BB_I2C_DELAY(1);

	return(res);
}

bool I2C1_WriteMaster(uint16_t DevAddr, uint16_t MemAddr, uint16_t MemAddrSize, uint8_t *buffer, uint8_t buff_len)
{
	// START condition
	I2C_SendStart();

	// Send device address + R/W
	I2C_SendByte(DevAddr << 1);

	// Send send memory address
	if (MemAddrSize > 1)	I2C_SendByte(MemAddr >> 8);

	I2C_SendByte(MemAddr);

	// Send data
	while(buff_len--)
		I2C_SendByte(*(buffer++));

	// STOP condition
	I2C_SendStop();
}

bool I2C1_ReadMaster(uint16_t DevAddr, uint16_t MemAddr, uint16_t MemAddrSize, uint8_t *buffer, uint8_t buff_len)
{
	// START condition
	I2C_SendStart();

	// Send device address + R/W
	I2C_SendByte(DevAddr << 1);

	// Send send memory address
	if (MemAddrSize > 1)	I2C_SendByte(MemAddr >> 8);
	I2C_SendByte(MemAddr);

	I2C_SendRestart();
	// Send device address + R/W
	I2C_SendByte((DevAddr << 1) | 1);

	// Send data
	while(buff_len--)
		*(buffer++)=I2C_ReadByte();

	// STOP condition
	I2C_SendStop();
}

bool I2C1_WriteMasterDMA(uint16_t DevAddr, uint16_t MemAddr, uint16_t MemAddrSize, uint8_t *buffer, uint8_t buff_len)
{

}

bool I2C1_ReadMasterDMA(uint16_t DevAddr, uint16_t MemAddr, uint16_t MemAddrSize, uint8_t *buffer, uint8_t buff_len)
{

}

bool I2C1_DMABusy(void)
{

}

#endif  // I2C_USE_BITBANG

/*
 * uart.c
 *
 *  Created on: Jun 2, 2019
 *      Author: stimey
 */

#include "main.h"

#ifdef STM32F303X8

#include "stm32f303x8.h"

#else

#include "stm32l4xx.h"

#endif

#define UART_BUFF_SIZE 2048

typedef struct {
	volatile uint16_t head;
	volatile uint16_t tail;
	volatile uint8_t buffer[UART_BUFF_SIZE];
} TCircularBuffer;

TCircularBuffer TxBuff, RxBuff;

void USART2_IRQHandler()
{
	if (USART2->ISR & USART_ISR_RXNE) // character received
	{
		if (((RxBuff.head+1) % UART_BUFF_SIZE) != RxBuff.tail)
		{
			RxBuff.buffer[RxBuff.head]=USART2->RDR;
			RxBuff.head = (RxBuff.head+1) % UART_BUFF_SIZE;
		}
		else
		{
			RxBuff.buffer[RxBuff.head]=USART2->RDR;
			RxBuff.buffer[RxBuff.head]='?';
		}
	};

	if ((USART2->ISR & USART_ISR_TXE) | (USART2->ISR & USART_ISR_TC)) // character ready for transmit
	{

		if (TxBuff.head != TxBuff.tail) 		// send next character if available
		{
			USART2->TDR = TxBuff.buffer[TxBuff.tail];
			TxBuff.tail = (TxBuff.tail + 1) % UART_BUFF_SIZE;
		}
		else if (USART2->ISR & USART_ISR_TC)	// transmit complete, disable TXE and TC interrupts
		{
			USART2->CR1 &= ~(USART_CR1_TCIE | USART_CR1_TXEIE);
		}
		else if (USART2->ISR & USART_ISR_TXE) 	// no more characters to send, disable TXE interrupt
		{
			USART2->CR1 &= ~(USART_CR1_TXEIE);
		}
	};
}

void uart_init(uint32_t baudrate)
{
#ifdef STM32F303X8
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
#else
    RCC->APB1ENR1 |= RCC_APB1ENR1_USART2EN;
#endif

    USART2->CR1 = 0;   // disable uart for configuration

    USART2->CR2 = 0;   //
    USART2->CR3 = 0x00000000; // set to 0x000000C0 for DMA mode
    USART2->BRR = (F_CPU / baudrate);   // we use HSI for UART clock

    USART2->CR1 = USART_CR1_RXNEIE | USART_CR1_TE | USART_CR1_RE | USART_CR1_UE; // UART Enabled, TX/RX Enabled

    TxBuff.head = TxBuff.tail = 0;
    RxBuff.head = RxBuff.tail = 0;

    NVIC_EnableIRQ(USART2_IRQn);
}

uint16_t uart_rx_avail()
{
	return((RxBuff.head-RxBuff.tail) % UART_BUFF_SIZE);
}

uint8_t uart_getch()
{
	uint8_t ch;

	while(RxBuff.head==RxBuff.tail) {/* NOTHING */} ;

	ch = RxBuff.buffer[RxBuff.tail];

	RxBuff.tail = (RxBuff.tail + 1) % UART_BUFF_SIZE;

	return ch;
}

void uart_putch(char ch)
{
	// wait for room in the buffer
	while (((TxBuff.tail+1) % UART_BUFF_SIZE)==TxBuff.head) /* NOTHING */ ;
	TxBuff.buffer[TxBuff.head]=ch;
	TxBuff.head = (TxBuff.head + 1) % UART_BUFF_SIZE;

	// if Tx int's are done, then restart transmit
	USART2->CR1 |= (USART_CR1_TCIE | USART_CR1_TXEIE);
}

void uart_puts(char *ch)
{
	while(*ch)
		uart_putch(*(ch++));
}


static const char HEXTABLE[]="0123456789ABCDEF";

static inline void uart_print_hex_nibble(uint8_t outv)
{
	uart_putch(HEXTABLE[outv & 0xF]);
}

void uart_print_hex_byte(uint8_t outv)
{
	uart_print_hex_nibble((outv >> 4) & 0x0F);
	uart_print_hex_nibble(outv & 0x0F);
}

void uart_print_hex_short(uint16_t outv)
{
	uart_print_hex_byte(outv >> 8);
	uart_print_hex_byte(outv & 0xFF);
}

void uart_print_hex_word(uint32_t outv)
{
	uart_print_hex_short(outv >> 16);
	uart_print_hex_short(outv & 0xFFFF);
}


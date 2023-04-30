/*
 * gpio.c
 *
 *  Created on: Jun 2, 2019
 *      Author: stimey
 */

#include "stm32l4xx.h"

void init_port_a()
{
    RCC->AHB2ENR = RCC->AHB2ENR | RCC_AHB2ENR_GPIOAEN;

    GPIOA->MODER =  // 0 = GP Input, 1 = GP Output, 2 = Spec Func, 3 = Analog Input
    			(2 << 30) | // PA15 - UART2 TX
                (2 << 28) | // PA14 - SWCLK
                (2 << 26) | // PA13 - SWDIO
                (0 << 24) | // PA12 - NOT USED [Will be USB]
                (0 << 22) | // PA11 - NOT USED [Will be USB]
                (2 << 20) | // PA10 - GPO HV_EN
                (2 << 18) | //  PA9 - TIM1 CH2 - COIL PULSE
				(2 << 16) | //  PA8 - LPTIM2 OUT - HV PWM
                (1 << 14) | //  PA7 - OPAMP ENABLE
                (0 << 12) | //  PA6 - GPI - TRIGGER PRESSED
                (2 << 10) | //  PA5 - TIM2 CH1 - HPHONE
                (3 << 8) |  //  PA4 - ADC - V_BAT
                (2 << 6) |  //  PA3 - TIM2 CH4  - SPKR
                (2 << 4) |  //  PA2 - UART2 RX
                (3 << 2) |  //  PA1 - ADC - V_DIO_AMP - minus input to internal opamp
                (3 << 0);   //  PA0 - ADC - V_DIO - direct input from opamp

    GPIOA->OTYPER = // 1 = Open Drain, 0 = Push / Pull
                (0 << 15) | // PA15 - UART2 TX
                (0 << 14) | // PA14 - SWCLK
                (0 << 13) | // PA13 - SWDIO
                (0 << 12) | // PA12 - NOT USED (Will be USB)
                (0 << 11) | // PA11 - NOT USED (Will be USB)
                (0 << 10) | // PA10 - GPO HV_EN
                (0 << 9) |  //  PA9 - TIM1 CH2 - COIL PULSE
				(0 << 8) |  //  PA8 - LPTIM2 OUT - HV PWM
                (0 << 7) |  //  PA7 - OPAMP ENABLE
                (1 << 6) |  //  PA6 - GPI - TRIGGER PRESSED
                (0 << 5) |  //  PA5 - TIM2 CH1 - HPHONE
                (1 << 4) |  //  PA4 - ADC - V_BAT
                (0 << 3) |  //  PA3 - TIM2 CH4  - SPKR
                (0 << 2) |  //  PA2 - UART2 RX
                (1 << 1) |  //  PA1 - ADC - V_DIO_AMP - minus input to internal opamp
                (1 << 0);   //  PA0 - ADC - V_DIO - direct input from opamp


    GPIOA->OSPEEDR = // 0 = SLOW (up to 12 MHz, 125 ns Rise/Fall), 1 = MEDIUM (up to 41 MHz, 25 ns Rise/Fall), 3=Fast (up to 50 MHz, ~10 ns Rise/Fall)
    			(0 << 30) | // PA15 - UART2 TX
                (3 << 28) | // PA14 - SWCLK
                (3 << 26) | // PA13 - SWDIO
                (0 << 24) | // PA12 - NOT USED (Will be USB)
                (0 << 22) | // PA11 - NOT USED (Will be USB)
                (0 << 20) | // PA10 - GPO HV_EN
                (1 << 18) | //  PA9 - TIM1 CH2 - COIL PULSE
                (0 << 16) | //  PA8 - LPTIM2 OUT - HV PWM
                (0 << 14) | //  PA7 - OPAMP ENABLE
                (0 << 12) | //  PA6 - GPI - TRIGGER PRESSED
                (0 << 10) | //  PA5 - TIM2 CH1 - HPHONE
                (0 << 8) |  //  PA4 - ADC - V_BAT
                (0 << 6) |  //  PA3 - TIM2 CH4  - SPKR
                (0 << 4) |  //  PA2 - UART2 RX
                (0 << 2) |  //  PA1 - ADC - V_DIO_AMP - minus input to internal opamp
                (0 << 0);   //  PA0 - ADC - V_DIO - direct input from opamp


    GPIOA->PUPDR =  // 0 = NO PU/PD, 1=PU, 2=PD, 3 = NOT USED
   				(1 << 30) | // PA15 - UART2 TX
                (2 << 28) | // PA14 - SWCLK
                (1 << 26) | // PA13 - SWDIO
                (1 << 24) | // PA12 - NOT USED (Will be USB)
                (1 << 22) | // PA11 - NOT USED (Will be USB)
                (0 << 20) | // PA10 - GPO HV_EN
                (2 << 18) | //  PA9 - TIM1 CH2 - COIL PULSE
                (0 << 16) | //  PA8 - LPTIM2 OUT - HV PWM
                (2 << 14) | //  PA7 - OPAMP ENABLE
                (1 << 12) | //  PA6 - GPI - TRIGGER PRESSED
                (0 << 10) | //  PA5 - TIM2 CH1 - HPHONE
                (0 << 8) |  //  PA4 - ADC - V_BAT
                (0 << 6) |  //  PA3 - TIM2 CH4  - SPKR
                (1 << 4) |  //  PA2 - UART2 RX
                (0 << 2) |  //  PA1 - ADC - V_DIO_AMP - minus input to internal opamp
                (0 << 0);   //  PA0 - ADC - V_DIO - direct input from opamp

                    // AFRL
    GPIOA->AFR[1] =  (3 << 28 ) | // PA15 - UART 2 RX - AF3
                (0 << 24 ) | // PA14 - SWDCLK - AF0
                (0 << 20 ) | // PA13 - SWDIO - AF0
                (0 << 16 ) | // PA12 - NOT USED (Will be USB)
                (0 << 12 ) | // PA11 - NOT USED (Will be USB)
                (0 << 8 ) |  // PA10 - GPO HV_EN
                (1 << 4 ) |  //  PA9 - TIM1 CH2 - COIL PULSE
                (14 << 0 ) ; //  PA8 - LPTIM2 OUT - HV PWM

                    // AFRH
    GPIOA->AFR[0] =  (0 << 28 ) | //  PA7 - OPAMP ENABLE
                (0 << 24 ) | //  PA6 - GPI - TRIGGER PRESSED
                (1 << 20 ) | //  PA5 - TIM2 CH1 - HPHONE
                (0 << 16 ) | //  PA4 - ADC - V_BAT
                (1 << 12 ) | //  PA3 - TIM2 CH4  - SPKR
                (7 << 8 ) |  //  PA2 - UART2 TX
                (0 << 4 ) |  //  PA1 - ADC - V_DIO_AMP - minus input to internal opamp
                (0 << 0 ) ;  //  PA0 - ADC - V_DIO - direct input from opamp

    GPIOA->ODR = 0; // all GP outputs LOW (active High transistor drives - leave low by default)
}

void init_port_b()
{
    RCC->AHB2ENR = RCC->AHB2ENR | RCC_AHB2ENR_GPIOBEN;

    GPIOB->MODER = // 0 = GP Input, 1 = GP Output, 2 = Spec Func, 3 = Analog Input
                (2 << 14) | // PB7 - I2C1 SDA - 4 for I2C
                (2 << 12) | // PB6 - I2C1 SCL - 4 for I2C
                (0 << 10) | // PB5 - GPI - OK BTN
                (1 << 8) |  // PB4 - GPO - AUDIO EN
                (1 << 6) |  // PB3 - LD3 - output LED
                (0 << 4) |  // PB2 -NOT PRESENT
                (3 << 2) |  // PB1 - ADC - HV_CHG
                (3 << 0);   // PB0 - ADC - ICOIL_SENSE

    GPIOB->OTYPER = // 1 = Open Drain, 0 = Push / Pull
                (1 << 7) | // PB7 - I2C1 SDA
                (1 << 6) | // PB6 - I2C1 SCL
                (0 << 5) | // PB5 - GPI - OK Button
                (0 << 4) | // PB4 - GPO - AUDIO EN
                (0 << 3) | // PB3 - LD3 - output LED
                (0 << 2) | // PB2 - NOT PRESENT
                (1 << 1) | // PB1 - ADC - HV_CHG
                (1 << 0);  // PB0 - ADC - ICOIL_SENSE

    GPIOB->OSPEEDR = // 0 = SLOW (up to 12 MHz, 125 ns Rise/Fall), 1 = MEDIUM (up to 41 MHz, 25 ns Rise/Fall), 3=Fase (up to 50 MHz, ~10 ns Rise/Fall)
                (1 << 14) | // PB7 - I2C1 SDA
                (1 << 12) | // PB6 - I2C1 SCL
                (0 << 10) | // PB5 - GPI - OK BUTTON
                (0 << 8) | 	// PB4 - GPO - AUDIO EN
                (0 << 6) | 	// PB3 - LD3 - output LED
                (0 << 4) | 	// PB2 - NOT PRESENT
                (0 << 2) | 	// PB1 - ADC - HV_CHG
                (0 << 0);  	// PB0 - ADC - ICOIL_SENSE

    GPIOB->PUPDR = // 0 = NO PU/PD, 1=PU, 2=PD, 3 = NOT USED
                (1 << 14) | // PB7 - I2C1 SDA
                (1 << 12) | // PB6 - I2C1 SCL
                (1 << 10) | // PB5 - GPI - OK BUTTON
                (2 << 8) | 	// PB4 - GPO - AUDIO EN
                (2 << 6) | 	// PB3 - LD3 - output LED
                (0 << 4) | 	// PB2 - NOT PRESENT
                (0 << 2) | 	// PB1 - ADC - HV_CHG
                (0 << 0); 	// PB0 - ADC - ICOIL_SENSE

                    // AFRL
    GPIOB->AFR[1] =  0;

                    // AFRH
    GPIOB->AFR[0] =
                (4 << 28 ) | // PB7 -I2C1 SDA
                (4 << 24 ) | // PB6 -I2C1 SCL
                (0 << 20 ) | // PB5 - OK BUTTON
                (0 << 16 ) | // PB4 - AUDIO EN
                (0 << 12 ) | // PB3 - LD3 - Output LED
                (0 << 8 ) | // PB2 - NOT PRESENT
                (0 << 4 ) | // PB1 - ADC - HV_CHG
                (0 << 0 ) ; // PB0 - ADC - ICOIL_SENSE
}

void init_port_c()
{
    RCC->AHB2ENR = RCC->AHB2ENR | RCC_AHB2ENR_GPIOCEN;
	GPIOC->MODER =  // 0 = GP Input, 1 = GP Output, 2 = Spec Func, 3 = Analog Input
			(0 << 14 ) |	// PC15 - Up Button / QUAD A
			(0 << 12 );		// PC14 - Down Button / QUAD B

    GPIOC->OTYPER = // 1 = Open Drain, 0 = Push / Pull
                (0 << 15) | // PC15 - Up Button / QUAD A
                (0 << 14) ; // PC14 - Down Button / QUAD B

    GPIOC->OSPEEDR = // 0 = SLOW (up to 12 MHz, 125 ns Rise/Fall), 1 = MEDIUM (up to 41 MHz, 25 ns Rise/Fall), 3=Fase (up to 50 MHz, ~10 ns Rise/Fall)
    			(0 << 30) | // PC15 - Up Button / QUAD A
                (0 << 28) ; // PC14 - Down Button / QUAD B

    GPIOC->PUPDR =  // 0 = NO PU/PD, 1=PU, 2=PD, 3 = NOT USED
   				(1 << 30) | // PC15 - Up Button / QUAD A
                (1 << 28) ; // PC14 - Down Button / QUAD B

    GPIOC->AFR[1] = // AFRL
    			(0 << 28 ) | // PC15 - Up Button / QUAD A
                (0 << 24 ) ; // PC14 - Down Button / QUAD B
}


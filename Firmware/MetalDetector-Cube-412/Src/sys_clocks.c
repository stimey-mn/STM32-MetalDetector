#include "sys_clocks.h"

#ifdef STM32F303X8

#include "stm32f30x.h"

/**
 * Initialize the system clock.
 *
 * Uses the external 8 MHz clock from the ST-Link to generate the
 * 72 MHz system clock.
 */
void
init_clock()
{
    uint32_t t=0;

	RCC->CFGR = 0;                          // reset to power on reset value

    RCC->CR &= ! (RCC_CR_HSEON | RCC_CR_PLLON); // turn off HSE and PLL

	RCC->CFGR = RCC_CFGR_PLLMULL4 | RCC_CFGR_HPRE_DIV1 | RCC_CFGR_PPRE1_DIV1 | RCC_CFGR_PPRE2_DIV1 | RCC_CFGR_MCO_NOCLOCK;

	// enable PLL
	RCC->CR |= RCC_CR_PLLON;

	// Wait until the PLL is stable
	while (!(RCC->CR & RCC_CR_PLLRDY)) {
		if (!(--t)) {
			return;
		}
	}

    // enable used device clocks
    RCC->AHBENR |= 0x10460007;
    RCC->APB1ENR |= 0x02220007; 
    RCC->APB2ENR |= 0x00000801;
    
    RCC->CFGR3 = 0x00030003; // USART2 and I2C1 use HSI as clock source
    
}

#else

#include "stm32l4xx.h"

void
init_clock()
{
}

#endif

/**
 * Initialize the systick and setup systick time handling and interrupt
 *
 */





























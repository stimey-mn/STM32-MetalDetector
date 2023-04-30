/*
 * sys_tick.c
 *
 *  Created on: Jun 2, 2019
 *      Author: stimey
 */

// the following is the number of overflows from the systick counter

#include <stdint.h>

#ifdef STM32F303X8

//#include "stm32f3x.h"

#include "stm32f303x8.h"

#else

#include "stm32l4xx.h"

#endif

#include "sw_quadrature.h"

volatile uint32_t systick_overflows;


uint32_t get_system_ticks(void)
{
	return systick_overflows;
}

void delay_ms(uint32_t dly)
{
	uint32_t starttick=systick_overflows;
	while((systick_overflows-starttick) < dly) {/* NOTHNG */} ;
}

void delay_us(uint32_t dly)
{
	volatile uint32_t cnt=0;
	while(cnt++ < dly) {  /* NOTHING */ } ;
#ifndef STM32F303X8
	cnt=0;
	while(cnt++ < dly) {  /* NOTHING */ } ;
	cnt=0;
	while(cnt++ < dly) {  /* NOTHING */ } ;
	cnt=0;
	while(cnt++ < dly) {  /* NOTHING */ } ;
	cnt=0;
	while(cnt++ < dly) {  /* NOTHING */ } ;
#endif

}


void SysTick_Handler(void)
{
    systick_overflows++;

    service_sw_quadrature();
}

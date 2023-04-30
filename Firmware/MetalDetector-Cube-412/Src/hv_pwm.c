/*
 * lptim2.c
 *
 *  Created on: Apr 12, 2020
 *      Author: stimey
 *
 *      Configure LPTIM2 to use HSI16, 30 usec T period, 8 reps / interrupt ~ 4 interrupts / millisecond
 *      ON time set based on difference between target and actual HV voltage
 */

#include "stm32l4xx.h"

#include "hv_pwm.h"
#include "adc.h"

#define LPTIM_CLKS_PER_USEC 16

#define MIN_CYCLE_TIME 30
#define MAX_ON_TIME (MIN_CYCLE_TIME/2)

#define HV_EN_BIT_FLAG (1 << 10)
#define HV_EN_ON() 	(GPIOA->ODR |= HV_EN_BIT_FLAG)
#define HV_EN_OFF() (GPIOA->ODR &= ~HV_EN_BIT_FLAG)

int hv_target_voltageX10;

void set_hv_target_voltage(int voltsX10)
{
	hv_target_voltageX10 = voltsX10;
	if (voltsX10==0)
		LPTIM2->CMP=0;	// force STOP of PWM output
}

void LPTIM2_IRQHandler()
{
	// hv_target_voltageX10 = !hv_target_voltageX10;

	if (hv_target_voltageX10 > 0)
	{
		int curr_hv_chg=read_hv_charge_levelx10();

	    GPIOA->MODER =  (GPIOA->MODER  & ~(3 << 16)) | (2 << 16);// 0 = GP Input, 1 = GP Output, 2 = Spec Func, 3 = Analog Input
	    if (hv_target_voltageX10 > curr_hv_chg)
	    	LPTIM2->CMP = LPTIM2->ARR - MAX_ON_TIME * LPTIM_CLKS_PER_USEC;
	    else
			LPTIM2->CMP = LPTIM2->ARR-1;
		HV_EN_ON();
	}
	else
	{
		LPTIM2->CMP = LPTIM2->ARR-1;
		HV_EN_OFF();
	    GPIOA->MODER =  (GPIOA->MODER  & ~(3 << 16)) | (1 << 16);// 0 = GP Input, 1 = GP Output, 2 = Spec Func, 3 = Analog Input
	};

	LPTIM2->ICR = LPTIM_ICR_UECF;
	LPTIM2->CR |= LPTIM_CR_SNGSTRT;	// start single conversion
}

void init_hv_pwm()
{
	// enable clock to LPTIM, LPTIM clock source is HSI16
	RCC->APB1ENR2 |= RCC_APB1ENR2_LPTIM2EN;

	LPTIM2->CR &= ~LPTIM_CR_ENABLE;	// disable to STOP
	LPTIM2->CFGR = 0; // LPTIM_CFGR_PRELOAD; // reverse polarity of the output

	LPTIM2->CR |= LPTIM_CR_ENABLE;	// ENABLE LPTIM2 to set remaing registers
	LPTIM2->ARR = (MIN_CYCLE_TIME)*LPTIM_CLKS_PER_USEC;
	LPTIM2->CMP = LPTIM2->ARR-1;
	LPTIM2->RCR = 8;	// 8 repititions of OPM cycle, then do interrupt do set target

    NVIC_EnableIRQ(LPTIM2_IRQn);
	LPTIM2->IER |= LPTIM_IER_UEIE;

	LPTIM2->CR |= LPTIM_CR_SNGSTRT;	// start single conversion
}

void shutdown_hv_pwm()
{
	set_hv_target_voltage(0);
}


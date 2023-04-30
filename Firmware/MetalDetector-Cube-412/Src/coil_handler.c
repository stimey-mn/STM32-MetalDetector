/*
 * tim1.c
 *
 *  Created on: Jul 6, 2019
 *      Author: stimey
 *
 *      TIM1 is utilized as follows:
 *      	CH2 - PA09 - Coil Pulse - OPM
 *      	CH1 - (not connected to any pin) used to flag HV charge pump to pause for disch pulse
 *
 *
 */

#ifdef STM32F303X8
#include "stm32f303x8.h"
#else
#include "stm32l4xx.h"
#endif

#include "adc.h"
#include "coil_handler.h"
#include "sys_tick.h"
#include "uart.h"
#include "printf.h"
#include "math.h"

int coil_current_readings[NUM_COIL_SAMPLES];
float hs_m, hs_c, ls_m, ls_e;

void init_tim1(void)
{
	// turn on CLOCK for TIM1
#ifdef STM32F303X8
	RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;

	// configure for 50 kHz timer rate from HSI clock with divisor set for  8 MHz base clock
	TIM1->PSC = 0;

#else
	RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;

	// @ 16 mhz base clock, this gives ~1us res
	TIM1->PSC = 8;
#endif

	TIM1->ARR = 200;
	TIM1->CCMR1 =
			TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_0 |
			TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_0 ;
	TIM1->CCR2 = 40;	// 40 us
	TIM1->CCER = TIM_CCER_CC2E;
	TIM1->BDTR = TIM_BDTR_MOE;
	TIM1->CR2 = (6 << TIM_CR2_MMS2_Pos) ;	// OC3 triggers TRGO2 for ADC sync
	TIM1->CR1 |= TIM_CR1_OPM;
}

void tim1_set_ch_pwm(int chno, int value)
{
	if (chno==1)
	{
		if (value>TIM1->ARR)
			TIM1->CCR1=TIM1->ARR;
		else
			TIM1->CCR1 = value;

		TIM1->EGR |= TIM_EGR_UG;	// update timer registers
	};
}

void coil_pulse_init()
{
	init_tim1();
}

void setup_adc_dma()
{
	// initialize the DMA2 CH 3 for ADC1
	DMA2_Channel3->CCR &= ~DMA_CCR_EN; // disable the channel
	DMA2_Channel3->CPAR = (uint32_t)&ADC1->DR;
	DMA2_Channel3->CMAR = (uint32_t)&coil_current_readings;
	DMA2_Channel3->CNDTR = NUM_COIL_SAMPLES;
	DMA2_Channel3->CCR = ((2 << DMA_CCR_MSIZE_Pos) | (2 << DMA_CCR_PSIZE_Pos) | DMA_CCR_MINC | DMA_CCR_CIRC) &  ~DMA_CCR_DIR ;
	DMA2_CSELR->CSELR = (DMA2_CSELR->CSELR & ~(0xF << 8));	// set DMA2 CH3 to trigger from ADC2 (source 0)
	DMA2_Channel3->CCR |= DMA_CCR_EN;

	// setup ADC1 for TIM15 trigger, DMA operation, each trigger will read Channel 5 then Channel 6, 2.5 sample times

	// setup ADC2 sequence to read Channel 5 then Channel 15
	ADC1->SQR1 = (0 << 0) | // Length of sequence = 1
			(5 << 6) |	// first read Channel 5 (direct V_dio) OR channel 6 (V_dio_amp)
			(15 << 12);  // second read output of OPAMP (I coil * PGA)

	ADC1->SMPR1 =
				(0 << 0) | // channel 6 sample time = 2.5
				(0 << 3); // channel 15 sample time = 2.5

	ADC1->CFGR = ADC_CFGR_JQDIS | // injected queue disabled
			ADC_CFGR_OVRMOD |
			ADC_CFGR_EXTEN_1 | // ADC_CFGR_EXTEN_1 |	// hardware trigger on any edge
			(1 << ADC_CFGR_EXTSEL_Pos) | // external source - event 1 - TIM1 CH2
			ADC_CFGR_CONT |
			ADC_CFGR_DMAEN;
			// | ADC_CFGR_DMACFG; ADC1 is used in DMA ONE SHOT mode - requires reset after completion
	delay_us(100);

	// enable ADC2
	ADC1->CR |= ADC_CR_ADEN;
	DMA2->IFCR = DMA_IFCR_CGIF3;	// reset the xfer complete flag
	delay_us(100);
	ADC1->CR |= ADC_CR_ADSTART;		// to-do - trigger this on the falling edge of the pulse output
}

void coil_pulse_trigger()
{
	// turn on opamp
	GPIOA->ODR |= (1 << 7);

	// reset ADC DMA
	setup_adc_dma();

	// trigger pulse output
	TIM1->CR1 |= TIM_CR1_CEN | TIM_CR1_OPM;
}

int coil_pulse_done()
{
	// CEN is cleared by HW in OPM mode
	return ((TIM1->CR1 & TIM_CR1_CEN)==0) && ((DMA2->ISR & DMA_ISR_TCIF3)>0);
}

int calibrate()	// 1 = success, 0 = failure
{
	return 1;
}

float x_arr[NUM_COIL_SAMPLES];
float y_arr[NUM_COIL_SAMPLES];

// do a linear curve fitting to the provided data points, return m and b (as in y = mx + c)

int do_linear_regression(int num_data_points, float *x_arr, float* y_arr, float *m, float *b)
{
	int i;
	float sumx, sumx2, sumy, sumxy;

	sumx = sumx2 = sumy = sumxy = 0;

    for(i=0;i<=num_data_points-1;i++)
    {
        sumx = sumx + x_arr[i];
        sumx2 = sumx2 + x_arr[i]*x_arr[i];
        sumy = sumy + y_arr[i];
        sumxy = sumxy + x_arr[i] * y_arr[i];
    }
    *b =((sumx2 * sumy - sumx * sumxy) * 1.0 / (num_data_points * sumx2 - sumx * sumx)*1.0);
    *m = ((num_data_points * sumxy - sumx * sumy) * 1.0 / (num_data_points * sumx2 - sumx * sumx) * 1.0);
    return 1;
}

/*
void test_lin_estimation()
{
	volatile float r1, r2;

	// test the linear estimation routines
	x_arr[0]=10;
	x_arr[1]=12;

	y_arr[0]=100;
	y_arr[1]=200;

	do_linear_regression(2, x_arr, y_arr, &r1, &r2);
}
*/

void debug_dump_buffer()
{
	int i;

	char vbuff[30];

	for(i=0; i < NUM_COIL_SAMPLES; i++)
	{
		uart_print_hex_byte(i);
		uart_putch(':');
		uart_print_hex_word(coil_current_readings[i]);
		uart_putch(13);
	}

	uart_puts("-----------");
	snprintf_(vbuff, sizeof(vbuff), "hs_m: %.3f", hs_m);
	snprintf_(vbuff, sizeof(vbuff), "hs_c: %.3f", hs_c);
	snprintf_(vbuff, sizeof(vbuff), "ls_m: %.3f", ls_m);
	snprintf_(vbuff, sizeof(vbuff), "ls_e: %.3f\r\n\r\r", ls_e);

	// hs_m, hs_c, ls_m, ls_e;

}

int analyze_results()
{
	hs_m = hs_c = ls_e = ls_m = 0;

	int idx, hs_start, hs_end;

	int ls_start, ls_end, ls_end_avg_value;

	// study the result set

	// high side starts at index 16 and goes until the ADC value goes below 3100

	idx = 16;
	hs_start = idx;
	hs_end = idx;

	while((coil_current_readings[idx] > 3100) && (idx < NUM_COIL_SAMPLES))
		idx++;

	hs_end = idx;
	if (hs_end > hs_start)
	{
		// consider doing linear regression here
		hs_m =  (float)(coil_current_readings[hs_end] - coil_current_readings[hs_start]) / (float)(hs_end - hs_start);
		hs_c = (float)(coil_current_readings[hs_end]) - (hs_m * (float)(hs_end));

		int i;

		for(i=0; i<(hs_end-hs_start); i++)
		{
			x_arr[i]=hs_start + i;
			y_arr[i]=coil_current_readings[hs_start+i];
		}
		do_linear_regression(hs_end-hs_start, x_arr, y_arr, &hs_m, &hs_c);
	}

	while((coil_current_readings[idx] > 2500) && (idx < NUM_COIL_SAMPLES))
		idx++;

	ls_end_avg_value = coil_current_readings[NUM_COIL_SAMPLES-1];

	// solve the logarithmic equation for the decay portion
	ls_start = idx;

	while((coil_current_readings[idx] > (ls_end_avg_value + 50)) && (idx < NUM_COIL_SAMPLES))
		idx++;

	ls_end = idx;

	if (ls_start < ls_end)
	{
		float mv, bv;
		// do logarithmic linear estimation
		int i;

		// first take the logarithmic value of each Y
		for(i=0; i < (ls_end - ls_start); i++)
		{
			x_arr[i]=i;
			y_arr[i]=log(coil_current_readings[i + ls_start] - ls_end_avg_value);
		}

		// do the linear estimation part
		do_linear_regression(ls_end - ls_start, x_arr, y_arr, &mv, &bv);

		//
		ls_e = mv;
		ls_m = bv;	// we aren't so concerned about getting the numerical number, just a relative gauge

		return ls_end - ls_start;
	}
	else
		return 0;
//	debug_dump_buffer();

}

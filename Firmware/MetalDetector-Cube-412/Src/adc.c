/*
 * adc.c
 *
 *  Created on: Jun 22, 2019
 *      Author: stimey
 *
 *  ADC channels assigned as follows:
 *  	ADC #2 - TMR1 triggered - store results via DMA - scanning sequence CH5 / CH15
 *  		CH5  (PA0)	- Coil current across detection diodes input via external OPAMP
 *  		CH6  (PA1) 	- Coil current across detection diodes input via 2nd external OPAMP
 *  		CH15 (PB0)	- Coil current across sense resistor
 *
 *  	ADC #1 - continuous 2 sequence scan with AUTO delay and 16x over-sampling
 *  		CH9  (PA4) 	- Battery Voltage
 *  		CH12 (PB1) 	- Coil HV Supply Voltage
 *
 */

#include "adc.h"
#include <stdint.h>
#include "sys_tick.h"

uint8_t adc1_calibrated, adc2_calibrated;

#include "stm32l4xx.h"


unsigned int bat_hv_results[2];

void setup_adc()	 // setup ADC's
{
	// ------ TURN ON THE CLOCKS TO THE ADC's -------

	// NOTE - NOTE - NOTE : fADC = HCLCK = 48 MHz (Running from MSI should meet 50% requirement)
	RCC->AHB2ENR |= RCC_AHB2ENR_ADCEN;	// turn on the clock to the ADC
	delay_us(100);

	// Exit Power Down, and enable VRegs
	ADC1->CR = (ADC1->CR & ~ADC_CR_DEEPPWD);
	delay_us(100);
	ADC2->CR = (ADC2->CR & ~ADC_CR_DEEPPWD);
	delay_us(100);

	ADC1->CR |= ADC_CR_ADVREGEN; // enable ADC1 VREG
	delay_us(200); // per datasheet, 20 uS setting up time for VREG

	ADC2->CR |= ADC_CR_ADVREGEN; // enable ADC2 VREG
	delay_us(200);// per datasheet, 20 uS setting up time for VREG

	// ------ CALIBRATE ADC's -------

	// turn ADC's OFF during calibration
	ADC1->CR &= ~ADC_CR_ADEN;
	ADC2->CR &= ~ADC_CR_ADEN;

	ADC1->CR &= ~ADC_CR_ADCALDIF; ADC1->CR |= ADC_CR_ADCAL;
	delay_us(100);
	ADC2->CR &= ~ADC_CR_ADCALDIF; ADC2->CR |= ADC_CR_ADCAL;
	delay_us(100);	// 116 ADC clocks = 3 uSec

	// dual channel calibration for ADC1
	// ADC1->CR |= ADC_CR_ADCALDIF; ADC1->CR |= ADC_CR_ADCAL;
	// delay_us(100);	// 116 ADC clocks = 3 uSec


	// ensure clocks are turned ON to DMA1 & 2
	RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN | RCC_AHB1ENR_DMA1EN;
	delay_us(100);



	// initialize the DMA1 CH 1 for ADC1
	DMA2_Channel4->CCR &= ~DMA_CCR_EN; // disable the channel
	DMA2_Channel4->CPAR = (uint32_t)&ADC2->DR;
	DMA2_Channel4->CMAR = (uint32_t)&bat_hv_results;
	DMA2_Channel4->CNDTR = 2;	// 2 transfers
	DMA2_Channel4->CCR = ((2 << DMA_CCR_MSIZE_Pos) | (2 << DMA_CCR_PSIZE_Pos) | DMA_CCR_MINC | DMA_CCR_CIRC) &  ~DMA_CCR_DIR ;
	DMA1_CSELR->CSELR = (DMA1_CSELR->CSELR & ~(0xF << 12));	// set DMA2 CH4 to trigger from ADC2 (source 0)
	DMA2_Channel4->CCR |= DMA_CCR_EN;

	// setup ADC2 to read Channels X and Y
	ADC2->SQR1 = (1 << 0) | // Length of sequence = 2
			(9 << 6) |	// first read Channel 9 PA4
			(16 << 12);  // second read cHANNEL 16 PB1

	ADC2->SMPR1 =
				(4 << 0) | // channel 5 sample time = 47.5
				(4 << 3); // channel 6 sample time = 47.5

	// setup ADC2 for continuous operation, over-sampling, circular DMA buffer - operation with no intervention
	ADC2->CFGR = ADC_CFGR_JQDIS | // injected queue disabled
			ADC_CFGR_OVRMOD |
			ADC_CFGR_DMAEN |
			ADC_CFGR_CONT |
			ADC_CFGR_DMACFG; // ADC circular mode

			// ADC_CFGR_EXTEN_0 |	// SOFTWARE TRIGGERED
			// (0 << ADC_CFGR_EXTSEL_Pos); // external source

	// configure 16 x over sampling
	ADC2->CFGR2 =
			ADC_CFGR2_OVSR_0 | ADC_CFGR2_OVSR_1 | // 16x over-sampling
			// ADC_CFGR2_OVSS_2 | // shift the results 4 bits to the left - i.e. normalize the results
			ADC_CFGR2_ROVSE; // regular over-sampling enabled

	// enable ADC2
	ADC2->CR |= ADC_CR_ADEN;
	delay_us(100);
	ADC2->CR |= ADC_CR_ADSTART;
}

void reset_adc1_dma()
{
	// reset ADC1 and DMA for next pulse transfer
}

int adc1_dma_done()
{
	// return TRUE if all transfers have been completed

	return 0;
}

void shutdown_adc() // shutdown the ADC's for sleep mode
{
	// stop any pending conversions

	// reset DMA transfers

	// disable ADC

	// power down DMA

	RCC->AHB2ENR &= ~RCC_AHB2ENR_ADCEN;	// turn OFF the clock to the ADC
}

int read_hv_charge_level()
{
	return bat_hv_results[0];
}

int read_battery_level()
{
	return bat_hv_results[1];
}

int read_hv_charge_levelx10()
{	// 47K/2K resistor divider = 70V max @ 32760
	return (bat_hv_results[1] + 44) / 88;
}

int read_battery_levelx10()
{	// 3.3V * 58K/10K = 19.2V as maximum voltage @ 32767 ADC value
	return (bat_hv_results[0] + 157) / 315; // 4 is the diode drop
};

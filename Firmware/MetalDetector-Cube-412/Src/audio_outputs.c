/*
 * audio_outputs.c
 *
 *  Created on: Jun 2, 2019
 *      Author: stimey
 */


// We use TIM2 PWM mode to generate the speaker outputs for the metal detector
#include <coil_handler.h>
#include "stm32l4xx.h"
#include "audio_outputs.h"
#include "math.h"

// PA3 (CH4) is SPEAKER, PA5 (CH1) is Head Phone
// TIM2 is PWM with a 20 uSec period
//
// CH1 and CH4 - SPEAKER and HEAD PHONE
// TIM2 is fed from SYS_CLOCK at 16 MHz (so PWM range is 0 - 319, VOLUME range is 0-159)
//
// DMA1 CH4 is used to update PWM registers
//
// // TIM16_UP is used to fire DMA channel 3, DMA channel 3 writes to TIM2 PWM registers
//


#define SINE_TABLE_SIZE 200
const int8_t base_sine_table[SINE_TABLE_SIZE] =
		{
		0,   3,   7,  11,  15,  19,  23,  27,  31,  35,  39,  43,  46,  50,  54,  57,  61,  64,  68,  71,  74,  77,  80,  83,  86,  89,  92,  95,  97, 100, 102, 105, 107, 109, 111, 113, 114, 116, 118, 119, 120, 121, 123, 123, 124, 125, 125, 126, 126, 126,
		127, 126, 126, 126, 125, 125, 124, 123, 123, 121, 120, 119, 118, 116, 114, 113, 111, 109, 107, 105, 102, 100,  97,  95,  92,  89,  86,  83,  80,  77,  74,  71,  68,  64,  61,  57,  54,  50,  46,  43,  39,  35,  31,  27,  23,  19,  15,  11,   7,   3,
		0,  -3,  -7, -11, -15, -19, -23, -27, -31, -35, -39, -43, -46, -50, -54, -57, -61, -64, -68, -71, -74, -77, -80, -83, -86, -89, -92, -95, -97,-100,-102,-105,-107,-109,-111,-113,-114,-116,-118,-119,-120,-121,-123,-123,-124,-125,-125,-126,-126,-126,
		-127,-126,-126,-126,-125,-125,-124,-123,-123,-121,-120,-119,-118,-116,-114,-113,-111,-109,-107,-105,-102,-100, -97, -95, -92, -89, -86, -83, -80, -77, -74, -71, -68, -64, -61, -57, -54, -50, -46, -43, -39, -35, -31, -27, -23, -19, -15, -11,  -7,  -3
		};

#define SPKR_BUFFER 0
#define HPHONE_BUFFER 3

// buffer to hold waveforms for DMA xfer
uint16_t audio_buffer[SINE_TABLE_SIZE];

int speaker_max_vol_pct, headphone_max_vol_pct;

void init_audio_outputs(void)
{

	speaker_max_vol_pct = 200;
	headphone_max_vol_pct = 400;

	RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN;
	TIM2->ARR = 120;
	// TIM2->RCR = 0;	// update after every timer cycle

	TIM2->CR2 |= TIM_CR2_CCDS; // dma request on update

	// set all channels to PWM
	TIM2->CCMR1 = TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_0 ;
	TIM2->CCMR2 = TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4M_0 ;

	TIM2->CCER = TIM_CCER_CC1E | TIM_CCER_CC1P |
					TIM_CCER_CC4E | TIM_CCER_CC4P;	// set polarity and enables

	// enable the counter
	TIM2->CR1 &= ~TIM_CR1_UDIS; // auto update on change
	TIM2->CR1 |= TIM_CR1_ARPE | TIM_CR1_URS;	// dma request only on overflow, ARR is buffered

	TIM2->DIER |= TIM_DIER_UDE;					// enable DMA updates
	TIM2->DCR = 0x000D;	// 1 xfer / burst, CCR1 is update target
	TIM2->CR1 |= TIM_CR1_URS;
	TIM2->CR1 |= TIM_CR1_CEN;

	// initialize the DMA, CH2
	RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;

	// configure DMA for 16 bit transfers, increment memory address, memory to peripheral, circular buffer
	DMA1_Channel2->CCR &= ~DMA_CCR_EN; // disable the channel
	DMA1_Channel2->CCR = (1 << DMA_CCR_MSIZE_Pos) | (2 << DMA_CCR_PSIZE_Pos) | DMA_CCR_MINC | DMA_CCR_DIR | DMA_CCR_CIRC;
	DMA1_Channel2->CNDTR = SINE_TABLE_SIZE;
	DMA1_Channel2->CMAR = (uint32_t)&audio_buffer;
	DMA1_CSELR->CSELR = (DMA1_CSELR->CSELR & ~(0xF << 4)) | (4 << 4);	// set DMA1 CH3 to trigger from TIM2
	DMA1_Channel2->CPAR = (uint32_t)&TIM2->DMAR;

	DMA1_Channel2->CCR |= DMA_CCR_EN;

	set_audio_volume(0);
}

uint16_t currvol;


// intended to be called from detection logic - provides audio feedback
void set_audio_volume(uint16_t new_vol) // pct * 10
{
	int halfvol, basevol, ctr;

	// if detection level > 10%, enable audio
	if (new_vol > 100)
		GPIOB->ODR |= (1 << 4);
	else
		GPIOB->ODR &= ~(1 << 4);

	currvol = new_vol;
	if (currvol > 1000)
		currvol = 1000;

	if (headphone_max_vol_pct > 1000)
		headphone_max_vol_pct = 1000;
	if (speaker_max_vol_pct > 1000)
		speaker_max_vol_pct = 1000;

	halfvol = (TIM2->ARR-8) / 2;
	for(ctr=0; ctr < SINE_TABLE_SIZE; ctr++)
	{
		uint16_t outv;
		basevol = ((base_sine_table[ctr] * halfvol * currvol) / 1000) / 128;
		outv = basevol * speaker_max_vol_pct / 1000 + halfvol;

		audio_buffer[ctr] = outv;
	}
}

void set_speaker_max_vol(uint16_t maxvol) // pct * 10
{
	speaker_max_vol_pct = maxvol;
	set_audio_volume(currvol);
}

uint16_t get_speaker_max_vol() // pct * 10
{
	return speaker_max_vol_pct;
}

void set_headphone_max_vol(uint16_t maxvol) // pct * 10
{
	headphone_max_vol_pct = maxvol;
	set_audio_volume(currvol);
}

uint16_t get_headphone_max_vol()	// pct * 10
{
	return headphone_max_vol_pct;
}



/*
 * PI Metal Detector
 * John Clymer
 * July 2019 v1
 * Mar 2023 V2
 *
 * Implement a PI metal detector using STM32L412 or STM32F303
 *
 * See gpio.c for pin assignment and peripheral usage
 *
 * DMA's assigned:
 * 	DMA1: 	CH1 = ADC1, CH2=ADC2 CH6=TIM2 update of CCR1 - CCR4 in burst mode
 * 	DMA2:	CH6=I2C1_TX, CH7=I2C1_RX
 *
 */

#include "printf.h"
#include "coil_handler.h"
#include "main.h"
#include <stdint.h>
#include "sys_tick.h"
#include "gpio.h"
#include "uart.h"
#include "audio_outputs.h"
#include "oled.h"
#include "adc.h"
#include "click.h"
#include "sw_quadrature.h"
#include "menu.h"
#include "hv_pwm.h"
#include <math.h>
#include <stdlib.h>

uint32_t MCU_Reset_Reasons;


#include "stm32l412xx.h"

void init_sys()
{
	int i;

  /* Set Interrupt Group Priority */
  SCB->AIRCR = 0x05fa0000 | 0300;	// magic code and new configuration

  MCU_Reset_Reasons = RCC->CSR >> 24;
  RCC->CSR |= RCC_CSR_LSION | RCC_CSR_RMVF;	// clear reset flags, turn LSI on

  // turn on HSI and MSI, HSI = 16 MHz
  RCC->CR |= RCC_CR_HSION | RCC_CR_MSION;
  while(!(RCC->CR & RCC_CR_HSIRDY)) /* NOTHING */ ; // wait for HSI startup

    /* Enable systick and configure 1ms tick (default clock after Reset is HSI) */
  SysTick->LOAD  = (F_CPU / 1000) - 1;                         /* set reload register */
  SysTick->VAL   = 0;                                             /* Load the SysTick Counter Value */
  SysTick->CTRL  = 7;                         /* Enable SysTick IRQ and SysTick Timer */

  RCC->CFGR = (RCC->CFGR & ~(0x03)) | 0x01; // switch to HSI16

  // enable 2 wait states, pre-fetch enabled
  FLASH->ACR = (FLASH->ACR & ~0x07) | 0x0102; // prefetch enabled, 2 wait states for 48 MHz operation
  for(i=0; i< 1000; i++) /* NOTHING */ ; // busy wait loop

  while(!(RCC->CR & RCC_CR_MSIRDY)) /* NOTHING */ ; // wait for MSI to be ready before switching frequencies
  RCC->CR = (RCC->CR & ~0x000000F0) | 0x000000B0;	// set MSI range 11 - 48 MHz
  RCC->CR |= 0x00000008;							// change MSI range

  for(i=0; i< 1000; i++) /* NOTHING */ ; // busy wait loop
  while(!(RCC->CR & RCC_CR_MSIRDY)) /* NOTHING */ ; // wait for MSI ready

  RCC->CR &= ~0x00000008;	// reset MSI change request

  // switch to MSI
  RCC->CFGR = (RCC->CFGR & ~(0x03)); // switch to MSI

  RCC->CCIPR = // 0x0028_2008;	// ADC clocked by system clock; LPTIM clocked by LSI; I2C, USART2 clocked by HSI
		  (2 << RCC_CCIPR_LPTIM2SEL_Pos) | 	// LPTIM2 clocked by HSI - HV PWM
		  (1 << RCC_CCIPR_LPTIM1SEL_Pos) | 	// LPTIM1 clocked by LSI - RTC
		  (2 << RCC_CCIPR_I2C1SEL_Pos) | 	// I2C clocked by HSI
		  (2 << RCC_CCIPR_USART1SEL_Pos)| 	// USART1 clocked by HSI
		  (2 << RCC_CCIPR_USART2SEL_Pos);	// USART2 clocked by HSI

  // enable separate usage, bus, etc faults to aid in troubleshooting
  SCB->SHCSR |= SCB_SHCSR_USGFAULTENA_Msk | SCB_SHCSR_BUSFAULTENA_Msk | SCB_SHCSR_MEMFAULTENA_Msk;

  RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN | RCC_AHB1ENR_DMA2EN;

  // Enable hardware FPU
  SCB->CPACR |= ((3UL << 10*2) | /* set CP10 Full Access */
                 (3UL << 11*2) ); /* set CP11 Full Access */
}


int disp_update_cnt;	//used to toggle active selection @ 400 ms on/off rate

typedef enum TKNOB_MODE {
	KNOB_MODE_NONE,			// knob rotation changes nothing
	KNOB_MODE_VOLUME,		// knob rotation changes volume
	KNOB_MODE_VCHG,			// knob rotation changes Vchg
	KNOB_MODE_TPULSE,		// knob rotation changes Tpulse
	KNOB_MODE_EOL
} TKNOB_MODE;

TKNOB_MODE knob_mode;


char *fp10_to_str(char *buff, int val)	// fixed format XX.X or -X.X
{
	int wp=val / 10;
	*(buff++) = ((wp / 10) % 10) | 0x30;
	*(buff++) = (wp % 10) | 0x30;
	*(buff++) = '.';
	*(buff++) = (val % 10) | 0x30;
	*(buff) = 0;	// null terminate the string
	return(buff);
}

#define NUMERIC_FONT UniFont_8x16
//#define NUMERIC_FONT Font_11x18
#define LABEL_FONT Font_7x10

void dispfp10(char *label, int val, int x, int y)
{
  OLED_SetCursor(x, y+2);
  OLED_WriteString(label, LABEL_FONT, White);
  OLED_SetCursor(x+1+4*LABEL_FONT.FontWidth,y);

  if (val < 0)
  {
	  val=-val;
	  if (val>99) val=99;	// -9.9 is lowest possible number
	  OLED_WriteChar('-', NUMERIC_FONT , White);
  }
  else if (val < 100)
  {
	  OLED_WriteChar(' ', NUMERIC_FONT , White);
  }
  else
  {
	  if (val>999) val=999;
	  OLED_WriteChar((val / 100) | 0x30, NUMERIC_FONT , White);
  }

  val = val % 100;
  OLED_WriteChar((val / 10) | 0x30, NUMERIC_FONT , White);
  // current column = x + 29 + 22
  OLED_DrawPixel(x+29+2*NUMERIC_FONT.FontWidth,y+14, White);
  OLED_DrawPixel(x+29+2*NUMERIC_FONT.FontWidth,y+15, White);
  OLED_DrawPixel(x+29+1+2*NUMERIC_FONT.FontWidth,y+14, White);
  OLED_DrawPixel(x+29+1+2*NUMERIC_FONT.FontWidth,y+15, White);

  OLED_SetCursor(x+29+2+2*NUMERIC_FONT.FontWidth, y);
  OLED_WriteChar((val % 10) | 0x30, NUMERIC_FONT , White);
}

void disp_volume(int vol)
{
	int hght, startpos, i;
	OLED_COLOR pix_color;

	// display "Vol" on bottom right
	OLED_SetCursor(OLED_WIDTH-22, OLED_HEIGHT-11);
	if ((disp_update_cnt & 0x04) && (knob_mode==KNOB_MODE_VOLUME))
		OLED_WriteString("   ", Font_7x10, White);
	else
		OLED_WriteString("Vol", Font_7x10, White);

	hght = OLED_HEIGHT-12;
	startpos = hght * (1000-vol) / 1000;

	for(i=0; i < hght; i++)
	{
		if (i < startpos) pix_color=Black;
		else pix_color = White;

		OLED_DrawPixel(OLED_WIDTH-3, i, pix_color);
		OLED_DrawPixel(OLED_WIDTH-4, i, pix_color);
		OLED_DrawPixel(OLED_WIDTH-5, i, pix_color);
		OLED_DrawPixel(OLED_WIDTH-6, i, pix_color);
	}
}

unsigned int graph_buffer_max_value=3200;
unsigned int graph_buffer_min_value=1024;

// how many samples in the buffer to make filtered ls_e
#define MAX_LS_E_ARR 8

float tgt_ls_e = 0.116;	// our "target" ls_e, deviations from this result in audio signals

int ls_e_cnt = 0;	// how many ls_e's have we processed since triggered
float ls_e_arr[MAX_LS_E_ARR];
float filtered_ls_e;

uint8_t graph_buffer[100];

void disp_graph()
{
	// display area = 100 pixels wide (0.5 uSec/column), 48 pixels tall
	int OFFSET_X = 2;
	int OFFSET_Y = 15;
	int FRAME_HEIGHT = 48;
	int FRAME_WIDTH = 100;

	int thisval;

	int idx;
	int x, y;
	char vbuff[20];

	x = OFFSET_X + FRAME_WIDTH / 2;
	y = OFFSET_Y;

	OLED_SetCursor(x, y+5);
	snprintf_(vbuff, sizeof(vbuff), "le %5.3f", filtered_ls_e);
	OLED_WriteString(vbuff, LABEL_FONT, White);


	for(idx=0; idx < FRAME_WIDTH ; idx++)
	{
		thisval = coil_current_readings[idx];
		if (thisval < graph_buffer_min_value)
			thisval = graph_buffer_min_value;
		else if (thisval > graph_buffer_max_value)
			thisval = graph_buffer_max_value;

		thisval = (thisval - graph_buffer_min_value) * FRAME_HEIGHT / (graph_buffer_max_value - graph_buffer_min_value);

		if (thisval<=FRAME_HEIGHT)
		{
			OLED_DrawPixel(OFFSET_X+idx, OFFSET_Y+FRAME_HEIGHT-thisval, White);
		}
	}
}



void shift_graph_buffer()
{
	int idx;

	idx = (sizeof(graph_buffer)/sizeof(graph_buffer[0]))-2;
	while(idx >= 0)
	{
		graph_buffer[idx+1]=graph_buffer[idx];
		idx--;
	}
}

int calibrated;
#define TRIG_PRESSED (~(GPIOA->IDR >> 6) & 1)

void update_display(int vbat10, int vchg10, int vol, int vchg_tgt)
{
	disp_update_cnt++;

	OLED_Fill(Black);

	dispfp10("Vbat", vbat10, 4, 1);

	if ((disp_update_cnt & 0x04) && (knob_mode==KNOB_MODE_VCHG))
		dispfp10("Vtgt", vchg_tgt, 60, 1);
	else
		dispfp10("Vchg", vchg10, 60, 1);


	if ((calibrated==0) && (TRIG_PRESSED))
	{
		OLED_SetCursor(2, 38);
		OLED_WriteString("Press Knob 1 Second", LABEL_FONT, White);
		OLED_SetCursor(2, 50);
		OLED_WriteString("to Calibrate", LABEL_FONT, White);
	}

	disp_volume(vol);
	disp_graph();

	GPIOB->ODR = GPIOB->ODR | (1 << 3); // LED on

	OLED_UpdateScreen();

	GPIOB->ODR = GPIOB->ODR & ~(1 << 3); // LED off
}

// uint8_t vchg_buffer[200];

uint32_t last_pulse_time;

#define OK_BTN_PRESSED (~(GPIOB->IDR >> 5) & 1)





int main(void)
{
	int32_t vbat10, vchg10, volume, last_tick;

	volatile int32_t vbat_reading, vchg_target;

	int this_ok_pressed, last_ok_pressed;
    int this_knob_pos, last_knob_pos, knob_change;
    int pulse_sent;

    calibrated = 0;
    last_knob_pos = 0;

  init_sys();

  init_port_a();
  init_port_b();
  init_port_c();

  init_hv_pwm();
  set_hv_target_voltage(0);

  uart_init(115200);	// bring up debug port

  OLED_Init();

  init_audio_outputs();

  setup_adc();

  coil_pulse_init();

  knob_mode = KNOB_MODE_VOLUME;

  volume = 400;	// default to 40% - TO-DO: Retain in a Flash memory block
  set_speaker_max_vol(volume);	// set the audio at full volume

  vchg_target=200; // 20 volts - valid range 15 - 40v

  while(1)
  {
      if (get_system_ticks()-last_tick > 100)
      {
    	  last_tick=get_system_ticks();

    	  	//
    	  vbat_reading = read_battery_levelx10();	// approximate - gives full scale of 12.8 volts
    	  vbat10 = (vbat10 + vbat_reading) / 2; // 2 input IIR filter

    	  // ~ 20 counts per 100 milli-volts
    	  if (quad_ok_pressed())
    	  {

    	  }
    	  else
    	  {
			  vchg10 = read_hv_charge_levelx10();
    	  }

          update_display(vbat10, vchg10, volume, vchg_target);
      }

      // handle knob presses - short press = change knob parameter, long press (1 sec) = enter menu system
      this_ok_pressed = quad_ok_pressed();
	  if (this_ok_pressed && !last_ok_pressed)
	  {
		  // change selection parameter (Vchg, Volume, Tpulse, etc)
		  knob_mode = knob_mode + 1;
		  if (knob_mode == KNOB_MODE_EOL) knob_mode=KNOB_MODE_NONE;
	  }
	  last_ok_pressed = this_ok_pressed;

	  if (TRIG_PRESSED)
	  {
	  	  set_hv_target_voltage(vchg_target);
	  	  if (((get_system_ticks()-last_pulse_time) > 4) && (vchg_target-3 < vchg10))
	  	  {
	  		  last_pulse_time = get_system_ticks();

	  		  // trigger a coil pulse
	  		  coil_pulse_trigger();
	  		  pulse_sent = 1;
	  	  }
	  	  else if (coil_pulse_done() && pulse_sent)
		  {
	  		  pulse_sent = 0;

	  		  // for ~ 30 samples, this takes ~ 2 milliseconds
	  		  // TO-DO : re-do the math using either :  1) LOG lookup table and/or 2) 64 bit fixed point math
 			  analyze_results();

 			  ls_e_arr[(ls_e_cnt++) % MAX_LS_E_ARR] = ls_e;

 			  if (ls_e_cnt > 3)
 			  {
 				  int i;
 				  float ls_e_tot;

 				  ls_e_tot = 0;
 				  for(i=0; i < MAX_LS_E_ARR; i++)
 					  ls_e_tot += ls_e_arr[i];

 				  filtered_ls_e = ls_e_tot / MAX_LS_E_ARR;
 			  }
 			  else
 				  filtered_ls_e = tgt_ls_e;	// to keep the beeper silenced

 			 if (calibrated)
			 {
				 uint16_t newvol;
				 float fnewvol=(fabs(tgt_ls_e) - fabs(filtered_ls_e)) * 100000.0;
				 newvol = abs(fnewvol);
				 if (newvol > 1000)
					 newvol = 1000;	// limit to 100%

				 set_audio_volume(newvol);
			 }
 			 else
 				 set_audio_volume(0);

		  }
	  }
	  else
	  {
		  int i;

		  // ensure 0 volume
		  set_audio_volume(0);

		  // zero out the display buffer
		  for(i=0; i < sizeof(graph_buffer)/sizeof(graph_buffer[0]); i++)
			  graph_buffer[i]=0;

		  for(i=0; i < sizeof(coil_current_readings)/sizeof(coil_current_readings[0]); i++)
			  coil_current_readings[i]=coil_current_readings[i] * 0.9;

		  ls_e_cnt = 0;	// how many ls_e's have we processed since triggered
		  filtered_ls_e = 0;

		  // turn off charge pump
		  set_hv_target_voltage(0);
	  };


	  if (quad_ok_pressed_time()>1000)
	  {
		  if ((TRIG_PRESSED) && (vchg_target-3 < vchg10))
		  {
			  // calibration
			  calibrated = 1;
			  tgt_ls_e = filtered_ls_e;
		  }
		  // enter menu mode
		  reset_ok_pressed_time();
	  }
	  else
		  GPIOA->ODR &= ~(1 << 6); // low output = disabled

	  // handle knob rotation
	  this_knob_pos = quad_get_position();
	  if (this_knob_pos != last_knob_pos)
	  {
		  knob_change = this_knob_pos - last_knob_pos;
		  switch(knob_mode)
		  {
		  case KNOB_MODE_VOLUME:
			  volume+=(knob_change * 5);

			  if (volume < 0) volume = 0;
			  else if (volume > 100) volume = 100;

			  set_speaker_max_vol(volume*10);
			  break;

		  case KNOB_MODE_VCHG:
			  vchg_target += knob_change;
			  if (vchg_target < 100)
				  vchg_target = 100;
			  if (vchg_target > 500)
				  vchg_target = 500;
		  	  break;

		  case KNOB_MODE_TPULSE:
		  	  break;

		  case KNOB_MODE_EOL:
		  case KNOB_MODE_NONE:
			  break;
		  }
	  }
	  last_knob_pos = this_knob_pos;
  }
}

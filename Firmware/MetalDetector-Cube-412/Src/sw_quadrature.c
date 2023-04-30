/*
 * sw_quadrature.c
 *
 *  Created on: Jul 10, 2019
 *      Author: stimey
 */

#include "sw_quadrature.h"
#include "sys_tick.h"
#include "gpio.h"

// enable the following DEFINE for encoders with 4 counts per detent
// this enables counting by 4, but adding 2 counts as an offset
// this helps for when a count is missed or the detents don't properly line up on the encoder quite right

#define ENC_4_COUNTS_PER_DETENT

#include "stm32l4xx.h"

volatile int int_quad_position;
volatile int quad_position;
volatile int btn_pressed_time;

void service_sw_quadrature(void)
{
	static unsigned skip_count=0;	// number of times we skipped 2 quadrature ticks
	static unsigned ran_once=0;
	static unsigned char ticks_since_changed=0;

	static unsigned short lastinpval;

	unsigned short inpval = (GPIOC->IDR >> 14) & 0x03;
	int btn_pressed;

	if (ran_once==0)
	{
		ran_once=1;
		lastinpval = inpval;
	}

    // check for new position
    if (inpval!=lastinpval)
    {
    	if (((lastinpval==0) && (inpval==2)) ||
    			((lastinpval==2) && (inpval==3)) ||
				((lastinpval==3) && (inpval==1)) ||
				((lastinpval==1) && (inpval==0)))
    	{
    		int_quad_position=int_quad_position-1;
    	}
    	else if (((lastinpval==0) && (inpval==1)) ||
    			((lastinpval==1) && (inpval==3)) ||
				((lastinpval==3) && (inpval==2)) ||
				((lastinpval==2) && (inpval==0)))
    	{
    		int_quad_position=int_quad_position+1;
    	}
    	else
    		skip_count++;

    	ticks_since_changed=0;
    }
#ifdef ENC_4_COUNTS_PER_DETENT
    else
    {
    	if (ticks_since_changed > 100)
    	{
    		int_quad_position=(int_quad_position & ~0x3) | 2;
    	}
    	else
    		ticks_since_changed++;
    }

    quad_position=int_quad_position / 4;
#else
    quad_position=int_quad_position;
#endif

    lastinpval = inpval;

    btn_pressed = ((GPIOB->IDR & (1 << 5))==0);

    // TO-DO - better debouncing on the button press / release
    if (btn_pressed) btn_pressed_time++;
    else			 btn_pressed_time=0;

    if (btn_pressed_time>32000) btn_pressed_time=32000;
}


volatile int quad_get_position()
{
	return quad_position;
}

volatile int quad_ok_pressed()
{
	return btn_pressed_time > 20;
}

volatile int quad_ok_pressed_time()
{
	return btn_pressed_time;
}

void reset_ok_pressed_time()
{
	btn_pressed_time=0;
}

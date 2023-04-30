/*
 * sys_tick.h
 *
 *  Created on: Jun 2, 2019
 *      Author: stimey
 */

#ifndef INC_SYS_TICK_H_
#define INC_SYS_TICK_H_

#include <stdint.h>

extern volatile uint32_t systick_overflows;

uint32_t get_system_ticks(void);

void delay_us(uint32_t dly);
void delay_ms(uint32_t dly);

#endif /* INC_SYS_TICK_H_ */

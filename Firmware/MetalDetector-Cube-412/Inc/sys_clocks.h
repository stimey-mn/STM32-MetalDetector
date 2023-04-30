#ifndef __SYS_CLOCKS_H__
#define __SYS_CLOCKS_H__

#include <stdint.h>

void init_clock();

extern void delay_us(uint32_t us);

extern volatile uint32_t tick_count;

#endif // __SYS_CLOCKS_H__

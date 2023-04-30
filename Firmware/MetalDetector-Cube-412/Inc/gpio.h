/*
 * gpio.h
 *
 *  Created on: Jun 2, 2019
 *      Author: stimey
 */

#ifndef GPIO_H_
#define GPIO_H_

#include "main.h"

#ifdef STM32F303X8

#include "stm32f303x8.h"

#else

#include "stm32l4xx.h"

#endif

void init_port_a();
void init_port_b();
void init_port_c();


#endif /* GPIO_H_ */

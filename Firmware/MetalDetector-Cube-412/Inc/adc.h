/*
 * adc.h
 *
 *  Created on: Jun 22, 2019
 *      Author: stimey
 */

#ifndef __ADC_H__
#define __ADC_H__

#include <stdint.h>

void setup_adc();	 // setup ADC's
void shutdown_adc(); // shutdown the ADC's for sleep mode

int read_hv_charge_level();
int read_battery_level();

int read_hv_charge_levelx10();
int read_battery_levelx10();

#endif /* ADC_H_ */

/*
 * tim1.h
 *
 *  Created on: Jul 7, 2019
 *      Author: stimey
 */

#ifndef COIL_HANDLER_H_
#define COIL_HANDLER_H_

#define NUM_COIL_SAMPLES 100
extern int coil_current_readings[NUM_COIL_SAMPLES];

void init_tim1(void);
void tim1_set_ch_pwm(int chno, int value);

void coil_pulse_init();
void coil_pulse_trigger();
int coil_pulse_done();

int analyze_results();
extern float hs_m, hs_c, ls_m, ls_e;	// high side slope and offset, low side co-efficient and exponent

#endif /* COIL_HANDLER_H_ */

/*
 * lptim2.h
 *
 *  Created on: Apr 12, 2020
 *      Author: stimey
 *
 * LPTIM2 drives the HV PWM.
 *
 *
 *
 *
 *
 */

#ifndef HV_PWM_H_
#define HV_PWM_H_

void init_hv_pwm();
void shutdown_hv_pwm();
void set_hv_target_voltage(int voltsX10);

#endif /* HV_PWM_H_ */

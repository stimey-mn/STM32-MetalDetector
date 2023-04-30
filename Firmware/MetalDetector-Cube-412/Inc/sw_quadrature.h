/*
 * sw_quadrature.h
 *
 *  Created on: Jul 10, 2019
 *      Author: stimey
 */

#ifndef SW_QUADRATURE_H_
#define SW_QUADRATURE_H_

volatile int quad_get_position();
volatile int quad_ok_pressed();

// how long the OK button has been held
volatile int quad_ok_pressed_time();
void reset_ok_pressed_time();

// the following routine should get called on a regular / periodic basis -
// fast enough that it won't miss a quadrature count

void service_sw_quadrature(void);

#endif /* SW_QUADRATURE_H_ */

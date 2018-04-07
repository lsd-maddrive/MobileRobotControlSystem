/* 
 * File:   motor_control.h
 */

#ifndef MOTOR_CONTROL_H
#define	MOTOR_CONTROL_H

#include <xc.h>
#include "hard.h"

/************************** PUBLIC FUNCTION **************************/
void motor_a_set_power(int16_t power);
void motor_b_set_power(int16_t power);
void motors_stop();
void motor_init();

#endif	/* MOTOR_CONTROL_H */


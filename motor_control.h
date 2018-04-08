/* 
 * File:   motor_control.h
 */

#ifndef MOTOR_CONTROL_H
#define	MOTOR_CONTROL_H

#include <xc.h>
#include "hard.h"

enum
    {
        CLOCK_FREQUENCY = 16000000,
        PWM_FREQUENCY = 16000,
        PWM_PRESCALER = 1,
        PWM_PERIOD = CLOCK_FREQUENCY/( PWM_FREQUENCY * PWM_PRESCALER) - 1,  // max 32767
        P1DC_MAX = PWM_PERIOD,
        P1DC_MIN = 0,        
    };
/************************** PUBLIC FUNCTION **************************/
void motor_a_set_power(int8_t power);
void motor_b_set_power(int8_t power);
void motors_stop();
void motor_init();

#endif	/* MOTOR_CONTROL_H */


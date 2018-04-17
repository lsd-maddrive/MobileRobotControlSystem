/* 
 * File:   motor_control.h
 */

#ifndef MOTOR_CONTROL_H
#define	MOTOR_CONTROL_H

#include <xc.h>
#include "hard.h"

/* Описание двигателей:
 * PWM 5, 6 - один двигатель
 * PWM 3, 11 - другой двигатель
 */

/****************************** PUBLIC FUNCTION ******************************/
void motor_set_power(int8_t power, uint8_t motor);
void motors_stop();
void motor_init();
/****************************** PUBLIC FUNCTION ******************************/
#endif	/* MOTOR_CONTROL_H */


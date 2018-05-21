/* 
 * File:   motor_control.h
 */

#ifndef MOTOR_CONTROL_H
#define	MOTOR_CONTROL_H

#include <xc.h>
#include "hard.h"

/* Описание драйвера для двигателя:
 * PWM 5, 6 - правый двигатель
 * PWM 3, 11 - левый двигатель
 */

void motor_set_power(int8_t power, uint8_t motor);
void motors_stop();
void motor_init();

#endif	/* MOTOR_CONTROL_H */


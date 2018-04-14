/* 
 * File:   robot_control.h
 */

#ifndef ROBOT_CONTROL_H
#define	ROBOT_CONTROL_H

#include <xc.h>
#include "encoder.h"
#include "hard.h"
#include "motor_control.h"
#include "timer.h"

void init_periphery();          // Инициализация всей переферии
void test_motor_control();      // Проверка работы модуля motor_control
void test_uart();               // Проверка работы модуля uart
void test_encoder();            // Проверка работы модуля encoder по uart
void test_software_timer();     // Проверка работы модуля software_timer
#endif	/* ROBOT_CONTROL_H */


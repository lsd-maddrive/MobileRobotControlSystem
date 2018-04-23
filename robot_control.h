/* 
 * File:   robot_control.h
 */

#ifndef ROBOT_CONTROL_H
#define	ROBOT_CONTROL_H

#include "encoder.h"
#include "hard.h"
#include "motor_control.h"
#include "rangefinder.h"
#include "timer.h"
#include "uart.h"

void init_periphery();          // Инициализация всей переферии

void turn_around_to(int16_t);   // Поворот на указанный угол


void test_motor_control();      // Проверка работы модуля motor_control
void test_uart();               // Проверка работы модуля uart
void test_software_timer();     // Проверка работы модуля software_timer
void test_encoder();            // Проверка работы модуля encoder по uart
void test_rangefinder();        // Проверка работы модуля rangefinder

#endif	/* ROBOT_CONTROL_H */


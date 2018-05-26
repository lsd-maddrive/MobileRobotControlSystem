/* 
 * File:   robot_test.h
 */

#ifndef ROBOT_TEST_H
#define	ROBOT_TEST_H

#include "robot_control.h"

void test_motor_control();      // Проверка работы модуля motor_control
void test_uart();               // Проверка работы модуля uart
void test_software_timer();     // Проверка работы модуля software_timer
void test_encoder();            // Проверка работы модуля encoder по uart
void test_rangefinder();        // Проверка работы модуля rangefinder
void test_turn_around_by();     // Проверка базового действия - поворота двигателя
void test_move_to();            // Проверка базового действия - перемещение по координатам

#endif	/* ROBOT_TEST_H */


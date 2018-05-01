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

typedef struct 
{
    int16_t x;
    int16_t y;
    int16_t angle;
    uint16_t range;
    uint8_t minSpeed;
    uint8_t maxSpeed;
    uint8_t currentSpeed;
    uint8_t acceleration;
    uint8_t deceleration;
} Robot_data;

void init_periphery();          // Инициализация всей переферии
void turn_around_to(int16_t);   // Поворот на указанный угол

#endif	/* ROBOT_CONTROL_H */


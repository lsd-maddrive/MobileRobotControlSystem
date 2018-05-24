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

typedef enum 
{
    ROBOT_INITIALIZED = 1,
    ROBOT_IN_PROGRESS = 2,
    ROBOT_FINISHED = 3,
    ROBOT_CANT_MOVE = 4,
} Robot_status;

typedef struct 
{
    Robot_status status;
    uint8_t currentSpeed;
    int8_t speedDifference;
    int16_t x;
    int16_t y;
    int16_t angle;
    uint16_t range;
    
    uint8_t minSpeed;
    uint8_t maxSpeed;
    uint8_t acceleration;
    uint8_t deceleration;
} Robot_data;



void turn_around_to(int16_t angle);     // Поворот на указанный угол
void move_to(int16_t x, int16_t y);     // Движение к указанной координате
void move_forward(uint16_t distance);   // Движение вперед на указанное расстояние

void init_periphery();                  // Инициализация всей переферии

void move_with_obstacle_avoidance_get_coordinates(int16_t x, int16_t y);
void move_with_obstacle_avoidance_do();

void log_transmit();

#endif	/* ROBOT_CONTROL_H */


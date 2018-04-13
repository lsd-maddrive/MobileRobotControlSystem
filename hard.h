/* 
 * File:   hard.h
 */

#ifndef HARD_H
#define	HARD_H

#include <xc.h>

enum
{
    MOTOR_A_1,
    MOTOR_A_2,
    MOTOR_B_1,
    MOTOR_B_2,
};

enum
{
    CLOCK_FREQUENCY = 16000000UL,
    PWM_FREQUENCY = 16000,
    PWM_PRESCALER = 1,
    PWM_PERIOD = 999UL, //CLOCK_FREQUENCY/( PWM_FREQUENCY * PWM_PRESCALER) - 1,  // max 32767
    P1DC_MAX = PWM_PERIOD,
    P1DC_MIN = 0,        
};

static uint8_t numberOfOverflows = 0;// Кол-во переполнений таймера23

void GPIO_init();                   // Инициализация и настройка портов ввода/вывода
void PWM_init();                    // Инициализация PWM
void PWM_set(uint8_t, uint8_t);     // PWM установка скважности для указанной ножки
void interrupt_INT0_init();         // Инициализация и разрешение прерывания от INT0
void TIM23_init();                  // Инициализация таймера 23
uint32_t return_time_of_TIM23();    // Вернуть время таймера 23

#endif	/* HARD_H */


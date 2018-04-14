/* 
 * File:   hard.h
 */

#ifndef HARD_H
#define	HARD_H

#include <xc.h>

enum
{
    MOTOR_LEFT, 
    MOTOR_LEFT_1,
    MOTOR_LEFT_2,
    MOTOR_RIGHT,
    MOTOR_RIGHT_1,
    MOTOR_RIGHT_2,
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

#define ENCODER_PIN1 PORTFbits.RF6
#define ENCODER_PIN2 PORTFbits.RF7
#define ENCODER_TYPE_OF_INTERRUPT INTCON2 & ~(1 << 0)
enum
{
    ENCODER_POSITIVE_EDGE = ~1,
    ENCODER_NEGATIVE_EDGE = 1
};

void GPIO_init();                   // Инициализация и настройка портов ввода/вывода

/**************************** MOTOR_CONTROL (PWM) ****************************/
void PWM_init();                    // Инициализация PWM
void PWM_set(uint8_t, uint8_t);     // PWM установка скважности для указанной ножки
/**************************** MOTOR_CONTROL (PWM) ****************************/

/******************************* ENCODER (INT0) *******************************/
void interrupt_INT0_init();             // Инициализация и разрешение прерывания
inline void change_type_of_interrupt(); // Изменить тип прерывания
inline void reset_interrupt_flag();     // Обнулить флаг прерывания
/******************************* ENCODER (INT0) *******************************/

/************************** SOFTWARE TIMER (TIM23)  **************************/
static uint8_t numberOfOverflows = 0;   // Кол-во переполнений таймера23
void TIM23_init();                      // Инициализация таймера 23
uint32_t return_time_of_TIM23();        // Вернуть время таймера 23
/************************** SOFTWARE TIMER (TIM23)  **************************/

#endif	/* HARD_H */


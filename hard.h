/* 
 * File:   hard.h
 */

#ifndef HARD_H
#define	HARD_H

#include <xc.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>

typedef enum
{
    false = 0,
    true = 1,
} bool_t;

enum
{
    MOTOR_LEFT = 0, 
    MOTOR_LEFT_1 = MOTOR_LEFT + 1,
    MOTOR_LEFT_2 = MOTOR_LEFT + 2,
    MOTOR_RIGHT = 3,
    MOTOR_RIGHT_1 = MOTOR_RIGHT + 1,
    MOTOR_RIGHT_2 = MOTOR_RIGHT + 2,
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

#define ENCODER_LEFT_PIN1 PORTFbits.RF6
#define ENCODER_LEFT_PIN2 PORTFbits.RF7
#define ENCODER_RIGHT_PIN1 PORTFbits.RF6
#define ENCODER_RIGHT_PIN2 PORTFbits.RF7

#define ENCODER_LEFT_TYPE_OF_INTERRUPT INTCON2bits.INT0EP
#define ENCODER_RIGHT_TYPE_OF_INTERRUPT INTCON2bits.INT1EP
enum
{
    ENCODER_POSITIVE_EDGE = 0,
    ENCODER_NEGATIVE_EDGE = 1
};

enum
{
    US_IN_COUNT = 2,          // кол-во тактов счетчика в 1 мкс 
    US_TO_COUNT_LSHIFT = 1,   // кол-во тактов счетчика в 1 мкс (для сдвига влево <<)
};

void GPIO_init();                   // Инициализация и настройка портов ввода/вывода

/**************************** MOTOR_CONTROL (PWM) ****************************/
void PWM_init();
void PWM_set(uint8_t, uint8_t);
/**************************** MOTOR_CONTROL (PWM) ****************************/

/*************************** ENCODER (INT0 и INT1) ***************************/
void encoders_interrupt_init();
inline void encoder_left_change_type_of_interrupt();
inline void encoder_right_change_type_of_interrupt();
inline void encoder_left_reset_interrupt_flag();
inline void encoder_right_reset_interrupt_flag();

/*************************** ENCODER (INT0 и INT1) ***************************/

/************************** SOFTWARE TIMER (TIM23)  **************************/
void hard_timer_init();
uint32_t hard_timer_return_time();
uint8_t  hard_timer_return_overflows();
/************************** SOFTWARE TIMER (TIM23)  **************************/

/**************************** RANGE FINDER (INT4)  ****************************/
#define RANGEFINDER_INPUT PORTAbits.RA15
#define RANGEFINDER_OUTPUT PORTDbits.RD8
#define RANGEFINDER_TYPE_OF_INTERRUPT INTCON2bits.INT4EP
void rangefinder_init_interrupt();
inline void rangefinder_change_type_of_interrupt();
/**************************** RANGE FINDER (INT4)  ****************************/

#endif	/* HARD_H */


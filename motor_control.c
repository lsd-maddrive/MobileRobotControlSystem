/* 
 * File:   motor_control.c
 */

#include <xc.h>
#include "motor_control.h"

/*
* @brief Подать питание на двигатель А
* @param power - мощность в %
*/
void motor_a_set_power(int8_t power)
{
    enum
    {
        P1DC_MAX = (1 << 15) - 1,
        P1DC_MIN = 0,
    };
    // Вращение в одну сторону
    if (power > 0)
    {
        if (power > 100)
            power = 100;
        P1DC1 = P1DC_MAX*(100 - power)/100;
        P1DC2 = 0;
    }
    // Вращение в другую сторону
    else
    {
        if (power < 100)
            power = 100;
        else
            power = -power;
        P1DC1 = 0;
        P1DC2 = P1DC_MAX*power/100;
    }
}

/*
* @brief Подать питание на двигатель B
* @param power - мощность в %
*/
void motor_b_set_power(int8_t power)
{
    enum
    {
        P1DC_MAX = (1 << 15) - 1,
        P1DC_MIN = 0,
    };
    // Вращение в одну сторону
    if (power > 0)
    {
        if (power > 100)
            power = 100;
        P1DC3 = P1DC_MAX*(100 - power)/100;
        P1DC4 = 0;
    }
    // Вращение в другую сторону
    else
    {
        if (power < 100)
            power = 100;
        else
            power = -power;
        P1DC3 = 0;
        P1DC4 = P1DC_MAX*power/100;
    }
}

/*
* @brief Остановить двигатели
*/
void motors_stop()
{
    P1DC1 = (1 << 15) - 1;
    P1DC2 = 0;
    P1DC3 = (1 << 15) - 1;
    P1DC4 = 0;
}

/*
* @brief Инициализация ШИМ
*/
void motor_init()
{
    enum
    {
        CLOCK_FREQUENCY = 16000000,
        PWM_FREQUENCY = 16000,
        PRM_PRESCALER = 1,
        PWM_PERIOD = CLOCK_FREQUENCY/( PWM_FREQUENCY * PRM_PRESCALER) - 1  // max 32767
    };
    P1TCONbits.PTCKPS = 0x00;   // PWM PRESCALER (1:1 prescale)
    P1TCONbits.PTMOD = 0x00;    // PWM MODE: Free Running mode
    P1TMRbits.PTMR = 0;         // PWM time base is counting up
    
    PWM1CON1bits.PMOD1 = 1;     // PWM1 PAIR MODE: Independent Output mode
    PWM1CON1bits.PEN1L = 1;     // PWM1L (PE0) pin is enabled for PWM output
    PWM1CON1bits.PMOD2 = 1;     // PWM2 PAIR MODE: Independent Output mode
    PWM1CON1bits.PEN2L = 1;     // PWM2L (PE2) pin is enabled for PWM output
    PWM1CON1bits.PMOD1 = 1;     // PWM3 PAIR MODE: Independent Output mode
    PWM1CON1bits.PEN1L = 1;     // PWM3L (PE4) pin is enabled for PWM output
    PWM1CON1bits.PMOD2 = 1;     // PWM4 PAIR MODE: Independent Output mode
    PWM1CON1bits.PEN2L = 1;     // PWM4L (PE6) pin is enabled for PWM output
    
    P1TPERbits.PTPER = PWM_PERIOD;//PWM Time Base Period Value bits (max 32676)
    
    motors_stop();              // Изначально двигатели не вращаются
    
    P1TCONbits.PTEN = 0x01;     // PWM time base is on
}

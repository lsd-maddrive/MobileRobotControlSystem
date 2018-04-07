/* 
 * File:   motor_control.c
 */

#include <xc.h>
#include "motor_control.h"

/*
* @brief Остановить двигатели
*/
void motors_stop()
{
    
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
    
    PWM1CON1bits.PMOD1 = 1;     // PWM1 I/O pin pair is in Independent Output mode
    PWM1CON1bits.PEN1L = 1;     // PWM1 pin is enabled for PWM output
    PWM1CON1bits.PMOD2 = 1;     // PWM2 I/O pin pair is in Independent Output mode
    PWM1CON1bits.PEN2L = 1;     // PWM2 pin is enabled for PWM output
    PWM1CON1bits.PMOD1 = 1;     // PWM3 I/O pin pair is in Independent Output mode
    PWM1CON1bits.PEN1L = 1;     // PWM3 pin is enabled for PWM output
    PWM1CON1bits.PMOD2 = 1;     // PWM4 I/O pin pair is in Independent Output mode
    PWM1CON1bits.PEN2L = 1;     // PWM4 pin is enabled for PWM output
    
    P1TPERbits.PTPER = PWM_PERIOD;//PWM Time Base Period Value bits (max 32676)
    
    motors_stop();              // Изначально двигатели не вращаются
    
    P1TCONbits.PTEN = 0x01;     // PWM time base is on
}

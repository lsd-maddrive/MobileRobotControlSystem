/*
 * File:   main.c
 */
#include "hard.h"


/*
* @brief Инициализация портов ввода/вывода
* @note TRISx - регистр направления порта
* @note PORTx - регистр чтения/записи 
*/
void GPIO_init()
{
    // A0
    TRISA &= ~(1 << 0);         // PORTA0 - output
    PORTA |= (1 << 0);          // PORTA0 - Высокий уровень

    
    // INT0 - RF6
    TRISA |= (1 << 6) |         // PORTF6 - input (encoder - INT0)
             (1 << 7);          // PORTF7 - input (encoder)
    
}


/*
* @brief Инициализация ШИМ
*/
void PWM_init()
{
    
}


/*
* @brief Инициализация внешнего прерывание от источника INT0
*/
void interrupt_INT0_init()
{
    INTCON2 &= ~0x0001;     // INT0 setup to interupt on rising edge
    IFS0bits.INT0IF = 0;    // INT0 reset interrupt flag 
    IEC0bits.INT0IE = 1;    // INT0 interupt enable
}
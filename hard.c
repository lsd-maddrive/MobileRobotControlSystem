/*
 * File:   main.c
 */
#include "hard.h"


/*
* @brief Инициализация портов ввода/вывода
*/
void GPIO_init()
{
    // Регистр направления порта:
    TRISA &= ~0x01;         // PORTA1 - output
    // Регистр чтения/записи
    PORTA |= 0x01;          // PORTA1 - Высокий уровень
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
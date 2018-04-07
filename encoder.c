/*
 * File:   main.c
 */
#include "encoder.h"

/*
* @brief Инициализация энкодера
*/
void encoder_init()
{
    angle_of_encoder = 0;
    interrupt_INT0_init();
}


/*
* @brief Прерывание от энкодера (INT0)
*/
void __attribute__((__interrupt__)) encoder_interrupt(void)
{
    // ПОП обработки прерывания
    angle_of_encoder++;
    
    IFS0bits.INT0IF = 0;    // INT0 reset interrupt flag 
}

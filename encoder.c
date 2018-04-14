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
    if (ENCODER_TYPE_OF_INTERRUPT == ENCODER_POSITIVE_EDGE)
    {
        change_type_of_interrupt();
        if (ENCODER_PIN2 == 1)
            angle_of_encoder++;
        else
            angle_of_encoder--;
    }
    else
    {
        change_type_of_interrupt();
        if (ENCODER_PIN2 == 1)
            angle_of_encoder--;
        else
            angle_of_encoder++;
    }
    
    reset_interrupt_flag();
}

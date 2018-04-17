/*
 * File:   main.c
 */
#include "encoder.h"

volatile int16_t angle_of_encoder;

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
void __attribute__((interrupt, no_auto_psv)) _INT0Interrupt(void)
{
    if ( (ENCODER_TYPE_OF_INTERRUPT) == ENCODER_POSITIVE_EDGE)
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

/*
* @brief Получить значение угла поворота энкодера
* @return значение угла поворота энкодера
*/
int16_t get_angle()
{
    return angle_of_encoder;
}
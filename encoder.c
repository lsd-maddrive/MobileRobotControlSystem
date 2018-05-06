/*
 * File:   main.c
 */
#include "encoder.h"
#include "uart.h"

/* Принцип работы энкодера:
 * У энкодеров по 4 ножки:
 * - PIN_x_1 (оранжевый)
 * - PIN_x_2 (зеленый)
 * - GND (желтый)
 * - VCC (белый)
 * Left encoder:
 * PIN_1 = RF6 = INT0
 * PIN_1 = RF7
 * Right encoder:
 * PIN_1 = RF8 = AN20 = INT1
 * PIN_1 = RF9 = AN21 
 * Если энкодер поворачивается, на ножках генерируются импульсы, 
 * Нужно внешнее прерывание на изменение уровня одной из ножек.
 * Во время прерывания проверяем состояние второй ноги.
 */

volatile int16_t pulsesLeft;
volatile int16_t pulsesRight;
extern UART_module* debug;


/*
* @brief Инициализация энкодера
*/
void encoders_init()
{
    pulsesLeft = 0;
    pulsesRight = 0;
    encoders_interrupt_init();
}

/*
* @brief Прерывание от левого энкодера (INT0)
*/
void __attribute__((interrupt, no_auto_psv)) _INT0Interrupt(void)
{
    UART_transmit(debug, "kek1", 4);
    if ( (ENCODER_LEFT_TYPE_OF_INTERRUPT) == ENCODER_POSITIVE_EDGE)
    {
        if (ENCODER_LEFT_PIN2 == 1)
            pulsesLeft++;
        else
            pulsesLeft--;
    }
    else
    {
        if (ENCODER_LEFT_PIN2 == 1)
            pulsesLeft--;
        else
            pulsesLeft++;
    }
    
    encoder_left_change_type_of_interrupt();
    encoder_left_reset_interrupt_flag();
}

/*
* @brief Прерывание от правого энкодера (INT1)
*/
void __attribute__((interrupt, no_auto_psv)) _INT1Interrupt(void)
{
    UART_transmit(debug, "kek2", 4);
    if ( (ENCODER_RIGHT_TYPE_OF_INTERRUPT) == ENCODER_POSITIVE_EDGE)
    {
        if (ENCODER_RIGHT_PIN2 == 1)
            pulsesRight++;
        else
            pulsesRight--;
    }
    else
    {
        if (ENCODER_RIGHT_PIN2 == 1)
            pulsesRight--;
        else
            pulsesRight++;
    }
    
    encoder_right_change_type_of_interrupt();
    encoder_right_reset_interrupt_flag();
}

/*
* @brief Получить значение угла поворота левого энкодера
* @return значение угла поворота левого энкодера
*/
int16_t encoder_left_get_pulses()
{
    return pulsesLeft;
}

/*
* @brief Получить значение угла поворота правого энкодера
* @return значение угла поворота правого энкодера
*/
int16_t encoder_right_get_pulses()
{
    return pulsesRight;
}

/*
* @brief Обнулить кол-во импульсов энкодеров
*/
void encoders_reset_angle()
{
    pulsesLeft = 0;
    pulsesRight = 0;
}
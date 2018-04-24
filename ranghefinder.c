/* 
 * File:   rangefinder.c
 */

/*Ультразвуковой дальномер SRF-05
 * - вход датчика - подаем импульс длительностью 10 мкс
 * - выход датчика - расстояние до препятствия (прерывание)
 */

#include "rangefinder.h"

static uint16_t range;
static Timer timer; 

/*
* @brief Инициализация ултьтразвукового дальномера
*/
void rangefinder_init()
{
    rangefinder_init_interrupt();
    soft_timer_init(&timer);
    range = 0;
}

/*
* @brief Подать импульс на датчик
*/
void rangefinder_give_impulse()
{
    RANGEFINDER_OUTPUT = 1;
    timer_start_ms(&timer, 10);
    while(timer_report(&timer) != FINISHED);
    RANGEFINDER_OUTPUT = 0;
}

/*
* @brief Получить измеренное расстояние
*/
uint16_t rangefinder_get_range()
{
    return range;
}      

/*
* @brief Прерывание от дальномера (INT4)
*/
void __attribute__((interrupt, no_auto_psv)) _INT4Interrupt(void)
{
    enum
    {
        SOUND_SPEED = 340   // м/сек
    };
    if ( (RANGEFINDER_TYPE_OF_INTERRUPT) == ENCODER_POSITIVE_EDGE)
    {
        timer_start_ms(&timer, 25);
    }
    else
    {
        if( timer_report(&timer) != WORKING)
            range = 0;
        else
            range = (uint32_t)SOUND_SPEED*timer_get_elapsed_time(&timer)/1000 >> 1;
    }
    rangefinder_change_type_of_interrupt();
}
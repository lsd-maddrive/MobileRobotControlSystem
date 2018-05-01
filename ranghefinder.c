/* 
 * File:   rangefinder.c
 */

/*Ультразвуковой дальномер SRF-05
 * - вход датчика - подаем импульс длительностью 10 мкс
 * - выход датчика - расстояние до препятствия
 */

#include "rangefinder.h"

static uint32_t range;
static Timer timerForPulse; 

/*
* @brief Инициализация ултьтразвукового дальномера
*/
void rangefinder_init()
{
    rangefinder_init_interrupt();
    soft_timer_init(&timerForPulse);
    range = 0;
}

/*
* @brief Подать импульс на датчик
*/
void rangefinder_give_impulse()
{
    // Заблаговременно устанавливаем ножку в 0
    RANGEFINDER_OUTPUT = 0;
    timer_start_us(&timerForPulse, 5);
    while(timer_report(&timerForPulse) == WORKING);
    
    // Подаем импульс 10 мкс
    RANGEFINDER_OUTPUT = 1;
    timer_start_us(&timerForPulse, 10);
    while(timer_report(&timerForPulse) == WORKING);
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
    static uint32_t lastTime;
    uint32_t nowTime = hard_timer_return_time();
    
    if ( (RANGEFINDER_TYPE_OF_INTERRUPT) == ENCODER_POSITIVE_EDGE)
    {
        lastTime = nowTime;
        RANGEFINDER_TYPE_OF_INTERRUPT = ENCODER_NEGATIVE_EDGE;
    }
    else
    {
        range = nowTime - lastTime;
        RANGEFINDER_TYPE_OF_INTERRUPT = ENCODER_POSITIVE_EDGE;
    }
    IFS3bits.INT4IF = 0;      // Очистка флага прерывания
}
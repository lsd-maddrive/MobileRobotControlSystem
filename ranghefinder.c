/* 
 * File:   rangefinder.c
 */

/*Ультразвуковой дальномер SRF-05
 * - вход датчика - подаем импульс длительностью 10 мкс
 * - выход датчика - расстояние до препятствия (прерывание)
 */

#include "rangefinder.h"

static uint32_t range;
static Timer timerForPulse; 
static Timer timerForEcho; 

/*
* @brief Инициализация ултьтразвукового дальномера
*/
void rangefinder_init()
{
    rangefinder_init_interrupt();
    soft_timer_init(&timerForPulse);
    soft_timer_init(&timerForEcho);
    range = 0;
}

/*
* @brief Подать импульс на датчик
*/
void rangefinder_give_impulse()
{
    // Заблаговременно устанавливаем ножку в 0
    //RANGEFINDER_OUTPUT = 0;
    //timer_start_us(&timerForPulse, 50);
    //while(timer_report(&timerForPulse) == WORKING);
    
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
    static uint32_t time;
    
    if ( (RANGEFINDER_TYPE_OF_INTERRUPT) == ENCODER_POSITIVE_EDGE)
    {
        RANGEFINDER_TYPE_OF_INTERRUPT = ENCODER_NEGATIVE_EDGE;
        
        //timer_start_ms(&timerForEcho, 100);
        time = hard_timer_return_time();
        //range += 10;
    }
    else
    {
        RANGEFINDER_TYPE_OF_INTERRUPT = ENCODER_POSITIVE_EDGE;
        //IFS3bits.INT4IF = 0;      // Очистка флага прерывания
        range = hard_timer_return_time() - time;
        
        //range = (uint32_t)timer_get_elapsed_time(&timerForEcho);
        //range++;
    }
    IFS3bits.INT4IF = 0;      // Очистка флага прерывания
}
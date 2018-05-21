/* 
 * File:   rangefinder.c
 */

/* Подключение ультразвукового дальномера SRF-05:
 * Ножка датчика                Подключение к dsPIC  
 * ECHO - output                RA15 (INT4) - input
 * TRIG - input                 RD8 (IC1) - output
 * GND                          GND
 * VCC                          5 V (but dsPIC only 3.3V)
 *
 * Принцип работы ультразвукового дальномера SRF-05
 * - на ножку TRIG датчика подаем импульс длительностью 10 мкс;
 * - с ножки ECHO датчика считываем импульс, длительность которого 
 * пропорциональна расстоянию до препятствия.
 */

#include "rangefinder.h"
#include "timer.h"

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
* @brief Подать импульс на датчик длительностью минимум 10 мс
*/
void rangefinder_give_impulse()
{
    // Заблаговременно устанавливаем ножку в 0
    RANGEFINDER_OUTPUT = 0;
    timer_start_us(&timerForPulse, 5);
    while(timer_report(&timerForPulse) == TIMER_WORKING);
    
    // Подаем импульс 10 мкс
    RANGEFINDER_OUTPUT = 1;
    timer_start_us(&timerForPulse, 10);
    while(timer_report(&timerForPulse) == TIMER_WORKING);
    RANGEFINDER_OUTPUT = 0;
}


/*
* @brief Получить измеренное расстояние (в мм)
*/
uint16_t rangefinder_get_range()
{
    return (float)range*0.085 + 14.3;
}      


/*
* @brief Прерывание от дальномера (INT4)
*/
void __attribute__((interrupt, no_auto_psv)) _INT4Interrupt(void)
{
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
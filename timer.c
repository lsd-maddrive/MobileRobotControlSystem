/* 
 * File:   timer.c
 */

#include <xc.h>
#include "timer.h"

/* 
 * @brief Создание таймера (конструктор)
 * @return возвращает указатель на объейкт структуры Timer
*/
Timer* create_timer()
{
    Timer* ptrTimer = malloc( sizeof(*ptrTimer) );
    ptrTimer->status = CREATED;
    return ptrTimer;
}

/*
 * @brief Удаление таймера (деструктор)
 * param ptrTimer - указатель на объект структуры Timer
*/
void delete_timer(Timer* ptrTimer)
{
    free(ptrTimer);
}

/*
 * @brief Запуск отчета таймера в мкс
 * param timer - указатель на объект структуры Timer
 * param time_us - время, которое будет отсчитывать таймер, в мкс
*/
void start_timer_us(Timer* ptrTimer, uint32_t time_us)
{
    if (time_us != 0)
    {
        // Инициализация объекта текущим временем и задержкой
        ptrTimer->fixedTime = return_time_of_TIM23();
        ptrTimer->restTime = time_us;
        ptrTimer->status = WORKING;
        ptrTimer->fixedNumberOfOverflowsTM23 = numberOfOverflows;
        // Проверка на переполнение конечного времени
        if ( (ptrTimer->fixedTime + ptrTimer->restTime) < ptrTimer->fixedTime)
            ptrTimer->isEndTimeFull = 1;
        else
            ptrTimer->isEndTimeFull = 0;
    }
    else
    {
        ptrTimer->status = FINISHED;
    }
}

/*
 * @brief Запуск отчета таймера в мс
 * param timer - указатель на объект структуры Timer
 * param time_us - время, которое будет отсчитывать таймер, в мкс
*/
void start_timer_ms(Timer* ptrTimer, uint32_t time_ms)
{
    if (time_ms != 0)
    {
        // Инициализация объекта текущим временем и задержкой
        ptrTimer->fixedTime = return_time_of_TIM23();
        ptrTimer->restTime = 1000*time_ms; // переполнение нестрашно
        ptrTimer->status = WORKING;
        // Проверка на переполнение конечного времени
        if ( (ptrTimer->fixedTime + ptrTimer->restTime) < ptrTimer->fixedTime)
            ptrTimer->isEndTimeFull = 1;
        else
            ptrTimer->isEndTimeFull = 0;
    }
    else
    {
        ptrTimer->status = FINISHED;
    }
}

/*
 * @brief Возвращает статус таймера
 * param timer - указатель на объект структуры Timer
*/
uint8_t report_timer(Timer* ptrTimer)
{
    uint32_t timeNow = return_time_of_TIM23();
    // Если таймер мк не переполнился
    if (ptrTimer->fixedNumberOfOverflowsTM23 == numberOfOverflows)
    {
        // Если конечное время не переполнено
        if (!ptrTimer->isEndTimeFull)
        {
            // Если время еще не наступило
            if ( (ptrTimer->fixedTime + ptrTimer->restTime) < timeNow )
            {
                ptrTimer->restTime = timeNow - ptrTimer->fixedTime;
                ptrTimer->fixedTime = timeNow;
            }
            // Если время пришло
            else
            {
                ptrTimer->restTime = 0;
                ptrTimer->status = FINISHED;
            }
        }
        // Если конечное время переполнено
        else
        {
            ptrTimer->restTime = timeNow - ptrTimer->fixedTime;
            ptrTimer->fixedTime = timeNow;
        }
    }
    // Если таймер мк переполнился
    else
    {
        // Если конечное время не переполнено
        if (!ptrTimer->isEndTimeFull)
        {
            ptrTimer->restTime = 0;
            ptrTimer->status = FINISHED;
        }
        // Если конечное время переполнено
        else
        {
            ptrTimer->isEndTimeFull = 0;
            // Если время еще не наступило
            if ( (ptrTimer->fixedTime + ptrTimer->restTime) < timeNow )
            {
                ptrTimer->restTime = timeNow - ptrTimer->fixedTime;
                ptrTimer->fixedTime = timeNow;
            }
            // Если время пришло
            else
            {
                ptrTimer->restTime = 0;
                ptrTimer->status = FINISHED;
            }
        }
    }
    return ptrTimer->status;
}
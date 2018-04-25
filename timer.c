/* 
 * File:   timer.c
 */

#include "timer.h"

/****************************** PRIVATE FUNCTION ******************************/
/* 
 * @brief Возвращает true, если время пришло, в противном случае false
 * @return возвращает указатель на объект структуры Timer
*/
inline uint8_t is_timer_end(Timer* ptrTimer, uint32_t nowCount)
{
    uint8_t hardTimerOverflows = hard_timer_return_overflows();
    
    if ( (ptrTimer->endOverflows < hardTimerOverflows) || 
         ( (ptrTimer->endOverflows == hardTimerOverflows) && (ptrTimer->endCount <= nowCount) ) )
        return 1;
    return 0;
}
/****************************** PRIVATE FUNCTION ******************************/



/****************************** PUBLIC FUNCTION *******************************/
/* 
 * @brief Создание таймера (конструктор)
 * @return возвращает указатель на объейкт структуры Timer
*/
void soft_timer_init(Timer* ptrTimer)
{
    ptrTimer->status = CREATED;
    
    ptrTimer->startCount = 0;
    ptrTimer->startOverflows = 0;
    
    ptrTimer->restCount = 0;
    ptrTimer->restOverflows = 0;
    
    ptrTimer->endCount = 0;
    ptrTimer->endOverflows = 0;
}

/*
 * @brief Удаление таймера (деструктор)
 * @param ptrTimer - указатель на объект структуры Timer
*/
void timer_delete(Timer* ptrTimer)
{
    free(ptrTimer);
}

/*
 * @brief Запуск отчета таймера в мкс
 * @param ptrTimer - указатель на объект структуры Timer
 * @param time_us - время, которое будет отсчитывать таймер, в мкс
*/
void timer_start_us(Timer* ptrTimer, uint16_t time_us)
{
    if (time_us != 0)
    {
        // Инициализация объекта текущим временем
        ptrTimer->startCount = hard_timer_return_time();
        ptrTimer->startOverflows = hard_timer_return_overflows();
        // Инициализация объекта оставшимся временем до срабатывания таймера
        ptrTimer->restCount = ((uint32_t)time_us ) << US_TO_COUNT_LSHIFT;
        ptrTimer->restOverflows = 0;
        // Инициализация объекта временем срабатывания таймера
        ptrTimer->endCount = ptrTimer->startCount + ptrTimer->restCount;
        ptrTimer->endOverflows = ( (ptrTimer->startCount + ptrTimer->restCount) < ptrTimer->startCount ) ? 1:0;
        
        ptrTimer->status = WORKING;
    }
    else
    {
        ptrTimer->status = FINISHED;
    }
}

/*
 * @brief Запуск отчета таймера в мс
 * param ptrTimer - указатель на объект структуры Timer
 * param time_us - время, которое будет отсчитывать таймер, в мкс
*/
void timer_start_ms(Timer* ptrTimer, uint16_t time_ms)
{
    enum 
    {
        US_IN_MS = 1000,
        COUNT_IN_MS = US_IN_MS*US_IN_COUNT,
    };
    if (time_ms != 0)
    {
        // Инициализация объекта текущим временем
        ptrTimer->startCount = hard_timer_return_time();
        ptrTimer->startOverflows = hard_timer_return_overflows();
        // Инициализация объекта оставшимся временем до срабатывания таймера
        ptrTimer->restCount = (uint32_t)time_ms * COUNT_IN_MS;
        ptrTimer->restOverflows = 0; //( (ptrTimer->startCount + ptrTimer->restCount) < ptrTimer->startCount) ? 0 : 0
        // Инициализация объекта временем срабатывания таймера
        ptrTimer->endCount = ptrTimer->startCount + ptrTimer->restCount;
        ptrTimer->endOverflows = ptrTimer->startOverflows + ptrTimer->restOverflows;
        
        ptrTimer->status = WORKING;
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
uint8_t timer_report(Timer* ptrTimer)
{
    uint32_t nowCount = hard_timer_return_time();
    if ( ptrTimer->status == WORKING && is_timer_end(ptrTimer, nowCount) )
    {
        ptrTimer->status = FINISHED;
    }
    return ptrTimer->status;
}

/*
 * @brief Возвращает оставшееся время работы таймера
 * @param timer - указатель на объект структуры Timer
 * @return оставшееся время работы таймера в мкс
*/
uint32_t timer_get_rest_time(Timer* ptrTimer)
{
    uint32_t nowCount = hard_timer_return_time();
    // Если время вышло
    if ( ptrTimer->status == WORKING && is_timer_end(ptrTimer, nowCount) )
    {
        ptrTimer->restCount = 0;
        ptrTimer->restOverflows = 0;
        ptrTimer->status = FINISHED;
    }
    ptrTimer->restCount = ptrTimer->endCount - nowCount;
    return ( ptrTimer->restCount >> US_TO_COUNT_LSHIFT );
}

/*
 * @brief Возвращает истекшее время работы таймера
 * @param timer - указатель на объект структуры Timer
 * @return время работы таймера в мкс
*/
uint32_t timer_get_elapsed_time(Timer* ptrTimer)
{
    const uint32_t MAX_COUNT = 4294967295UL;
    uint32_t nowCount = hard_timer_return_time();
    
    if( ptrTimer->startOverflows == hard_timer_return_overflows() )
        return ( (ptrTimer->startCount - nowCount) << 1 );
    return ( (MAX_COUNT - ptrTimer->startCount + nowCount) << 1 );
}
/****************************** PUBLIC FUNCTION *******************************/

/* 
 * File:   soft_timer.h
 */

#ifndef SOFT_TIMER_H
#define	SOFT_TIMER_H

#include "hard.h"

enum TimerStatus
{
    CREATED,    // исходное состояние после конструктора
    WORKING,    // таймер в процессе работы
    WAITING,    // пауза
    STOPPED,    // остановлен
    FINISHED    // время истекло
};

typedef struct 
{
    uint32_t startCount;        // начальное состояние счетчика
    uint8_t  startOverflows;    // начальное кол-во переполнений счетчика
    
    uint32_t restCount;         // осталось тиков счетчика
    uint8_t  restOverflows;     // осталось переполнений счетчика
    
    uint32_t endCount;          // конечное состояние счетчика
    uint8_t  endOverflows;      // конечное кол-во переполнений таймера
    
    uint8_t  status;            // статус таймера
} Timer;

Timer* timer_create();                  // конструктор
void timer_delete(Timer*);              // деструктор

void timer_start_us(Timer*, uint16_t);  // начать отчет времени в мкс
void timer_start_ms(Timer*, uint16_t);  // начать отчет времени в мс
void timer_continue(Timer*);            // продолжить отчет времени
void timer_wait(Timer*);                // сделать паузу
void timer_stop(Timer*);                // полностью остановить таймер

uint8_t timer_report(Timer*);           // вернуть статус таймера
uint16_t timer_get_rest_time(Timer*);   // вернуть оставшееся время таймера

#endif	/* SOFT_TIMER_H */


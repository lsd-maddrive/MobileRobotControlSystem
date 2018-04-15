/* 
 * File:   software_timer.h
 */

#ifndef SOFTWARE_TIMER_H
#define	SOFTWARE_TIMER_H

/* Некоторые нюансы работы таймера из железа:
 * - Частота мк - 16 МГц
 * - Регистр 32 битный
 * - Предделитель 8
 * - 1 тик таймер23 = 0.5 мкс
 * - 1 период таймера23 = 2147.48365 сек = 36 минут
 */

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

Timer* create_timer();                  // конструктор
void delete_timer(Timer*);              // деструктор

void start_timer_us(Timer*, uint16_t);  // начать отчет времени в мкс
void start_timer_ms(Timer*, uint16_t);  // начать отчет времени в мс
void continue_timer(Timer*);            // продолжить отчет времени
void wait_timer(Timer*);                // сделать паузу
void stop_timer(Timer*);                // полностью остановить таймер

uint8_t report_timer(Timer*);           // вернуть статус таймера

#endif	/* SOFTWARE_TIMER_H */


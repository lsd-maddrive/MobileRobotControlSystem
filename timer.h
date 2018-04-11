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
 * - макс. время до "сбоя" таймера23 = 8 * 2^32 / 16МГц = 2147 сек или 35 мин
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
    uint32_t fixedTime;     // время старта таймера
    uint32_t restTime;      // остаток времени
    uint8_t status;         // статус таймера
    uint8_t isEndTimeFull;  // факт переполнения конечного времени таймера (fixedTime + restTime >= 2^32)
    uint8_t fixedNumberOfOverflowsTM23;//завиксированное кол-во переполнений TM23
} Timer;
Timer* create_timer();                  // конструктор
void delete_timer(Timer*);               // деструктор

void start_timer_us(Timer*, uint32_t);   // начать отчет времени в мкс
void start_timer_ms(Timer*, uint32_t);   // начать отчет времени в мс
void continue_timer(Timer*);             // продолжить отчет времени
void wait_timer(Timer*);                 // сделать паузу
void stop_timer(Timer*);                 // полностью остановить таймер

uint8_t report_timer(Timer*);            // вернуть статус таймера

#endif	/* SOFTWARE_TIMER_H */


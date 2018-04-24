/* 
 * File:   soft_timer.h
 */

#ifndef SOFT_TIMER_H
#define	SOFT_TIMER_H

#include "hard.h"

enum TimerStatus
{
    CREATED,
    WORKING,
    WAITING,
    STOPPED,
    FINISHED
};

typedef struct 
{
    uint32_t startCount;
    uint8_t  startOverflows;
    
    uint32_t restCount;
    uint8_t  restOverflows;
    
    uint32_t endCount;
    uint8_t  endOverflows;
    
    uint8_t  status;
} Timer;

void soft_timer_init();                  // конструктор
void timer_delete(Timer*);              // деструктор

void timer_start_us(Timer*, uint16_t);
void timer_start_ms(Timer*, uint16_t);
void timer_continue(Timer*);
void timer_wait(Timer*);
void timer_stop(Timer*);

uint8_t timer_report(Timer*);
uint32_t timer_get_rest_time(Timer*);
uint32_t timer_get_elapsed_time(Timer*);

#endif	/* SOFT_TIMER_H */


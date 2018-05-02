/* 
 * File:   rangefinder.h
 */

#ifndef RANGEFINDER_H
#define	RANGEFINDER_H

#include "hard.h"
#include "timer.h"

void rangefinder_init();            // Инициализация датчика
void rangefinder_give_impulse();    // Подать импульс на датчик
uint16_t rangefinder_get_range();   // Получить измеренное расстояние (в мм)

#endif	/* RANGEFINDER_H */


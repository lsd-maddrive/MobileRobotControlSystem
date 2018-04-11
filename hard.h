/* 
 * File:   hard.h
 */

#ifndef HARD_H
#define	HARD_H

#include <xc.h>


void GPIO_init();                   // Инициализация и настройка портов ввода/вывода
void PWM_init();                    // Инициализация PWM
void interrupt_INT0_init();         // Инициализация и разрешение прерывания от INT0
void TIM23_init();                  // Инициализация таймера 23
uint32_t return_time_of_TIM23();    // Вернуть время таймера 23

#endif	/* HARD_H */


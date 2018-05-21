 /* 
 * File:   encoder.h
 */

#ifndef ENCODER_H
#define	ENCODER_H

#include "hard.h"

void encoders_init();               // Инициализация энкодеров
int16_t encoder_left_get_pulses();  // Получить кол-во импульсов левого энкодера
int16_t encoder_right_get_pulses(); // Получить кол-во импульсов правого энкодера
void encoders_reset_pulses();       // Обнуление кол-ва импульсов энкодеров

#endif	/* ENCODER_H */


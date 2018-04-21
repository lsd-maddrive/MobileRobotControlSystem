 /* 
 * File:   encoder.h
 */

#ifndef ENCODER_H
#define	ENCODER_H

#include "hard.h"

void encoders_init();               // Инициализация энкодеров
int16_t encoder_left_get_angle();   // Получить значение угла поворота левого энкодера
int16_t encoder_right_get_angle();  // Получить значение угла поворота правого энкодера
//void encoders_reset_angle();      // Обнуление значения угла поворота

#endif	/* ENCODER_H */


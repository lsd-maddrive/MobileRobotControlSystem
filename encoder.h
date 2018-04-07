/* 
 * File:   encoder.h
 */

#ifndef ENCODER_H
#define	ENCODER_H

#include <xc.h>
#include "hard.h"

// Описание:
// У энкодера 2 ножки
// На ножках генерируются импульс, если энкодер поворачивается
// Нужно внешнее прерывание на изменение уровня одной из ножек
// Во время прерывания проверяем вторую ногу

/************************** PUBLIC FUNCTION **************************/
static int16_t angle_of_encoder;

void encoder_init(void);
void __attribute__((__interrupt__)) encoder_interrupt(void);

#endif	/* ENCODER_H */


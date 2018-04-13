 /* 
 * File:   encoder.h
 */

#ifndef ENCODER_H
#define	ENCODER_H

#include <xc.h>
#include "hard.h"

// Принцип работы энкодера:
// У энкодера 2 ножки
// На ножках генерируются импульс, если энкодер поворачивается
// Нужно внешнее прерывание на изменение уровня одной из ножек
// Во время прерывания проверяем вторую ногу

#define PIN1 PORTFbits.RF6
#define PIN2 PORTFbits.RF7
#define TYPE_OF_INTERRUPT INTCON2 & ~(1 << 0)
enum
{
    POSITIVE_EDGE = ~1,
    NEGATIVE_EDGE = 1
};
/************************** PUBLIC FUNCTION **************************/
volatile static int16_t angle_of_encoder;

void encoder_init(void);
#endif	/* ENCODER_H */


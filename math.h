/* 
 * File:   math.h
 */

#ifndef MATH_H
#define	MATH_H

#include <xc.h>

#define	PI 3.1416
#define	RAD_TO_DEGREE 57.296
#define	DEGREE_TO_RAD 0.0174533

uint32_t power(uint16_t, uint8_t);
double sqrt(uint16_t);
double atan(float);
float sin(int a);
float cos(int a);

#endif	/* MATH_H */


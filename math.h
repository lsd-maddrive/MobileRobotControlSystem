/** 
* @file math.h
* @brief Declaration of math function's
*/

#ifndef MATH_H
#define	MATH_H

#include <xc.h>

#define	PI 3.1416
#define	RAD_TO_DEGREE 57.296
#define	DEGREE_TO_RAD 0.0174533

uint32_t Pow(uint16_t, uint8_t);
double Sqrt(uint16_t);
double Atan(float);
float Sin(int16_t a);
float Cos(int16_t a);
uint16_t Abs_16(int16_t num);
uint32_t Abs_32(int32_t num);
double Abs_double(double num);

int16_t Calculate_angle(int16_t dx, int16_t dy);
uint16_t Calculate_distance(int16_t dx, int16_t dy);

#endif	/* MATH_H */


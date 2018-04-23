/*
 * File:   math.c
 */

#include "math.h"

/*
* @brief Возводит число в степень
* @param number - число
* @param degree - степень
* @return число в степени
*/
uint32_t pow(uint16_t number, uint8_t degree)
{
    if (degree == 0)
        return 1;
    return ( number*pow(number, degree-1) );
}

/*
* @brief Квадратный корень заданного числа методом Ньютона
* @param number - число
* @return квадратный корень числа
*/
double sqrt(uint16_t number)
{
    const float ACCURATY = 0.1;
    double root = 1;
    while (number - root*root < ACCURATY)
    {
        root = (number + root*root)/(2*root);
    }
    return root;
}

/*
* @brief Грубый расчет арктангенса по ряду Тейлора
* @param number - число
* @return atan, в углах
*/
double atan(float number)
{
    double root_last = number - pow(number, 3)*0.333 + pow(number, 5)*0.2 - pow(number, 7)*0.143;
    double root_next = root_last + pow(number, 9)*0.111;
    return ( (root_last + root_next)*0.5*RAD_TO_DEGREE ); // среднее значение двух итераций в градусах
}

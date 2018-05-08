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
uint32_t power(uint16_t number, uint8_t degree)
{
    if (degree == 0)
        return 1;
    return (uint32_t)( number*power(number, degree-1) );
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
    double root_last = number - power(number, 3)*0.333 + power(number, 5)*0.2 - power(number, 7)*0.143;
    double root_next = root_last + power(number, 9)*0.111;
    return ( (root_last + root_next)*0.5*RAD_TO_DEGREE ); // среднее значение двух итераций в градусах
}

/*
* @brief Грубый расчет синуса по ряду Тейлора
* @param a - угол в градусах
* @return sin
*/
float sin(int16_t  a)
{
    float x = a*DEGREE_TO_RAD;
    float buf = x*x*x*0.166667;
    float answer = x - buf;
    
    buf *= x*x*0.05;
    answer += buf;
    
    buf *= x*x*0.02381;
    answer -= buf;
    
    return answer;
}


/*
* @brief Грубый расчет косинуса по ряду Тейлора
* @param a - угол в градусах
* @return cos
*/
float cos(int16_t  a)
{
    float x = a*DEGREE_TO_RAD;
    float buf = x*x*0.5;
    float answer = 1 - buf;
    
    buf *= x*x*0.083333;
    answer += buf;
    
    buf *= x*x*0.033333;
    answer -= buf;
    
    buf *= x*x*0.017857;
    answer += buf;
    
    return answer;
}

/*
* @brief Модуль числа
* @param num - число
* @return модуль числа
*/
/*
uint16_t abso(int16_t num)
{
    return (num > 0)?num:-num;
}
*/

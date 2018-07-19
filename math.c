/** 
* @file math.c
* @brief Implementation of math function's
*/
#include "math.h"

/**
* @brief Возводит число в степень
* @param number - число
* @param degree - степень
* @return число в степени
*/
uint32_t Power(uint16_t number, uint8_t degree)
{
    if (degree == 0)
        return 1;
    return (uint32_t)( number*Power(number, degree-1) );
}

/**
* @brief Квадратный корень заданного числа методом Ньютона
* @param number - число
* @return квадратный корень числа
*/
double Sqrt(uint16_t number)
{
    const float ACCURATY = 0.1;
    double root = 1;
    while (Abs_double(number - root*root) > ACCURATY)
    {
        root = (number + root*root)/(2*root);
    }
    return root;
}

/**
* @brief Грубый расчет арктангенса по ряду Тейлора
* @note Погрешность обычно не больше 1 градуса
* @param number - число
* @return atan, в углах
*/
double Atan(float number)
{
    // Ряд Тейлора работает только для аргументов <1, поэтому, если больше, то берем обратное число.
    // В конце преобразуем полученное число в нужное
    uint8_t isInverse = 0;
    if(number > 1)
    {
        number = 1/number;
        isInverse = 1;
    }
    double root_last = number - Power(number, 3)*0.333 + Power(number, 5)*0.2 - Power(number, 7)*0.143;
    double root_next = root_last + Power(number, 9)*0.111;
    double answer = ( (root_last + root_next)*0.5*RAD_TO_DEGREE ); // среднее значение двух итераций в градусах
    if (isInverse)
        return (90 - answer);
    return answer;
}

/**
* @brief Грубый расчет синуса по ряду Тейлора
* @param a - угол в градусах [-360; +360]
* @return sin
*/
float Sin(int16_t  a)
{
    // Данный ряд работает при углах [0; 90]
    // Предполагаем, что значение аргумента лежит в диапазоне [-360; +360], тогда:
    uint8_t isNegative = 0;
    // 1-ый квадрант
    if ( (a <= -270) && (a >= -360) )
        a = 360 + a;
    // 2-ой квадрант
    else if ( (a >= 90) && (a <= 180) )
        a = 180 - a;
    else if ( (a <= -180) && (a >= -270) )
        a = -180 - a;
    // 3-ий квадрант
    else if ( (a >= 180) && (a <= 270) )
    {
        isNegative = 1;
        a = a - 180;
    }
    else if ( (a <= -90) && (a >= -180) )
    {
        isNegative = 1;
        a = a + 180;
    }
    // 4-ый квадрант

    else if ( (a <= 0) && (a >= -90) )
    {
        isNegative = 1;
        a = -a;
    }
    else if ( (a >= 270) && (a <= 360) )
    {
        isNegative = 1;
        a = 360 - a;
    }

    float x = a*DEGREE_TO_RAD;
    float buf = x*x*x*0.166667;
    float answer = x - buf;
    buf *= x*x*0.05;
    answer += buf;
    buf *= x*x*0.02381;
    answer -= buf;

    if (isNegative)
        return -answer;
    return answer;
}


/**
* @brief Грубый расчет косинуса по ряду Тейлора
* @param a - угол в градусах [-360; +360]
* @return cos
*/
float Cos(int16_t  a)
{
    // Данный ряд работает при углах [0; 90]
    // Предполагаем, что значение аргумента лежит в диапазоне [-360; +360], тогда:
    uint8_t isNegative = 0;
    // 1-ый квадрант
    if ( (a <= -270) && (a >= -360) )
    {
        a = 360 + a;
    }
    // 2-ой квадрант
    else if ( (a >= 90) && (a <= 180) )
    {
        isNegative = 1;
        a = 180 - a;
    }
    else if ( (a <= -180) && (a >= -270) )
    {
        isNegative = 1;
        a = -180 - a;
    }
    // 3-ий квадрант
    else if ( (a >= 180) && (a <= 270) )
    {
        isNegative = 1;
        a = a - 180;
    }
    else if ( (a <= -90) && (a >= -180) )
    {
        isNegative = 1;
        a = a + 180;
    }
    // 4-ый квадрант

    else if ( (a <= 0) && (a >= -90) )
    {
        a = -a;
    }
    else if ( (a >= 270) && (a <= 360) )
    {
        a = 360 - a;
    }

    float x = a*DEGREE_TO_RAD;
    float buf = x*x*0.5;
    float answer = 1 - buf;
    buf *= x*x*0.083333;
    answer += buf;
    buf *= x*x*0.033333;
    answer -= buf;
    buf *= x*x*0.017857;
    answer += buf;

    if (isNegative)
        return -answer;
    return answer;
}

/**
* @brief Модуль 2-ух байтного числа
* @param num - число
* @return модуль числа
*/
uint16_t Abs_16(int16_t num)
{
    return (num > 0)?num:-num;
}


/**
* @brief Модуль 4-ех байтного числа
* @param num - число
* @return модуль числа
*/
uint32_t Abs_32(int32_t num)
{
    return (num > 0)?num:-num;
}

/**
* @brief Модуль числа типа double
* @param num - число
* @return модуль числа
*/
double Abs_double(double num)
{
    return (num > 0)?num:-num;
}

/**
* @brief Расчет угла поворота по длинам двух катетов
* @param dx - катет по горизонтали
* @param dy - катет по вертикали
* @return angle - угол в диапазоне [-180; +180] градусов
*/
int16_t Calculate_angle(int16_t dx, int16_t dy)
{
    /// Избегаем деления на ноль:
    if (dy == 0)
    {
        if (dx > 0)
        {
            return 90;
        }
        else if (dx == 0)
        {
            return 0;
        }
        return -90;
    }
    if (dx == 0)
    {
        if (dy > 0)
        {
            return 0;
        }
        return 180;
    }
    /// Основной расчет:
    int16_t angle = Atan( Abs_double((float)dx/dy) );
    if (dx > 0 && dy < 0)       /// 4-ый квадрант
        angle = 180 - angle;
    else if (dx < 0 && dy < 0)  /// 3-ий квадрант
        angle -= 180;
    else if (dx < 0 && dy > 0)  /// 2-ой квадрант
        angle = -angle;
    else                        /// 1-ый квадрант
        angle = angle;

    return angle;
}

/** 
* @brief Расчет гипатенузы по двум катетам
* @param dx - катет по горизонтали
* @param dy - катет по вертикали
*/
uint16_t Calculate_distance(int16_t dx, int16_t dy)
{
    return Sqrt(dx*dx + dy*dy);
}
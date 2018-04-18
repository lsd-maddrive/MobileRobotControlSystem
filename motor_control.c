/* 
 * File:   motor_control.c
 */

#include "motor_control.h"

/*
* @brief Подать питание на двигатель
* @param power - мощность в %
* @param motor - название двигателя
*/
void motor_set_power(int8_t power, uint8_t motor)
{
    // Вращение в одну сторону
    if (power > 0)
    {
        if (power > 100)
            power = 100;
        PWM_set(power, motor+1);
        PWM_set(0, motor+2);
    }
    // Вращение в другую сторону
    else
    {
        if (power < -100)
            power = 100;
        else
            power = -power;
        PWM_set(0, motor+1);
        PWM_set(power, motor+2);
    }
}

/*
* @brief Остановить двигатели
*/
void motors_stop()
{
    PWM_set(0, MOTOR_LEFT_1);
    PWM_set(0, MOTOR_LEFT_2);
    PWM_set(0, MOTOR_RIGHT_1);
    PWM_set(0, MOTOR_RIGHT_2);
}

/*
* @brief Инициализация ШИМ
*/
void motor_init()
{
    PWM_init();
    motors_stop();
}

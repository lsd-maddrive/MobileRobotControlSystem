/* 
 * File:   motor_control.c
 */

#include "motor_control.h"

/* Подключение двигателей:
 * Двигатель        Подключение к драйверу          Подключение к dsPIC  
 * Правый           PWM 5, 6                        RE6, RE4
 * Левый            PWM 3, 11                       RE2, RE0
 */

/*
* @brief Подать питание на двигатель
* @param power - мощность в % от максимальной
* @param motor - название двигателя (MOTOR_LEFT, MOTOR_RIGHT)
*/
void motor_set_power(int8_t power, uint8_t motor)
{
    // Вращение вперед (не обязательно по часовой стрелке)
    if (power > 0)
    {
        if (power > 100)
            power = 100;
        PWM_set(power, motor+1);
        PWM_set(0, motor+2);
    }
    // Вращение назад (не обязательно против часовой стрелке)
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
* @brief Инициализация двигателей
*/
void motor_init()
{
    PWM_init();
    motors_stop();
}

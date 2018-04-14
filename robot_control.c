/* 
 * File:   robot_control.c
 */

#include <xc.h>
#include "robot_control.h"

/* 
 * @brief Инициализация всей переферии
*/
void init_periphery()
{
    GPIO_init();
    motor_init();
    encoder_init();
    //software_timer_init();
    //findranger_init();
}

/* 
 * @brief Тест работы модуля motor_control на разных мощностях и в разных направлениях
*/
void test_motor_control()
{
    enum
    {
        NUMBER_OF_SPEED = 14,
    };
    int8_t power[NUMBER_OF_SPEED] = {10, 40, 70, 100, 70, 40, 10, -10, -40, -70, -100, -70, -40, -10};
    uint32_t countOfDelay;
    uint32_t countOfSpeed;
    for(countOfSpeed = 0; countOfSpeed < NUMBER_OF_SPEED; countOfSpeed++)
    {
        motor_set_power(power[countOfSpeed], MOTOR_LEFT);
        motor_set_power(power[countOfSpeed], MOTOR_RIGHT);
        for(countOfDelay = 0; countOfDelay < 10000000; countOfDelay++);
    }
}

/* 
 * @brief Тест работы модуля uart
*/
void test_uart()
{
    
}

/* 
 * @brief Тест работы модуля encoder
*/
void test_encoder()
{
    
}

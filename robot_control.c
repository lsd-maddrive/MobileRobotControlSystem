/* 
 * File:   robot_control.c
 */

#include "robot_control.h"
#include <stdio.h>  // временное решение

UART_module* debug;
Timer* timer; 

/* 
 * @brief Инициализация всей переферии
 */
void init_periphery() 
{
    GPIO_init();
    motor_init();
    encoders_init();
    debug = UART_init(UART_1, UART_BAUD_RATE_9600);
    hard_timer_init();
    timer = timer_create();
    //findranger_init();
}

/* 
 * @brief Тест работы модуля motor_control на разных мощностях и в разных направлениях
 */
void test_motor_control() 
{
    enum 
    {
        NUMBER_OF_SPEED = 18,
    };
    int8_t power[NUMBER_OF_SPEED] = 
    {
         9,  18,  24,  28,  30,  28,  24,  18,  9, 
        -9, -18, -24, -28, -30, -28, -24, -18, -9
    };
    uint32_t countOfDelay;
    uint32_t countOfSpeed;
    for (countOfSpeed = 0; countOfSpeed < NUMBER_OF_SPEED; countOfSpeed++) 
    {
        motor_set_power(power[countOfSpeed], MOTOR_LEFT);
        motor_set_power(power[countOfSpeed], MOTOR_RIGHT);
        for (countOfDelay = 0; countOfDelay < 400000; countOfDelay++);
    }
    motors_stop();
}

/* 
 * @brief Тест работы модуля uart
 */
void test_uart() 
{
    UART_transmit(debug, "Test UART is succeed\n", 21);
}

/* 
 * @brief Тест модуля software_timer
 */
void test_software_timer() 
{
    timer_start_ms(timer, 10000);   // запуск таймера на 10 сек
    while( timer_report(timer) != FINISHED) 
    {
        uint32_t countOfDelay;
        for (countOfDelay = 0; countOfDelay < 5000000; countOfDelay++);
        /* КАСТЫЛЬ СНИЗУ: жрет 60 байт (0.2% от максимума) памяти данных!!!!!*/
        char buffer[12];
        sprintf(buffer, "%lu", timer_get_rest_time(timer) );
        UART_transmit(debug, buffer, 12);
        UART_transmit(debug, "\n", 1);
        /* КАСТЫЛЬ СВЕРХУ: жрет 1307 байт (1.5% от максимума) памяти программы!!!!!*/
    }
}

/* 
 * @brief Тест работы модуля encoder
 */
void test_encoder() 
{
    int32_t angleLeft = encoder_left_get_angle();
    int32_t angleRight = encoder_right_get_angle();
    uint8_t count, countOfDelay;
    
    motor_set_power(10, MOTOR_LEFT);
    motor_set_power(10, MOTOR_RIGHT);
    for (count = 0; count < 10; count++)
    {
        /* КАСТЫЛЬ СНИЗУ: жрет много памяти данных!!!!!*/
        char buffer[12];
        sprintf(buffer, "%lu", angleLeft);
        UART_transmit(debug, buffer, 12);
  
        sprintf(buffer, "%lu", angleRight);
        UART_transmit(debug, buffer, 12);
        /* КАСТЫЛЬ СВЕРХУ: жрет много памяти программы!!!!!*/
        for (countOfDelay = 0; countOfDelay < 400000; countOfDelay++);
    }
    motors_stop();
}

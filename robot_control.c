/* 
 * File:   robot_control.c
 */

#include "robot_control.h"
#include <stdio.h>  // временное решение

UART_module* debug;
Timer* soft_timer; 

/* 
 * @brief Инициализация всей переферии
 */
void init_periphery() 
{
    GPIO_init();
    motor_init();
    encoder_init();
    debug = UART_init(UART_1, UART_BAUD_RATE_9600);
    soft_timer = create_timer();
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
    int8_t power[NUMBER_OF_SPEED] = {10, 20, 30, 50, 30, 20, 10, -10, -20, -30, -50, -30, -20, -10};
    uint32_t countOfDelay;
    uint32_t countOfSpeed;
    for (countOfSpeed = 0; countOfSpeed < NUMBER_OF_SPEED; countOfSpeed++) 
    {
        motor_set_power(power[countOfSpeed], MOTOR_LEFT);
        motor_set_power(power[countOfSpeed], MOTOR_RIGHT);
        for (countOfDelay = 0; countOfDelay < 10000000; countOfDelay++);
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
    start_timer_ms(soft_timer, 10000);   // запуск таймера на 10 сек
    while(1) 
    {
        uint32_t countOfDelay;
        for (countOfDelay = 0; countOfDelay < 5000000; countOfDelay++);
        /* КАСТЫЛЬ СНИЗУ: жрет 60 байт (0.2% от максимума) памяти данных!!!!!*/
        char buffer[12];
        sprintf(buffer, "%lu", get_rest_time(soft_timer) );
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
    /* КАСТЫЛЬ СНИЗУ: жрет много памяти данных!!!!!*/
    char buffer[12];
    sprintf(buffer, "%lu", get_angle());
    UART_transmit(debug, buffer, 12);
    /* КАСТЫЛЬ СВЕРХУ: жрет много памяти программы!!!!!*/
}

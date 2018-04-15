/* 
 * File:   robot_control.c
 */

#include <xc.h>
#include "robot_control.h"
#include <stdio.h>

UART_module* debug;
/* 
 * @brief Инициализация всей переферии
 */
void init_periphery() {
    GPIO_init();
    motor_init();
    encoder_init();
    debug = UART_init(UART_1, UART_BAUD_RATE_9600);
    //software_timer_init();
    //findranger_init();
}

/* 
 * @brief Тест работы модуля motor_control на разных мощностях и в разных направлениях
 */
void test_motor_control() {

    enum {
        NUMBER_OF_SPEED = 14,
    };
    int8_t power[NUMBER_OF_SPEED] = {10, 40, 70, 100, 70, 40, 10, -10, -40, -70, -100, -70, -40, -10};
    uint32_t countOfDelay;
    uint32_t countOfSpeed;
    for (countOfSpeed = 0; countOfSpeed < NUMBER_OF_SPEED; countOfSpeed++) {
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
    Timer* timer = create_timer();  // конструктор таймера
    start_timer_ms(timer, 10000);   // запуск таймера на 10 сек
    while(1) 
    {
        uint32_t countOfDelay;
        for (countOfDelay = 0; countOfDelay < 5000000; countOfDelay++);
        /* КАСТЫЛЬ СНИЗУ: жрет 60 байт (0.2% от максимума) памяти данных!!!!!*/
        char buffer[12];
        sprintf(buffer, "%lu", get_rest_time(debug) );
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
    
}

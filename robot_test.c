/* 
 * File:   robot_test.c
 */

#include "robot_test.h"
#include "math.h"
#include "text.h"

// Глобальные и статические переменные:
extern UART_module* debug;
extern Timer timer; 
extern Timer timerSub; 
extern Robot_data robot;

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
    UART_transmit(debug, "Case 1. Test UART_transmit() is succeed\n\r", 41);
    UART_write_string(debug, "Case 2. Test UART_write_string() is succeed\n\r\0");
    uint32_t count;
    for (count = 0; count < 3000000; count++);
}

/* 
 * @brief Тест модуля software_timer
 */
void test_software_timer() 
{
    int8_t count = -5;
    char buffer[12];

    UART_write_string(debug, "Test 50 sec!\n\r\0");
    timer_start_ms(&timer, 50000);
    while(timer_report(&timer) == WORKING)
    {
        num2str(count+=5, buffer); 
        UART_write_string(debug, buffer);
        UART_write_string(debug, " sec from 50.\n\r\0");
        
        UART_write_string(debug, "- hard_time: ");
        num2str(hard_timer_return_time(), buffer); 
        UART_write_string(debug, buffer);
        UART_write_string(debug, "\n\r\0");
        
        UART_write_string(debug, "- elapsed time: \0");
        num2str(timer_get_elapsed_time(&timer), buffer); 
        UART_write_string(debug, buffer);
        UART_write_string(debug, "\n\r\0");
        
        UART_write_string(debug, "- rest time:    \0");
        num2str(timer_get_rest_time(&timer), buffer); 
        UART_write_string(debug, buffer);
        UART_write_string(debug, "\n\r\n\r\0");
        
        timer_start_ms(&timerSub, 5000);
        while(timer_report(&timerSub) == WORKING);
    }
}

/* 
 * @brief Тест работы модуля encoder
 */
void test_encoder() 
{
    int32_t pulsesLeft = encoder_left_get_pulses();
    int32_t pulsesRight = encoder_right_get_pulses();
    uint8_t count, countOfDelay;
    
    motor_set_power(10, MOTOR_LEFT);
    motor_set_power(10, MOTOR_RIGHT);
    for (count = 0; count < 10; count++)
    {
        char buffer[12];
        num2str(pulsesLeft, buffer);
        UART_transmit(debug, buffer, 12);
  
        num2str(pulsesRight, buffer);
        UART_transmit(debug, buffer, 12);
        for (countOfDelay = 0; countOfDelay < 400000; countOfDelay++);
    }
    motors_stop();
}

/* 
 * @brief Тест работы модуля rangefinder
 */
void test_rangefinder()
{
    rangefinder_give_impulse();
    timer_start_ms(&timer, 100);
    while(timer_report(&timer) == WORKING);
    uint32_t range = rangefinder_get_range();
    
    char buffer[12];
    num2str(range, buffer); 
    UART_write_string(debug, "range = ");
    UART_write_string(debug, buffer);
    UART_write_string(debug, "\n\r\n\r");
    
    timer_start_ms(&timer, 500);
    while(timer_report(&timer) == WORKING);
}
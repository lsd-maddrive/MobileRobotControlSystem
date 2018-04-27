/* 
 * File:   robot_control.c
 */

/* Описание системы управления
 * Система управления строится на 2 базовых действиях:
 * - поворот на месте 
 * - прямолинейное движение
 *
 * 1. Поворот на месте
 * Во время поворота на месте двигатели вращаются в противоположные стороны с
 * одинаковой мощностью, пропорциональной заданной скорости робота.
 * Среднее кол-во импульсов с обоих энкодеров переводится в угол путем умножения
 * кол-ва импульсов на коэффициент, полученный опытным путем в ходе калибровки.
 *
 * 2. Прямолинейное движение
 * Во время прямолинейного движения двигатели вращаются в одинаковые стороны с
 * средней мощностью, пропорциональной заданной скорости робота.
 * Если суммарное кол-во импульсов одного из энкодеров будет отличаться, то мощ-
 * ность одного из двигателей будет увеличена, а другого - уменьшена.
 * Кол-во импульсов энкодера, соответствующее единице расстояния определяется
 * опытнм путем в ходе калибровки.
 *
 * Наверно, будем считать аддитивную погрешность определения координат - 1 см.
 */

#include "robot_control.h"
#include "math.h"
#include "text.h"
#include <stdio.h>  // временное решение

enum Start_data
{
    ROBOT_START_MIN_SPEED = 10,     // Исходное значение минимальной скорости см/сек
    ROBOT_START_MAX_SPEED = 10,     // Исходное значение максимальной скорости см/сек
    ROBOT_START_ACCELERATION = 10,  // Исходное значение ускорения см/сек^2
    ROBOT_START_DECELERATION = 10,  // Исходное значение замедления см/сек^2
};

enum Calibration
{
    PULSES_IN_REVOLUTION = 500,
    PULSES_IN_METER = 1000,
};

typedef struct 
{
    int16_t x;
    int16_t y;
    int16_t angle;
    uint16_t range;
    uint8_t minSpeed;
    uint8_t maxSpeed;
    uint8_t currentSpeed;
    uint8_t acceleration;
    uint8_t deceleration;
} Robot_data;


// Глобальные и статические переменные:
static UART_module* debug;
static Timer timer; 
static Timer timerSub; 
static Robot_data robot =
{
    .x = 0, .y = 0, .angle = 0, .range = 0, 
    .minSpeed = ROBOT_START_MIN_SPEED,
    .maxSpeed = ROBOT_START_MAX_SPEED,
    .currentSpeed = 0,
    .acceleration = ROBOT_START_ACCELERATION,
    .deceleration = ROBOT_START_DECELERATION
};

/* 
 * @brief Поворот на указанный угол
 */
inline void turn_around_by(int16_t angle)
{
    encoders_reset_angle();
    if ( angle > 0) // поворот по часовой
    {
        motor_set_power(robot.minSpeed,  MOTOR_LEFT);
        motor_set_power(-robot.minSpeed, MOTOR_RIGHT);
        uint16_t needPulses = PULSES_IN_REVOLUTION*angle/360;
        while(encoder_right_get_pulses() < needPulses);
    }
    else if ( angle < 0) // поворот против часовой
    {
        motor_set_power(-robot.minSpeed, MOTOR_LEFT);
        motor_set_power(robot.minSpeed,  MOTOR_RIGHT);
        uint16_t needPulses = PULSES_IN_REVOLUTION*(-1*angle)/360;
        while(encoder_left_get_pulses() < needPulses);
    }
    else
    {
        return;
    }
    motors_stop();
    robot.angle =+ angle;
}

/* 
 * @brief Поворот к указанному углу
 */
void turn_around_to(int16_t angle)
{
    turn_around_by(angle - robot.angle);
}

/* 
 * @brief Прямолинейное движение вперед на указанное расстояние (в см)
 * @param distance - расстояния в см
 */
void move_forward(uint16_t distance)
{
    const uint8_t PULSES_HYSTERESIS = 10;
    const float PROPORTIONAL_REGULATOR = 0.2;
    const uint16_t needPulses = PULSES_IN_METER*distance;
    uint16_t averagePulses = ( encoder_left_get_pulses() + encoder_right_get_pulses() ) >> 1;
    
    while(averagePulses < needPulses)
    {
        if( (encoder_left_get_pulses() > (encoder_right_get_pulses() + PULSES_HYSTERESIS) ) ||
            ( encoder_right_get_pulses() > (encoder_left_get_pulses() + PULSES_HYSTERESIS) ) )
        {
            uint8_t speedChange = ( encoder_left_get_pulses() - encoder_right_get_pulses() )*PROPORTIONAL_REGULATOR;
            motor_set_power(robot.minSpeed + speedChange,  MOTOR_LEFT);
            motor_set_power(robot.minSpeed - speedChange, MOTOR_RIGHT);
        }
        else
        {
            motor_set_power(robot.minSpeed,  MOTOR_LEFT);
            motor_set_power(robot.minSpeed, MOTOR_RIGHT);
        }
        averagePulses = ( encoder_left_get_pulses() + encoder_right_get_pulses() ) >> 1;
    }
    motors_stop();
}

/* 
 * @brief Прямолинейное движение к указанной координате
 */
void move_to(int16_t x, int16_t y)
{
    int16_t dx = (robot.x - x);
    int16_t dy = (robot.y - y);
    uint16_t distance = sqrt( dx*dx + dy*dy );
    int16_t angle = atan( (float)dx/dy );
    if (dx >= 0 && dy <= 0) // 4-ый квадрант
        angle =+ 180;
    else if (dx <= 0 && dy <= 0) // 3-ий квадрант
        angle =- 180;
    turn_around_by(angle);
    move_forward(distance);
    robot.x =+ x; 
    robot.y =+ y; 
}
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
    soft_timer_init(&timer);
    soft_timer_init(&timerSub);
    rangefinder_init();
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
    while(1)
    {
        UART_write_string(debug, "Test UART is succeed\n\r");
        uint32_t count;
        for (count = 0; count < 3000000; count++);
    }
    
}

/* 
 * @brief Тест модуля software_timer
 */
void test_software_timer() 
{
    uint8_t count = 0;
    char buffer[12];

    UART_write_string(debug, "Test 20 sec!\n\r");
    timer_start_ms(&timer, 20000);
    while(timer_report(&timer) == WORKING)
    {
        num2str(count++, buffer); 
        UART_write_string(debug, buffer);
        UART_write_string(debug, " sec from 20.\n\r");
        
        UART_write_string(debug, "- hard_time: ");
        num2str(hard_timer_return_time(), buffer); 
        UART_write_string(debug, buffer);
        UART_write_string(debug, "\n\r\0");
        
        UART_write_string(debug, "- elapsed time: ");
        num2str(timer_get_elapsed_time(&timer), buffer); 
        UART_write_string(debug, buffer);
        UART_write_string(debug, "\n\r\0");
        
        UART_write_string(debug, "- rest time:    ");
        num2str(timer_get_rest_time(&timer), buffer); 
        UART_write_string(debug, buffer);
        UART_write_string(debug, "\n\r\n\r\0");
        
        timer_start_ms(&timerSub, 1000);
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
        /* КАСТЫЛЬ СНИЗУ: жрет много памяти данных!!!!!*/
        char buffer[12];
        sprintf(buffer, "%lu", pulsesLeft);
        UART_transmit(debug, buffer, 12);
  
        sprintf(buffer, "%lu", pulsesRight);
        UART_transmit(debug, buffer, 12);
        /* КАСТЫЛЬ СВЕРХУ: жрет много памяти программы!!!!!*/
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
    uint32_t range = rangefinder_get_range()*17/200;
    
    char buffer[12];
    num2str(range, buffer); 
    UART_write_string(debug, "range = ");
    UART_write_string(debug, buffer);
    UART_write_string(debug, "\n\r\n\r");
    
    timer_start_ms(&timer, 500);
    while(timer_report(&timer) == WORKING);
}

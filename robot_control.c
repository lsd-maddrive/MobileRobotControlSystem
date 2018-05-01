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




// Глобальные и статические переменные:
UART_module* debug;
Timer timer; 
Timer timerSub; 
Robot_data robot =
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

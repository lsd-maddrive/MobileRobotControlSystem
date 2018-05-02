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
 * Допущения:
 * - Наверно, будем считать аддитивную погрешность определения координат - 1 см.
 * - Предполагаем, что препятствие - прямоугольный параллепипед с максимальными
 * размерами ширины/длины - 50 см.
 *
 */

#include "robot_control.h"
#include "math.h"
#include "text.h"
#include "string.h" // для memcpy

enum Initial_data
{
    ROBOT_START_MIN_SPEED = 10,     // Исходное значение минимальной скорости см/сек
    ROBOT_START_MAX_SPEED = 10,     // Исходное значение максимальной скорости см/сек
    ROBOT_START_ACCELERATION = 10,  // Исходное значение ускорения см/сек^2
    ROBOT_START_DECELERATION = 10,  // Исходное значение замедления см/сек^2
};

enum Calibration
{
    PULSES_IN_REVOLUTION = 500,
    PULSES_IN_CM = 10,
    ROBOT_WIDTH = 200,  // 20 см
    ROBOT_LENGTH = 300,  // 30 см
    ROBOT_HALF_WIDTH = ROBOT_WIDTH >> 1,
    ROBOT_HALF_LENGTH = ROBOT_LENGTH >> 1,
    OBSTACLE_DANGEROUS_DISTANCE = 500,  // 50 см
};

typedef struct 
{
    int16_t x;
    int16_t y;
} Target_point;

typedef struct 
{
    uint16_t range;
    int16_t angle;
} Measured_point;



// Глобальные и статические переменные:
UART_module* debug;
Timer timer; 
Timer timerSub;
Robot_data robot =
{
    .status = ROBOT_INITIALIZED,
    .x = 0, .y = 0, .angle = 0, .range = 0, 
    .minSpeed = ROBOT_START_MIN_SPEED,
    .maxSpeed = ROBOT_START_MAX_SPEED,
    .currentSpeed = 0,
    .acceleration = ROBOT_START_ACCELERATION,
    .deceleration = ROBOT_START_DECELERATION
};
static Target_point target = 
{
    .x = 0, .y = 0
};
static Measured_point intermediatePoint_1 = 
{
    .range = 0, .angle = 0
};
static Measured_point intermediatePoint_2 = 
{
    .range = 0, .angle = 0
};

/****************************** PRIVATE FUNCTION ******************************/
/* 
 * @brief Расчет угла по длинам двух катетов
 */
int16_t calculate_angle(int16_t dx, int16_t dy)
{
    int16_t angle = atan( (float)dx/dy );
    if (dx >= 0 && dy <= 0) // 4-ый квадрант
        angle =+ 180;
    else if (dx <= 0 && dy <= 0) // 3-ий квадрант
        angle =- 180;
    return angle;
}

/* 
 * @brief Расчет гипатенузы по двум катетам
 */
uint16_t calculate_distance(int16_t dx, int16_t dy)
{
    return sqrt(dx*dx + dy*dy);
}

/* 
 * @brief Получить медианное значение среди трех измерений дальности
 */
uint16_t getMedianRange()
{
    rangefinder_give_impulse();
    timer_start_ms(&timer, 100);
    while(timer_report(&timer) == TIMER_WORKING);
    uint16_t range_1 = rangefinder_get_range();
    
    rangefinder_give_impulse();
    timer_start_ms(&timer, 100);
    while(timer_report(&timer) == TIMER_WORKING);
    uint16_t range_2 = rangefinder_get_range();
    
    rangefinder_give_impulse();
    timer_start_ms(&timer, 100);
    while(timer_report(&timer) == TIMER_WORKING);
    uint16_t range_3 = rangefinder_get_range();
    
    if (range_1 >= range_2)
    {
        if (range_1 >= range_3)
            return (range_2 >= range_3) ? range_2 : range_3;
        else
            return range_1;
    }
    else
    {
        if (range_2 >= range_3)
            return (range_1 > range_3) ? range_1 : range_3;
    }
        
    return range_2;
}

/* 
 * @brief Определение курсового угла и расстояния до левого края препятствия
 * @note Предполагается, что диаметральная плоскость робота проходит через препятствие,
 * @note т.е. для наискорейшего нахождения левой границы нужно поворачивать против часовой.
 */
void measure_left_border()
{
    enum
    {
        MAX_DIF_RANGE = 300,    // 30 см
        MAX_DIF_ANGLE = 60,     // 60 градусов
        ITERATION_DIF_ANGLE = 5,// 5 градуса
    };
    int16_t initialAngle = robot.angle;
    int16_t measuredAngle = robot.angle;
    uint16_t oldRange = getMedianRange();
    uint16_t newRange;
    while(measuredAngle - initialAngle > MAX_DIF_ANGLE)
    {
        measuredAngle -= ITERATION_DIF_ANGLE; 
        turn_around_by(-ITERATION_DIF_ANGLE);
        oldRange = newRange;
        newRange = getMedianRange();
        if (newRange - oldRange > MAX_DIF_RANGE)
        {
            uint8_t count;
            for(count = 0; count < ITERATION_DIF_ANGLE; count++)
            {
                turn_around_by(1);
                oldRange = newRange;
                newRange = getMedianRange();
                if (oldRange - newRange < MAX_DIF_RANGE)
                {
                    measuredAngle += count;
                    break;
                }  
            }
            intermediatePoint_1.angle = measuredAngle;
            intermediatePoint_1.range = newRange;
            return;   // левая граница найдена
        } 
    }
    intermediatePoint_1.angle = robot.angle;
    intermediatePoint_1.range = 0;
    return; // левая граница не найдена
}

/* 
 * @brief Определение курсового угла и расстояния до правого края препятствия
 * @note Предполагается, что диаметральная плоскость робота проходит через препятствие,
 * @note т.е. для наискорейшего нахождения правой границы нужно поворачивать по часовой.
 */
void measure_right_border()
{
    enum
    {
        MAX_DIF_RANGE = 300,    // 30 см
        MAX_DIF_ANGLE = 60,     // 60 градусов
        ITERATION_DIF_ANGLE = 5,// 5 градуса
    };
    int16_t initialAngle = robot.angle;
    int16_t measuredAngle = robot.angle;
    uint16_t oldRange = getMedianRange();
    uint16_t newRange;
    while(measuredAngle - initialAngle > MAX_DIF_ANGLE)
    {
        measuredAngle += ITERATION_DIF_ANGLE; 
        turn_around_by(+ITERATION_DIF_ANGLE);
        oldRange = newRange;
        newRange = getMedianRange();
        if (newRange - oldRange > MAX_DIF_RANGE)
        {
            uint8_t count;
            for(count = 0; count < ITERATION_DIF_ANGLE; count++)
            {
                turn_around_by(-1);
                oldRange = newRange;
                newRange = getMedianRange();
                if (oldRange - newRange < MAX_DIF_RANGE)
                {
                    measuredAngle -= count;
                    break;
                }  
            }
            robot.angle = measuredAngle;
            intermediatePoint_2.angle = measuredAngle;
            intermediatePoint_2.range = newRange;
            return;   // правая граница найдена
        } 
    }
    intermediatePoint_2.angle = robot.angle;
    intermediatePoint_2.range = 0;
    robot.angle = measuredAngle;
    return; // правая граница не найдена
}
/****************************** PRIVATE FUNCTION ******************************/


/****************************** PUBLIC FUNCTION *******************************/
/* 
 * @brief Поворот на указанный угол
 */
void turn_around_by(int16_t angle)
{
    uint16_t needPulses;
    encoders_reset_angle();
    if ( angle > 0) // поворот по часовой
    {
        motor_set_power(robot.minSpeed,  MOTOR_LEFT);
        motor_set_power(-robot.minSpeed, MOTOR_RIGHT);
        needPulses = PULSES_IN_REVOLUTION*angle/360;  
    }
    else if ( angle < 0) // поворот против часовой
    {
        motor_set_power(-robot.minSpeed, MOTOR_LEFT);
        motor_set_power(robot.minSpeed,  MOTOR_RIGHT);
        needPulses = PULSES_IN_REVOLUTION*(-1*angle)/360;
    }
    else
    {
        return;
    }
    while(encoder_right_get_pulses() < needPulses);
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
 * @note с П-регулятором
 * @param distance - расстояние (в см)
 */
void move_forward(uint16_t distance)
{
    // Константы, полученные эмпирическим путем:
    const uint8_t PULSES_HYSTERESIS = 10;
    const float PROPORTIONAL_REGULATOR = 0.2;
    
    const uint16_t needPulses = PULSES_IN_CM*distance;
    uint16_t nowPulses = ( encoder_left_get_pulses() + encoder_right_get_pulses() ) >> 1;
    uint8_t speedChange = 0;
    robot.range = 0;
    
    while(nowPulses < needPulses)
    {
        // Пассивная проверка на наличие препятствия (не влияет на движение робота):
        if (timer_report(&timer) != TIMER_WORKING)
        {
            rangefinder_give_impulse();
            timer_start_ms(&timer, 100);
            robot.range = rangefinder_get_range();
        }
        // Активная проверка на наличие препятствия (робот остановливается):
        if (robot.range < OBSTACLE_DANGEROUS_DISTANCE)
        {
            motors_stop();
            if (getMedianRange() < OBSTACLE_DANGEROUS_DISTANCE)
            {
                distance = nowPulses/PULSES_IN_CM;
                break;
            }
        }
        // Алгоритм подстройки скорости двигателей:
        if( (encoder_left_get_pulses() > (encoder_right_get_pulses() + PULSES_HYSTERESIS) ) ||
            ( encoder_right_get_pulses() > (encoder_left_get_pulses() + PULSES_HYSTERESIS) ) )
        {
            speedChange = ( encoder_left_get_pulses() - encoder_right_get_pulses() )*PROPORTIONAL_REGULATOR;
            motor_set_power(robot.minSpeed + speedChange,  MOTOR_LEFT);
            motor_set_power(robot.minSpeed - speedChange, MOTOR_RIGHT);
        }
        else
        {
            motor_set_power(robot.minSpeed + speedChange,  MOTOR_LEFT);
            motor_set_power(robot.minSpeed - speedChange, MOTOR_RIGHT);
        }
        nowPulses = ( encoder_left_get_pulses() + encoder_right_get_pulses() ) >> 1;
    }
    motors_stop();
    robot.x = sin(robot.angle)*distance;
    robot.y = cos(robot.angle)*distance;
}

/* 
 * @brief Прямолинейное движение к указанной координате
 * @param x, y - декартовы координаты
 */
void move_to(int16_t x, int16_t y)
{
    int16_t dx = (robot.x - x);
    int16_t dy = (robot.y - y);
    uint16_t distance = calculate_distance(dx, dy);
    int16_t angle = calculate_angle(dx, dy);
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

void move_with_obstacle_avoidance_get_coordinates(int16_t x, int16_t y)
{
    target.x = x;
    target.y = y;
    robot.status = ROBOT_INITIALIZED;
}

/*
Robot_status move_with_obstacle_avoidance_do()
{
    enum Step
    {
        STEP_1_ROTATE_TO_ANGLE = 1,
        STEP_2_MOVE_FORWARD = 2,
        STEP_3_MEASURE_LEFT_BORDER = 3,
        STEP_4_ROTATE_TO_ANGLE = 4,
        STEP_5_MEASURE_RIGTH_BORDER = 5,
        STEP_6_CALCULATE_INTERMEDIATE_POINTS = 6,
        STEP_7_ROTATE_TO_ANGLE = 7,
        STEP_8_MOVE_FORWARD = 8,
        STEP_9_ROTATE_TO_ANGLE = 9,
        STEP_10_MOVE_FORWARD = 10,
        STEP_11_ROTATE_TO_ANGLE = 11,
        STEP_12_MEASURE_BOARD = 12,
        STEP_13_CALCULATE_INTERMEDIATE_POINTS = 13,
        STEP_14_ROTATE_TO_ANGLE = 14,
        STEP_15_MOVE_FORWARD = 15,
        STEP_16_ROTATE_TO_ANGLE = 16,
        STEP_17_MOVE_FORWARD = 17,
    };
    static uint8_t stepCount = STEP_1_ROTATE_TO_ANGLE;
    if (robot.status == ROBOT_IN_PROGRESS)
    {
        switch(stepCount)
        {
            case STEP_1_ROTATE_TO_ANGLE:
            {
                int16_t dx = target.x - robot.x;
                int16_t dy = target.y - robot.y;
                turn_around_to( calculate_angle(dx, dy) );
                stepCount++;
                break;
            }
            case STEP_2_MOVE_FORWARD:   // остановится, если обнаружит препятствие или достигнет цели
            {
                int16_t dx = target.x - robot.x;
                int16_t dy = target.y - robot.y;
                move_forward( calculate_distance(dx, dy) );
                stepCount++;
                break;
            }
            case STEP_3_MEASURE_LEFT_BORDER:
            {
                measure_left_border();  // измеренные значения сохраняются в intermediatePoint_1
                stepCount++;
                break;
            }
            case STEP_4_ROTATE_TO_ANGLE:
            {
                int16_t dx = target.x - robot.x;
                int16_t dy = target.y - robot.y;
                turn_around_to( calculate_angle(dx, dy) );
                stepCount++;
                break;
            }
            case STEP_5_MEASURE_RIGTH_BORDER:
            {
                measure_left_border();  // измеренные значения сохраняются в intermediatePoint_2
                stepCount++;
                break;
            }
            case STEP_6_CALCULATE_INTERMEDIATE_POINTS:
            {
                measure_left_border();
                stepCount++;
                break;
            }
            default:
            {
                break;
            }
        }
    }
    else if (robot.status == ROBOT_INITIALIZED)
    {
        stepCount = 0;
    }
    else if (robot.status == ROBOT_FINISHED)
    {
        
    }
    return 0;   // change
}
*/
/* 
 * @brief Передача по uart информацию о роботе (первые 10 байт структуры robot)
 */
void log_transmit()
{
    uint8_t arrOfData[10];
    memcpy(arrOfData, &robot, 10);
    UART_transmit(debug, arrOfData, 10);
    UART_transmit(debug, "\n\r", 2);
}
/****************************** PUBLIC FUNCTION *******************************/

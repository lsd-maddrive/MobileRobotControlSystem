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
    ROBOT_START_MIN_SPEED = 30,     // Исходное значение минимальной скорости см/сек
    ROBOT_START_MAX_SPEED = 50,     // Исходное значение максимальной скорости см/сек
    ROBOT_START_ACCELERATION = 8,  // Исходное значение ускорения см/сек^2
    ROBOT_START_DECELERATION = 8,  // Исходное значение замедления см/сек^2
};

enum Calibration
{
    // Характеристики робота:
    ROBOT_WIDTH = 300,  // 30 см
    ROBOT_LENGTH = 400,  // 40 см
    ROBOT_HALF_WIDTH = ROBOT_WIDTH >> 1,
    ROBOT_HALF_LENGTH = ROBOT_LENGTH >> 1,
    // Характеристики энкодера:
    PULSES_IN_360_DEGREE_COUNTER_CLOCKWISE_ROTATION = 675,
    PULSES_IN_360_DEGREE_CLOCKWISE_ROTATION = 710,
    PULSES_IN_CM = 9, // в теории (если коэф. сцепления = 1) = 5.79, на практике хорошо будет 8.5
    // Характеристики дальномера:
    RANGEFINDER_ANGLE = 15,
    RANGEFINDER_HALF_ANGLE = 7,
    RANGEFINDER_MAX_DISTANCE = 1000, // 100 см (по datasheet 4 - 4.5 метра)
    // Другие характеристики:
    OBSTACLE_DANGEROUS_DISTANCE = 500,  // 50 см
};

// Сканирование пространства:
enum
{
    MAX_ANGLE_OF_ROTATION_WHEN_MEASURE = 45,    // 45 градусов
    ANGLE_OF_ROTATION_WHEN_MEASURE = 5,         // 5 градусов
    NUMBER_OF_MEASUREMENTS_LEFT = 45/5 + 1,
    NUMBER_OF_MEASUREMENTS_RIGHT = 45/5,
    NUMBER_OF_MEASUREMENTS_ALL = NUMBER_OF_MEASUREMENTS_LEFT + NUMBER_OF_MEASUREMENTS_RIGHT,
    
    RADIUS_OF_OBSTACLE_SEARCH = 500,
    RADUIS_OF_MOVEMENT = 250,
};

typedef struct 
{
    int16_t x;
    int16_t y;
} Target_point;

/************************* GLOBAL AND STATIC VARIABLES ************************/
extern UART_module* debug;
Timer timer; 
Timer timerSub;
static Timer timerForSmoothChangeSpeedDelay;
static Timer timerForSmoothChangeSpeedDeadZone;
Robot_data robot =
{
    .status = ROBOT_INITIALIZED,
    .currentSpeed = 0,
    .speedDifference = 0,
    .x = 0, .y = 0, .angle = 0, .range = 0, 
    
    .minSpeed = ROBOT_START_MIN_SPEED,
    .maxSpeed = ROBOT_START_MAX_SPEED,
    .acceleration = ROBOT_START_ACCELERATION,
    .deceleration = ROBOT_START_DECELERATION
};
static Target_point target = 
{
    .x = 0, .y = 0
};
/************************* GLOBAL AND STATIC VARIABLES ************************/



/****************************** PRIVATE FUNCTION ******************************/
/* 
 * @brief Расчет угла поворота по длинам двух катетов
 * @param dx - катет по горизонтали
 * @param dy - катет по вертикали
 * @return angle - угол в диапазоне (-180; 180] градусов
 */
int16_t calculate_angle(int16_t dx, int16_t dy)
{
    // Избегаем деления на ноль:
    if (dy == 0)
    {
        if (dx > 0)
        {
            return 90;
        }
        else if (dx == 0)
        {
            return 0;
        }
        return -90;
    }
    if (dx == 0)
    {
        if (dy > 0)
        {
            return 0;
        }
        return 180;
    }
    // Основной расчет:
    int16_t angle = atan( abs_double((float)dx/dy) );
    if (dx > 0 && dy < 0)       // 4-ый квадрант
        angle = 180 - angle;
    else if (dx < 0 && dy < 0)  // 3-ий квадрант
        angle -= 180;
    else if (dx < 0 && dy > 0)  // 2-ой квадрант
        angle = -angle;
    else                        // 1-ый квадрант
        angle = angle;

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
 * @brief Определение массива показаний дальномера слева от курсового угла
 * @param указатель на массив расстояний
 */
void measure_left(uint16_t* arrRanges)
{
    // Поворачиваемся в крайнее левое положение
    turn_around_by(-MAX_ANGLE_OF_ROTATION_WHEN_MEASURE);
    
    // Измеряем массив расстояний
    uint8_t countAngle;
    for (countAngle = 0; countAngle < NUMBER_OF_MEASUREMENTS_LEFT; countAngle++)
    {
        turn_around_by(ANGLE_OF_ROTATION_WHEN_MEASURE);
        *(arrRanges++) = getMedianRange();
    }
}

/* 
 * @brief Определение массива показаний дальномера справа от курсового угла
 * @param указатель на массив расстояний
 */
void measure_right(uint16_t* arrRanges)
{
    // Поворачиваемся в крайнее правое положение
    turn_around_by(+MAX_ANGLE_OF_ROTATION_WHEN_MEASURE);
    
    // Запись данных с конца массива
    arrRanges += NUMBER_OF_MEASUREMENTS_RIGHT;
    
    // Измеряем массив расстояний
    uint8_t countAngle;
    for (countAngle = 0; countAngle <= 10; countAngle++)
    {
        turn_around_by(-ANGLE_OF_ROTATION_WHEN_MEASURE);
        *(arrRanges--) = getMedianRange();
    }
}

/* 
 * @brief Вовзвращает 1, если будет обнаружено препятствие, иначе 0
 * @param указатель на массив расстояний
 * @return 1, если обнаружено препятствие, 0, если препятствия нет
 */
uint8_t is_there_obstacle(uint16_t* arrRanges)
{
    uint8_t count;
    for(count = 0; count < NUMBER_OF_MEASUREMENTS_ALL; count++)
    {
        if (arrRanges[count] < RADIUS_OF_OBSTACLE_SEARCH)
            return 1;
    }
    return 0;
}

/* 
 * @brief Проверка на совпадение координат робота и цели
 * @return 1, если координаты совпадают с небольшой погрешностью, иначе 0
 */
uint8_t is_robot_in_target()
{
    enum
    {
        ALLOWABLE_FAULT = 5, // 5 см
    };
    int16_t dx, dy;
    dx = abs_16( target.x - robot.x );
    dy = abs_16( target.y - robot.y );
    
    if(dx <= ALLOWABLE_FAULT && dy <= ALLOWABLE_FAULT)
        return 1;
    return 0;  
}


/* 
 * @brief Плавное увеличение значения текущей скорости робота
 * @return 1, если максимальная скорость достигнута, иначе 0
 */
uint8_t smooth_increase_current_speed()
{
    if (robot.currentSpeed < robot.maxSpeed)
    {
        if( timer_report(&timerForSmoothChangeSpeedDelay) != TIMER_WORKING )
        {
            timer_start_ms(&timerForSmoothChangeSpeedDelay, 100);
            robot.currentSpeed += (robot.acceleration >> 3);
        }
    }
    else
    {
        return 1;
    }
    return 0;
}


/* 
 * @brief Плавное уменьшение значения текущей скорости робота
 */
void smooth_decrease_current_speed()
{
    UART_write_string(debug, "dec\n\r");
    if (robot.currentSpeed > robot.minSpeed)
    {
        if( timer_report(&timerForSmoothChangeSpeedDelay) != TIMER_WORKING )
        {
            timer_start_ms(&timerForSmoothChangeSpeedDelay, 100);
            robot.currentSpeed -= (robot.deceleration >> 3);
        }
    }
}


/* 
 * @brief Плавное изменение значения текущей скорости робота
 * Процесс изменения скорости робота делится на 4 этапа:
 * 1. ROBOT_ACCELERATION_PROCESS
 * Происходит увеличение скорости робота с значения robot.speedMin до robot.speedMax.
 * Этот процесс заканчивается двумя способами:
 * - если робот достигает максимальной скорости => устанавливается статус ROBOT_ACCELERATION_STOP;
 * - если кол-во импульсов переваливает значение половины необходимых.
 * 2. ROBOT_ACCELERATION_STOP
 * На этом этапе скорость робота не меняется и остается максимальной.
 * Этот процесс заканчивается, если кол-во импульсов переваливает значение половины необходимых.
 * 3. ROBOT_DECELERATION_STOP
  * На этом этапе скорость робота не меняется и остается максимальной.
 * Этот процесс длится ровно столько же, сколько длился 2-ой этап.
 * 4. ROBOT_DECELERATION_PROCESS
 * Происходит уменьшение скорости робота до значения robot.speedMin.
 * Процесс заканчивается, когда кол-во импульсов достигает необходимого.
 */
void smooth_change_current_speed(uint32_t nowPulses, uint32_t needPulses)
{
    enum
    {
        ROBOT_ACCELERATION_PROCESS = 0,
        ROBOT_ACCELERATION_STOP = 1,
        ROBOT_DECELERATION_STOP = 2,
        ROBOT_DECELERATION_PROCESS = 3,
    };
    static uint8_t statusOfChange = ROBOT_ACCELERATION_PROCESS;
    
    // Ускорение - первые 2 этапа:
    if(nowPulses < (needPulses >> 1) )
    {
        if( statusOfChange == ROBOT_ACCELERATION_PROCESS)
        {
            if (smooth_increase_current_speed() )
            {
                statusOfChange == ROBOT_ACCELERATION_STOP;
                timer_start_ms(&timerForSmoothChangeSpeedDeadZone, 30000);
            }
        }  
        else if (statusOfChange != ROBOT_ACCELERATION_STOP )
            statusOfChange = ROBOT_ACCELERATION_PROCESS;
    }
    
    // Замедление - последние 2 этапа:
    else
    {
        switch(statusOfChange)
        {
            case ROBOT_ACCELERATION_STOP:
            {
                uint32_t timeOfDeadZone = timer_get_elapsed_time(&timerForSmoothChangeSpeedDeadZone)/1000;
                timer_start_ms(&timerForSmoothChangeSpeedDeadZone, timeOfDeadZone);
                statusOfChange == ROBOT_DECELERATION_STOP;
                {
                    uint8_t buffer[20] = {0};
                    num2str(timeOfDeadZone, buffer); 
                    UART_write_string(debug, "DeadZone:");
                    UART_write_string(debug, buffer);
                }
                break;
            }
            case ROBOT_DECELERATION_STOP:
            {
                if( timer_report(&timerForSmoothChangeSpeedDeadZone) != TIMER_WORKING)
                    statusOfChange == ROBOT_DECELERATION_PROCESS;
                break;
            }
            case ROBOT_DECELERATION_PROCESS:
            {
                smooth_decrease_current_speed();
                break;
            }
            default:
            {
                statusOfChange == ROBOT_DECELERATION_PROCESS;
            }
        }
    }
}

/* 
 * @brief Сохранение прямолинейности движения за счет подстроки скважности ШИМ 
 * двигателей с помощью ПИ регулятора.
 * @note Скорость левого двигателя уменьшается на robot.speedDifference.
 * @note Скорость правого двигателя увеличивается на robot.speedDifference.
 * @note Данная функция только меняет значение robot.speedDifference.
 * @note Стоит быть начеку, чтобы фактическая мощность (скажность) не превысила 100%
 */
void PI_regulator()
{
    // Константы, полученные эмпирическим путем:
    const uint8_t PULSES_HYSTERESIS = 0;
    const float P_REGULATOR = 2;
    const float I_REGULATOR = 1;
    
    // Основной алгоритм:
    int16_t leftPulses = encoder_left_get_pulses();
    int16_t rightPulses = encoder_right_get_pulses();
    static float integralComponent = 0;
    
    if( leftPulses > (rightPulses + PULSES_HYSTERESIS) )
    {
        if(integralComponent < (robot.maxSpeed - robot.minSpeed) )
            integralComponent += I_REGULATOR;
    }
    else if( rightPulses > (leftPulses + PULSES_HYSTERESIS) )
    {
        if(integralComponent > ( (int8_t)robot.minSpeed - robot.maxSpeed) )
            integralComponent -= I_REGULATOR;
        
    }
    robot.speedDifference = (leftPulses - rightPulses)*P_REGULATOR + integralComponent;
}
/****************************** PRIVATE FUNCTION ******************************/


/****************************** PUBLIC FUNCTION *******************************/
/* 
 * @brief Поворот на указанный угол c плавным изменением скорости
 */
void turn_around_by(int16_t angle)
{
    int32_t needPulses;
    encoders_reset_pulses();
    if ( angle > 0) // поворот по часовой
    {
        robot.currentSpeed = robot.minSpeed;
        needPulses = (int32_t)PULSES_IN_360_DEGREE_CLOCKWISE_ROTATION*angle/360;      
        while(encoder_left_get_pulses() < needPulses)
        {
            motor_set_power(robot.currentSpeed, MOTOR_LEFT);
            motor_set_power(-robot.currentSpeed,  MOTOR_RIGHT);
            smooth_change_current_speed(encoder_left_get_pulses(), needPulses);
        }
    }
    else if ( angle <= 0) // поворот против часовой
    {
        robot.currentSpeed = robot.minSpeed;
        needPulses = (int32_t)PULSES_IN_360_DEGREE_COUNTER_CLOCKWISE_ROTATION*(-1)*angle/360; 
        while(encoder_right_get_pulses() < needPulses )
        {
            motor_set_power(-robot.currentSpeed, MOTOR_LEFT);
            motor_set_power(robot.currentSpeed,  MOTOR_RIGHT);
            smooth_change_current_speed(encoder_right_get_pulses(), needPulses);
        }
    }
    else
    {
        return;
    }
    motors_stop();
    robot.currentSpeed = 0;
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
 *
 * @note В цикле выполяются следующие 4 действия, пока среднее кол-во импульсов
 * на энкодерах не будет больше или равно, чем нужно.
 * 1. Пассивная проверка на наличие препятствие, т.е. сравнение показания
 * дальномера с критическим значением при включенном двигателе. Если датчик
 * обнаружит препятствие, то запускается активная проверка.
 * 2. Активная проверка, т.е. полная остановка двигателя с выполнением 3-ех
 * считываний показаний дальномера. Если медианное значение дальности среди
 * считанных значений меньше критического, тогда цикл останавливается, а 
 * координаты робота обновляются фактическими. Иначе - продолжаем цикл.
 * 3. Плавное изменение скорости двигателей от минимальной к максимальной и
 * обратно.
 * 4. Сохранение прямолинейности движения за счет подстроки скважности ШИМ 
 * двигателей с помощью ПИ регулятора.
 
 * @param distance - расстояние (в см)
 */
void move_forward(uint16_t distance)
{
    // Инициализация переменных исходными значениями:
    const int16_t needPulses = PULSES_IN_CM*distance;
    encoders_reset_pulses();
    int16_t nowPulses = 0;
    robot.range = 0;
    robot.currentSpeed = robot.minSpeed;
    
    // Основной цикл:
    while(nowPulses < needPulses)
    {
        // 1. Пассивная проверка на наличие препятствия (не влияет на движение робота):
        if (timer_report(&timer) != TIMER_WORKING)
        {
            rangefinder_give_impulse();
            timer_start_ms(&timer, 100);
            robot.range = rangefinder_get_range();
        }
        // 2. Активная проверка на наличие препятствия (робот остановливается):
        if (robot.range < OBSTACLE_DANGEROUS_DISTANCE)
        {
            /*
            motors_stop();
            if (getMedianRange() < OBSTACLE_DANGEROUS_DISTANCE)
            {
                distance = nowPulses/PULSES_IN_CM;
                break;
            }
            */
        }
        // 3. Плавное изменение скорости двигателей
        smooth_change_current_speed(nowPulses, needPulses);
        
        // 4. Сохранение прямолинейности движения с помощью П-регулятора:
        PI_regulator();
        
        // Обновление мощности двигателей, при этом мощность должна быть в интервале [0; maxSpeed]:
        int8_t actualSpeed = robot.currentSpeed - robot.speedDifference;
        if(actualSpeed > robot.maxSpeed)
            actualSpeed = robot.maxSpeed;
        else if(actualSpeed < 0)
            actualSpeed = 0;    
        motor_set_power(actualSpeed,  MOTOR_LEFT);
        
        actualSpeed = robot.currentSpeed + robot.speedDifference;
        if(actualSpeed > robot.maxSpeed)
            actualSpeed = robot.maxSpeed;
        else if(actualSpeed < 0)
            actualSpeed = 0; 
        motor_set_power(actualSpeed, MOTOR_RIGHT);
        
        // Обновление текущего кол-ва импульсов:
        nowPulses = ( encoder_left_get_pulses() + encoder_right_get_pulses() ) >> 1;
    }
    robot.currentSpeed = 0;
    motors_stop();
    robot.x += sin(robot.angle)*distance;
    robot.y += cos(robot.angle)*distance;
}

/* 
 * @brief Прямолинейное движение к указанной координате
 * @param x, y - декартовы координаты
 */
void move_to(int16_t x, int16_t y)
{
    target.x = x;
    target.y = y;
    if( !is_robot_in_target() )
    {
        int16_t dx = (x - robot.x);
        int16_t dy = (y - robot.y);
        uint16_t distance = calculate_distance(dx, dy);
        int16_t angle = calculate_angle(dx, dy);
        turn_around_to(angle);
        move_forward(distance);
    }
     
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
    soft_timer_init(&timerForSmoothChangeSpeedDelay);
    soft_timer_init(&timerForSmoothChangeSpeedDeadZone);
    rangefinder_init();
    
    UART_write_string(debug, "Start\n\r");
}

/* 
 * @brief Задать координаты для движения без столкновения с препятствиями
 */
void move_with_obstacle_avoidance_get_coordinates(int16_t x, int16_t y)
{
    target.x = x;
    target.y = y;
    robot.status = ROBOT_INITIALIZED;
}

/* 
 * @brief Движение без столкновения с препятствиями
 */
void move_with_obstacle_avoidance_do()
{
    enum Step
    {
        STEP_1_ROTATE = 1,
        STEP_2_IS_ROBOT_IN_TARGET = 2,
        STEP_3_MEASURE_LEFT = 3,
        STEP_4_MEASURE_RIGHT = 4,
        STEP_5_IS_THERE_OBSTACLE = 5,
        STEP_6_ROTATE = 6,
        STEP_7_MEASURE_LEFT = 7,
        STEP_8_MEASURE_RIGHT = 8,
        STEP_9_IS_THERE_OBSTACLE = 9,
        STEP_10_MOVE_FORWARD = 10,
        STEP_11_ROTATE = 11,
        STEP_12_MOVE_FORWARD = 12,
        STEP_13_CANT_MOVE = 13,
    };
    static uint8_t stepCount = STEP_1_ROTATE;
    static uint16_t arrRanges[NUMBER_OF_MEASUREMENTS_ALL];
    
    switch(stepCount)
    {
        case STEP_1_ROTATE:
        {
            turn_around_to(calculate_angle(target.x - robot.x, target.y - robot.y));
            stepCount++;
            break;
        }
        case STEP_2_IS_ROBOT_IN_TARGET:
        {
            if( is_robot_in_target() )
            {
                robot.status = ROBOT_FINISHED;
            }
            else
            {
                stepCount++;
                robot.status = ROBOT_IN_PROGRESS;
            }
            break;
        }
        case STEP_3_MEASURE_LEFT:
        {
            measure_left(arrRanges);
            stepCount++;
            break;
        }
        case STEP_4_MEASURE_RIGHT:
        {
            measure_right(arrRanges);
            stepCount++;
            break;
        }
        case STEP_5_IS_THERE_OBSTACLE:
        {
            if ( is_there_obstacle(arrRanges) )
                stepCount++;
            else
                stepCount = STEP_10_MOVE_FORWARD;
            break;
        }
        case STEP_6_ROTATE:
        {
            turn_around_by(-90);
            stepCount++;
            break;
        }
        case STEP_7_MEASURE_LEFT:
        {
            measure_left(arrRanges);
            stepCount++;
            break;
        }
        case STEP_8_MEASURE_RIGHT:
        {
            measure_right(arrRanges);
            stepCount++;
            break;
        }
        case STEP_9_IS_THERE_OBSTACLE:
        {
            if ( is_there_obstacle(arrRanges) )
                stepCount++;
            else
                stepCount = STEP_10_MOVE_FORWARD;
            break;
        }
        case STEP_10_MOVE_FORWARD:
        {
            move_forward(RADUIS_OF_MOVEMENT);
            stepCount++;
            break;
        }
        case STEP_11_ROTATE:
        {
            turn_around_by(-90);
            stepCount = STEP_3_MEASURE_LEFT;
            break;
        }
        case STEP_12_MOVE_FORWARD:
        {
            move_forward(RADUIS_OF_MOVEMENT);
            stepCount = STEP_2_IS_ROBOT_IN_TARGET;
            break;
        }
        case STEP_13_CANT_MOVE:
        {
            robot.status = ROBOT_CANT_MOVE;
            break;
        }
        default:
        {
            break;
        }
    }
}

/* 
 * @brief Передача по uart информацию о роботе (первые 10 байт структуры robot)
 */
void log_transmit()
{
    //uint8_t arrOfData[10];
    //memcpy(arrOfData, &robot, 10);
    //UART_transmit(debug, arrOfData, 10);
    //UART_transmit(debug, "\n\r", 2);
    
    char buf[12];
    UART_write_string(debug, "\nlog:");
    
    //num2str(robot.status, buf);
    //UART_write_string(debug, "\r\nstatus: ");
    //UART_write_string(debug, buf);
    
    num2str(robot.x, buf);
    UART_write_string(debug, "\nrobot.x: ");
    UART_write_string(debug, buf);
    
    num2str(robot.y, buf);
    UART_write_string(debug, "\nrobot.y: ");
    UART_write_string(debug, buf);
    
    //num2str(target.x, buf);
    //UART_write_string(debug, "\r\ntarget.x: ");
    //UART_write_string(debug, buf);
    
    //num2str(target.y, buf);
    //UART_write_string(debug, "\r\ntarget.y: ");
    //UART_write_string(debug, buf);
    
    num2str(robot.angle, buf);
    UART_write_string(debug, "\nrobot.angle: ");
    UART_write_string(debug, buf);
    
    UART_write_string(debug, "\r\n");
}
/****************************** PUBLIC FUNCTION *******************************/

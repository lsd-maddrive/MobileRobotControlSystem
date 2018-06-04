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
    ROBOT_START_MIN_SPEED = 40,     // Исходное значение минимальной скорости см/сек
    ROBOT_START_MAX_SPEED = 60,     // Исходное значение максимальной скорости см/сек
    ROBOT_START_ACCELERATION = 8,   // Исходное значение ускорения см/сек^2
    ROBOT_START_DECELERATION = 8,   // Исходное значение замедления см/сек^2
};


typedef enum 
{
    MOVE_FORWARD = 0,
    ROTATE_CLOCKWISE = 1,
    ROTATE_COUNTER_CLOCKWISE = 2,
}Movement_t;


typedef enum 
{
    SECTOR_CLEAR = 0,
    OBSTACLE_TO_THE_LEFT = 1,
    OBSTACLE_TO_THE_RIGHT = 2,
}Result_of_scan_t;


enum Calibration
{
    // Характеристики робота:
    ROBOT_WIDTH = 28,           // 28 см
    ROBOT_LENGTH = 28,          // 28 см
    ROBOT_RADIUS = 20,          // 20 см
    ROBOT_SAFETY_CORRIDOR = 30, // 30 см
    ROBOT_HALF_WIDTH = ROBOT_WIDTH >> 1,
    ROBOT_HALF_LENGTH = ROBOT_LENGTH >> 1,
    // Характеристики энкодера:
    PULSES_IN_360_DEGREE_COUNTER_CLOCKWISE_ROTATION = 735,
    PULSES_IN_360_DEGREE_CLOCKWISE_ROTATION = 730,
    PULSES_IN_CM = 9, // в теории (если коэф. сцепления = 1) = 5.79, на практике хорошо будет 8.5
    // Характеристики дальномера:
    RANGEFINDER_ANGLE = 15,
    RANGEFINDER_HALF_ANGLE = 7,
    RANGEFINDER_MAX_DISTANCE = 1000, // 100 см (по datasheet 4 - 4.5 метра)
    // Другие характеристики:
    OBSTACLE_DANGEROUS_DISTANCE = 50,  // 50 см
};

// Сканирование пространства:
enum
{
    MAX_ANGLE_OF_ROTATION_WHEN_MEASURE = 45,    // 45 градусов
    ANGLE_OF_ROTATION_WHEN_MEASURE = 5,         // 5 градусов
    NUMBER_OF_ROTATIONS_LEFT = 9,               // 45/5
    NUMBER_OF_ROTATIONS_RIGHT = 9,              // 45/5
    NUMBER_OF_MEASUREMENTS_LEFT = 10,           // [-MAX : ITER : 0] = 45/5 + 1
    NUMBER_OF_MEASUREMENTS_RIGHT = 10,          // [0 : ITER : +MAX] = 45/5 + 1
    
    RADIUS_OF_OBSTACLE_SEARCH = 40,             // 60 см
    RADIUS_OF_MOVEMENT = 20,                    // 40 см
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
static uint16_t arrRangesLeft[NUMBER_OF_MEASUREMENTS_LEFT];
static uint16_t arrRangesRight[NUMBER_OF_MEASUREMENTS_RIGHT];
Robot_data robot =
{
    .status = ROBOT_INITIALIZED,
    .currentSpeed = 0,
    .speedRegulator = 0,
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
static uint8_t distanceToSafetyCorridor[NUMBER_OF_MEASUREMENTS_LEFT] = 
{29, 33, 39, 46, 57, 74, 102, 159, 255, 255};
/************************* GLOBAL AND STATIC VARIABLES ************************/



/****************************** PRIVATE FUNCTION ******************************/
/* 
 * @brief Расчет угла поворота по длинам двух катетов
 * @param dx - катет по горизонтали
 * @param dy - катет по вертикали
 * @return angle - угол в диапазоне [-180; +180] градусов
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
    uint16_t range_1 = rangefinder_do();
    uint16_t range_2 = rangefinder_do();
    uint16_t range_3 = rangefinder_do();
    
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
 * @brief Определение массива показаний дальномера слева на 45 градусов от курсового угла
 * @note Обратно не поворачиваем
 */
void measure_left()
{
    uint16_t* ptrArr = arrRangesLeft;
    uint8_t countOfRotation;
    *(ptrArr++) = getMedianRange();
    for (countOfRotation = 0; countOfRotation < NUMBER_OF_ROTATIONS_LEFT; countOfRotation++)
    {
        turn_around_by(-ANGLE_OF_ROTATION_WHEN_MEASURE);
        *(ptrArr++) = getMedianRange();
    }
}


/* 
 * @brief Определение массива показаний дальномера справа на 45 градусов от курсового угла
 * @note Обратно не поворачиваем
 */
void measure_right()
{
    uint16_t* ptrArr = arrRangesRight;
    uint8_t countOfRotation;
    *(ptrArr++) = getMedianRange();
    for (countOfRotation = 0; countOfRotation < NUMBER_OF_ROTATIONS_RIGHT; countOfRotation++)
    {
        turn_around_by(ANGLE_OF_ROTATION_WHEN_MEASURE);
        *(ptrArr++) = getMedianRange();
    }
}


/* 
 * @brief Определение массива показаний дальномера справа и слева от курсового угла
 * @note С поворотом обратно
 */
void measure()
{
    measure_left();
    turn_around_by(MAX_ANGLE_OF_ROTATION_WHEN_MEASURE);
    measure_right();
    turn_around_by(-MAX_ANGLE_OF_ROTATION_WHEN_MEASURE);
}

/* 
 * @brief Анализ массива расстояний, полученных от дальномера
 * @return OBSTACLE_TO_THE_LEFT или OBSTACLE_TO_THE_RIGHT, если обнаружено препятствие, 
   иначе SECTOR_CLEAR
 */
Result_of_scan_t where_is_there_obstacle()
{
    int8_t count;
    int16_t* ptrArrLeft = arrRangesLeft + NUMBER_OF_MEASUREMENTS_LEFT;
    int16_t* ptrArrRight = arrRangesRight + NUMBER_OF_MEASUREMENTS_RIGHT;
    int8_t maxCount = 0;
    
    for(count = NUMBER_OF_MEASUREMENTS_LEFT; count > 0; count--)
    {
        if( ( *(--ptrArrLeft) < distanceToSafetyCorridor[count] ) &&
            ( *ptrArrLeft < RADIUS_OF_OBSTACLE_SEARCH ) )
        {
            maxCount = -count;
            break;
        }
    }
    for(count = NUMBER_OF_MEASUREMENTS_RIGHT; count > maxCount; count--)
    {
        if( ( *(--ptrArrRight) < distanceToSafetyCorridor[count] ) &&
            ( *ptrArrRight < RADIUS_OF_OBSTACLE_SEARCH ) )
        {
            maxCount = count;
            break;
        }
    }
    
    if(count < 0)
        return OBSTACLE_TO_THE_LEFT;
    else if (count > 0)
        return OBSTACLE_TO_THE_RIGHT; 
    return SECTOR_CLEAR;
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
    
    if( (dx <= ALLOWABLE_FAULT) && (dy <= ALLOWABLE_FAULT) )
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
                statusOfChange = ROBOT_DECELERATION_PROCESS;
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
    int16_t leftPulses = abs_16( encoder_left_get_pulses() );
    int16_t rightPulses = abs_16( encoder_right_get_pulses() );
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
    robot.speedRegulator = (leftPulses - rightPulses)*P_REGULATOR + integralComponent;
}


/* 
 * @brief Обновление скорости робота с учетом влияния ПИ-регулятора и плавного
 * изменения скорости.
 * @note скорость робота в любом случае остается в интервале [0; maxSpeed]
 */
void update_robot_speed(Movement_t type)
{
    int8_t actualSpeed = robot.currentSpeed - robot.speedRegulator;
    if(actualSpeed > robot.maxSpeed)
        actualSpeed = robot.maxSpeed;
    else if(actualSpeed < 0)
        actualSpeed = 0;   
    if ( (type == MOVE_FORWARD) || (type == ROTATE_CLOCKWISE) )
        motor_set_power(actualSpeed,  MOTOR_LEFT);
    else
        motor_set_power(-actualSpeed,  MOTOR_LEFT);
    
    actualSpeed = robot.currentSpeed + robot.speedRegulator;
    if(actualSpeed > robot.maxSpeed)
        actualSpeed = robot.maxSpeed;
    else if(actualSpeed < 0)
        actualSpeed = 0; 
    if ( (type == MOVE_FORWARD) || (type == ROTATE_COUNTER_CLOCKWISE) )
        motor_set_power(actualSpeed, MOTOR_RIGHT);
    else
        motor_set_power(-actualSpeed, MOTOR_RIGHT);
}


/* 
 * @brief Пассивная проверка на наличие препятствия
 * @note Не влияет на движение робота
 */
void passive_obstacle_check()
{
    if (timer_report(&timer) != TIMER_WORKING)
    {
        rangefinder_give_impulse();
        timer_start_ms(&timer, 100);
        robot.range = rangefinder_get_range();
    }
}


/* 
 * @brief Активная проверка на наличие препятствия
 * @note робот останавливается во время проверки
 * @param ptrDistance - указатель на расстояние, необходимое проехать
 */
void active_obstacle_check(uint16_t* ptrDistance)
{
    if (robot.range < OBSTACLE_DANGEROUS_DISTANCE)
    {
        /*
        motors_stop();
        if (getMedianRange() < OBSTACLE_DANGEROUS_DISTANCE)
        {
            *distance = nowPulses/PULSES_IN_CM;
            break;
        }
        */
    }
}
/****************************** PRIVATE FUNCTION ******************************/


/****************************** PUBLIC FUNCTION *******************************/
/* 
 * @brief Поворот на указанный угол c плавным изменением скорости
 * @param angle [-30к; + 30к]
 */
void turn_around_by(int16_t angle)
{
    int16_t nowPulses = 0;
    int32_t needPulses;
    encoders_reset_pulses();
    robot.currentSpeed = robot.minSpeed;
    if ( angle > 0) // поворот по часовой
    {
        needPulses = (int32_t)PULSES_IN_360_DEGREE_CLOCKWISE_ROTATION*angle/360;  
        while(nowPulses < needPulses)
        {
            update_robot_speed(ROTATE_CLOCKWISE);
            smooth_change_current_speed(nowPulses, needPulses);
            PI_regulator();
            nowPulses = ( encoder_left_get_pulses() - encoder_right_get_pulses() ) >> 1;
        }
    }
    else if ( angle <= 0) // поворот против часовой
    {
        needPulses = (int32_t)PULSES_IN_360_DEGREE_COUNTER_CLOCKWISE_ROTATION*(-1)*angle/360; 
        while(nowPulses < needPulses )
        {
            update_robot_speed(ROTATE_COUNTER_CLOCKWISE);
            smooth_change_current_speed(nowPulses, needPulses);
            PI_regulator();
            nowPulses = ( encoder_right_get_pulses() - encoder_left_get_pulses() ) >> 1;
        }
    }
    else
    {
        return;
    }
    motors_stop();
    robot.currentSpeed = 0;
    robot.angle += angle;
}

/* 
 * @brief Поворот к указанному углу по кратчайшему направлению
 * @param angle - курсовой угол, который должен быть у робота
 */
void turn_around_to(int16_t angle)
{
    int16_t short_angle = angle - robot.angle;
    while( abs(short_angle) > 180)
    {
        if (short_angle > 180)
            short_angle -= 360;
        else if (short_angle < -180)
            short_angle += 360;
    }
    
    turn_around_by(short_angle);
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
    const int16_t needPulses = PULSES_IN_CM*distance;
    encoders_reset_pulses();
    int16_t nowPulses = 0;
    robot.range = 0;
    robot.currentSpeed = robot.minSpeed;
    
    while(nowPulses < needPulses)
    {
        passive_obstacle_check();
        active_obstacle_check(&distance);
        smooth_change_current_speed(nowPulses, needPulses);
        PI_regulator();
        update_robot_speed(MOVE_FORWARD);
        nowPulses = ( encoder_left_get_pulses() + encoder_right_get_pulses() ) >> 1;
    }
    
    robot.currentSpeed = 0;
    robot.y += cos(robot.angle)*distance;
    robot.x += sin(robot.angle)*distance;
    motors_stop();
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
        turn_around_to(calculate_angle(dx, dy));
        move_forward(calculate_distance(dx, dy));
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
 * @brief Движение без столкновения с препятствиями
 */
void move_with_obstacle_avoidance(int16_t x, int16_t y)
{
    target.x = x;
    target.y = y;
    robot.status = ROBOT_INITIALIZED;
    while ( !is_robot_in_target() )
    {
        int16_t dx = target.x - robot.x;
        int16_t dy = target.y - robot.y;
        uint16_t distance = calculate_distance(dx, dy);
        turn_around_to(calculate_angle(dx, dy));
        
        measure_right();
        turn_around_by(-MAX_ANGLE_OF_ROTATION_WHEN_MEASURE);
        measure_left();
        log_transmit();
        
        if( where_is_there_obstacle() == OBSTACLE_TO_THE_LEFT )
        {
            turn_around_by(+135);

            test_measure(); //measure();
            
            if( where_is_there_obstacle() != SECTOR_CLEAR)
                break;
            else
            {
                move_forward(RADIUS_OF_MOVEMENT << 1);
                turn_around_by(-90);
                move_forward(RADIUS_OF_MOVEMENT << 1);
            }
        }
        else if( where_is_there_obstacle() == OBSTACLE_TO_THE_RIGHT )
        {
            turn_around_by(-45);

            test_measure(); //measure();
            
            if( where_is_there_obstacle() != SECTOR_CLEAR)
                break;
            else
            {
                move_forward(RADIUS_OF_MOVEMENT << 1);
                turn_around_by(+90);
                move_forward(RADIUS_OF_MOVEMENT << 1);
            }
        }
        else
        {
            turn_around_by(45);
            if (distance < RADIUS_OF_MOVEMENT)
                move_forward(distance);
            else
                move_forward(RADIUS_OF_MOVEMENT);
        }
    }
}

/* 
 * @brief Передача по uart информацию о роботе (первые 10 байт структуры robot)
 */
void log_transmit()
{
    char buf[12];
    UART_write_string(debug, "\nlog:");
    
    int8_t count;
    for(count = NUMBER_OF_MEASUREMENTS_LEFT-1; count >= 0; count--)
    {
        num2str(arrRangesLeft[count], buf);
        UART_write_string(debug, " ");
        UART_write_string(debug, buf);
    }
    for(count = 0; count < NUMBER_OF_MEASUREMENTS_RIGHT; count++)
    {
        num2str(arrRangesRight[count], buf);
        UART_write_string(debug, " ");
        UART_write_string(debug, buf);
    }
    
}
/****************************** PUBLIC FUNCTION *******************************/

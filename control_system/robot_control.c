/** 
* @file robot_control.c
* @brief Implementation of robot control system
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
#include "pid_regulator.h"
#include "smooth_change_speed.h"
#include "adc.h"
#include "math.h"
#include "text.h"
#include "robot_test.h"
#include "string.h" // для memcpy

/// Исходные данные
enum Initial_data
{
	ROBOT_START_MIN_SPEED = 30,		///< Исходное значение минимальной скважности, duty_cycle
	ROBOT_START_MAX_SPEED = 70,		///< Исходное значение максимальной скважности, duty_cycle
};


/// Направление движения
typedef enum 
{
    MOVE_FORWARD = 0,
    ROTATE_CLOCKWISE = 1,
    ROTATE_COUNTER_CLOCKWISE = 2,
}Movement_t;


/// Результаты сканирования пространства
typedef enum 
{
    SECTOR_CLEAR = 0,
    OBSTACLE_TO_THE_LEFT = 1,
    OBSTACLE_TO_THE_RIGHT = 2,
}Result_of_scan_t;


/// Калибровочные константы
enum Calibration
{
	/// Характеристики робота (ширина 28, длина 28):
    ROBOT_INTERNAL_RADIUS = 14,
    ROBOT_EXTERNAL_RADIUS = 20,
    ROBOT_SAFETY_CORRIDOR_RADIUS = 30,
    ROBOT_INTERNAL_DIAMETER = 28,
    ROBOT_EXTERNAL_DIAMETER = 40,
    ROBOT_SAFETY_CORRIDOR_DIAMETER = 60,
	/// Характеристики энкодера:
    PULSES_IN_360_DEGREE_COUNTER_CLOCKWISE_ROTATION = 720,
    PULSES_IN_360_DEGREE_CLOCKWISE_ROTATION = 720,
    PULSES_IN_CM = 9, // в теории (если коэф. сцепления = 1) = 5.79, на практике хорошо будет 8.5
	/// Характеристики дальномера:
	RANGEFINDER_ANGLE = 15,
	RANGEFINDER_HALF_ANGLE = 7,
	RANGEFINDER_MAX_DISTANCE = 1000,    ///< 100 см (по datasheet 4 - 4.5 метра)
	/// Другие характеристики:
	OBSTACLE_DANGEROUS_DISTANCE = 50,   ///< 50 см
};

/// Сканирование пространства
enum
{
    MAX_ANGLE_OF_ROTATION_WHEN_MEASURE = 45,    ///< 45 градусов
    ANGLE_OF_ROTATION_WHEN_MEASURE = 5,         ///< 5 градусов
    NUMBER_OF_ROTATIONS = 9,                    ///< 45/5
    NUMBER_OF_ROTATIONS_LEFT = 9,               ///< 45/5
    NUMBER_OF_ROTATIONS_RIGHT = 9,              ///< 45/5
    NUMBER_OF_MEASUREMENTS_LEFT = 10,           ///< [-MAX : ITER : 0] = 45/5 + 1
    NUMBER_OF_MEASUREMENTS_RIGHT = 10,          ///< [0 : ITER : +MAX] = 45/5 + 1
    
    RADIUS_OF_OBSTACLE_SEARCH = 40,             ///< 60 см
    RADIUS_OF_MOVEMENT = 20,                    ///< 40 см
};


/// Декартовы координаты цели
typedef struct 
{
    int16_t x;
    int16_t y;
} Target_point;


/// Информация о препятствии
typedef struct 
{
    int16_t BorderLeft;
    int16_t BorderRight;
    uint8_t Distance;
} Obstacle;

/************************* GLOBAL AND STATIC VARIABLES ************************/
extern UART_module* debug;
Timer timer; 
Timer timerSub;
static Timer timerForTest;
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
};
static Target_point target = 
{
    .x = 0, .y = 0
};
static Obstacle obstacle =
{
    .BorderLeft = 0, .BorderRight = 0, .Distance = 0,
};
static uint8_t distanceToSafetyCorridor[NUMBER_OF_MEASUREMENTS_LEFT] = 
{
    29, 33, 39, 46, 57, 74, 102, 159, 255, 255
};
/************************* GLOBAL AND STATIC VARIABLES ************************/



/****************************** PRIVATE FUNCTION ******************************/
/** 
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


/** 
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


/** 
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


/** 
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


/** 
* @brief Определение, было ли обнаружено при сканировании препятствие
* @param pDist - указатель на массив расстояний
* @param pSafety - указатель на массив максимальных расстояний, при которых считается,
что препятствия нет
* @return 1, если препятствие обнаружено, иначе 0
*/
uint8_t is_it_obstacle(uint16_t* pDist, uint8_t* pSafety)
{
    return ( ( *pDist < *pSafety ) && ( *pDist < RADIUS_OF_OBSTACLE_SEARCH ) );
}


/** 
* @brief Анализ массива расстояний, полученных от дальномера
* @return OBSTACLE_TO_THE_LEFT или OBSTACLE_TO_THE_RIGHT, если обнаружено препятствие, 
иначе SECTOR_CLEAR
*/
Result_of_scan_t where_is_obstacle()
{
    enum
    {
        ZERO_OBSTACLES = 0,
        ONE_OBSTACLE = 1,
        SEVERAL_OBSTACLES = 2,
        SOME_BIG_VALUE = 50,
    };
    int8_t angle;
    uint16_t* ptrArrLeft = arrRangesLeft + NUMBER_OF_MEASUREMENTS_LEFT;
    uint16_t* ptrArrRight = arrRangesRight;
    uint8_t* ptrDistance = distanceToSafetyCorridor;
    uint8_t numberOfObstacles = 0;
    obstacle.BorderLeft = SOME_BIG_VALUE;
    obstacle.BorderRight = -SOME_BIG_VALUE;

    for(angle = -MAX_ANGLE_OF_ROTATION_WHEN_MEASURE; angle <= 0; angle+=5)
    {
        if( is_it_obstacle(--ptrArrLeft, ptrDistance ) && (numberOfObstacles == ZERO_OBSTACLES) )
        {
            numberOfObstacles = ONE_OBSTACLE;
            obstacle.BorderLeft = angle;
        }
        else if( !is_it_obstacle(ptrArrLeft, ptrDistance ) && (numberOfObstacles == ONE_OBSTACLE) )
        {
            numberOfObstacles = SEVERAL_OBSTACLES;
            obstacle.BorderRight = angle;
        }
        else if( is_it_obstacle(ptrArrLeft, ptrDistance ) && (numberOfObstacles == SEVERAL_OBSTACLES) )
        {
            obstacle.BorderRight = angle;
        }
        ptrDistance++;
    }
    ptrDistance = distanceToSafetyCorridor + NUMBER_OF_ROTATIONS;
    for(angle = 0; angle < MAX_ANGLE_OF_ROTATION_WHEN_MEASURE; angle+=5)
    {
        if( is_it_obstacle(ptrArrRight, --ptrDistance ) && (numberOfObstacles == ZERO_OBSTACLES) )
        {
            numberOfObstacles = ONE_OBSTACLE;
            obstacle.BorderLeft = angle;
        }
        else if( !is_it_obstacle(ptrArrRight, ptrDistance ) && (numberOfObstacles == ONE_OBSTACLE) )
        {
            numberOfObstacles = SEVERAL_OBSTACLES;
            obstacle.BorderRight = angle;
        }
        else if( is_it_obstacle(ptrArrLeft, ptrDistance ) && (numberOfObstacles == SEVERAL_OBSTACLES) )
        {
            obstacle.BorderRight = angle;
        }
        ptrArrRight++;
    }

    //obstacle.Distance = sqrt();

    if( obstacle.BorderLeft <= 0 )
        obstacle.BorderLeft = (arrRangesLeft[(uint8_t)(obstacle.BorderLeft*(-0.2))])*Sin(obstacle.BorderLeft);
    else
        obstacle.BorderLeft = (arrRangesLeft[(uint8_t)(obstacle.BorderRight*(0.2))])*Sin(obstacle.BorderLeft);

    if( obstacle.BorderRight >= 0 )
        obstacle.BorderRight = (arrRangesLeft[9-(uint8_t)(obstacle.BorderRight*0.2)])*Sin(obstacle.BorderRight);
    else
        obstacle.BorderRight = (arrRangesLeft[(uint8_t)(obstacle.BorderLeft*(-0.2))])*Sin(obstacle.BorderRight);


    if( abs(obstacle.BorderLeft) > abs(obstacle.BorderRight) )
        return OBSTACLE_TO_THE_LEFT;
    else if ( abs(obstacle.BorderLeft) < abs(obstacle.BorderRight) )
        return OBSTACLE_TO_THE_RIGHT;
    return SECTOR_CLEAR;
}


/**
* @brief Проверка на совпадение координат робота и цели
* @return 1, если координаты совпадают с небольшой погрешностью, иначе 0
*/
uint8_t is_robot_in_target()
{
    enum
    {
        ALLOWABLE_FAULT = 5, ///< 5 см
    };
    int16_t dx, dy;
    dx = Abs_16( target.x - robot.x );
    dy = Abs_16( target.y - robot.y );
    
    if( (dx <= ALLOWABLE_FAULT) && (dy <= ALLOWABLE_FAULT) )
        return 1;
    return 0;  
}


/** 
* @brief Обновление скорости робота с учетом влияния ПИ-регулятора и плавного
* изменения скорости.
* @note скорость робота в любом случае остается в интервале [0; maxSpeed]
*/
void update_robot_speed(Movement_t type)
{
    //#define TEST_MODE       // activate timer
    //#define TEST_ENCODER    // with timer
    //#define TEST_PWM        // with timer

    #ifdef TEST_MODE
    if( timer_report(&timerForTest) != TIMER_WORKING )
    {
        timer_start_ms(&timerForTest, 50);
        #ifdef TEST_ENCODER
        {
            char buffer[12];
            num2str(encoder_left_get_pulses(), buffer); 
            UART_write_string(debug, buffer);
            UART_write_string(debug, " ");
        }
        {
            char buffer[12];
            num2str(encoder_right_get_pulses(), buffer); 
            UART_write_string(debug, buffer);
            UART_write_string(debug, " ");
        }
        #endif
        #ifdef TEST_PWM
        {
            char buffer[12];
            num2str(robot.currentSpeed, buffer); 
            UART_write_string(debug, buffer);
            UART_write_string(debug, " ");
        }
        #endif
        UART_write_string(debug, "; ");
    } 
    #endif  // TEST_MODE

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


/** 
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


/** 
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
            smooth_change_current_speed(&robot.currentSpeed, nowPulses, needPulses);
            robot.speedRegulator = PI_regulator();
            nowPulses = ( encoder_left_get_pulses() - encoder_right_get_pulses() ) >> 1;
        }
    }
    else if ( angle <= 0) // поворот против часовой
    {
        needPulses = (int32_t)PULSES_IN_360_DEGREE_COUNTER_CLOCKWISE_ROTATION*(-1)*angle/360; 
        while(nowPulses < needPulses )
        {
            update_robot_speed(ROTATE_COUNTER_CLOCKWISE);
            smooth_change_current_speed(&robot.currentSpeed, nowPulses, needPulses);
            robot.speedRegulator = PI_regulator();
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

/**
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
*
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
        smooth_change_current_speed(&robot.currentSpeed, nowPulses, needPulses);
        //robot.speedRegulator = PI_regulator();
        update_robot_speed(MOVE_FORWARD);
        nowPulses = ( encoder_left_get_pulses() + encoder_right_get_pulses() ) >> 1;
        test_adc();
    }
    
    robot.currentSpeed = 0;
    robot.y += Cos(robot.angle)*distance;
    robot.x += Sin(robot.angle)*distance;
    motors_stop();
}

/** 
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
        turn_around_to(Calculate_angle(dx, dy));
        move_forward(Calculate_distance(dx, dy));
    }
}

/** 
* @brief Инициализация всей переферии
*/
void init_periphery() 
{
    GPIO_init();
    adc_init(16);
    motor_init();
    encoders_init();
    debug = UART_init(UART_1, UART_BAUD_RATE_9600);
    hard_timer_init();
    soft_timer_init(&timer);
    soft_timer_init(&timerForTest);
    soft_timer_init(&timerSub);
    smooth_change_current_speed_init();
    rangefinder_init();
    
    UART_write_string(debug, "Start\n\r");
}


/** 
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
        uint16_t distance = Calculate_distance(dx, dy);
        turn_around_to(Calculate_angle(dx, dy));
        
        measure_right();
        turn_around_by(-MAX_ANGLE_OF_ROTATION_WHEN_MEASURE);
        measure_left();
        log_transmit();
        
        /// Объезжаем препятствие справа
        if( where_is_obstacle() == OBSTACLE_TO_THE_LEFT )
        {
            //uint16_t borderLeft = obstacle.BorderLeft;
            uint16_t borderRight = obstacle.BorderRight;
            turn_around_by(+135);

            test_measure(); //measure();
            
            if( where_is_obstacle() != SECTOR_CLEAR)
                break;
            else
            {
                
                char buf[12] = {};
                num2str(borderRight, buf);
                UART_write_string(debug, " %");
                UART_write_string(debug, buf);
                UART_write_string(debug, "% ");
                
                move_forward(ROBOT_SAFETY_CORRIDOR_RADIUS + borderRight);
                turn_around_by(-90);
                move_forward(RADIUS_OF_MOVEMENT << 1);
            }
        }
        /// Объезжаем препятствие слева
        else if( where_is_obstacle() == OBSTACLE_TO_THE_RIGHT )
        {
            uint16_t borderLeft = obstacle.BorderLeft;
            //uint16_t borderRight = obstacle.BorderRight;
            turn_around_by(-45);

            test_measure(); //measure();
            
            if( where_is_obstacle() != SECTOR_CLEAR)
                break;
            else
            {
                char buf[12] = {};
                num2str(borderLeft, buf);
                UART_write_string(debug, " %");
                UART_write_string(debug, buf);
                UART_write_string(debug, "% ");
                
                move_forward(ROBOT_SAFETY_CORRIDOR_RADIUS - borderLeft);
                turn_around_by(+90);
                move_forward(RADIUS_OF_MOVEMENT << 1);
            }
        }
        /// Едем прямо
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

/** 
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

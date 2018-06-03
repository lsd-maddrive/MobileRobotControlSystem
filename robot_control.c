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
    ROBOT_START_ACCELERATION = 8,   // Исходное значение ускорения см/сек^2
    ROBOT_START_DECELERATION = 8,   // Исходное значение замедления см/сек^2
};

enum Calibration
{
    // Характеристики робота:
    ROBOT_WIDTH = 300,  // 30 см
    ROBOT_LENGTH = 400,  // 40 см
    ROBOT_HALF_WIDTH = ROBOT_WIDTH >> 1,
    ROBOT_HALF_LENGTH = ROBOT_LENGTH >> 1,
    // Характеристики энкодера:
    PULSES_IN_360_DEGREE_COUNTER_CLOCKWISE_ROTATION = 690,
    PULSES_IN_360_DEGREE_CLOCKWISE_ROTATION = 690,
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
    NUMBER_OF_MEASUREMENTS_LEFT = 45/5 + 1,     // [MAX:ITER:0]
    NUMBER_OF_MEASUREMENTS_RIGHT = 45/5,        // [MAX:ITER:ITER]
    NUMBER_OF_MEASUREMENTS_ALL = NUMBER_OF_MEASUREMENTS_LEFT + NUMBER_OF_MEASUREMENTS_RIGHT,
    
    RADIUS_OF_OBSTACLE_SEARCH = 35,            // 60 см
    RADIUS_OF_MOVEMENT = 20,                   // 40 см
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
static uint16_t arrRanges[NUMBER_OF_MEASUREMENTS_ALL];
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
 * и запись их в массив расстояний в соответствующие ячейки
 */
void measure_left()
{
    uint16_t* ptrArr = arrRanges;
    // Поворачиваемся в крайнее левое положение
    turn_around_by(-MAX_ANGLE_OF_ROTATION_WHEN_MEASURE);
    
    // Измеряем массив расстояний
    uint8_t countAngle;
    for (countAngle = 0; countAngle < NUMBER_OF_MEASUREMENTS_LEFT; countAngle++)
    {
        turn_around_by(ANGLE_OF_ROTATION_WHEN_MEASURE);
        *(ptrArr++) = getMedianRange();
    }
}


/* 
 * @brief Определение массива показаний дальномера справа от курсового угла
  * и запись их в массив расстояний в соответствующие ячейки
 */
void measure_right()
{
    uint16_t* ptrArr = arrRanges;
    // Поворачиваемся в крайнее правое положение
    turn_around_by(+MAX_ANGLE_OF_ROTATION_WHEN_MEASURE);
    
    // Запись данных с конца массива
    ptrArr += NUMBER_OF_MEASUREMENTS_ALL;
    
    // Измеряем массив расстояний
    uint8_t countAngle;
    for (countAngle = 0; countAngle <= 10; countAngle++)
    {
        turn_around_by(-ANGLE_OF_ROTATION_WHEN_MEASURE);
        *(ptrArr--) = getMedianRange();
    }
}


/* 
 * @brief Определение массива показаний дальномера справа и слева от курсового угла
 */
void measure()
{
    measure_left();
    measure_right();
}

/* 
 * @brief Вовзвращает 1, если будет обнаружено препятствие, иначе 0
 * @param указатель на массив расстояний
 * @return 1, если обнаружено препятствие, 0, если препятствия нет
 */
uint8_t is_there_obstacle()
{
    uint8_t count;
    int16_t* ptrArr = arrRanges;
    for(count = 0; count < NUMBER_OF_MEASUREMENTS_ALL; count++)
    {
        if (*ptrArr++ < RADIUS_OF_OBSTACLE_SEARCH)
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
            UART_write_string(debug, "i\n\r");
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
            UART_write_string(debug, "d\n\r");
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


/* 
 * @brief Обновление скорости робота с учетом влияния ПИ-регулятора и плавного
 * изменения скорости.
 * @note скорость робота в любом случае остается в интервале [0; maxSpeed]
 */
void update_robot_speed()
{
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
            //smooth_decrease_current_speed(encoder_left_get_pulses(), needPulses);
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
        update_robot_speed();
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
        int16_t angle = calculate_angle(dx, dy);
        turn_around_to(angle);
        robot.angle = angle;
        test_measure(); //measure();
        if( is_there_obstacle() )
        {
            turn_around_by(-90);
            robot.angle -= angle;
            test_measure(); //measure();
            if( is_there_obstacle() )
                break;
            else
            {
                move_forward(RADIUS_OF_MOVEMENT);
                turn_around_by(90);
                move_forward(RADIUS_OF_MOVEMENT << 1);
            }
        }
        else
        {
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
    
    uint8_t count;
    for(count = 0; count < NUMBER_OF_MEASUREMENTS_ALL; count++)
    {
        num2str(arrRanges[count], buf);
        UART_write_string(debug, " ");
        UART_write_string(debug, buf);
    }
    
}
/****************************** PUBLIC FUNCTION *******************************/

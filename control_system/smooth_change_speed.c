/** 
* @file smooth_change_speed.c
* @brief Implementation of smooth change speed regulator
*/


#include "smooth_change_speed.h"
#include "timer.h"


/// Declaration of private functions:
uint8_t smooth_increase_current_speed(uint8_t* currentSpeed);
void smooth_decrease_current_speed(uint8_t* currentSpeed);

/// Global and static object's
static Timer timerForSmoothChangeSpeedDelay;
static Timer timerForSmoothChangeSpeedDeadZone;

/// Some constans:
enum
{
	PWM_DUTY_CYCLE_MIN = 30,	///< Исходное значение минимальной скважности, duty_cycle
	PWM_DUTY_CYCLE_MAX = 70,	///< Исходное значение максимальной скважности, duty_cycle
	ROBOT_ACCELERATION = 1,		///< Исходное значение ускорения duty_cycle/(50мс)
	ROBOT_DECELERATION = 1,		///< Исходное значение замедления duty_cycle/(50мс)
};

/****************************** PUBLIC FUNCTION *******************************/

/** 
* @brief Инициализация таймеров плавного изменения скорости
*/
void smooth_change_current_speed_init()
{
	soft_timer_init(&timerForSmoothChangeSpeedDelay);
    soft_timer_init(&timerForSmoothChangeSpeedDeadZone);
}


/** 
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
void smooth_change_current_speed(uint8_t* currentSpeed, uint32_t nowPulses, uint32_t needPulses)
{
    enum
    {
        ROBOT_ACCELERATION_PROCESS = 0,
        ROBOT_ACCELERATION_STOP = 1,
        ROBOT_DECELERATION_STOP = 2,
        ROBOT_DECELERATION_PROCESS = 3,
    };
    static uint8_t statusOfChange = ROBOT_ACCELERATION_PROCESS;
    
    /// Ускорение - первые 2 этапа:
    if(nowPulses < (needPulses >> 1) )
    {
        if( statusOfChange == ROBOT_ACCELERATION_PROCESS)
        {
            if (smooth_increase_current_speed(currentSpeed) )
            {
                statusOfChange = ROBOT_ACCELERATION_STOP;
                timer_start_ms(&timerForSmoothChangeSpeedDeadZone, 30000);
            }
        }  
        else if (statusOfChange != ROBOT_ACCELERATION_STOP )
            statusOfChange = ROBOT_ACCELERATION_PROCESS;
    }
    
    /// Замедление - последние 2 этапа:
    else
    {
        switch(statusOfChange)
        {
            case ROBOT_ACCELERATION_STOP:
            {
                uint32_t timeOfDeadZone = timer_get_elapsed_time(&timerForSmoothChangeSpeedDeadZone)*0.001;
                timer_start_ms(&timerForSmoothChangeSpeedDeadZone, timeOfDeadZone);
                statusOfChange = ROBOT_DECELERATION_STOP;
                break;
            }
            case ROBOT_DECELERATION_STOP:
            {
                if( timer_report(&timerForSmoothChangeSpeedDeadZone) != TIMER_WORKING)
                    statusOfChange = ROBOT_DECELERATION_PROCESS;
                break;
            }
            case ROBOT_DECELERATION_PROCESS:
            {
                smooth_decrease_current_speed(currentSpeed);
                break;
            }
            default:
            {
                statusOfChange = ROBOT_DECELERATION_PROCESS;
            }
        }
    }
}

/****************************** PUBLIC FUNCTION *******************************/

/****************************** PRIVATE FUNCTION ******************************/

/** 
* @brief Плавное увеличение значения текущей скорости робота
* @param currentSpeed - указатель на текущую скорость робота
* @return 1, если максимальная скорость достигнута, иначе 0
*/
uint8_t smooth_increase_current_speed(uint8_t* currentSpeed)
{
    if (*currentSpeed < PWM_DUTY_CYCLE_MAX)
    {
        if( timer_report(&timerForSmoothChangeSpeedDelay) != TIMER_WORKING )
        {
            timer_start_ms(&timerForSmoothChangeSpeedDelay, 50);
            *currentSpeed += ROBOT_ACCELERATION;
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
* @param currentSpeed - указатель на текущую скорость робота
*/
void smooth_decrease_current_speed(uint8_t* currentSpeed)
{
    if (*currentSpeed > PWM_DUTY_CYCLE_MIN)
    {
        if( timer_report(&timerForSmoothChangeSpeedDelay) != TIMER_WORKING )
        {
            timer_start_ms(&timerForSmoothChangeSpeedDelay, 50);
            *currentSpeed -= ROBOT_DECELERATION;
        }
    }
}

/****************************** PRIVATE FUNCTION ******************************/

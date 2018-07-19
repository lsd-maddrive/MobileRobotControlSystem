/** 
* @file pid_regulator.c
* @brief Implementation of pid regulator
*/

#include "pid_regulator.h"
#include "encoder.h"

/** 
* @brief Расчет значения ПИ-регулятора.
* @note Скорость левого двигателя уменьшится на result.
* @note Скорость правого двигателя увеличивается на result.
* @return значение ПИ-регулятора
*/
int8_t PI_regulator()
{
	enum
	{
		REGULATOR_MAX_VALUE = 50,	///< 1/2 of max duty cycle
	};
	
    /// Константы, полученные эмпирическим путем:
    const uint8_t HYSTERESIS_COEFFICIENT = 0;
    const float P_COEFFICIENT = 1;
    const float I_COEFFICIENT = 1;
	//const float D_COEFFICIENT = 0;
	
	/// Переменные алгоритма:
    static float integralComponent = 0;
	int16_t leftPulses = abs_16( encoder_left_get_pulses() );
    int16_t rightPulses = abs_16( encoder_right_get_pulses() );
	int16_t error = leftPulses - rightPulses;
	int16_t result;
    
	/// П-составляющая:
	result = P_COEFFICIENT*error;
	
	/// И-составляющая:
    if( error - HYSTERESIS_COEFFICIENT > 0 )
    {
        if(integralComponent < REGULATOR_MAX_VALUE )
            integralComponent += I_COEFFICIENT;
    }
    else if( error + HYSTERESIS_COEFFICIENT < 0 )
    {
        if(integralComponent > (-REGULATOR_MAX_VALUE) )
            integralComponent -= I_COEFFICIENT;
    }
	result += integralComponent;
	
	/// Вернуть значение регулятора
    return result;
}

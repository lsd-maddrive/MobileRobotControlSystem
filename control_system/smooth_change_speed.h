/** 
* @file smooth_change_speed.h
* @brief Declaration of smooth change speed regulator
*/

#include <xc.h>

#ifndef SMOOTH_CHANGE_SPEED_H
#define	SMOOTH_CHANGE_SPEED_H

void smooth_change_current_speed_init();
void smooth_change_current_speed(uint8_t* currentSpeed, uint32_t nowPulses, uint32_t needPulses);

#endif	/* SMOOTH_CHANGE_SPEED_H */


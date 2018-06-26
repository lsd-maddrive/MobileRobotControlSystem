/* 
 * File: adc.h
 */

#ifndef ADC_H
#define	ADC_H

#include "hard.h" 

int adc_init(uint8_t channel);
int16_t adc_read();

#endif	/* ADC_H */


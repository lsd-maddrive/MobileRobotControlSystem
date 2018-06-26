/* 
 * File: adc.c
 */

#include "adc.h" 

int adc_init(uint8_t channel)
{
    AD1CON1bits.ADON = 0;
    
    // 1. Select port pins as analog inputs
    if (channel <= 15)
        AD1PCFGL &= ~(1 << channel);        
    else if (channel <= 31)
        AD1PCFGH &= ~(1 << (channel-16));
    else
        return -1;
    
    // 2. Select voltage reference source to match expected range on analog inputs
    AD1CON2 = 0x0000;
    
    // 3.Select the analog conversion clock to match desired data rate with processor clock
    AD1CON3bits.SAMC = 0b11111;     // Sample time 
    AD1CON3bits.ADCS = 0b11111111;  // Conversion clock select 
    
    // 4. Determine how many S/H channels will be used
    AD1CHS0 = channel;
    
    // 5.Select the appropriate sample/conversion sequence 
    AD1CON1bits.ASAM = 1;        // Auto sample
    
    // 6. Select how conversion results are presented in the buffer
    AD1CON1bits.SSRC = 0b111;    // Auto convertion
    AD1CON1bits.AD12B = 1;       // 0 = 10 bit ADC; 1 = 12 bit ADC
    
    // 7. Turn on ADC module	
    AD1CON1bits.ADON = 1;
    
    return 0;
}

int16_t adc_res = 0;

int16_t adc_read()
{	
    if ( AD1CON1bits.DONE ) {
        AD1CON1bits.DONE = 0;            // reset DONE bit
        adc_res = ADC1BUF0;
    }
	return adc_res;       			// read ADC1 data      
}
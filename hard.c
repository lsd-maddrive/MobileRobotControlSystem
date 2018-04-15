/*
 * File:   main.c
 */
#include "hard.h"


/*
* @brief Инициализация портов ввода/вывода
* @note TRISx - регистр направления порта
* @note PORTx - регистр чтения/записи 
*/
void GPIO_init()
{
    // A0
    TRISA &= ~(1 << 0);         // PORTA0 - output
    PORTA |= (1 << 0);          // PORTA0 - Высокий уровень

    // INT0 - RF6
    TRISA |= (1 << 6) |         // PORTF6 - input (encoder - INT0)
             (1 << 7);          // PORTF7 - input (encoder)
    
}


/*
* @brief Инициализация ШИМ
*/
void PWM_init()
{
    P1TCONbits.PTCKPS = 0x00;   // PWM PRESCALER (1:1 prescale)
    P1TCONbits.PTMOD = 0x00;    // PWM MODE: Free Running mode
    P1TMRbits.PTMR = 0;         // PWM time base is counting up
    
    PWM1CON1bits.PMOD1 = 1;     // PWM1 PAIR MODE: Independent Output mode
    PWM1CON1bits.PEN1L = 1;     // PWM1L (PE0) pin is enabled for PWM output
    PWM1CON1bits.PMOD2 = 1;     // PWM2 PAIR MODE: Independent Output mode
    PWM1CON1bits.PEN2L = 1;     // PWM2L (PE2) pin is enabled for PWM output
    PWM1CON1bits.PMOD1 = 1;     // PWM3 PAIR MODE: Independent Output mode
    PWM1CON1bits.PEN1L = 1;     // PWM3L (PE4) pin is enabled for PWM output
    PWM1CON1bits.PMOD2 = 1;     // PWM4 PAIR MODE: Independent Output mode
    PWM1CON1bits.PEN2L = 1;     // PWM4L (PE6) pin is enabled for PWM output
    
    P1TPERbits.PTPER = PWM_PERIOD;//PWM Time Base Period Value bits (max 32676)
        
    P1TCONbits.PTEN = 0x01;     // PWM time base is on
}

void PWM_set(uint8_t duty_cycle, uint8_t pin)
{
    switch(pin)
    {
        case MOTOR_LEFT_1:
        {
            P1DC1 = PWM_PERIOD*(100UL - duty_cycle)/100 << 1;
            break;
        }
        case MOTOR_LEFT_2:
        {
            P1DC2 = PWM_PERIOD*(100 - duty_cycle)/100 << 1;
            break;
        }
        case MOTOR_RIGHT_1:
        {
            P1DC3 = PWM_PERIOD*(100 - duty_cycle)/100 << 1;
            break;
        }
        case MOTOR_RIGHT_2:
        {
            P1DC4 = PWM_PERIOD*(100 - duty_cycle)/100 << 1;
            break;
        }
    }
                
}

/*
* @brief Инициализация внешнего прерывание от источника INT0
*/
void interrupt_INT0_init()
{
    INTCON2 &= ~0x0001;     // INT0 setup to interupt on rising edge
    IFS0bits.INT0IF = 0;    // INT0 reset interrupt flag 
    IEC0bits.INT0IE = 1;    // INT0 interupt enable
}

/*
* @brief Меняет тип прерывания от INT0: с прерывания по верхнему уровню на нижний и наоборот
*/
void change_type_of_interrupt()
{
    if ( (ENCODER_TYPE_OF_INTERRUPT) == ENCODER_POSITIVE_EDGE)
        INTCON2 |= ENCODER_NEGATIVE_EDGE;
    else
        INTCON2 &= ENCODER_POSITIVE_EDGE;
}

/*
* @brief Обнуляет флаг прерывания от INT0
*/
void reset_interrupt_flag()
{
    IFS0bits.INT0IF = 0;
}

/*
* @brief Инициализация таймер 23
* @note таймер23 будет использоваться для создания задержки
*/
void TIM23_init()
{
    T3CON = 0;              // Stop any operations Timer3
    T2CON = 0;              // Stop any operations Timer2
    
    T2CONbits.T32= 1;       // Enable 32-bit mode
    T2CONbits.TCKPS = 1;    // Timer2 prescale 1:8  
    TMR2 =  0x0000;         // Clear least significant half word
    TMR3 =  0x0000;         // Clear most significant half word
    PR2 = 0xFFFF;           // Timer23 period register
    PR3 = 0xFFFF;           // Timer23 period register
    
    IPC2bits.T3IP = 0x01;   // Set Timer3 Interrupt Priority Level
    IFS0bits.T3IF = 0;      // Clear Timer3 Interrupt Flag
    IEC0bits.T3IE = 1;      // Enable Timer3 interrupt
    
    T2CONbits.TON = 1;      // Start 32-bit Timer
}

/*
* @brief Прерывание по переполнению таймера23
*/
void __attribute__((interrupt,no_auto_psv)) _T3Interrupt( void )
{
    IFS0bits.T3IF = 0;      // Очистка флага прерывания
    T2CONbits.TON = 0;      // Выключение таймера
    T3CONbits.TON = 0;      // Выключение таймера
    
   	TIM23NumberOfOverflows++;
    
    T3CONbits.TON = 1;      // Включение таймера
	T2CONbits.TON = 1;      // Включение таймера
}

/*
* @brief Вернуть время таймера 23
*/
uint32_t return_time_of_TIM23()
{
    return (TMR3 << 16) + TMR2;
}
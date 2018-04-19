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

    // INT1 - RE8; RE9
    TRISE |= (1 << 8) |         // PORTE8 - input (encoder2 - INT1)
             (1 << 9);          // PORTE8 - input (encoder2)
    
    // INT0 - RF6; RF7
    TRISF |= (1 << 6) |         // PORTF6 - input (encoder1 - INT0)
             (1 << 7);          // PORTF7 - input (encoder1)
    
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

/*
* @brief Установка скважности PWM
*/
void PWM_set(uint8_t duty_cycle, uint8_t pin)
{
    switch(pin)
    {
        case MOTOR_LEFT_1:
        {
            PDC1 = PWM_PERIOD*(100UL - duty_cycle)/100 << 1;
            break;
        }
        case MOTOR_LEFT_2:
        {
            PDC2 = PWM_PERIOD*(100UL - duty_cycle)/100 << 1;
            break;
        }
        case MOTOR_RIGHT_1:
        {
            PDC3 = PWM_PERIOD*(100UL - duty_cycle)/100 << 1;
            break;
        }
        case MOTOR_RIGHT_2:
        {
            PDC4 = PWM_PERIOD*(100UL - duty_cycle)/100 << 1;
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

/* Некоторые нюансы работы аппаратного таймера:
 * - Частота мк - 16 МГц
 * - Регистр 32 битный
 * - Предделитель 8
 * - 1 тик таймер23 = 0.5 мкс
 * - 1 период таймера23 = 2147.48365 сек = 36 минут
 */

/*
* @brief Инициализация аппаратного таймера
* @note Выбран 32-битный таймер TIM23
*/
void hard_timer_init()
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
    
   	hardTimerOverflows++;
    
    T3CONbits.TON = 1;      // Включение таймера
	T2CONbits.TON = 1;      // Включение таймера
}

/*
* @brief Вернуть кол-во прерываний аппаратного таймера
*/
uint8_t hard_timer_return_overflows()
{
    return hardTimerOverflows;
}

/*
* @brief Вернуть время аппаратного таймера
*/
uint32_t hard_timer_return_time()
{
    return (TMR3 << 16) + TMR2;
}
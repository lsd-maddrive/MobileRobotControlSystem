/*
 * File:   main.c
 */
#include "hard.h"
#include "uart.h"

static uint8_t hardTimerOverflows;

/*
* @brief Инициализация портов ввода/вывода
* @note TRISx - регистр направления порта
* @note PORTx - регистр чтения/записи 
*/
void GPIO_init()
{
    // A0
    TRISA &= ~(1 << 0);         // PORTA0 - output
    PORTA |= (1 << 0);          // PORTA0 - высокий уровень
    
    // Rangefinder
    TRISA |= (1 << 15);         // PORTA15 - input (rangefinder input - INT4)
    TRISD &= ~(1 << 8);         // PORTD8 - output
    PORTD &= ~(1 << 8);         // PORTD8 - низкий уровень (rangefinder output)
    
    // Encoder
    TRISE |= (1 << 8) |         // PORTE8 - input (encoder2 - INT1)
             (1 << 9);          // PORTE8 - input (encoder2)
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
* @brief Инициализация внешнего прерывание энкодеров - INT0 и INT1
*/
void encoders_interrupt_init()
{
    INTCON2 &= ~0x0003;     // INT0 and INT1 setup to interupt on rising edge
    IFS0bits.INT0IF = 0;    // INT0 reset interrupt flag 
    IEC0bits.INT0IE = 1;    // INT0 interupt enable
    
    IFS1bits.INT1IF = 0;    // INT1 reset interrupt flag 
    IEC1bits.INT1IE = 0;    // INT1 interupt enable
}

/*
* @brief Меняет тип прерывания левого энкодера
* @note Левый энкодер - INT0
*/
void encoder_left_change_type_of_interrupt()
{
    if ( (ENCODER_LEFT_TYPE_OF_INTERRUPT) == ENCODER_POSITIVE_EDGE)
        ENCODER_LEFT_TYPE_OF_INTERRUPT = ENCODER_NEGATIVE_EDGE;
    else
        ENCODER_LEFT_TYPE_OF_INTERRUPT = ENCODER_POSITIVE_EDGE;
}

/*
* @brief Меняет тип прерывания правого энкодера
* @note Правый энкодер - INT1
*/
void encoder_right_change_type_of_interrupt()
{
    if ( (ENCODER_LEFT_TYPE_OF_INTERRUPT) == ENCODER_POSITIVE_EDGE)
        ENCODER_RIGHT_TYPE_OF_INTERRUPT = ENCODER_NEGATIVE_EDGE;
    else
        ENCODER_RIGHT_TYPE_OF_INTERRUPT = ENCODER_POSITIVE_EDGE;
}

/*
* @brief Обнуляет флаг прерывания от левого энкодера (INT0)
*/
inline void encoder_left_reset_interrupt_flag()
{
    IFS0bits.INT0IF = 0;
}

/*
* @brief Обнуляет флаг прерывания от правого энкодера (INT1)
*/
inline void encoder_right_reset_interrupt_flag()
{
    IFS1bits.INT1IF = 0;
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
    hardTimerOverflows = 0;
    T3CONbits.TON = 0;      // Stop any operations Timer3
    T2CONbits.TON = 0;      // Stop any operations Timer2
    T2CONbits.T32 = 1;      // Enable 32-bit mode
    T2CONbits.TCS = 0;      // Select internal instruction clock cycle
    T2CONbits.TGATE = 0;    // Disable Gates Mode
    T2CONbits.TCKPS = 0b01; // prescale 1:8 
    TMR3 =  0x00;           // Clear most significant half word
    TMR2 =  0x00;           // Clear least significant half word
    PR3 = 0xFFFF;           // Timer23 period register
    PR2 = 0x0000;           // Timer23 period register
    
    IPC2bits.T3IP = 0x05;   // Set Timer3 Interrupt Priority Level
    IFS0bits.T3IF = 0;      // Clear Timer3 Interrupt Flag
    IEC0bits.T3IE = 1;      // Enable Timer3 interrupt
    
    T3CONbits.TON = 1;      // Start 32-bit Timer
    T2CONbits.TON = 1;      // Start 32-bit Timer
    
}

/*
* @brief Прерывание по переполнению таймера23
*/
void __attribute__((interrupt,no_auto_psv)) _T3Interrupt( void )
{
   	//hardTimerOverflows++;
    
    IFS0bits.T3IF = 0;      // Очистка флага прерывания
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

/*
* @brief Инициализация внешнего прерывания дальномера - INT4
*/
void rangefinder_init_interrupt()
{
    RANGEFINDER_TYPE_OF_INTERRUPT = 0; // INT4 setup to interupt on rising edge
    IFS3bits.INT4IF = 0;    // INT4 reset interrupt flag 
    IEC3bits.INT4IE = 1;    // INT4 interupt enable
    
}

/*
* @brief Изменить тип внешнего прерывания дальномера - INT4
*/
inline void rangefinder_change_type_of_interrupt()
{
    if ( RANGEFINDER_TYPE_OF_INTERRUPT == ENCODER_POSITIVE_EDGE)
        RANGEFINDER_TYPE_OF_INTERRUPT = ENCODER_NEGATIVE_EDGE;
    else
        RANGEFINDER_TYPE_OF_INTERRUPT = ENCODER_POSITIVE_EDGE;
}
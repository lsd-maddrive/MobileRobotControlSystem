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
* @brief Инициализация таймер 23
* @note таймер23 будет использоваться для создания задержки
*/
void TIM23_init()
{
    T2CON = 0;              // Stop any operations master Timer2
    T3CON = 0;              // Stop any operations slave Timer3
    
    T2CONbits.T32= 1;       // Enable 32-bit mode
    T2CONbits.TCKPS = 1;    // Timer2 prescale 1:8  
    TMR2 =  0x0000;         // Clear least significant half word
    TMR3 =  0x0000;         // Clear most significant half word
    PR2 = 0xFFFFFFFF;       // Timer23 period register
    
    IPC1bits.T2IP = 0x01;   // Set priority level of interrupt = 1
    IFS0bits.T2IF = 0;      // Clear the Timer2 interrupt status flag
    IEC0bits.T2IE = 1;      // Enable Timer2 interrupts
    
    T2CONbits.TON = 1;      // Enable Timer2
    T3CONbits.TON = 1;      // Enable Timer3
    
}

/*
* @brief Прерывание по переполнению таймера23
*/
void __attribute__((interrupt,no_auto_psv)) _T2Interrupt( void )
{
    IFS0bits.T1IF = 0;      // Очистка флага прерывания
    T2CONbits.TON = 0;      // Выключение таймера
    T3CONbits.TON = 0;      // Выключение таймера
    
   	numberOfOverflows++;
    
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
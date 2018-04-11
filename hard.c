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
    T2CONbits.TCKPS = 3;    // Timer2 prescale 1:256  
    TMR2 =  0x0000;         // Clear least significant half word
    TMR3 =  0x0000;         // Clear most significant half word
    PR2 = 0xFFFFFFFF;       // Timer23 period register
    
    T2CONbits.TON = 1;
    T3CONbits.TON = 1;
    
    T1CONbits.TCKPS = 3;    // Timer1 prescale 1:256  
    T1CONbits.TON = 1;      // Enable Timer1 and start the counter
}

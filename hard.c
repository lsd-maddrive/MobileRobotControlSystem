/*
 * File:   main.c
 */
#include "hard.h"

void GPIO_init()
{
    // Регистр направления порта:
    TRISA &= ~0x01;         // PORTA1 - output
    // Регистр чтения/записи
    PORTA |= 0x01;          // PORTA1 - Высокий уровень
}

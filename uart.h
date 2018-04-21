/* 
 * File:   uart.h
 */

#ifndef UART_H
#define	UART_H

#include "hard.h"   

enum
{
    UART_DATA_BUFFER_SIZE = 256,    // Размер буфера uart
};

typedef enum
{
    UART_1 = 0,                     // Первый модуль uart
    UART_2 = 1,                     // Второй модуль uart
} Uart_number_t;

typedef enum
{
    UART_BAUD_RATE_9600 = CLOCK_FREQUENCY/16/9600 - 1,      // 103
    UART_BAUD_RATE_115200 = CLOCK_FREQUENCY/16/115200 - 1,  // 8
} Uart_boud_rate_t;

typedef struct 
{
    //bool initialized;
    
    uint8_t write_big_endian_mode;
    uint8_t write_buffer[UART_DATA_BUFFER_SIZE];
    uint8_t i_write_head_byte;
    uint8_t i_write_tail_byte;
    uint8_t n_write_bytes_available;
    uint8_t write_overflow;
    
    uint8_t read_buffer[UART_DATA_BUFFER_SIZE];
    uint8_t i_read_head_byte;
    uint8_t i_read_tail_byte;
    uint8_t n_read_bytes_available;
    uint8_t read_overflow;
    
    volatile unsigned int* reg_mode;            // UARTx Mode Register
    volatile unsigned int* reg_status;          // UARTx Status and Control Register
    volatile unsigned int* reg_receive;         // UARTx Receive Register
    volatile unsigned int* reg_transmit;        // UARTx Transmit Register
    volatile unsigned int* reg_baud_rate;       // UARTx Baud Rate Register
    volatile unsigned int* reg_interrupt_flag;  // UARTx Baud Interrupt Flag Register
    
    unsigned int interrupt_flag_tx_mask;
    unsigned int interrupt_flag_rx_mask;

} UART_module;

UART_module* UART_init(Uart_number_t, Uart_boud_rate_t);// Инициализация модуля uart
void UART_receive(UART_module*, uint8_t*, uint8_t);     // Получение данных по uart
void UART_transmit(UART_module*, char*, uint8_t);    // Передача данных по uart


#endif	/* UART_H */


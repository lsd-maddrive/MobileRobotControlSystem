/* 
 * File:   uart.c
 */

#include "uart.h"

#define SET_REG_BIT(reg, mask)      ( (reg) |= (mask) )
#define RESET_REG_BIT(reg, mask)    ( (reg) &= ~(mask) )

enum
{
    UART_STA_DATA_ACCESS_BIT = (1 << 0),
    UART_STA_BUFFER_FULL_BIT = (1 << 9),
    UART1_TX_FLAG = (1 << 12),
    UART1_RX_FLAG = (1 << 11),
    UART2_TX_FLAG = (1 << 15),
    UART2_RX_FLAG = (1 << 14),
};

UART_module UART_module_base[] = 
{
    {
        .i_write_head_byte = 0, .i_write_tail_byte = 0, 
        .n_write_bytes_available = 0, .write_overflow = false, .i_read_head_byte = 0,  
        .i_read_tail_byte = 0, .n_read_bytes_available = 0,  .read_overflow = false,
        
        .reg_mode = &U1RXREG, .reg_status = &U1STA, .reg_receive = &U1RXREG,
        .reg_transmit = &U1TXREG, .reg_baud_rate = &U1BRG, .reg_interrupt_flag = &IFS0,
        .interrupt_flag_tx_mask = UART1_TX_FLAG, .interrupt_flag_rx_mask = UART1_RX_FLAG
    },
    {
        .i_write_head_byte = 0, .i_write_tail_byte = 0, 
        .n_write_bytes_available = 0, .write_overflow = false, .i_read_head_byte = 0,  
        .i_read_tail_byte = 0, .n_read_bytes_available = 0,  .read_overflow = false,

        .reg_mode = &U2RXREG, .reg_status = &U2STA, .reg_receive = &U2RXREG,
        .reg_transmit = &U2TXREG, .reg_baud_rate = &U2BRG, .reg_interrupt_flag = &IFS1,
        .interrupt_flag_tx_mask = UART2_TX_FLAG, .interrupt_flag_rx_mask = UART2_RX_FLAG
    }
};

UART_module* debug;

/* 
 * @brief Инициализация модуля UART
 * @return указатель на объект структуры UART_module_config_t
*/
UART_module* UART_init(Uart_number_t numberOfModule, Uart_boud_rate_t boudRate)
{
    // Проверка корректности аргументов:
    if ( (numberOfModule != UART_1) && (numberOfModule != UART_2) )
        return NULL;
    
    UART_module* module = &UART_module_base[numberOfModule];
    
    if(numberOfModule == UART_1)
    {
        U1MODEbits.UARTEN = 0;  // UART1 is disabled
        U1MODEbits.UEN = 0;     // Bits8,9 TX,RX enabled, CTS,RTS not
        U1MODEbits.BRGH = 0;
        
        U1BRG = boudRate;
        _U1TXIF = 0;            // TX interrupt flag reset
        _U1RXIF = 0;            // RX interrupt flag reset
        
        _U1TXIE = 1;            // TX interrupt Enable
        _U1RXIE = 1;            // RX interrupt Enable
        
        _U1TXIP = 1;            // TX interrupt prioputy 
        _U1RXIP = 1;            // RX interrupt prioputy 
        
        U1STAbits.UTXISEL0 = 0;
        U1STAbits.UTXISEL1 = 0;
        
        U1MODEbits.UARTEN = 1;  // UARTx is enabled
        U1STAbits.UTXEN = 1;
    }
    else
    {
        U2MODEbits.UARTEN = 0;  // UART2 is disabled
        
        U2BRG = boudRate;
        _U2TXIF = 0;            // TX interrupt flag reset
        _U2RXIF = 0;            // RX interrupt flag reset
        
        _U2TXIE = 1;            // TX interrupt Enable
        _U2RXIE = 1;            // RX interrupt Enable
        
        _U2TXIP = 1;            // TX interrupt prioputy 
        _U2RXIP = 1;            // RX interrupt prioputy 
        
        U2STAbits.UTXISEL0 = 0;
        U2STAbits.UTXISEL1 = 0;
        
        U2MODEbits.UARTEN = 1;
        U2STAbits.UTXEN = 1;
    }
    
    return module;
}

/* 
 * @brief Проверка доступности байтов
*/
uint8_t UART_bytes_available( UART_module* module )
{
    return module->n_read_bytes_available;
}

/* 
 * @brief Обработчик прерывания UART
*/
void rx_interrupt_handler( volatile UART_module* module )
{
    if ( (*(module->reg_status) & UART_STA_DATA_ACCESS_BIT) && !module->read_overflow ) 
    {
        module->read_buffer[module->i_read_head_byte++] = *module->reg_receive;
        if ( ++module->n_read_bytes_available == UART_DATA_BUFFER_SIZE ) 
            module->read_overflow = true;
    } 
    else
        RESET_REG_BIT( *(module->reg_interrupt_flag), module->interrupt_flag_rx_mask );
}

/* 
 * @brief ПОП обработки прерывания UART1
*/
void __attribute__( (__interrupt__, auto_psv) ) _U1RXInterrupt()
{
    rx_interrupt_handler( &UART_module_base[0] );
}

/* 
 * @brief ПОП обработки прерывания UART2
*/
void __attribute__( (__interrupt__, auto_psv) ) _U2RXInterrupt()
{
    rx_interrupt_handler( &UART_module_base[1] );
}

/* 
 * @brief получить байт c ,буфера uart модуля
 * @return байт данных
*/
uint8_t UART_get_byte( UART_module* module )   
{
    module->read_overflow = false;
    
    module->n_read_bytes_available--;
    return module->read_buffer[module->i_read_tail_byte++];
}

/* 
 * @brief получить данные с uart
 * @return байт данных
*/
void UART_receive( UART_module* module, uint8_t* buffer, uint8_t length )
{
    if ( module == NULL )
        return;
    
    int16_t i = 0;
    for ( i = 0; i < length; i++ ) 
    {
        if ( UART_bytes_available( module ) == 0 )
            return;

        buffer[i] = UART_get_byte( module );
    }
}

/* 
 * @brief обработчик прерывания по UART TX
 * @return байт данных
*/
void tx_interrupt_handler( volatile UART_module* module )
{
    while ( !(*(module->reg_status) & UART_STA_BUFFER_FULL_BIT) )
    {
        if ( module->n_write_bytes_available )
        {
            *module->reg_transmit = module->write_buffer[module->i_write_tail_byte++];
            module->n_write_bytes_available--;
            module->write_overflow = false;
        } else 
        {
            RESET_REG_BIT( *(module->reg_interrupt_flag), module->interrupt_flag_tx_mask );
            break;
        }
    }
}

/* 
 * @brief ПОП обработки прерывания UART1
*/
void __attribute__( (__interrupt__, auto_psv) ) _U1TXInterrupt()
{
    tx_interrupt_handler( &UART_module_base[0] );
}

/* 
 * @brief ПОП обработки прерывания UART2
*/
void __attribute__( (__interrupt__, auto_psv) ) _U2TXInterrupt()
{
    tx_interrupt_handler( &UART_module_base[1] );
};

/* 
 * @brief передать байт по UART
*/
void UART_write_byte( UART_module* module, const uint8_t byte )
{
    while ( module->write_overflow );
    
    module->write_buffer[module->i_write_head_byte++] = byte;
    module->n_write_bytes_available++;

    SET_REG_BIT( *(module->reg_interrupt_flag), module->interrupt_flag_tx_mask );
    
    if ( module->n_write_bytes_available == UART_DATA_BUFFER_SIZE )
        module->write_overflow = true;
}

/* 
 * @brief передать массив данных по UART
*/
void UART_transmit( UART_module* module, const char* buffer, const uint8_t length )
{
    if ( module == NULL )
        return;
    
    uint16_t count = 0;
    for ( count = 0; count < length; count++ ) 
    {
        UART_write_byte( module, buffer[count] );
    }
}

/* 
 * @brief передать строку по UART
*/
void UART_write_string( UART_module* module, const char* buffer)
{
    if ( module == NULL )
        return;
    
    uint8_t count;
    while( *buffer != '\0' )
    {
        UART_write_byte( module, *(buffer++) );
        if (count++ == UART_DATA_MAX_NUMBER_OF_BUFFER_BYTE)
            break;
    }
}
 
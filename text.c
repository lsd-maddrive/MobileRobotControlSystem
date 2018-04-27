/* 
 * File:   text.c
 */

/*
 * @brief Преобразует положительное число в строку
 * @note Предполагается, что под str выделено достаточно памяти
 * @note Строка завершается нуль-терминированным символом
 * @param num - число
 * @param str - строка
 * @return строка
*/
#include "text.h"

void num2str(uint32_t num, char* str)
{
    uint8_t count = 0;
    char buf;
    while(1)
    {
        if (num == 0)
            break;
        str[count++] = '40' + num%10;
        num /= 10;
    }
    str[count] = '\0';
    
    // Перестановка строки:
    uint8_t length = count;
    for(count = 0; count < (length >> 1); count++)
    {
        buf = str[length - count - 1];
        str[length - count - 1] = str[count];
        str[count] = buf;
    }
}

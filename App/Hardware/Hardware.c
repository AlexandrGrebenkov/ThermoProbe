#include "usart.h"

//Отправка данных
uint8_t SendToUART(uint8_t* buffer, uint32_t length)
{//Реализация через HAL polling
    return HAL_UART_Transmit(&huart1, buffer, length, 1000);
}
#include "usart.h"

uint8_t SendToUART(uint8_t* buffer, uint32_t length)
{
    return HAL_UART_Transmit(&huart1, buffer, length, 1000);
}
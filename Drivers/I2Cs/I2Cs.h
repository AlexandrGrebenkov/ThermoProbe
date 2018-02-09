#ifndef MULTI_I2Cs_H
#define MULTI_I2Cs_H

#include <stdint.h>
#include "gpio.h"

#define I2CS_NO_ERROR                0x00
#define I2CS_NO_ADDR_ACK_ERROR       0x01
#define I2CS_NO_ACK_ERROR            0x02

typedef struct
{
  uint16_t              SDA_PIN;
  GPIO_TypeDef *        SDA_PORT;
  uint16_t              SCL_PIN;
  GPIO_TypeDef *        SCL_PORT; 
  
  uint8_t               SlaveAddress;
  uint8_t               ErrorCode;
}i2cs;

uint32_t i2csWrite(i2cs* I2Cs, uint8_t *pBuf, uint32_t addr, uint32_t size);
uint32_t i2csRead(i2cs* I2Cs, uint8_t *pBuf, uint32_t addr, uint32_t size);
uint32_t i2csTest(i2cs* I2Cs);

#endif
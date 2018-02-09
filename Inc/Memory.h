#ifndef __MEMORY_H
#define __MEMORY_H
#include <stdint.h>

#define MemParamBaseAddr        0x8080000 //базовый адрес для конфигов датчика

#define SysStructSize           256
#define CharactStructSize       32

#define MemParamAddr            0
#define MemSensor1Addr          SysStructSize + 1*CharactStructSize
#define MemSensor2Addr          SysStructSize + 2*CharactStructSize
#define MemSensor3Addr          SysStructSize + 3*CharactStructSize
#define MemSensor4Addr          SysStructSize + 4*CharactStructSize

typedef enum
{
  SMM_Rate0,
  SMM_Rate1,
  SMM_Rate2,
}SensorMemMap;

typedef enum
{
  PMM_FirstStart,
  PMM_Serial,
}ParamMemMap;

typedef enum
{
  M_Read,
  M_Write
}RW_Action;

uint8_t WriteToEEPROM(uint32_t data, uint32_t address);
uint32_t ReadFromEEPROM(uint32_t address);

uint8_t WriteCharacteristic(uint8_t number);
uint8_t ReadCharacteristic(uint8_t number);
#endif
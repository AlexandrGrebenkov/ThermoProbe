#include "Memory.h"
#include "stm32l0xx_hal.h"
#include "DataTypes.h"
const uint32_t SensorsAddr[4] = {MemSensor1Addr, MemSensor2Addr, MemSensor3Addr, MemSensor4Addr};

uint8_t WriteToEEPROM(uint32_t data, uint32_t address)
{
  HAL_FLASHEx_DATAEEPROM_Unlock();
  HAL_FLASHEx_DATAEEPROM_Program(FLASH_TYPEPROGRAMDATA_WORD, address*4 + MemParamBaseAddr, data);
  HAL_FLASHEx_DATAEEPROM_Lock();
  return 1;
}

uint32_t ReadFromEEPROM(uint32_t address)
{
  return *(uint32_t*)(MemParamBaseAddr + address*4);
}

/******************************************
«апись характеристики из RAM в ROM
uint8_t number: номер характеристики
*******************************************/
uint8_t WriteCharacteristic(uint8_t number)
{
  uint8_t mec = 0;
  converter bc;
  
  HAL_FLASHEx_DATAEEPROM_Unlock();
  
  bc.f = TP.TSens[number].Rate[0];
  mec = HAL_FLASHEx_DATAEEPROM_Program(FLASH_TYPEPROGRAMDATA_WORD, SensorsAddr[number] + SMM_Rate0*4 + MemParamBaseAddr, bc.l32);

  bc.f = TP.TSens[number].Rate[1];
  mec = HAL_FLASHEx_DATAEEPROM_Program(FLASH_TYPEPROGRAMDATA_WORD, SensorsAddr[number] + SMM_Rate1*4 + MemParamBaseAddr, bc.l32);
  
  bc.f = TP.TSens[number].Rate[2];
  mec = HAL_FLASHEx_DATAEEPROM_Program(FLASH_TYPEPROGRAMDATA_WORD, SensorsAddr[number] + SMM_Rate2*4 + MemParamBaseAddr, bc.l32);

  HAL_FLASHEx_DATAEEPROM_Lock();
  return mec;
}

/******************************************
„тение характеристики из ROM в RAM 
uint8_t number: номер характеристики
*******************************************/
uint8_t ReadCharacteristic(uint8_t number)
{
  uint8_t mec = 0;
  converter bc;
  bc.l32 = *(uint32_t*)(MemParamBaseAddr + SensorsAddr[number] + SMM_Rate0*4);
  TP.TSens[number].Rate[0] = bc.f;
  bc.l32 = *(uint32_t*)(MemParamBaseAddr + SensorsAddr[number] + SMM_Rate1*4);
  TP.TSens[number].Rate[1] = bc.f;
  bc.l32 = *(uint32_t*)(MemParamBaseAddr + SensorsAddr[number] + SMM_Rate2*4);
  TP.TSens[number].Rate[2] = bc.f;
  return mec;
}















#ifndef __APP_H
#define __APP_H

#include "DataHelpers.h"

//������ ������ �������
typedef enum
{
  StreamMode,
  FullStreamMode,
  RequestMode,
  WriteParameter,
  ReadParameter,
}Modes;

//-----------������� ������ � ������ �������------------
uint8_t SetCharacteristic(uint8_t number, float* rates);
void SetDeviceMode(Modes mode);
Modes GetDeviceMode();
void SetSerial(uint32_t serial);
uint32_t GetSerial();

uint8_t GetStatusRegister();
uint16_t GetTAvg();

#endif
#ifndef __APP_H
#define __APP_H

#include "DataHelpers.h"

//���������� ������� ��� ����������:
#define IntegrSize 2

//������ ������ �����
#define RxBuffSize 256

//������ ������ �������
typedef enum {
  StreamMode = 1,//��������� �����
  FullStreamMode,//����������� �����
  RequestMode,//����� "������-�����"
  WriteParameter,//������ ���������
  ReadParameter,//������ ���������
} Modes;

//������� ������ ������ ������-��������
typedef enum {
  sf_Sensor1 = (1 << 0),
  sf_Sensor2 = (1 << 1),
  sf_Sensor3 = (1 << 2),
  sf_Sensor4 = (1 << 3),
  sf_Humiddity = (1 << 7)
} StatusFlags;

//-----------������� ������ � ������ �������------------
uint8_t SetCharacteristic(uint8_t number, float *rates);
void SetDeviceMode(Modes mode);
Modes GetDeviceMode();
void SetSerial(uint32_t serial);
uint32_t GetSerial();

uint8_t GetStatusRegister();
uint16_t GetTAvg();
void GetTSensorData(uint8_t number, uint16_t* TObj, uint8_t* ec);

uint8_t *GetUARTBuffer();
void AddToRxBuffer(uint8_t data);
void ResetRxBuffer();

void Multi_I2Cs_Init(void);
void DataInit(void);

#endif
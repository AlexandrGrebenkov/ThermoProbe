#ifndef __DATATYPES_H
#define __DATATYPES_H

#include "I2Cs.h"
#include "DataHelpers.h"
#include "App.h"

//��������� ������ ������� �����������
typedef struct
{
  float         T;      //�����������(� ��������)
  uint16_t      Tobj;   //����������� � ��������
  uint8_t       ec;     //������
  i2cs          I2C;
  float         Rate[3];//������������
  uint16_t      Integr[IntegrSize];
  uint8_t       IntegrCounter;
}tSensor;

//��������� ����� ����������������� ����������
typedef struct
{
  uint8_t       RxBuf[RxBuffSize];  //����� �����
  uint8_t       Counter;            //������� �������� ����
}serial;

//��������� �������
typedef struct
{
  tSensor       TSens[4];
  uint16_t      Tavg;           //������� ����������� � ��������
  StatusFlags   StatusReg;      //��������� �������(���������, ������...)
  Modes         Mode;           //����� ������ ���������� (0-����� "������")
  serial        UART;
  uint32_t      SerialNumber;   //�������� ����� �������
}termoprobe;

#endif
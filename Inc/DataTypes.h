#ifndef __DATATYPES_H
#define __DATATYPES_H

#include "I2Cs.h"
#define IntegrSize      2

#define LO(x) (uint8_t)((x >> 0)&0xFF);
#define HI(x) (uint8_t)((x >> 8)&0xFF);

typedef union
{
  uint8_t       b[4];
  float         f;
  uint32_t      l32;
}converter;

typedef enum
{
  StreamMode,
  FullStreamMode,
  RequestMode,
  WriteParameter,
  ReadParameter,
}Modes;

typedef enum
{
  C_ChangeMode, //����� ������
  C_GetData,    //������ ���������� �����������
  C_GetFullData,//������ �� ���� ��������
  C_GetRates,   //������ ������������� �����������
  C_SetRates,   //��������� ����� ������������� ����������
  C_GetInfoBlock,//������ ����-����� �������
  C_Done,       //���������
  C_GetSerial,  //������ ��������� ������  
  C_SetSerial,  //������ ��������� ������  
}NW_Commands;

#define OwnAddress 0x37
#define TargetAddress 0xB2  

//��������� ������ ������� �����������
typedef struct
{
  float         T;//�����������(� ��������)
  uint16_t      Tobj;   //����������� � ��������
  uint8_t       ec;     //������
  i2cs          I2C;
  float         Rate[3];//������������
  uint16_t      Integr[IntegrSize];
  uint8_t       IntegrCounter;
}tSensor;

typedef struct
{
  uint8_t       RxBuf[256];
  uint8_t       Counter;
}serial;

//��������� �������
typedef struct
{
  tSensor       TSens[4];
  uint16_t      Tavg;           //������� ����������� � ��������
  uint8_t       StatusReg;      //��������� �������(���������, ������...)
  Modes         Mode;           //����� ������ ���������� (0-����� "������")
  serial        UART;
  uint32_t      SerialNumber;   //�������� ����� �������
}termoprobe;

void Multi_I2Cs_Init(void);
void DataInit(void);

extern termoprobe TP;//data.c
void CalcAverage(void);
void Integr(uint8_t ch);
#endif
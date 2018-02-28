#ifndef __SERIAL_H
#define __SERIAL_H

#include <stdint.h>

#define OwnAddress 0x37
#define TargetAddress 0xB2  

typedef enum
{
  C_ChangeMode = 1, //����� ������
  C_GetData,    //������ ���������� �����������
  C_GetFullData,//������ �� ���� ��������
  C_GetRates,   //������ ������������� �����������
  C_SetRates,   //��������� ����� ������������� ����������
  C_GetInfoBlock,//������ ����-����� �������
  C_Done,       //���������
  C_GetSerial,  //������ ��������� ������  
  C_SetSerial,  //������ ��������� ������  
}NW_Commands;

void Parser(uint8_t *buffer);
uint8_t SendMeasure();

#endif
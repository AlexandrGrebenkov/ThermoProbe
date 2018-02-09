#ifndef __DATATYPES_H
#define __DATATYPES_H

#include "I2Cs.h"
#include "DataHelpers.h"
#include "App.h"

#define IntegrSize      2

//Структура одного датчика температуры
typedef struct
{
  float         T;//Температура(В градусах)
  uint16_t      Tobj;   //Температура в попугаях
  uint8_t       ec;     //Ошибки
  i2cs          I2C;
  float         Rate[3];//Коэффициенты
  uint16_t      Integr[IntegrSize];
  uint8_t       IntegrCounter;
}tSensor;

typedef struct
{
  uint8_t       RxBuf[256];
  uint8_t       Counter;
}serial;

//Структура прибора
typedef struct
{
  tSensor       TSens[4];
  uint16_t      Tavg;           //Средняя температура в попугаях
  uint8_t       StatusReg;      //Статусный регистр(состояние, ошибки...)
  Modes         Mode;           //Режим работы Термозонда (0-режим "Потока")
  serial        UART;
  uint32_t      SerialNumber;   //Серийный номер датчика
}termoprobe;

void Multi_I2Cs_Init(void);
void DataInit(void);

void CalcAverage(void);
void Integr(uint8_t ch);

extern termoprobe TP;//data.c

#endif
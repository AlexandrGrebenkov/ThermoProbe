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
  C_ChangeMode, //Смена режима
  C_GetData,    //Запрос усреднённой температуры
  C_GetFullData,//Данные со всех датчиков
  C_GetRates,   //Запрос коэффициентов калибровкаи
  C_SetRates,   //Установка новых коэффициентов калибровки
  C_GetInfoBlock,//Запрос ИНФО-блока датчика
  C_Done,       //Выполнено
  C_GetSerial,  //Запрос серийного номера  
  C_SetSerial,  //Запись серийного номера  
}NW_Commands;

#define OwnAddress 0x37
#define TargetAddress 0xB2  

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

extern termoprobe TP;//data.c
void CalcAverage(void);
void Integr(uint8_t ch);
#endif
#ifndef __SERIAL_H
#define __SERIAL_H

#include <stdint.h>

#define OwnAddress 0x37
#define TargetAddress 0xB2  

typedef enum
{
  C_ChangeMode = 1, //Смена режима
  C_GetData,    //Запрос усреднённой температуры
  C_GetFullData,//Данные со всех датчиков
  C_GetRates,   //Запрос коэффициентов калибровкаи
  C_SetRates,   //Установка новых коэффициентов калибровки
  C_GetInfoBlock,//Запрос ИНФО-блока датчика
  C_Done,       //Выполнено
  C_GetSerial,  //Запрос серийного номера  
  C_SetSerial,  //Запись серийного номера  
}NW_Commands;

void Parser(uint8_t *buffer);
uint8_t SendMeasure();

#endif
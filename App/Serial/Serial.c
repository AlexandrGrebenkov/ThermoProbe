#include "Serial.h"
#include "DataHelpers.h"
#include "App.h"
#include "Hardware.h"
#include <string.h>
#include "Build_def.h"
#include "DataTypes.h"

extern const TInfo Info;//main

//Для отправки ответа в UART
uint8_t TxBuf[150];
uint8_t cntr;

//Разборщик посылок
void Parser(uint8_t *buffer)
{
    if (*(buffer + 0) == TargetAddress)
    {//Если посылка адресованна нам
        cntr = 0;
        TxBuf[cntr++] = OwnAddress;
        TxBuf[cntr++] = *(buffer + 1);//Шапка ответа
        switch (*(buffer + 1))
        {
        case C_ChangeMode:
        { //Смена режима
            SetDeviceMode((Modes)(*(buffer + 2)));
            TxBuf[cntr++] = 1;
            break;
        }
        case C_GetData:
        { //Запрос данных
            TxBuf[cntr++] = HI(GetTAvg());
            TxBuf[cntr++] = LO(GetTAvg());
            TxBuf[cntr++] = GetStatusRegister();
            break;
        }
        case C_GetRates:
        { //Запрос характеристии одного сенсора
            TxBuf[cntr++] = *(buffer + 2);
            SetCharacteristic(*(buffer + 2), (float *)(TxBuf + 3));
            break;
        }
        case C_SetRates:
        { //Запись одной характеристики
            SetCharacteristic(*(buffer + 2), (float *)(buffer + 3));
            TxBuf[cntr++] = 1;
            break;
        }
        case C_GetInfoBlock:
        {//Запрос информации о приборе
            memcpy(&TxBuf[2], &Info, 128);
            cntr = 128 + 2;
            break;
        }
        case C_GetSerial:
        { //Запрос серийного номера
            TxBuf[cntr++] = HI(GetSerial());
            TxBuf[cntr++] = LO(GetSerial());
            break;
        }
        case C_SetSerial:
        { //Запись серийного номера
            SetSerial(*(buffer + 2) * 256 + *(buffer + 3));
            break;
        }
        default:
        {
            cntr = 0;
            asm("NOP");
        }
        }
        SendToUART(TxBuf, cntr);//отправка ответа
    }
}

//Отпарвка результатов замера без запроса
uint8_t SendMeasure()
{
    cntr = 0;
    TxBuf[cntr++] = OwnAddress;
    TxBuf[cntr++] = C_GetData;
    TxBuf[cntr++] = HI(GetTAvg());
    TxBuf[cntr++] = LO(GetTAvg());
    TxBuf[cntr++] = GetStatusRegister();
    if (GetDeviceMode() == FullStreamMode)
    {
        TxBuf[1] = C_GetFullData;
        for (int i = 0; i < 4; i++)
        {
            uint16_t TObj;
            uint8_t ec;
            GetTSensorData(i, &TObj, &ec);
            TxBuf[cntr++] = HI(TObj);
            TxBuf[cntr++] = LO(TObj);
            TxBuf[cntr++] = ec;
        }
    }

   return SendToUART(TxBuf, cntr);
}
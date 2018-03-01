#include "Serial.h"
#include "DataHelpers.h"
#include "App.h"
#include "Hardware.h"
#include <string.h>
#include "Build_def.h"
#include "DataTypes.h"

extern const TInfo Info;//main

//��� �������� ������ � UART
uint8_t TxBuf[150];
uint8_t cntr;

//��������� �������
void Parser(uint8_t *buffer)
{
    if (*(buffer + 0) == TargetAddress)
    {//���� ������� ����������� ���
        cntr = 0;
        TxBuf[cntr++] = OwnAddress;
        TxBuf[cntr++] = *(buffer + 1);//����� ������
        switch (*(buffer + 1))
        {
        case C_ChangeMode:
        { //����� ������
            SetDeviceMode((Modes)(*(buffer + 2)));
            TxBuf[cntr++] = 1;
            break;
        }
        case C_GetData:
        { //������ ������
            TxBuf[cntr++] = HI(GetTAvg());
            TxBuf[cntr++] = LO(GetTAvg());
            TxBuf[cntr++] = GetStatusRegister();
            break;
        }
        case C_GetRates:
        { //������ ������������� ������ �������
            TxBuf[cntr++] = *(buffer + 2);
            SetCharacteristic(*(buffer + 2), (float *)(TxBuf + 3));
            break;
        }
        case C_SetRates:
        { //������ ����� ��������������
            SetCharacteristic(*(buffer + 2), (float *)(buffer + 3));
            TxBuf[cntr++] = 1;
            break;
        }
        case C_GetInfoBlock:
        {//������ ���������� � �������
            memcpy(&TxBuf[2], &Info, 128);
            cntr = 128 + 2;
            break;
        }
        case C_GetSerial:
        { //������ ��������� ������
            TxBuf[cntr++] = HI(GetSerial());
            TxBuf[cntr++] = LO(GetSerial());
            break;
        }
        case C_SetSerial:
        { //������ ��������� ������
            SetSerial(*(buffer + 2) * 256 + *(buffer + 3));
            break;
        }
        default:
        {
            cntr = 0;
            asm("NOP");
        }
        }
        SendToUART(TxBuf, cntr);//�������� ������
    }
}

//�������� ����������� ������ ��� �������
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
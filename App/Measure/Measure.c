#include "DataTypes.h"
#include "I2Cs.h"
#include "cmsis_os.h"
#include "Measure.h"
#include "crc.h"

extern termoprobe TP;//App.c

void TemperaturMeasument()
{
    uint8_t address = 0x5A;

    //для расчёта CRC-8
    uint8_t crc8;
    uint8_t crcbuf[5];
    crcbuf[0] = (address << 1);
    crcbuf[1] = 0x07;
    crcbuf[2] = (address << 1) + 1;

    uint8_t buf[3];
    for (int i = 0; i < 4; i++)
    {
        taskENTER_CRITICAL();
        buf[0] = 0x07; //0х07 - Номер регистра с температурой
        TP.TSens[i].ec = i2csRead(&TP.TSens[i].I2C, buf, address, 3);
        taskEXIT_CRITICAL();
        if (TP.TSens[i].ec == 0)
        { //Если нет ошибок
            crcbuf[3] = buf[0];
            crcbuf[4] = buf[1];
            crc8 = HAL_CRC_Calculate(&hcrc, (uint32_t *)crcbuf, 5);
            if (buf[2] != crc8)
                asm("NOP");
            float T = (float)((buf[1] << 8) + buf[0]);
            TP.TSens[i].Integr[TP.TSens[i].IntegrCounter++] = (uint16_t)(TP.TSens[i].Rate[0] + TP.TSens[i].Rate[1] * T + TP.TSens[i].Rate[2] * T * T); //Температура с датчика(в попугаях)
            Integr(i);
            TP.TSens[i].T = TP.TSens[i].Tobj * 0.02 - 273.15; //Температура в градусах цельсия
        }
    }

    TP.StatusReg &= ~(0x0F);
    for (int i = 0; i < 4; i++)
    { //Запись ошибок работы датчиков в статус-регистр
        if (TP.TSens[i].ec != 0)
            TP.StatusReg |= (1 << i);
    }
}

//---------------------------------------------
void Integr(uint8_t ch)
{
    uint32_t SUMM = 0;
    for (int i = 0; i < IntegrSize; i++)
    {
        SUMM += TP.TSens[ch].Integr[i];
    }
    //Расчёт среднего:
    TP.TSens[ch].Tobj = SUMM / IntegrSize;
    if (TP.TSens[ch].IntegrCounter >= IntegrSize)
        TP.TSens[ch].IntegrCounter = 0;
}

void CalcAverage(void)
{
    uint32_t SUMM = 0;
    uint8_t counter = 0;
    for (int i = 0; i < 4; i++)
    {
        if (TP.TSens[i].ec == 0)
        {
            SUMM += TP.TSens[i].Tobj;
            counter++;
        }
    }
    //Расчёт среднего:
    TP.Tavg = SUMM / counter;
}

//------------Датчик воды----------------
//StatusFlags flag: номер флага
//uint8_t value: Значение (0 или 1)
void SetStatusRegFlag(StatusFlags flag, uint8_t value)
{
    if (value)
        TP.StatusReg |= (flag);
    else
        TP.StatusReg &= ~(flag);
}
#include "DataTypes.h"
#include "Memory.h"

termoprobe TP;

/*
Стартовая инициализация данных
*/
const float DefaultCharacteristic[3] = {0, 1, 0};
void DataInit(void)
{
  uint32_t FirstStartFlag; //Флаг первого запуска

  FirstStartFlag = ReadFromEEPROM(PMM_FirstStart);
  if (FirstStartFlag == 0)
  { //Первый запуск прибора
    FirstStartFlag = 0xFF;
    WriteToEEPROM(FirstStartFlag, PMM_FirstStart);
    WriteToEEPROM(1, PMM_Serial);
    for (int i = 0; i < 4; i++)
    {
      SetCharacteristic(i, (float *)DefaultCharacteristic);
    }
  }
  else
  {
    for (int i = 0; i < 4; i++)
    {
      ReadCharacteristic(i);
    }
  }
  TP.SerialNumber = ReadFromEEPROM(PMM_Serial);

  TP.Mode = ReadParameter;
}

/*
Инициализация программных I2C
*/
void Multi_I2Cs_Init(void)
{
  TP.TSens[0].I2C.SCL_PIN = I2Cs_SCL_Pin;
  TP.TSens[0].I2C.SCL_PORT = I2Cs_SCL_GPIO_Port;
  TP.TSens[0].I2C.SDA_PIN = I2Cs_SDA0_Pin;
  TP.TSens[0].I2C.SDA_PORT = I2Cs_SDA0_GPIO_Port;

  TP.TSens[1].I2C.SCL_PIN = I2Cs_SCL_Pin;
  TP.TSens[1].I2C.SCL_PORT = I2Cs_SCL_GPIO_Port;
  TP.TSens[1].I2C.SDA_PIN = I2Cs_SDA1_Pin;
  TP.TSens[1].I2C.SDA_PORT = I2Cs_SDA1_GPIO_Port;

  TP.TSens[2].I2C.SCL_PIN = I2Cs_SCL_Pin;
  TP.TSens[2].I2C.SCL_PORT = I2Cs_SCL_GPIO_Port;
  TP.TSens[2].I2C.SDA_PIN = I2Cs_SDA2_Pin;
  TP.TSens[2].I2C.SDA_PORT = I2Cs_SDA2_GPIO_Port;

  TP.TSens[3].I2C.SCL_PIN = I2Cs_SCL_Pin;
  TP.TSens[3].I2C.SCL_PORT = I2Cs_SCL_GPIO_Port;
  TP.TSens[3].I2C.SDA_PIN = I2Cs_SDA3_Pin;
  TP.TSens[3].I2C.SDA_PORT = I2Cs_SDA3_GPIO_Port;
}

//Установка новых значений характеристики
uint8_t SetCharacteristic(uint8_t number, float *rates)
{
  TP.TSens[number].Rate[0] = *(rates + 0);
  TP.TSens[number].Rate[1] = *(rates + 1);
  TP.TSens[number].Rate[2] = *(rates + 2);
  return WriteCharacteristic(number);
}

void GetCharacteristic(uint8_t number, float *rates)
{
  *(rates + 0) = TP.TSens[number].Rate[0];
  *(rates + 1) = TP.TSens[number].Rate[1];
  *(rates + 2) = TP.TSens[number].Rate[2];
}

//------режим работы прибора---------
void SetDeviceMode(Modes mode)
{
  TP.Mode = mode;
}

Modes GetDeviceMode()
{
  return TP.Mode;
}
//------Серийный номер---------
void SetSerial(uint32_t serial)
{
  TP.SerialNumber = serial;
  WriteToEEPROM(TP.SerialNumber, PMM_Serial);
}

uint32_t GetSerial()
{
  return TP.SerialNumber;
}

//------Измерения----------------
uint16_t GetTAvg()
{
  return TP.Tavg;
}

uint8_t GetStatusRegister()
{
  return TP.StatusReg;
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
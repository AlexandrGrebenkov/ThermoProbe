#include "DataTypes.h"
#include "gpio.h"
#include "Memory.h"

termoprobe TP;

void Multi_I2Cs_Init(void)
{
  TP.TSens[0].I2C.SCL_PIN = I2Cs_SCL_Pin;
  TP.TSens[0].I2C.SCL_PORT = I2Cs_SCL_GPIO_Port;
  TP.TSens[0].I2C.SDA_PIN = I2Cs_SDA0_Pin;
  TP.TSens[0].I2C.SDA_PORT = I2Cs_SDA0_GPIO_Port;
/*  SWAP-PINs
  TP.TSens[0].I2C.SCL_PIN = I2Cs_SDA0_Pin;
  TP.TSens[0].I2C.SCL_PORT = I2Cs_SDA0_GPIO_Port;
  TP.TSens[0].I2C.SDA_PIN = I2Cs_SCL_Pin;
  TP.TSens[0].I2C.SDA_PORT = I2Cs_SCL_GPIO_Port;*/
  
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

void DataInit(void)
{
  uint32_t FirstStartFlag;
  
  FirstStartFlag = ReadFromEEPROM(PMM_FirstStart);
  if (FirstStartFlag == 0)
  {//Первый запуск прибора
    FirstStartFlag = 0xFF;
    WriteToEEPROM(FirstStartFlag, PMM_FirstStart);
    WriteToEEPROM(1, PMM_Serial);
    for (int i = 0; i < 4; i ++)
    {
      TP.TSens[i].Rate[0] = 0;
      TP.TSens[i].Rate[1] = 1;
      TP.TSens[i].Rate[2] = 0;
      WriteCharacteristic(i);
    }
  }
  else
  {
    for (int i = 0; i < 4; i ++)
    {
      ReadCharacteristic(i);
    }
  }
  TP.SerialNumber = ReadFromEEPROM(PMM_Serial);
  
  TP.Mode = ReadParameter;
}

void Integr(uint8_t ch)
{
  uint32_t SUMM = 0;
  for (int i = 0; i < IntegrSize; i++)
  {
      SUMM += TP.TSens[ch].Integr[i];
  }
  //Расчёт среднего:
  TP.TSens[ch].Tobj = SUMM/IntegrSize;
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
  TP.Tavg = SUMM/counter;
}
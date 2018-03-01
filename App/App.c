#include "DataTypes.h"
#include "Memory.h"

termoprobe TP; //Структура всего прибора

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

//Получить ссылку на буфер приёма
uint8_t *GetUARTBuffer()
{
  return TP.UART.RxBuf;
}
//Добавить байт в буффер приёма
void AddToRxBuffer(uint8_t data)
{
  if (TP.UART.Counter == RxBuffSize - 1)
    ResetRxBuffer();
  TP.UART.RxBuf[TP.UART.Counter++] = data;
}
//Очистить буфер приёма
void ResetRxBuffer()
{
  TP.UART.Counter = 0;
  TP.UART.RxBuf[0] = 0;
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

void GetTSensorData(uint8_t number, uint16_t* TObj, uint8_t* ec)
{
  *TObj = TP.TSens[number].Tobj;
  *ec = TP.TSens[number].ec;
}

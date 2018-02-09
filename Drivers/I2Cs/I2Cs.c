#include "I2Cs.h"

#define SET_SDA     HAL_GPIO_WritePin(I2Cs->SDA_PORT, I2Cs->SDA_PIN, GPIO_PIN_SET)
#define CLR_SDA     HAL_GPIO_WritePin(I2Cs->SDA_PORT, I2Cs->SDA_PIN, GPIO_PIN_RESET)
#define REL_SDA     SET_SDA
#define GET_SDA     HAL_GPIO_ReadPin(I2Cs->SDA_PORT, I2Cs->SDA_PIN)

#define SET_SCL     HAL_GPIO_WritePin(I2Cs->SCL_PORT, I2Cs->SCL_PIN, GPIO_PIN_SET)
#define CLR_SCL     HAL_GPIO_WritePin(I2Cs->SCL_PORT, I2Cs->SCL_PIN, GPIO_PIN_RESET)
#define REL_SCL     SET_SCL

//---------Задержки-----------------------
void i2csDelay(void)
{
  //HAL_Delay(1);
  for (uint32_t i = 0; i < 10; i++)
  {
    asm("NOP");
  }
}

void i2csSCL_low (void) // 5us
{
  i2csDelay();
}

void i2csSCL_high (void) // 5us
{
  i2csDelay();
}
//_______________________________________



void I2CsStart(i2cs* I2Cs)
{
  //CLR_SCL;
  SET_SCL;
  i2csDelay();
  
  CLR_SDA;
  
  i2csDelay();
  
  CLR_SCL;
  i2csDelay();
  //SET_SDA;
  //SET_SCL;
}

void I2CsStop(i2cs* I2Cs)
{
  CLR_SCL;
  i2csDelay();
  
  CLR_SDA;
  SET_SCL;
  i2csDelay();
  
  SET_SDA;
  i2csDelay();
}

void i2csReset(i2cs* I2Cs)
{
  uint32_t i;
  
  I2CsStart(I2Cs);
  
  i = 9;
  do
  {
    CLR_SCL;
    i2csSCL_low();
    SET_SCL;
    i2csSCL_high();
  }
  while (--i);
  
  CLR_SCL;
  
  I2CsStart(I2Cs);
  I2CsStop(I2Cs);
}


uint32_t i2csWriteByte(i2cs* I2Cs, uint8_t data)
{
  uint32_t i, ack;
  
  // передача байта
  i = 8;
  do
  {
    CLR_SCL;
    
    if (data & 0x80)
      SET_SDA;
    else
      CLR_SDA;
    
    data <<= 1;
    
    i2csSCL_high();
    SET_SCL;
    i2csSCL_low();
  }
  while (--i);
  
  CLR_SCL;
  
  // ожидание ack 
  REL_SDA;
  i2csDelay();
  SET_SCL;

  i = 10;
  ack = 1;
  do
  {
    i2csDelay();

    if ((ack = GET_SDA) == 0)
      break;
  }
  while (--i);
  
  CLR_SCL;
  i2csDelay();

  return ack;   
}

uint32_t i2csReadByte(i2cs* I2Cs, uint32_t set_ack)
{
  uint32_t i;
  uint8_t data = 0x00;

  // приём байта
  REL_SDA;
  
  i = 8;
  do
  {
    CLR_SCL;
    
    data <<= 1;
    
    i2csSCL_low();
    SET_SCL;
    i2csSCL_high();

    if (GET_SDA == 1)
      data |= 0x01;
  }
  while (--i);
  
  CLR_SCL;
  
  // выдать ack?
  if (set_ack != 0)
    CLR_SDA;    
  else
    SET_SDA;
  
  i2csDelay();
  SET_SCL;
  i2csSCL_high();
  CLR_SCL;
  i2csSCL_low();
  
  return data;
}

uint32_t i2csDummyWrite(i2cs* I2Cs, uint8_t *pBuf, uint32_t addr, uint32_t size)
{
  uint32_t i, ack;
  
  // передача адреса устройства
  i = 1;
  do    
  {
    I2CsStart(I2Cs);
    ack = i2csWriteByte(I2Cs, (addr << 1) | 0x00);
    if (ack == 0)
      break;
    else
      I2CsStop(I2Cs);
  }
  while (--i);
  
  if (ack != 0)
  {
    I2Cs->ErrorCode = I2CS_NO_ADDR_ACK_ERROR;
    return I2Cs->ErrorCode;
  }

  // передача адреса в памяти
  if (I2Cs->SlaveAddress != 0)
  {
    for (i = 0; i < I2Cs->SlaveAddress; i++)
    {
      if (i2csWriteByte(I2Cs, addr >> 8*(I2Cs->SlaveAddress - i - 1)) != 0)
      {
        I2Cs->ErrorCode = I2CS_NO_ACK_ERROR;
        return I2Cs->ErrorCode;        
      }          
    }
  }
  
  // передача данных
  if (size != 0)
  {
    do
    {
      if (i2csWriteByte(I2Cs, *pBuf++) != 0)
      {
        I2Cs->ErrorCode = I2CS_NO_ACK_ERROR;
        return I2Cs->ErrorCode;
      }
    }
    while (--size);
  }
  
  I2Cs->ErrorCode = I2CS_NO_ERROR;
  return I2CS_NO_ERROR; 
}


uint32_t i2csWrite(i2cs* I2Cs, uint8_t *pBuf, uint32_t addr, uint32_t size)
{
  uint32_t ec = i2csDummyWrite(I2Cs, pBuf, addr, size);

  I2CsStop(I2Cs);

  return ec; 
}

uint32_t i2csRead(i2cs* I2Cs, uint8_t *pBuf, uint32_t addr, uint32_t size)
{
  // dummy write
  uint32_t ec = i2csDummyWrite(I2Cs, pBuf, addr, 1);
  if (ec != 0)
    return ec;
  // передача адреса устройства
  I2CsStart(I2Cs);
  if (i2csWriteByte(I2Cs, (addr << 1) | 0x01) != 0)
  {
    I2Cs->ErrorCode = I2CS_NO_ACK_ERROR;
    I2CsStop(I2Cs);
    return I2Cs->ErrorCode;
  }
  
  // чтение данных
  if (size != 0)
  {
    do
    {
      *pBuf++ = i2csReadByte(I2Cs, (size != 1) ? 1 : 0);
    }
    while (--size);
  }
  
  I2CsStop(I2Cs);
  
  I2Cs->ErrorCode = I2CS_NO_ERROR;
  return I2CS_NO_ERROR; 
}

//Тестовая функция для проверки I2C
uint32_t i2csTest(i2cs* I2Cs)
{
  uint32_t i, ack;
  
  // передача адреса устройства
  i = 1000;
  do{
    I2CsStart(I2Cs);
    ack = i2csWriteByte(I2Cs, (I2Cs->SlaveAddress << 1) | 0x01);
    if(!ack) break; 
    else     I2CsStop(I2Cs);  
  }while (--i);
  I2CsStop(I2Cs);
  
  I2Cs->ErrorCode = (ack != 0) ? I2CS_NO_ADDR_ACK_ERROR : I2CS_NO_ERROR;
  return I2Cs->ErrorCode;
}

// I2C Error strings
static const char i2csNoErrorText[]        = { "\x08" "NO_ERROR" };
static const char i2csNoAddrAckErrorText[] = { "\x0B" "NO_ADDR_ACK" };
static const char i2csNoAckErrorText[]     = { "\x06" "NO_ACK" };

static const char *const pi2csErrorText[] = {
  i2csNoErrorText,         // 0
  i2csNoAddrAckErrorText,
  i2csNoAckErrorText       // 2
};

const char *i2csGetErrorName (uint32_t AErrorCode)
{
  if (AErrorCode > sizeof(pi2csErrorText)/sizeof(uint16_t) - 1)
  {
    AErrorCode = 0;
  }

  return pi2csErrorText[AErrorCode];
}






















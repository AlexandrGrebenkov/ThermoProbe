#ifndef __BUILD_DEF_H
#define __BUILD_DEF_H

//Структура для бутлоадера
typedef struct{                      //offset | len | example            | description
  uint8_t StructVer[8];              //  00   |  04 | 1.00               | Версия структуры
  uint8_t HardwareVer[30];           //  04   |  30 | PAB-1.1 (PAB-3.02) | Название прибора и его аппаратная версия
  uint8_t SoftDate[12];              //  34   |  10 | 2017.06.09         | Дата создания прошивки
  uint8_t Empty[78];
} TBootloadInfo;

//Макросы для формирования строк версии прошивки
#define BUILD_YEAR0 (__DATE__[ 7])
#define BUILD_YEAR1 (__DATE__[ 8])
#define BUILD_YEAR2 (__DATE__[ 9])
#define BUILD_YEAR3 (__DATE__[10])

#define BUILD_MONTH_IS_JAN (__DATE__[0] == 'J' && __DATE__[1] == 'a' && __DATE__[2] == 'n')
#define BUILD_MONTH_IS_FEB (__DATE__[0] == 'F')
#define BUILD_MONTH_IS_MAR (__DATE__[0] == 'M' && __DATE__[1] == 'a' && __DATE__[2] == 'r')
#define BUILD_MONTH_IS_APR (__DATE__[0] == 'A' && __DATE__[1] == 'p')
#define BUILD_MONTH_IS_MAY (__DATE__[0] == 'M' && __DATE__[1] == 'a' && __DATE__[2] == 'y')
#define BUILD_MONTH_IS_JUN (__DATE__[0] == 'J' && __DATE__[1] == 'u' && __DATE__[2] == 'n')
#define BUILD_MONTH_IS_JUL (__DATE__[0] == 'J' && __DATE__[1] == 'u' && __DATE__[2] == 'l')
#define BUILD_MONTH_IS_AUG (__DATE__[0] == 'A' && __DATE__[1] == 'u')
#define BUILD_MONTH_IS_SEP (__DATE__[0] == 'S')
#define BUILD_MONTH_IS_OCT (__DATE__[0] == 'O')
#define BUILD_MONTH_IS_NOV (__DATE__[0] == 'N')
#define BUILD_MONTH_IS_DEC (__DATE__[0] == 'D')

#define BUILD_MONTH0  ((BUILD_MONTH_IS_OCT || BUILD_MONTH_IS_NOV || BUILD_MONTH_IS_DEC) ? '1' : '0')
#define BUILD_MONTH1  ( \
  (BUILD_MONTH_IS_JAN) ? '1' : \
  (BUILD_MONTH_IS_FEB) ? '2' : \
  (BUILD_MONTH_IS_MAR) ? '3' : \
  (BUILD_MONTH_IS_APR) ? '4' : \
  (BUILD_MONTH_IS_MAY) ? '5' : \
  (BUILD_MONTH_IS_JUN) ? '6' : \
  (BUILD_MONTH_IS_JUL) ? '7' : \
  (BUILD_MONTH_IS_AUG) ? '8' : \
  (BUILD_MONTH_IS_SEP) ? '9' : \
  (BUILD_MONTH_IS_OCT) ? '0' : \
  (BUILD_MONTH_IS_NOV) ? '1' : \
  (BUILD_MONTH_IS_DEC) ? '2' : \
  /* error default */    '?' \
)

#define BUILD_DAY0    ((__DATE__[4] >= '0') ? (__DATE__[4]) : '0')
#define BUILD_DAY1    (__DATE__[ 5])

#endif
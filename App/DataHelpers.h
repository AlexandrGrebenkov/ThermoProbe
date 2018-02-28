#ifndef __DataHelpers_H
#define __DataHelpers_H

#include <stdint.h>

//Макросы для разбития 16-битного значения на байты
#define LO(x) (uint8_t)((x >> 0) & 0xFF);
#define HI(x) (uint8_t)((x >> 8) & 0xFF);

//Конвертор 32-битных данных
typedef union {
    uint8_t b[4];
    float f;        
    uint32_t l32;
} converter;

#endif
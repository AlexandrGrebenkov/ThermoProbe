#ifndef __DataHelpers_H
#define __DataHelpers_H

#include <stdint.h>

#define LO(x) (uint8_t)((x >> 0) & 0xFF);
#define HI(x) (uint8_t)((x >> 8) & 0xFF);

typedef union {
    uint8_t b[4];
    float f;
    uint32_t l32;
} converter;

#endif
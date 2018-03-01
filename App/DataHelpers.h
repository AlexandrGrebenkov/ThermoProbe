#ifndef __DataHelpers_H
#define __DataHelpers_H

#include <stdint.h>

//������� ��� �������� 16-������� �������� �� �����
#define LO(x) (uint8_t)((x >> 0) & 0xFF);
#define HI(x) (uint8_t)((x >> 8) & 0xFF);

//��������� 32-������ ������
typedef union {
    uint8_t b[4];
    float f;        
    uint32_t l32;
} converter;

#endif
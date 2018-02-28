#ifndef __MEASURE_H
#define __MEASURE_H

#include "App.h"

void TemperaturMeasument();
void CalcAverage(void);
void Integr(uint8_t ch);

void SetStatusRegFlag(StatusFlags flagPosition, uint8_t value);

#endif
#ifndef TRIG_LUTS
#define TRIG_LUTS

#define LUT_LENGTH 256

#include "main.h"

uint8_t mult_sin_lut_uint8(uint8_t value, uint8_t position);
int16_t mult_sin_lut_int16(uint8_t position, int16_t value);

#endif
#pragma once
#include "main.h"

// PWM output is 10-bit
#define LUT_LENGTH 1024
#define LUT_DEPTH 1024

#define PI_F 3.14159274f
#define PI2_F 2.0f * 3.14159274f

#define SQRT_1_2 0.7071067812f
#define SQRT_2_3 0.8164965809f
#define SQRT_3 1.7320508076f

namespace trig{

extern int16_t sin_lut[LUT_LENGTH];

void initialize();
int mult_sin(int, int);
int mult_cos(int, int);

}
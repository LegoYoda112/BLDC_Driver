#pragma once
#include "main.h"

namespace utils
{

int bound_int(int value, int min, int max);
int8_t bound_int8(int8_t value, int8_t min, int8_t max);
float fbound(float value, float min, float max);
float fbound_sym(float value, float max);

}
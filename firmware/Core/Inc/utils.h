#ifndef UTILS_H
#define UTILS_H

#include "main.h"
#include "stdbool.h"

// Returns the minimum of 3 provided int16s
int16_t int16_min3(int16_t a, int16_t b, int16_t c);

// Returns an integer bounded by a min and max
int bound(int value, int min, int max);

// Bounds an integer between a min and max, returns true if bounded
bool enforce_bound(int *value, int min, int max);

#endif
#pragma once
#include "drive.h"

namespace foc
{

int convert_to_electrical_angle(int, int8_t, uint16_t);

void make_voltage_at_electrical_angle(int16_t, int16_t, uint16_t, struct drive::PhaseVoltages *phase_voltages);
void clarke_transform(int A, int B, int C, int *alpha, int *beta);
void park_transform(int alpha, int beta, int angle, int *d, int *q);
void inverse_clarke_transform(int alpha, int beta, int *a, int *b, int *c);
void inverse_park_transform(int d, int q, int angle, int *alpha, int *beta);

}
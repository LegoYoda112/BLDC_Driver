#ifndef DRIVE_H
#define DRIVE_H

#include "main.h"
#include "cmsis_os.h"
#include "math.h"

#define DRIVE_FREQ 1000
#define PI 3.1415926535897932384626433832795

extern int enc_angle_int;
extern uint16_t enc_angle_uint12;
extern uint16_t elec_angle_uint16;
extern uint16_t target_elec_angle;
extern int elec_angle;
extern float angle;

void foc_interrupt();
void enable_foc_loop();
void disable_foc_loop();
void calc_elec_angle();

#endif
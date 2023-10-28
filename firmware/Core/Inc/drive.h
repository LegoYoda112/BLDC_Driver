#ifndef DRIVE_H
#define DRIVE_H

#include "main.h"
#include "cmsis_os.h"
#include "math.h"
#include "stdbool.h"
#include "sensors.h"
#include "trig_luts.h"

#define DRIVE_FREQ 1000
#define PI 3.1415926535897932384626433832795

extern int enc_angle_int;
extern int target_encoder_value;
extern uint16_t electrical_angle;
extern uint16_t electrical_angle_offset;
extern uint8_t angle;

extern uint16_t encoder_zeros[10];

extern TIM_HandleTypeDef htim7;

void foc_interrupt();
void enable_foc_loop();
void disable_foc_loop();
void calc_elec_angle();

#endif
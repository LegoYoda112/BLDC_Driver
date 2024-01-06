#ifndef DRIVE_H
#define DRIVE_H

#include "main.h"
#include "sensors.h"

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim4;

void start_drive_PWM();

void enable_DRV();
void disable_DRV();

void set_duty_phase_A(uint8_t value);
void set_duty_phase_B(uint8_t value);
void set_duty_phase_C(uint8_t value);

#endif
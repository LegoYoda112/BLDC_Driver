#pragma once
#include "stm32g4xx_hal.h"
#include "tim.h"

#include "analog.h"

namespace drive
{

void initialize();
void enable();
void enable_low_side();
void disable();
void disable_low_side();

void commutation_interrupt();

void set_phaseA_duty(uint16_t);
void set_phaseB_duty(uint16_t);
void set_phaseC_duty(uint16_t);

} // drive
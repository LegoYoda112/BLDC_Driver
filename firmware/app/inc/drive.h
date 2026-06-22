#pragma once
#include "stm32g4xx_hal.h"
#include "tim.h"

#include "analog.h"

#define PHASE_DUTY_MAX 1024.0f

namespace drive
{

void initialize();
void enable();
void enable_low_side();
void disable();
void disable_low_side();

void apply_phaseA_duty(uint16_t);
void apply_phaseB_duty(uint16_t);
void apply_phaseC_duty(uint16_t);
void apply_phase_duties(uint16_t, uint16_t, uint16_t);

struct PhaseVoltages {
    float phaseA_V;
    float phaseB_V;
    float phaseC_V;
};

struct PhaseCurrents {
    float phaseA_mA;
    float phaseB_mA;
    float phaseC_mA;
};

void set_target_phase_voltages(struct drive::PhaseVoltages* target_voltages);
void apply_phase_voltages(PhaseVoltages*);



void commutation_interrupt();

} // drive
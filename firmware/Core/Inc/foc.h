#ifndef FOC_H
#define FOC_H

#include "main.h"
#include "drive.h"
#include "sensors.h"


extern uint16_t vmotor_mV;

extern int16_t current_A_mA;
extern int16_t current_B_mA;
extern int16_t current_C_mA;

extern int16_t current_Alpha_mA;
extern int16_t current_Beta_mA;

extern int16_t angle;


/**
 * @brief Runs required FOC functions on TIM6 interrupt
 * 
 */
void foc_interrupt();

void clarke_transform(int16_t A, int16_t B, int16_t C, int16_t *alpha, int16_t *beta);
void park_transform(int16_t alpha, int16_t beta, uint8_t angle, int16_t *d, int16_t *q);

void inverse_clarke_transform(int16_t alpha, int16_t beta, int16_t *a, int16_t *b, int16_t *c);
void inverse_park_transform(int16_t V_d, int16_t V_q, uint8_t angle, int16_t *V_alpha, int16_t *V_beta);

int16_t min3(int16_t a, int16_t b, int16_t c);

void apply_duty_at_electrical_angle_int(uint8_t angle, uint8_t magnitude);

#endif
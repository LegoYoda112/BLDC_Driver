#ifndef FOC_H
#define FOC_H

#include "main.h"
#include "drive.h"
#include "sensors.h"
#include "math.h"
#include "trig_luts.h"
#include "utils.h"


extern uint16_t vmotor_mV;

extern int16_t current_A_mA;
extern int16_t current_B_mA;
extern int16_t current_C_mA;

extern int16_t current_Alpha_mA;
extern int16_t current_Beta_mA;

extern int16_t angle;

extern uint8_t electrical_angle_offset;

extern int16_t current_offsets[256];

/**
 * @brief Set target current setpoints in rotor space
 * 
 * @param D_setpoint_mA D "normal current"
 * @param Q_setpoint_mA Q "tangent current"
 * 
 * @note In normal operation, D should be set to 0, and Q set to the desired current
 * based on required torque.
 */
void set_current_setpoints(int D_setpoint_mA, int Q_setpoint_mA);

/**
 * @brief Runs required FOC functions on TIM6 interrupt at 10KHz
 * 
 */
void foc_interrupt();

/**
 * @brief Power invariant Clarke transform, converts from a 3-phase coordinate system to orthogonal coordinates
 * (with Alpha in line with phase A)
 * 
 * @param A Phase A value
 * @param B Phase B value
 * @param C Phase C value
 * @param alpha Pointer to Alpha value to be set
 * @param beta Pointer to Beta value to be set
 */
void clarke_transform(int16_t A, int16_t B, int16_t C, int16_t *alpha, int16_t *beta);

/**
 * @brief Park transform, converts from stator-aligned orthogonal coordinates to rotor aligned orthogonal coordinates.
 * D is aligned "normal" and Q is aligned "tangent".
 * 
 * @param alpha Alpha value
 * @param beta Beta value
 * @param angle Electrical angle as a 8-bit integer (255 = 2pi)
 * @param d Pointer to D value to be set
 * @param q Pointer to Q value to be set
 */
void park_transform(int16_t alpha, int16_t beta, uint8_t angle, int16_t *d, int16_t *q);

/**
 * @brief Inverse power invariant Clarke transform. See also `clarke_transform()`
 * 
 * @param alpha Alpha value
 * @param beta Beta value
 * @param a Pointer to A-phase value to be set
 * @param b Pointer to A-phase value to be set
 * @param c Pointer to A-phase value to be set
 * 
 */
void inverse_clarke_transform(int16_t alpha, int16_t beta, int16_t *a, int16_t *b, int16_t *c);

/**
 * @brief Inverse Park transform. See also `park_transform()`
 * 
 * @param d 
 * @param q 
 * @param angle 
 * @param alpha 
 * @param beta 
 */
void inverse_park_transform(int16_t d, int16_t q, uint8_t angle, int16_t *alpha, int16_t *beta);


void apply_duty_at_electrical_angle_int(uint8_t angle, uint8_t magnitude);

#endif
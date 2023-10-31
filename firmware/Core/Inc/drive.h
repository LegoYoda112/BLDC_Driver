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

/**
 * @brief Current encoder angle as an integer
 * 
 * Resolution is 12-bit (1 rev = 4096)
 */
extern int enc_angle_int;

/**
 * @brief Target encoder value set by position control
 * 
 */
extern int target_encoder_value;

/**
 * @brief Current electrical angle as a 9-bit uint.
 * 
 */
extern uint16_t electrical_angle;

/**
 * @brief Electrical angle offset as a 12-bit uint
 * 
 * This represents the physical offset between encoder zero (divided to nearest pole) and electrical zero
 * 
 */
extern uint16_t electrical_angle_offset;
extern uint8_t angle;

extern uint16_t encoder_zeros[10];

extern TIM_HandleTypeDef htim7;

/**
 * @brief Field Oriented Control interrupt function, called at 10 kHz
 * 
 * Runs all required functions to commutate motor
 * 
 */
void foc_interrupt();

/**
 * @brief Enables drive output section of foc_interrupt
 */
void enable_foc_loop();

/**
 * @brief Disables drive output section of foc_interrupt
 */
void disable_foc_loop();
void calc_elec_angle();

#endif
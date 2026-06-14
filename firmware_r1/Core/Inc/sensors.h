#ifndef SENSORS_H
#define SENSORS_H

#include "main.h"
#include "stdbool.h"

#define AMP_GAIN 40.0f
#define SHUNT_VALUE_R 0.002f
#define ADC_MIDPOINT 2048.0f
#define ADC_MAX 4096.0f

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;

extern uint16_t adc1_dma[2];
extern uint16_t adc2_dma[3];

extern uint16_t phase_resistance_mOhm[3];

extern uint16_t v_motor_mv;

/**
 * @brief Current sense 0 offset
 * 
 */
extern uint32_t adc2_calib_offset[3];

/**
 * @brief Current sense in AMPs
 * Call update_current_sense to update
 */
extern float current_sense[3];
extern float alpha_current;
extern float beta_current;

extern int enc_angle_int;
extern uint16_t enc_angle_uint12;

/**
 * @brief 
 * 
 */
void encoder_ISR();

/**
 * @brief Set absolute encoder offset. 
 * Sets the absolute encoder offset by power cycling IC then reading out startup pulses
 */
void set_encoder_absolute_offset();

/**
 * @brief Sets up DMA on ADC channels 2 and 3
 * 
 * 
 */
void start_ADC();

/**
 * @brief Runs calibration on the DRV current shunt amps.
 * Starts by running DRV internal calibration, then sets an external offset
 * 
 */
void calibrate_DRV_amps();

/**
 * @brief Re-calculate current sense (in amps)
 * Uses floating-point so ideally shouldn't be used in the FOC loop
 * 
 */
void update_current_sense();

float get_vmotor();

#endif
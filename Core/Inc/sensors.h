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

extern uint32_t adc1_dma[2];
extern uint32_t adc2_dma[3];
extern uint32_t adc2_calib_offset[3];

extern float current_sense[3];

extern int enc_angle_int;
extern uint16_t enc_angle_uint12;

void encoder_ISR();
void set_encoder_absolute_offset();
void start_ADC_DMA();
void calibrate_DRV_amps();
void update_current_sense();

#endif
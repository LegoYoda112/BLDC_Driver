#pragma once
#include "stm32g4xx_hal.h"
#include "adc.h"

#define CURRENT_FILTER_CONSTANT 0.95f
#define ZERO_FILTER_CONSTANT 0.9f
#define AMP_GAIN 40.0f
#define ADC_MAX 4096.0f
#define CURRENT_CONSTANT_MA AMP_GAIN / 
#define SHUNT_VALUE_R 0.002f

namespace analog
{   

/**
 * @brief Starts DMA
 * 
 */
void initialize();


/**
 * @brief Read the current bus voltage
 * 
 * @return Voltage in volts 
 */
float read_bus_voltage_v();

/**
 * @brief Gets the cached bus voltage from the last read
 * 
 *
 */
float get_cached_bus_voltage_v();


/**
 * @brief Calibrate DRV current sense amps
 * 
 */
void calibrate_current_sense();

/**
 * @brief Updates the IIR filter on phase current measurements
 * 
 */
void update_current_sense();

float get_phase_A_current_mA();
float get_phase_B_current_mA();
float get_phase_C_current_mA();

} // analog
#include "analog.h"

using namespace analog;

uint16_t adc1_dma[3];
uint16_t adc2_dma[3];
uint16_t adc2_dma_offset_measure[3];
uint16_t adc2_dma_offset[3];
float current_sense[3];

bool zero_current_sense = false;

void analog::initialize(){
    HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
    HAL_ADC_Start_DMA(&hadc1, reinterpret_cast<uint32_t*>(adc1_dma), 3);

    HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);
    HAL_ADC_Start_DMA(&hadc2, reinterpret_cast<uint32_t*>(adc2_dma), 3);

    analog::calibrate_current_sense();
}

float analog::get_bus_voltage_v(){
    return adc1_dma[0] * (0.000806f / 0.0637f) * 1.006f;
}

void analog::calibrate_current_sense(){
    // Perform DRV amp calibration
    HAL_GPIO_WritePin(DRV_CAL_GPIO_Port, DRV_CAL_Pin, (GPIO_PinState)(0));
    HAL_Delay(2);
    HAL_GPIO_WritePin(DRV_CAL_GPIO_Port, DRV_CAL_Pin, (GPIO_PinState)(1));
    // Wait for internal calibration
    HAL_Delay(20);

    // Zero out offsets
    adc2_dma_offset[0] = 0;
    adc2_dma_offset[1] = 0;
    adc2_dma_offset_measure[0] = 2048;
    adc2_dma_offset_measure[1] = 2048;
    // Measure new offset
    zero_current_sense = true;
    HAL_Delay(10);

    // Apply new offset and disable drv calibration
    zero_current_sense = false;
    adc2_dma_offset[0] = adc2_dma_offset_measure[0] + 1; // unsure why these constant offsets are needed,.
    adc2_dma_offset[1] = adc2_dma_offset_measure[1] + 4;


    HAL_GPIO_WritePin(DRV_CAL_GPIO_Port, DRV_CAL_Pin, (GPIO_PinState)(0));
}

void analog::update_current_sense(){
    // Run IIR filter on current
    // TODO: Make this less of a mess
    current_sense[0] = current_sense[0] * CURRENT_FILTER_CONSTANT + ((adc2_dma[0] - adc2_dma_offset[0])) * (1.0f - CURRENT_FILTER_CONSTANT);
    current_sense[1] = current_sense[1] * CURRENT_FILTER_CONSTANT + ((adc2_dma[1] - adc2_dma_offset[1])) * (1.0f - CURRENT_FILTER_CONSTANT);
    current_sense[2] = - current_sense[0] - current_sense[1];

    if(zero_current_sense){
        adc2_dma_offset_measure[0] = adc2_dma_offset_measure[0] * ZERO_FILTER_CONSTANT + ((adc2_dma[0])) * (1.0f - ZERO_FILTER_CONSTANT);
        adc2_dma_offset_measure[1] = adc2_dma_offset_measure[1] * ZERO_FILTER_CONSTANT + ((adc2_dma[1])) * (1.0f - ZERO_FILTER_CONSTANT);
    }
}

float analog::get_phase_A_current_mA(){
    return current_sense[0] * 9;
}

float analog::get_phase_B_current_mA(){
    return current_sense[1] * 9;
}

float analog::get_phase_C_current_mA(){
    return current_sense[2] * 9;
}
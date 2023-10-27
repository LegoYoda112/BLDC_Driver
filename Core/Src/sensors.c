#include "sensors.h"

uint32_t adc1_dma[2];
uint32_t adc2_dma[3];
uint32_t adc2_calib_offset[3];

float current_sense[3];

int enc_angle_int;
uint16_t enc_angle_uint12;

bool start_up_pulses = false;
uint16_t start_up_pulse_count = 0;

void encoder_ISR(){
  if(start_up_pulses || HAL_GPIO_ReadPin(IFB_GPIO_Port, IFB_Pin)){
    enc_angle_int ++;
  }else{
    enc_angle_int --;
  }
}

void set_encoder_absolute_offset(){
  // TODO: This is very very hacky and I do not like it

  // GET PULSE COUNT
  // Disable magnetic encoder
  start_up_pulses = true;
  HAL_GPIO_WritePin(ENC_EN_GPIO_Port, ENC_EN_Pin, 1);
  HAL_Delay(10);
  // Reset angle
  enc_angle_int = 0;
  // Enable magnetic encoder
  HAL_GPIO_WritePin(ENC_EN_GPIO_Port, ENC_EN_Pin, 0);
  HAL_Delay(10);
  start_up_pulse_count = enc_angle_int;

  // FIND DIRECTION
  start_up_pulses = false;
  HAL_GPIO_WritePin(ENC_EN_GPIO_Port, ENC_EN_Pin, 1);
  HAL_Delay(10);
  // Reset angle
  enc_angle_int = 0;
  // Enable magnetic encoder
  HAL_GPIO_WritePin(ENC_EN_GPIO_Port, ENC_EN_Pin, 0);
  HAL_Delay(10);
  // Adjust startup pulse direction
  if(enc_angle_int < 0){
    enc_angle_int = start_up_pulse_count;
  }else{
    enc_angle_int = -start_up_pulse_count;
  }


}

void start_ADC_DMA(){
  // Start adc1 DMA (vmot + temp)
  HAL_ADC_Start_DMA(&hadc1, adc1_dma, 2);

  // Start adc2 DMA (phase current shunts)
  HAL_ADC_Start_DMA(&hadc2, adc2_dma, 3);
}

// void get_vmot(){
//   return adc1_dma,
// }

void calibrate_DRV_amps(){
    // Perform DRV amp calibration
    HAL_GPIO_WritePin(DRV_CAL_GPIO_Port, DRV_CAL_Pin, 1);
    // Wait for internal calibration
    HAL_Delay(10);

    // Perform external calibration
    // TODO: Have average over a set of samples
    adc2_calib_offset[0] = adc2_dma[0];
    adc2_calib_offset[1] = adc2_dma[1];
    adc2_calib_offset[2] = adc2_dma[2];

    // Disable calibration
    HAL_GPIO_WritePin(DRV_CAL_GPIO_Port, DRV_CAL_Pin, 0);
}

void update_current_sense(){
    // TODO: Set up proper phase numbering
    current_sense[0] = ((((int)adc2_dma[0] - (int)adc2_calib_offset[0]) / ADC_MAX) * 3.3) / (AMP_GAIN * SHUNT_VALUE_R);
    current_sense[1] = ((((int)adc2_dma[1] - (int)adc2_calib_offset[1]) / ADC_MAX) * 3.3) / (AMP_GAIN * SHUNT_VALUE_R);
    current_sense[2] = ((((int)adc2_dma[2] - (int)adc2_calib_offset[2]) / ADC_MAX) * 3.3) / (AMP_GAIN * SHUNT_VALUE_R);
}
#include "drive.h"

void start_drive_PWM(){
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
}

void enable_DRV(){
    HAL_GPIO_WritePin(DRV_EN_GPIO_Port, DRV_EN_Pin, 1);
}

void disable_DRV(){
    HAL_GPIO_WritePin(DRV_EN_GPIO_Port, DRV_EN_Pin, 0);
}


void set_duty_phase_A(uint8_t value){
    TIM1->CCR3 = value;
}

void set_duty_phase_B(uint8_t value){
    TIM1->CCR2 = value;
}

void set_duty_phase_C(uint8_t value){
    TIM1->CCR1 = value;
}
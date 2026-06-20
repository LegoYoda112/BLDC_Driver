#include "drive.h"

using namespace drive;

void drive::initialize(){
    drive::enable();

    // Start phase PWM timers
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);

    // 10kHz commutation interrupt
    HAL_TIM_Base_Start_IT(&htim6);
}

void drive::commutation_interrupt(){
    analog::update_current_sense();
}

void drive::enable(){
    HAL_GPIO_WritePin(DRV_EN_GPIO_Port, DRV_EN_Pin, (GPIO_PinState) 1);
}

void drive::enable_low_side(){
    HAL_GPIO_WritePin(INLX_GPIO_Port, INLX_Pin, (GPIO_PinState) 1);
}

void drive::disable(){
    HAL_GPIO_WritePin(DRV_EN_GPIO_Port, DRV_EN_Pin, (GPIO_PinState) 0);
}

void drive::disable_low_side(){
    HAL_GPIO_WritePin(INLX_GPIO_Port, INLX_Pin, (GPIO_PinState) 1);
}


void drive::set_phaseA_duty(uint16_t value){
    TIM1->CCR3 = value;
}

void drive::set_phaseB_duty(uint16_t value){
    TIM1->CCR2 = value;
}

void drive::set_phaseC_duty(uint16_t value){
    TIM1->CCR1 = value;
}
#include "drive.h"

using namespace drive;

struct PhaseVoltages phase_voltages;

void drive::initialize(){
    drive::enable();

    // Start phase PWM timers
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);

    // 10kHz commutation interrupt
    HAL_TIM_Base_Start_IT(&htim6);
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
    HAL_GPIO_WritePin(INLX_GPIO_Port, INLX_Pin, (GPIO_PinState) 0);
}

void drive::apply_phaseA_duty(uint16_t value){
    TIM1->CCR3 = value;
}

void drive::apply_phaseB_duty(uint16_t value){
    TIM1->CCR2 = value;
}

void drive::apply_phaseC_duty(uint16_t value){
    TIM1->CCR1 = value;
}

void drive::set_target_phase_voltages(struct drive::PhaseVoltages* target_voltages){
    phase_voltages = *target_voltages;
}

void drive::apply_phase_duties(uint16_t value_A, uint16_t value_B, uint16_t value_C){
    //TODO: clip voltages here
    TIM1->CCR3 = value_A;
    TIM1->CCR2 = value_B;
    TIM1->CCR1 = value_C;
}

void drive::apply_phase_voltages(struct drive::PhaseVoltages* voltage){
    float voltage_to_duty = PHASE_DUTY_MAX / analog::get_cached_bus_voltage_v();

    apply_phase_duties(
        voltage->phaseA_V * voltage_to_duty,
        voltage->phaseB_V * voltage_to_duty,
        voltage->phaseC_V * voltage_to_duty
    );
}


void drive::commutation_interrupt(){

    // TODO: Clip phase voltages
    drive::apply_phase_voltages(&phase_voltages);
    analog::update_current_sense();
}

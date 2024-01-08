#include "drive.h"

enum DriveError drive_error = drive_error_none;
enum DriveState drive_state = drive_state_idle;

// If true, FOC is allowed to control motor phases
bool foc_active = false;

//////////// VARIABLES
int max_motor_current_mAmps = 1000;

int estimated_resistance_mOhms = -1;






void start_drive_timers(){
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

void enable_foc_loop(){
    foc_active = true;
}

void disable_foc_loop(){
    foc_active = false;
    set_duty_phases(0, 0, 0);
}





// Currently only checks phase A as that amp seems to be the most accurate
void estimate_phase_resistance(float voltage){
    if(drive_state != drive_state_idle) {
        return;
    }

    // Verify we have motor supply voltage
    if(check_supply_voltage() != drive_error_none){
        return;
    }

    disable_foc_loop();
    drive_state = drive_state_resistance_estimation;

    // Calculate required duty cycle for voltage
    float input_voltage = get_vmotor();
    float desired_voltage = voltage;
    int duty = (int)((desired_voltage / input_voltage) * 256);

    // Apply voltage
    set_duty_phases(0, duty, 0);
    osDelay(50);

    // Sample current
    int current_adc = 0;
    const int NUM_SAMPLES = 10;
    for(int i = 0; i < 10; i++){
        current_adc += adc2_dma[0];
        osDelay(1);
    }

    current_adc = current_adc / NUM_SAMPLES;

    // Reset phases
    set_duty_phases(0, 0, 0);

    float measured_shunt_voltage = ((current_adc / 4096.0 - 0.5) * 2.8f) / 40.0f;
    float measured_shunt_current = fabs(measured_shunt_voltage / 0.002f);
    
    // Calculate resistance
    estimated_resistance_mOhms = (int) (desired_voltage / measured_shunt_current * 1000);

                

    // Check if resistance is within expected values
    if(estimated_resistance_mOhms > MAX_MOTOR_RESISTANCE_OHM * 1000){
        drive_state = drive_state_error;
        drive_error = drive_error_high_resistance;
        return;
    }else if(estimated_resistance_mOhms < MIN_MOTOR_RESISTANCE_OHM * 1000){
        drive_state = drive_state_error;
        drive_error = drive_error_high_resistance;
        return;
    } else {
        drive_state = drive_state_idle;
        enable_foc_loop();
    }
}





enum DriveError check_supply_voltage() {
    if(get_vmotor() < MIN_SUPPLY_VOLTAGE_V){
        drive_error = drive_error_no_supply;
        drive_state = drive_state_error;
    }

    return drive_error;
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

void set_duty_phases(uint8_t A_value, uint8_t B_value, uint8_t C_value){
    set_duty_phase_A(A_value);
    set_duty_phase_B(B_value);
    set_duty_phase_C(C_value);
}
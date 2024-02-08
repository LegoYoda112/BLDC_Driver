#include "drive.h"

enum DriveError drive_error = drive_error_none;
enum DriveState drive_state = drive_state_disabled;

// If true, FOC is allowed to control motor phases
bool foc_active = false;

//////////// VARIABLES
int max_motor_current_mAmps = 1000;

int estimated_resistance_mOhms = -1;

int16_t current_offsets[256];




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
    HAL_GPIO_WritePin(INLX_GPIO_Port, INLX_Pin, 1);
}

void disable_foc_loop(){
    foc_active = false;
    set_duty_phases(0, 0, 0);
    HAL_GPIO_WritePin(INLX_GPIO_Port, INLX_Pin, 0);
}





void enter_drive_error(enum DriveError error){
    drive_error = error;
    drive_state = drive_state_error;
    disable_foc_loop();
}

void drive_state_machine(){

    switch(drive_state){
        case drive_state_disabled:
            disable_foc_loop();
            break;
        case drive_state_resistance_estimation:
            estimate_phase_resistance(5.0f);
            if(drive_state != drive_state_error){
                drive_state = drive_state_disabled;
            }
            break;

        case drive_state_encoder_calibration:
            calibrate_encoder(3.0f);
            if(drive_state != drive_state_error){
                drive_state = drive_state_disabled;
            }
            break;

        case drive_state_anti_cogging_calibration:
            anti_cogging_calibration(1.0f);
            if(drive_state != drive_state_error){
                drive_state = drive_state_disabled;
            }
            break;

        case drive_state_idle:
            enable_foc_loop();
            break;

        case drive_state_position_control:
            enable_foc_loop();
            break;
    }


    // Check 
    if(get_vmotor() < MIN_SUPPLY_VOLTAGE_V){
        enter_drive_error(drive_error_low_voltage);
    }
    osDelay(1);
}




void anti_cogging_calibration(float current){
    enable_foc_loop();


    anti_cogging_enabled = false;
    position_setpoint = 0;
    osDelay(1000);

        // int setpoint_electrical_angle = position_setpoint % (4096 / 8) / 2 + electrical_angle_offset;

        // current_offsets[setpoint_electrical_angle % 256] = current_Q_setpoint_mA;
        // uint8_t applied_electrical_angle = (uint8_t) (int) (-i) % 256;
        // apply_duty_at_electrical_angle_int(applied_electrical_angle, duty);

        // uint8_t measured_electrical_angle = enc_angle_int % (4096 / 8) / 2;
        
        // int offset = applied_electrical_angle - measured_electrical_angle;
                
        // if(offset < 127){
        //     offset += 255;
        // }

        // if(offset > 127){
        //     offset -= 255;
        // }

        // offset_average += offset;


    // Spin one electrical rev backward

    for(int i = 0; i < 256; i++){
        current_offsets[i] = 0;
    }

    for(int i = 0; i<256 * 4; i++){
        position_setpoint = -i * 2;
        osDelay(10);
        uint8_t setpoint_electrical_angle = (position_setpoint % (4096 / 8) / 2 + electrical_angle_offset);
        current_offsets[setpoint_electrical_angle] += current_Q_setpoint_mA;
    }

    for(int i = 0; i<256 * 4; i++){
        position_setpoint = i * 2 - 255 * 4 * 2;
        osDelay(10);
        uint8_t setpoint_electrical_angle = (position_setpoint % (4096 / 8) / 2 + electrical_angle_offset);
        current_offsets[setpoint_electrical_angle] += current_Q_setpoint_mA;
    }

    for(int i = 0; i < 256; i++){
        current_offsets[i] = current_offsets[i] / 8;
    }
    
    disable_foc_loop();
    anti_cogging_enabled = true;
    printf("lol");
}

// Currently only checks phase A as that amp seems to be the most accurate
void estimate_phase_resistance(float voltage){

    // Verify we have motor supply voltage
    if(check_supply_voltage() != drive_error_none){
        return;
    }

    disable_foc_loop();
    HAL_GPIO_WritePin(INLX_GPIO_Port, INLX_Pin, 1);

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
    }
}


void calibrate_encoder(float voltage){
    disable_foc_loop();
    HAL_GPIO_WritePin(INLX_GPIO_Port, INLX_Pin, 1);
    

    // Calculate required duty cycle for voltage
    float input_voltage = get_vmotor();
    float desired_voltage = voltage;
    int duty = (int)((desired_voltage / input_voltage) * 256);

    volatile int offset_average = 0;

    // Spin one electrical rev forward
    for(int i = 0; i<=256 * 8; i++){
        uint8_t applied_electrical_angle = (uint8_t) (int) (i) % 256;
        apply_duty_at_electrical_angle_int(applied_electrical_angle, duty);

        uint8_t measured_electrical_angle = enc_angle_int % (4096 / 8) / 2;
        
        int offset = applied_electrical_angle - measured_electrical_angle;
                
        if(offset < 127){
            offset += 255;
        }

        if(offset > 127){
            offset -= 255;
        }

        offset_average += offset;
        
        osDelay(5);
    }

    // Spin one electrical rev backward
    for(int i = 0; i<=256 * 8; i++){
        uint8_t applied_electrical_angle = (uint8_t) (int) (-i) % 256;
        apply_duty_at_electrical_angle_int(applied_electrical_angle, duty);

        uint8_t measured_electrical_angle = enc_angle_int % (4096 / 8) / 2;
        
        int offset = applied_electrical_angle - measured_electrical_angle;
                
        if(offset < 127){
            offset += 255;
        }

        if(offset > 127){
            offset -= 255;
        }

        offset_average += offset;
        
        osDelay(5);
    }

    electrical_angle_offset = offset_average / (256 * 16) + 127;
    set_duty_phases(0,0,0);
    printf("lol");
}


void apply_duty_at_electrical_angle_int(uint8_t angle, uint8_t magnitude){
    angle = (angle + (1.0 / 4.0) * 256);

    uint8_t A_value = mult_sin_lut_uint8((uint8_t) (angle + (0.0 / 3.0) * 256), magnitude);
    uint8_t B_value = mult_sin_lut_uint8((uint8_t) (angle + (1.0 / 3.0) * 256), magnitude);
    uint8_t C_value = mult_sin_lut_uint8((uint8_t) (angle + (2.0 / 3.0) * 256), magnitude);

    set_duty_phase_A(A_value);
    set_duty_phase_B(B_value);
    set_duty_phase_C(C_value);
}


enum DriveError check_supply_voltage() {
    if(get_vmotor() < MIN_SUPPLY_VOLTAGE_V){
        drive_error = drive_error_low_voltage;
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
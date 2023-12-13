#include "drive.h"


float power = 0;
uint16_t target_elec_angle = 0;
bool foc_drive_enabled = false;

bool motor_inverted = false;
bool motor_calibrated = false;

int motor_poles;
int counts_per_pole;
int target_encoder_value;

uint16_t electrical_angle;
uint16_t electrical_angle_offset;
uint16_t encoder_zeros[10];

// Enable DRV chip
void enable_DRV(){
    HAL_GPIO_WritePin(DRV_EN_GPIO_Port, DRV_EN_Pin, 1);
}

// Disable DRV chip
void disable_DRV(){
    HAL_GPIO_WritePin(DRV_EN_GPIO_Port, DRV_EN_Pin, 0);
}

// Set phase A PWM strength (8 bit)
void set_phase_A(uint8_t value){
    TIM1->CCR3 = value;
}
// Set phase B PWM strength (8 bit)
void set_phase_B(uint8_t value){
    TIM1->CCR2 = value;
}

// Set phase C PWM strength (8 bit)
void set_phase_C(uint8_t value){
    TIM1->CCR1 = value;
}

// TODO: Set phase ordering in calibration (should be relative to driver)
// TODO: Use CORDIC or LUT

// Apply duty vector at specified electrical angle
void apply_duty_at_electrical_angle(float angle, uint8_t duty){
    float shifted_angle = angle + PI/2.0f; // Shift so 0 is aligned with phase A
    volatile float A_value = sinf(shifted_angle);
    volatile float B_value = sinf(shifted_angle + 2.0f * PI / 3.0f);
    volatile float C_value = sinf(shifted_angle + 4.0f * PI / 3.0f);

    set_phase_A( duty * A_value + duty );
    set_phase_B( duty * B_value + duty );
    set_phase_C( duty * C_value + duty );
}

void apply_duty_at_electrical_angle_int(uint8_t angle, uint8_t duty){
    angle = (angle + (1.0f / 4.0f) * 256);

    uint8_t A_value = mult_sin_lut_uint8((uint8_t) (angle + (0.0f / 3.0f) * 256), duty);
    uint8_t B_value = mult_sin_lut_uint8((uint8_t) (angle + (1.0f / 3.0f) * 256), duty);
    uint8_t C_value = mult_sin_lut_uint8((uint8_t) (angle + (2.0f / 3.0f) * 256), duty);

    set_phase_A(A_value);
    set_phase_B(B_value);
    set_phase_C(C_value);
}

// Apply voltage vector at specified electrical angle
// Make sure to have run get_vmot at 
void apply_voltage_at_electrical_angle(uint8_t angle, uint16_t voltage_mv){
    volatile uint8_t duty = (256 * (int) voltage_mv) / v_motor_mv;
    apply_duty_at_electrical_angle_int(angle, duty);
}



// void apply_current_at_electrical_angle_open_loop(uint8_t angle, uint16_t current_mA){
    
//     // V = IR
//     A_voltage = phase_resistance_mOhm[0] * current_mA;
//     A_duty = mult_sin
// }

uint8_t angle = 0;
int max_voltage = 8000;

void foc_interrupt(){
    update_current_sense();
    // HAL_TIM_Base_Start(&htim7);
    
    // Run PD loop
    // TODO: Run this at a slower speed
    // int power = 0.004f * (target_encoder_value - enc_angle_int);
    if(foc_drive_enabled){
        // int power = 200;

        int voltage = 5 * (target_encoder_value - enc_angle_int);
        // int power = 3;
        electrical_angle = ((enc_angle_int - electrical_angle_offset) % 512);

        // angle = ((float)electrical_angle / 512.0f) * (PI * 2.0);

        // Convert electrical angle from uint9 to int8
        int8_t angle = electrical_angle >> 1;

        // Adjust direction based on power
        if(voltage > 0){
            angle += (1.0f/4.0f) * 256;
        }else{
            angle -= (1.0f/4.0f) * 256;
            voltage = -voltage;
        }
        // voltage = abs(voltage);

        // Adjust angle to 
        // Can probably get away without this now

        // Clamp power
        if(voltage > max_voltage){
            voltage = max_voltage;
        }

        apply_voltage_at_electrical_angle((uint8_t) (angle % 256), voltage);
        // apply_voltage_at_electrical_angle(angle, power);
    }

    // HAL_TIM_Base_Stop(&htim7);
    // volatile uint32_t executionTime = __HAL_TIM_GET_COUNTER(&htim7);
    // printf("test");
}

void enable_foc_loop(){
    foc_drive_enabled = true;
}

void disable_foc_loop(){
    foc_drive_enabled = false;
}

void clear_phases(){
    set_phase_A(0);
    set_phase_B(0);
    set_phase_C(0);
}

void spin_electrical_rads_sin(float revs, int calibration_voltage){
    for(int i = 0; i<=255; i++){
        // apply_voltage_at_electrical_angle((i / 100.0f) * rads, calibration_voltage);
        apply_duty_at_electrical_angle_int((uint8_t) (int) (i * revs) % 254, calibration_voltage);
        HAL_Delay(2);
    }
}

void spin_electrical_rev_forward(int calibration_voltage){
    for(int i = 0; i<=255; i++){
        // apply_voltage_at_electrical_angle((i / 100.0f) * rads, calibration_voltage);
        apply_duty_at_electrical_angle_int(i, calibration_voltage);
        HAL_Delay(2);
    }
}

void spin_electrical_rev_forward_os(int calibration_voltage){
    // for(int i = 0; i<=255; i++){
    //     // apply_voltage_at_electrical_angle((i / 100.0f) * rads, calibration_voltage);
    //     apply_duty_at_electrical_angle_int(i, calibration_voltage);
    //     osDelay(10);
    // }

    for(int i = 0; i<=500; i++){
        apply_duty_at_electrical_angle((2.0f * PI * i)/500.0f, calibration_voltage);
        osDelay(1);
    }
}



void spin_electrical_rev_backward(int calibration_voltage){
    for(int i = 0; i<=255; i++){
        // apply_voltage_at_electrical_angle((i / 100.0f) * rads, calibration_voltage);
        apply_duty_at_electrical_angle_int(255 - i, calibration_voltage);
        HAL_Delay(2);
    }
}

void estimate_phase_resistance(){
    // apply_voltage_at_electrical_angle(0, 1000);
    set_phase_A(0);
    set_phase_B(0);
    set_phase_C(0);
    // HAL_Delay(100);
    // clear_phases();
    // apply_voltage_at_electrical_angle(0, 0);
}

void calibrate_encoder_offset(int calibration_voltage){
    disable_foc_loop();

    get_vmotor();
    estimate_phase_resistance();
    motor_calibrated = false;

    // // 24 phase steps takes you around once
    // // Encoder is accurate (at least out to 40 rotations)

    // // 14 magnets (7 poles)
    // // 12 coils

    // apply_voltage_at_electrical_angle(0.0f, calibration_voltage);
    // HAL_Delay(80);
    // int start_enc = enc_angle_int;

    // apply_voltage_at_electrical_angle(0.5f, calibration_voltage);
    // HAL_Delay(80);

    // if(enc_angle_int > start_enc){
    //     motor_inverted = false;
    // }

    // apply_duty_at_electrical_angle_int(0, calibration_voltage);
    // HAL_Delay(100);

    // for(int i = 0; i < 5; i++){
    //     spin_electrical_rev_forward(calibration_voltage);
    //     encoder_zeros[i] = (enc_angle_int + 4096);
    // }

    // for(int i = 0; i < 5; i++){
    //     spin_electrical_rev_backward(calibration_voltage);
    //     encoder_zeros[i + 5] = (enc_angle_int + 4096);
    // }

    // clear_phases();

    
    // // TODO: Make this handle different pole counts
    // for(int i = 0; i < 10; i++){
    //     electrical_angle_offset += encoder_zeros[i] % 512;
    // }
    // electrical_angle_offset = electrical_angle_offset / 10;


    electrical_angle_offset = 162;

    motor_calibrated = true;
    enable_foc_loop();
    // disable_foc_loop();
}
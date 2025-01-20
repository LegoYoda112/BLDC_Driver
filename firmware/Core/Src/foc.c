#include "foc.h"

#define SQRT_1_2 0.7071067812f
#define SQRT_2_3 0.8164965809f
#define SQRT_3 1.7320508076f

// Control mode toggles
bool foc_active = false;
bool anti_cogging_enabled = false;

// Current Variables
uint16_t maximum_motor_current_mA = 5000;

int current_Q_setpoint_mA = 0;
int current_D_setpoint_mA = 0;

int16_t current_A_mA = 0;
int16_t current_B_mA = 0;
int16_t current_C_mA = 0;

int16_t current_A_offset_mA = 0;
int16_t current_C_offset_mA = 0;

int16_t current_A_mA_filtered = 0;
int16_t current_B_mA_filtered = 0;
int16_t current_C_mA_filtered = 0;

int16_t current_Alpha_mA = 0;
int16_t current_Beta_mA = 0;

int16_t current_D_mA = 0;
int16_t current_D_mA_error = 0;
int32_t current_D_mA_error_integral = 0;
int16_t current_Q_mA = 0;
int16_t current_Q_mA_error = 0;
int32_t current_Q_mA_error_integral = 0;

// Voltage Variables
uint16_t maximum_motor_voltage_mV = 25000;

int16_t voltage_D_mV = 0;
int16_t voltage_Q_mV = 0;

int16_t voltage_Alpha_mV = 0;
int16_t voltage_Beta_mV = 0;

int16_t voltage_a_mV = 0;
int16_t voltage_b_mV = 0;
int16_t voltage_c_mV = 0;

int direction = 1;
int hysteresis_offset_value = 4;
int hysteresis_offset = 0;
int prev_hysteresis_offset = 0;

// Gains
// TODO: Choose this based on motor resistance and inductance
float current_P_gain = 0.2f;
float current_I_gain = 0.015f;

// Angle
int prev_encoder_position = 0;
int encoder_velocity = 0;
uint8_t electrical_angle = 0;
uint8_t electrical_angle_offset = 0;
<<<<<<< Updated upstream
int8_t electrical_mechanical_ratio = 21;
=======

int position_setpoint = 0;
>>>>>>> Stashed changes

void set_current_setpoints(int D_setpoint_mA, int Q_setpoint_mA){
    current_Q_setpoint_mA = Q_setpoint_mA;
    current_D_setpoint_mA = D_setpoint_mA;
}


// Observer
float q_hat = 0;
float q_err = 0;
float v_hat = 0;
const float loop_rate = 10000.0f; 
volatile const float dt = 1.0/loop_rate;

int adjusted_enc_angle = 0;

bool first_run = true;

void current_control_loop(){
    // Calculate electrical angle as a uint8
    // 0 is aligned with phase A
    // 255 is just before wraparound
    if( (enc_angle_int - prev_encoder_position) > 0 && direction < 0 ){
        hysteresis_offset = -hysteresis_offset_value;
        direction = 1;
    } else if( (enc_angle_int - prev_encoder_position) < 0 && direction > 0 ){
        hysteresis_offset = hysteresis_offset_value;
        direction = -1;
    } else {
        // hysteresis_offset = 0;
    }

    adjusted_enc_angle = enc_angle_int + hysteresis_offset;
    electrical_angle = convert_to_electrical_angle(adjusted_enc_angle, electrical_mechanical_ratio, electrical_angle_offset);

    // Calculate velocity

    // "warm up" observer
    if(first_run == true){
        q_hat = (float) adjusted_enc_angle;
        prev_encoder_position = adjusted_enc_angle;
        first_run = false;
    }

    // Estimate new position
    q_hat += dt * v_hat;

    // FLOAT MATH BAD
    q_err = adjusted_enc_angle - q_hat;
    v_hat += 10.0f * q_err - 0.1 * v_hat;

    encoder_velocity = round(v_hat);
    prev_encoder_position = adjusted_enc_angle;

    // Calculate motor currents and transform them
    // 9 is a magic number
    // TODO: Document 9
    current_A_mA =  (adc2_dma[0] - adc2_calib_offset[0]) * 9;
    current_C_mA =  (adc2_dma[1] - adc2_calib_offset[1]) * 9;
    current_B_mA =  - (current_A_mA + current_C_mA); // Ignore noisy ADC

    // Perform clarke and park transform to get motor current in orthogonal rotor-centric coordinates
    clarke_transform(current_A_mA, current_B_mA, current_C_mA, &current_Alpha_mA, &current_Beta_mA);
    park_transform(current_Alpha_mA, current_Beta_mA, electrical_angle, &current_D_mA, &current_Q_mA);

    // Enforce limits on current
    current_Q_setpoint_mA = bound(current_Q_setpoint_mA, -maximum_motor_current_mA, maximum_motor_current_mA);

    // Feedforward for anti-cogging
    // if(anti_cogging_enabled){
    //     current_Q_setpoint_mA += current_offsets[electrical_angle] * 1.0f;
    // }

    // Q current P loop
    current_Q_mA_error = (current_Q_mA - current_Q_setpoint_mA);
    current_Q_mA_error_integral += current_Q_mA_error;
    voltage_Q_mV = current_Q_mA_error * current_P_gain + current_Q_mA_error_integral * current_I_gain;
    voltage_Q_mV = (int16_t) bound(voltage_Q_mV, -maximum_motor_voltage_mV, maximum_motor_voltage_mV); // Enforce duty bounds
    // D current P loop
    current_D_mA_error = -(current_D_mA - current_D_setpoint_mA);
    current_D_mA_error_integral += current_D_mA_error;
    voltage_D_mV = current_D_mA_error * current_P_gain + + current_D_mA_error_integral * current_I_gain;
    voltage_D_mV = (int16_t) bound(voltage_D_mV, -maximum_motor_voltage_mV, maximum_motor_voltage_mV); // Enforce duty bounds 

    // Perform inverse park and clarke transform to convert from rotor-centric voltage into phase voltages
    inverse_park_transform(voltage_D_mV, voltage_Q_mV, electrical_angle, &voltage_Alpha_mV, &voltage_Beta_mV);
    inverse_clarke_transform(voltage_Alpha_mV, voltage_Beta_mV, &voltage_a_mV, &voltage_b_mV, &voltage_c_mV);
    
    // Find minimum voltage to offset phase voltages to be all positive
    // Might be worth experimenting with centering phases around V_supply/2 to avoid this
    // there might be additional consequences
    int16_t min_voltage_mV = int16_min3(voltage_a_mV, voltage_b_mV, voltage_c_mV);

    // If FOC is enabled, set voltages
    if(foc_active){
        set_V_phases_mv(voltage_a_mV - min_voltage_mV, voltage_b_mV - min_voltage_mV, voltage_c_mV - min_voltage_mV);
    } else {
        // Zero out integrals if not
        current_Q_mA_error_integral = 0;
        current_D_mA_error_integral = 0;
    }
}

void clarke_transform(int16_t A, int16_t B, int16_t C, int16_t *alpha, int16_t *beta){
    *alpha = (int16_t) ( SQRT_2_3 * (1 * A  -  B / 2           -  C / 2         ) );
    *beta =  (int16_t) ( SQRT_2_3 * (0 * A  +  SQRT_3 * B / 2  -  SQRT_3 * C / 2) );
}

void park_transform(int16_t alpha, int16_t beta, uint8_t angle, int16_t *d, int16_t *q){
    *d = mult_sin_lut_int16(angle + 64, alpha) + mult_sin_lut_int16(angle, beta);
    *q = mult_sin_lut_int16(angle + 64, beta)  - mult_sin_lut_int16(angle, alpha);
}

void inverse_clarke_transform(int16_t alpha, int16_t beta, int16_t *a, int16_t *b, int16_t *c){
    *a = SQRT_2_3 * ( 1 * alpha );
    *b = SQRT_2_3 * ( - alpha / 2.0f + SQRT_3 * beta / 2.0f);
    *c = SQRT_2_3 * ( - alpha / 2.0f - SQRT_3 * beta / 2.0f);
}

void inverse_park_transform(int16_t d, int16_t q, uint8_t angle, int16_t *alpha, int16_t *beta){
    *alpha = mult_sin_lut_int16(angle + 64, d) + mult_sin_lut_int16(angle, q);
    *beta =  mult_sin_lut_int16(angle + 64, q) - mult_sin_lut_int16(angle, d);
}




/////////// LEGACY ///////////
// void apply_duty_at_electrical_angle_int(uint8_t angle, uint8_t magnitude){
//     angle = (angle + (1.0 / 4.0) * 256);

//     uint8_t A_value = mult_sin_lut_uint8((uint8_t) (angle + (0.0 / 3.0) * 256), magnitude);
//     uint8_t B_value = mult_sin_lut_uint8((uint8_t) (angle + (1.0 / 3.0) * 256), magnitude);
//     uint8_t C_value = mult_sin_lut_uint8((uint8_t) (angle + (2.0 / 3.0) * 256), magnitude);

//     set_duty_phase_A(A_value);
//     set_duty_phase_B(B_value);
//     set_duty_phase_C(C_value);
// }

// void spin_electrical_rads_sin(float revs, int calibration_voltage){
//     for(int i = 0; i<=255; i++){
//         // apply_voltage_at_electrical_angle((i / 100.0f) * rads, calibration_voltage);
//         apply_voltage_at_electrical_angle_int((uint8_t) (int) (i * revs) % 254, calibration_voltage);
//         osDelay(5);
//     }
// }

// void spin_electrical_rev_forward(int calibration_voltage){
//     for(int i = 0; i<=255; i++){
//         // apply_voltage_at_electrical_angle((i / 100.0f) * rads, calibration_voltage);
//         apply_voltage_at_electrical_angle_int(i, calibration_voltage);
//         osDelay(5);
//     }
// }


// void spin_electrical_rev_backward(int calibration_voltage){
//     for(int i = 0; i<=255; i++){
//         // apply_voltage_at_electrical_angle((i / 100.0f) * rads, calibration_voltage);
//         apply_voltage_at_electrical_angle_int(255 - i, calibration_voltage);
//         osDelay(5);
//     }
// }
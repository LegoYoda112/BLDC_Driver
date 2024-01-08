#include "foc.h"

#define SQRT_1_2 0.7071067812f
#define SQRT_2_3 0.8164965809f
#define SQRT_3 1.7320508076f

uint16_t voltage_supply_mV;

// Current Variables
int16_t current_setpoint_limit_mA = 1000;
int16_t current_setpoint_mA = 0;

int16_t current_A_mA = 0;
int16_t current_B_mA = 0;
int16_t current_C_mA = 0;

int16_t current_A_offset_mA = 2;
int16_t current_C_offset_mA = -5;

int16_t current_A_mA_filtered = 0;
int16_t current_B_mA_filtered = 0;
int16_t current_C_mA_filtered = 0;

int16_t current_Alpha_mA = 0;
int16_t current_Beta_mA = 0;

int16_t current_D_mA = 0;
int16_t current_Q_mA = 0;


// Voltage Variables
int8_t maximum_duty = 200;

int16_t voltage_D_mV = 0;
int16_t voltage_Q_mV = 0;

int16_t voltage_Alpha_mV = 0;
int16_t voltage_Beta_mV = 0;

int16_t voltage_a_mV = 0;
int16_t voltage_b_mV = 0;
int16_t voltage_c_mV = 0;

// Gains
// TODO: Choose this based on motor resistance and inductance
float current_P_gain = 0.05f;

// Angle
uint8_t electrical_angle = 0;
uint8_t electrical_angle_offset = -40;

int position_setpoint = 0;

void foc_interrupt(){
    // Calculate electrical angle as a uint8
    // 0 is aligned with phase A
    // 255 is just before wraparound
    electrical_angle = -enc_angle_int % (4096 / 8) / 2 - electrical_angle_offset;

    // Calculate motor voltage
    voltage_supply_mV = adc1_dma[0] * 13;

    // Calculate motor currents and transform them
    // 9 is a magic number
    // TODO: Document 9
    current_A_mA =  (adc2_dma[0] - 2048 + current_A_offset_mA) * 9;
    current_C_mA =  (adc2_dma[1] - 2048 - current_C_offset_mA) * 9;
    current_B_mA =  - (current_A_mA + current_C_mA);

    // Perform an IIR filter on current to cut down on noise and injected vibrations
    current_A_mA_filtered = current_A_mA_filtered * 0.9f + current_A_mA * 0.1f;
    current_B_mA_filtered = current_B_mA_filtered * 0.9f + current_B_mA * 0.1f;
    current_C_mA_filtered = current_C_mA_filtered * 0.9f + current_C_mA * 0.1f;

    // Perform clarke and park transform to get motor current in orthogonal rotor-centric coordinates
    clarke_transform(current_A_mA_filtered, current_B_mA_filtered, current_C_mA_filtered, &current_Alpha_mA, &current_Beta_mA);
    park_transform(current_Alpha_mA, current_Beta_mA, electrical_angle, &current_D_mA, &current_Q_mA);

    // Generate current setpoint 
    // TODO: Split this out into another timer interrupt
    current_setpoint_mA = -(position_setpoint - enc_angle_int) * 20;
    current_setpoint_limit_mA = 1000; // 1A current setpoint for testing
    // Enforce limits on current
    enforce_bound(&current_setpoint_mA, -current_setpoint_limit_mA, current_setpoint_limit_mA);

    // Q current P loop
    // TODO: Add an integral term?
    voltage_Q_mV = (current_Q_mA - current_setpoint_mA) * current_P_gain;
    enforce_bound(&voltage_Q_mV, -maximum_duty, maximum_duty);
    // D current P loop
    voltage_D_mV = -current_D_mA * current_P_gain;
    enforce_bound(&voltage_D_mV, -maximum_duty, maximum_duty);

    // Perform inverse park and clarke transform to convert from rotor-centric voltage into phase voltages
    inverse_park_transform(voltage_D_mV, voltage_Q_mV, electrical_angle, &voltage_Alpha_mV, &voltage_Beta_mV);
    inverse_clarke_transform(voltage_Alpha_mV, voltage_Beta_mV, &voltage_a_mV, &voltage_b_mV, &voltage_c_mV);
    // Find minimum voltage to offset phase voltages to be all positive
    // Might be worth experimenting with centering phases around V_supply/2 to avoid this
    // there might be additional consequences
    int16_t min_voltage_mV = int16_min3(voltage_a_mV, voltage_b_mV, voltage_c_mV);


    // If FOC is enabled, set voltages
    if(foc_active){
        // TODO, convert to setting actual voltages instead of duty cycle
        set_duty_phases(voltage_a_mV - min_voltage_mV, voltage_b_mV - min_voltage_mV, voltage_c_mV - min_voltage_mV);
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
void apply_duty_at_electrical_angle_int(uint8_t angle, uint8_t magnitude){
    angle = (angle + (1.0 / 4.0) * 256);

    uint8_t A_value = mult_sin_lut_uint8((uint8_t) (angle + (0.0 / 3.0) * 256), magnitude);
    uint8_t B_value = mult_sin_lut_uint8((uint8_t) (angle + (1.0 / 3.0) * 256), magnitude);
    uint8_t C_value = mult_sin_lut_uint8((uint8_t) (angle + (2.0 / 3.0) * 256), magnitude);

    set_duty_phase_A(A_value);
    set_duty_phase_B(B_value);
    set_duty_phase_C(C_value);
}

void spin_electrical_rads_sin(float revs, int calibration_voltage){
    for(int i = 0; i<=255; i++){
        // apply_voltage_at_electrical_angle((i / 100.0f) * rads, calibration_voltage);
        apply_voltage_at_electrical_angle_int((uint8_t) (int) (i * revs) % 254, calibration_voltage);
        osDelay(5);
    }
}

void spin_electrical_rev_forward(int calibration_voltage){
    for(int i = 0; i<=255; i++){
        // apply_voltage_at_electrical_angle((i / 100.0f) * rads, calibration_voltage);
        apply_voltage_at_electrical_angle_int(i, calibration_voltage);
        osDelay(5);
    }
}


void spin_electrical_rev_backward(int calibration_voltage){
    for(int i = 0; i<=255; i++){
        // apply_voltage_at_electrical_angle((i / 100.0f) * rads, calibration_voltage);
        apply_voltage_at_electrical_angle_int(255 - i, calibration_voltage);
        osDelay(5);
    }
}
#include "foc.h"
#include "math.h"
#include "trig_luts.h"

#define SQRT_1_2 0.7071067812f
#define SQRT_2_3 0.8164965809f
#define SQRT_3 1.7320508076f

uint16_t vmotor_mV;


// Current Variables
int16_t current_A_mA = 0;
int16_t current_B_mA = 0;
int16_t current_C_mA = 0;

int16_t current_A_mA_filtered = 0;
int16_t current_B_mA_filtered = 0;
int16_t current_C_mA_filtered = 0;

int16_t current_Alpha_mA;
int16_t current_Beta_mA;

int16_t current_D_mA;
int16_t current_Q_mA;


// Voltage Variables
int16_t voltage_D_mV;
int16_t voltage_Q_mV;

int16_t voltage_Alpha_mV;
int16_t voltage_Beta_mV;

int16_t voltage_a_mV;
int16_t voltage_b_mV;
int16_t voltage_c_mV;


// Angle
int16_t angle = 0;

uint8_t electrical_angle = 0;
uint8_t electrical_angle_offset = -40;

int position_setpoint = 0;

void foc_interrupt(){
    // HAL_GPIO_TogglePin(TEST_GPIO_Port, TEST_Pin);

    // Calculate electrical angle as a uint8
    // 0 is aligned with phase A
    // 255 is just before wraparound
    electrical_angle = -enc_angle_int % (4096 / 8) / 2 - electrical_angle_offset;

    // Calculate motor voltage
    vmotor_mV = adc1_dma[0] * 13;

    // Calculate motor currents and transform them
    current_A_mA =  (adc2_dma[0] - 2048 + 2) * 9;
    current_C_mA =  (adc2_dma[1] - 2048 - 5) * 9;
    current_B_mA =  - (current_A_mA + current_C_mA);

    current_A_mA_filtered = current_A_mA_filtered * 0.9f + current_A_mA * 0.1f;
    current_B_mA_filtered = current_B_mA_filtered * 0.9f + current_B_mA * 0.1f;
    current_C_mA_filtered = current_C_mA_filtered * 0.9f + current_C_mA * 0.1f;

    // HAL_GPIO_TogglePin(TEST_GPIO_Port, TEST_Pin);

    clarke_transform(current_A_mA_filtered, current_B_mA_filtered, current_C_mA_filtered, &current_Alpha_mA, &current_Beta_mA);
    park_transform(current_Alpha_mA, current_Beta_mA, electrical_angle, &current_D_mA, &current_Q_mA);

    // HAL_GPIO_TogglePin(TEST_GPIO_Port, TEST_Pin);

    // current_setpoint_filtered = ( current_setpoint_filtered )* 0.0f + (-(position_setpoint - enc_angle_int) * 40) * 1.0f;
    int current_setpoint = -(position_setpoint - enc_angle_int) * 20;
    // int current_setpoint = 0;
    int max_current = 700;
    if(current_setpoint > max_current){
        current_setpoint = max_current;
    }else if(current_setpoint < -max_current){
        current_setpoint = -max_current;
    }

    int max = 250;
    int qvalue = (current_Q_mA - current_setpoint) / 20.0f;
    // int qvalue = 0;

    if(qvalue > max){
        qvalue = max;
    }else if(qvalue < -max){
        qvalue = -max;
    }

    int dvalue = -current_D_mA / 20.0f;
    // int dvalue = 0;

    if(dvalue > max){
        dvalue = max;
    }else if(dvalue < -max){
        dvalue = -max;
    }

    // if(current_setpoint > max){
    //     current_setpoint = max;
    // }else if(current_setpoint < -max){
    //     current_setpoint = -max;
    // }

    inverse_park_transform(dvalue, qvalue, electrical_angle, &voltage_Alpha_mV, &voltage_Beta_mV);
    inverse_clarke_transform(voltage_Alpha_mV, voltage_Beta_mV, &voltage_a_mV, &voltage_b_mV, &voltage_c_mV);

    int16_t min_voltage = min3(voltage_a_mV, voltage_b_mV, voltage_c_mV);

    set_duty_phases(voltage_a_mV - min_voltage, voltage_b_mV - min_voltage, voltage_c_mV - min_voltage);

    //apply_duty_at_electrical_angle_int((uint8_t) angle, 20);

    // if(foc_active){
    //apply_duty_at_electrical_angle_int((uint8_t) electrical_angle + 64, 60);
    // }
    // set_duty_phases(0, 0, 0);
    // HAL_GPIO_TogglePin(TEST_GPIO_Port, TEST_Pin);
}

// Power invariant
void clarke_transform(int16_t A, int16_t B, int16_t C, int16_t *alpha, int16_t *beta){
    *alpha = (int16_t) ( SQRT_2_3 * (1 * A  -  B / 2           -  C / 2         ) );
    *beta =  (int16_t) ( SQRT_2_3 * (0 * A  +  SQRT_3 * B / 2  -  SQRT_3 * C / 2) );
}

void park_transform(int16_t alpha, int16_t beta, uint8_t angle, int16_t *d, int16_t *q){
    *d = mult_sin_lut_int16(angle + 64, alpha) + mult_sin_lut_int16(angle, beta);
    *q = mult_sin_lut_int16(angle + 64, beta)  - mult_sin_lut_int16(angle, alpha);
    // *q = beta * - alpha
}

void inverse_clarke_transform(int16_t alpha, int16_t beta, int16_t *a, int16_t *b, int16_t *c){
    *a = SQRT_2_3 * ( 1 * alpha );
    *b = SQRT_2_3 * ( - alpha / 2.0f + SQRT_3 * beta / 2.0f);
    *c = SQRT_2_3 * ( - alpha / 2.0f - SQRT_3 * beta / 2.0f);
}

void inverse_park_transform(int16_t V_d, int16_t V_q, uint8_t angle, int16_t *V_alpha, int16_t *V_beta){
    *V_alpha = mult_sin_lut_int16(angle + 64, V_d) + mult_sin_lut_int16(angle, V_q);
    *V_beta =  mult_sin_lut_int16(angle + 64, V_q) - mult_sin_lut_int16(angle, V_d);
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

int16_t min3(int16_t a, int16_t b, int16_t c) {
    int16_t min = a;
    if (b < min) {
        min = b;
    }
    if (c < min) {
        min = c;
    }
    return min;
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
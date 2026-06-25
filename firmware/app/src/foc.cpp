#include "foc.h"
#include "drive.h"
#include "trig.h"

using namespace foc;

/**
 * @brief Convert from encoder raw to electrical angle
 * 
 * @param mechanical_angle 
 * @param ratio 
 * @param offset 
 * @return uint16_t 
 */
int foc::convert_to_electrical_angle(int encoder_raw, int8_t ratio, uint16_t offset){
    // TODO: This is messy and not intuitive; does it really give a performance increase over just using floats
    int32_t temp = (encoder_raw % 16384) % (16384 / ratio);
    temp = temp * (ratio * 65536);
    temp = temp / 16384;
    return (temp - offset);
}

void foc::make_voltage_at_electrical_angle(int16_t voltage_D_mV, int16_t voltage_Q_mV, uint16_t electrical_angle, struct drive::PhaseVoltages *phase_voltages){
    int voltage_alpha_mV, voltage_beta_mV;
    int voltage_A_mV, voltage_B_mV, voltage_C_mV;
    inverse_park_transform(voltage_D_mV, voltage_Q_mV, electrical_angle, &voltage_alpha_mV, &voltage_beta_mV);
    inverse_clarke_transform(voltage_alpha_mV, voltage_beta_mV, &voltage_A_mV, &voltage_B_mV, &voltage_C_mV);

    phase_voltages->phaseA_V = voltage_A_mV / 1000.0f;
    phase_voltages->phaseB_V = voltage_B_mV / 1000.0f;
    phase_voltages->phaseC_V = voltage_C_mV / 1000.0f;
}


void foc::clarke_transform(int A, int B, int C, int *alpha, int *beta){
    *alpha = (int) ( SQRT_2_3 * (1 * A  -  B / 2           -  C / 2         ) );
    *beta =  (int) ( SQRT_2_3 * (0 * A  +  SQRT_3 * B / 2  -  SQRT_3 * C / 2) );
}

void foc::park_transform(int alpha, int beta, int angle, int *d, int *q){
    *d =   trig::mult_cos(angle, alpha) +   trig::mult_sin(angle, beta);
    *q = - trig::mult_sin(angle, alpha) +   trig::mult_cos(angle, beta);
}

void foc::inverse_clarke_transform(int alpha, int beta, int *a, int *b, int *c){
    *a = SQRT_2_3 * ( 1 * alpha );
    *b = SQRT_2_3 * ( - alpha / 2.0f + SQRT_3 * beta / 2.0f);
    *c = SQRT_2_3 * ( - alpha / 2.0f - SQRT_3 * beta / 2.0f);
}

void foc::inverse_park_transform(int d, int q, int angle, int *alpha, int *beta){
    *alpha = trig::mult_cos(angle, d) + trig::mult_sin(angle, q);
    *beta =  trig::mult_cos(angle, q) - trig::mult_sin(angle, d);
}
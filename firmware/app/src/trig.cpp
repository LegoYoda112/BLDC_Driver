#include "trig.h"
#include "main.h"
#include "math.h"

using namespace trig;

int16_t trig::sin_lut[LUT_LENGTH];

/**
 * @brief Precomputes look up tables
 * 
 */
void trig::initialize(){
    // Precompute SIN LUT
    for(int i = 0; i < LUT_LENGTH; i++){
        float value = sinf( (float) PI2_F * (i / (float) LUT_LENGTH));
        sin_lut[i] = value * LUT_DEPTH;
    }
}


/**
 * @brief 
 * 
 * @param position 10-bit angle 
 * @param value value to multiply
 * @return value * sin(position)
 */
int trig::mult_sin(int angle, int value){
    int mult_value = value * sin_lut[angle % 1024];
    return (int)(mult_value >> 10);
}

/**
 * @brief 
 * 
 * @param position 10-bit angle 
 * @param value value to multiply
 * @return value * cos(position)
 */
int trig::mult_cos(int angle, int value){
    return mult_sin(angle + 256, value);
}
#include "encoder.h"
#include "main.h"
#include <cstdint>

#include "string.h"
#include "usbd_cdc_if.h"

using namespace encoder;

int encoder_angle_raw = 0;
bool index_read = false;

void encoder::initialize(){
    // Enable encoder
    encoder_angle_raw = 0;
    index_read = false;
    
    // Absolute index
    #if (HW_REV == 1 || HW_REV == 2 || HW_REV == 3)
        HAL_GPIO_WritePin(ENC_EN_GPIO_Port, ENC_EN_Pin, (GPIO_PinState) 0);
        HAL_Delay(5);
        encoder_angle_raw = 4 * encoder_angle_raw; // hack
        index_read = false;

    #endif

}

int8_t lookup_table[] = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};
uint8_t enc_val = 0;

void encoder::interrupt(uint16_t pin){
    enc_val = enc_val << 2;
    enc_val = enc_val | (IFA_GPIO_Port->IDR & IFA_Pin) | (IFB_GPIO_Port->IDR & IFB_Pin) >> 2;

    encoder_angle_raw += lookup_table[enc_val & 0b1111];
}

void encoder::index_interrupt(){
    encoder_angle_raw = ((encoder_angle_raw + (2048*4)) >> 14) << 14;
    index_read = true;
}

int encoder::get_raw(){
    return encoder_angle_raw;
}

bool encoder::get_index_read(){
    return index_read;
}

uint8_t encoder::get_enc_val(){
    return enc_val;
}
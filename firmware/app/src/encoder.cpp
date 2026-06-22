#include "encoder.h"
#include "main.h"

using namespace encoder;

int encoder_angle_raw = 0;
bool index_read = false;

void encoder::initialize(){
    // Enable encoder
    encoder_angle_raw = 0;
    index_read = false;
    #if (HW_REV == 1 || HW_REV == 2 || HW_REV == 3)
        HAL_GPIO_WritePin(ENC_EN_GPIO_Port, ENC_EN_Pin, (GPIO_PinState) 0);
    #endif
}

void encoder::interrupt(){
  if((IFB_GPIO_Port->IDR & IFB_Pin)){
    encoder_angle_raw ++;
  }else{
    encoder_angle_raw --;
  }
}

void encoder::index_interrupt(){
    encoder_angle_raw = ((encoder_angle_raw + 2048) >> 12) << 12;
    index_read = true;
}

int encoder::get_raw(){
    return encoder_angle_raw;
}

bool encoder::get_index_read(){
    return index_read;
}
#include "encoder.h"

using namespace encoder;

int encoder_angle_raw = 0;

void encoder::interrupt(){
    if(HAL_GPIO_ReadPin(IFB_GPIO_Port, IFB_Pin)){
        encoder_angle_raw ++;
    } else {
        encoder_angle_raw --;
    }
}

void encoder::index_interrupt(){

}
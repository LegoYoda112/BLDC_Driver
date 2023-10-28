#include "comms.h"

void process_USB_rx(uint8_t* Buf, uint32_t *Len){
    target_encoder_value = Buf[0] * 100;

    char intStr[12];
    sprintf(intStr, "%d\r\n", target_encoder_value);
    CDC_Transmit_FS((uint8_t*)intStr, strlen(intStr));
}
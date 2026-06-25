#pragma once


#include "tim.h"

#ifdef __cplusplus
extern "C" {
#endif

void app_main(void);
void tim_elapsed_callback(TIM_HandleTypeDef*);
void gpio_interrupt_callback(uint16_t pin);
void usb_callback(uint8_t*, uint32_t);

#ifdef __cplusplus
}
#endif

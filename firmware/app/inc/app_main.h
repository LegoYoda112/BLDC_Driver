#ifndef APP_MAIN_H
#define APP_MAIN_H


#include "tim.h"

#ifdef __cplusplus
extern "C" {
#endif

void app_main(void);
void tim_elapsed_callback(TIM_HandleTypeDef*);
void gpio_interrupt_callback(uint16_t pin);

#ifdef __cplusplus
}
#endif

#endif // APP_MAIN_H
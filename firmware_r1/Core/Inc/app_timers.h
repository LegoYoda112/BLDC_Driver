#ifndef APP_TIMERS_H
#define APP_TIMERS_H

#include "main.h"

extern TIM_HandleTypeDef htim7;

void start_app_timers();
void app_delay_ms(uint16_t ms);

#endif
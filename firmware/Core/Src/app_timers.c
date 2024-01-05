#include "app_timers.h"

void start_app_timers(){
    HAL_TIM_Base_Start(&htim7);
}

void app_delay_ms(uint16_t ms){
    __HAL_TIM_SET_COUNTER(&htim7, 0);  // set the counter value a 0
	volatile int test = __HAL_TIM_GET_COUNTER(&htim7);
    while (__HAL_TIM_GET_COUNTER(&htim7) < ms * 10){
        volatile int test = __HAL_TIM_GET_COUNTER(&htim7);
    }; 
}
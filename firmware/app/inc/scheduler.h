#pragma once
#include "stm32g4xx_hal.h"
#include "tim.h"

#define CLOCK_RATE 1000000


namespace scheduler
{

/**
 * 
 * @brief Initializes timers
 * 
 */
void initialize();

/**
 * @brief Generate 
 * 
 * @param Period in seconds 
 * @return Period in clock ticks 
 */
int period_from_seconds(float seconds);

/**
 * @brief Generate clock tick period from desired rate
 * 
 * @param Rate in hertz 
 * @return Period in clock ticks
 */
int period_from_hertz(int hertz);

class PeriodicTask {
    public:
        int period_us;
        int last_run_us;

        TIM_HandleTypeDef* timer;

        PeriodicTask(TIM_HandleTypeDef* _timer) {
            timer = _timer;
        }
        
        /**
         * @brief Call every loop, if the task is due to run, this function will return true
         * 
         */
        bool tick();
};

} // scheduler
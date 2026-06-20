#include "app_main.h"
#include "stm32g4xx_hal.h"
#include "tim.h"
#include "structs.h"
#include "scheduler.h"

void app_main(void){
  HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_1);

  scheduler::initialize_scheduler();

  struct ControllerTarget target;
  target.position = 100;


  scheduler::PeriodicTask task(&htim2);
  task.period_us = scheduler::period_from_seconds(1.0f);

  while(true)
    if(task.tick()){
      // Toggle timer
      if(TIM15->CCR1 == 100){
        TIM15->CCR1 = 0;
      } else {
        TIM15->CCR1 = 100;
      }
    }

    target.position += 1;
}
#include "app_main.h"
// c lib includes
#include "string.h"
#include <stdio.h>
// ST HAL includes
#include "stm32g4xx_hal.h"
#include "tim.h"
#include "usbd_cdc_if.h"
// Custom lib includes
#include "structs.h"
#include "scheduler.h"
#include "analog.h"
#include "drive.h"
#include "encoder.h"
#include "led.h"


// struct ControllerTarget target;
int count = 0;

void app_main(void){
  HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_1);

  // Initialize all sub-modules
  scheduler::initialize();
  encoder::initialize();
  drive::initialize();
  analog::initialize();
  led::initialize();

  // Enable low side and zero phases
  drive::enable_low_side();
  drive::set_phaseA_duty(0);
  drive::set_phaseB_duty(0);
  drive::set_phaseC_duty(0);


  scheduler::PeriodicTask blink_task(&htim2);
  blink_task.period_us = scheduler::period_from_hertz(60);

  scheduler::PeriodicTask print_task(&htim2);
  print_task.period_us = scheduler::period_from_hertz(50);


  char print_buffer[100];

  while(true) {
    if(print_task.tick()){
      sprintf(print_buffer, "Encoder angle %d %d %d\r\n", 
        encoder::get_raw(),
        encoder::get_index_read(),
        count
      );
      CDC_Transmit_FS((uint8_t*) print_buffer, strlen(print_buffer));
    }


    if(blink_task.tick()){
      // Toggle timer
      led::set_hsv(count / 10.0f, 1.0f, 1.0f);
      count += 1;
    }
  }
}

void tim_elapsed_callback(TIM_HandleTypeDef *htim) {
  if(htim->Instance == TIM6) {
    drive::commutation_interrupt();
  }
}

void gpio_interrupt_callback(uint16_t pin){
  if(pin == IFA_Pin || pin == IFB_Pin){
    encoder::interrupt(pin);
  } else if(pin == IFC_Pin) {
    encoder::index_interrupt();
  }
};
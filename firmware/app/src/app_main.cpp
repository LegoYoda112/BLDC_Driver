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

  struct drive::PhaseVoltages motor_voltage;
  motor_voltage.phaseA_V = 2.0;
  motor_voltage.phaseB_V = 4.0;
  motor_voltage.phaseC_V = 6.0;
  drive::set_target_phase_voltages(&motor_voltage);

  scheduler::PeriodicTask blink_task(&htim2);
  blink_task.period_us = scheduler::period_from_hertz(60);

  scheduler::PeriodicTask print_task(&htim2);
  print_task.period_us = scheduler::period_from_hertz(50);

  scheduler::PeriodicTask analog_update_task(&htim2);
  analog_update_task.period_us = scheduler::period_from_hertz(1000);


  char print_buffer[100];

  while(true) {
    if(print_task.tick()){
      sprintf(print_buffer, "%.2f\r\n", 
        analog::get_cached_bus_voltage_v()
      );
      CDC_Transmit_FS((uint8_t*) print_buffer, strlen(print_buffer));
    }
    
    // Compute all non-current analog values
    if(analog_update_task.tick()){
      // TODO: Add temps
      analog::read_bus_voltage_v();
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
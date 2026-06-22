#include "led.h"
#include "tim.h"
#include "math.h"

using namespace led;

void led::initialize(){
    // Start timers
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_2);
}

void led::set_red_pwm(float value){
  TIM15->CCR1 = (value * R_LED_MAX);
}

void led::set_green_pwm(float value){
  TIM15->CCR2 = (value * G_LED_MAX);
}

void led::set_blue_pwm(float value){
  TIM3->CCR2 = (value * B_LED_MAX);
}

void led::set_hsv(float H, float S, float V){

  // Log dimming curve
  if(V < 0){
    V = 0;
  }
  V = V*V;

  float C = V * S;
  float H_prime = fmod(H, 360) / 60.0f;
  // volatile float mod_test = fabs((float)fmod(H_prime, 2) - 1.0f;
  float X = C * (1.0f - fabs( fmod(H_prime, 2.0f) - 1.0f));

  float r = 0;
  float g = 0;
  float b = 0;

  if(0 <= H_prime && H_prime < 1){
    r = C;
    g = X;
  }else if(1 <= H_prime && H_prime < 2){
    r = X;
    g = C;
  }else if(2 <= H_prime && H_prime < 3){
    g = C;
    b = X;
  }else if(3 <= H_prime && H_prime < 4){
    g = X;
    b = C;
  }else if(4 <= H_prime && H_prime < 5){
    r = X;
    b = C;
  }else if(5 <= H_prime && H_prime < 6){
    r = C;
    b = X;
  }

  float m = V - C;

  set_red_pwm(r + m);
  set_green_pwm(g + m);
  set_blue_pwm(b + m);
}
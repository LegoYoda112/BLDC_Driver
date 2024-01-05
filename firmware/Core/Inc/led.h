#ifndef LED_H
#define LED_H

#include "main.h"

extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim15;

void start_led_timers();

void set_led_red_pwm(float value);
void set_led_green_pwm(float value);
void set_led_blue_pwm(float value);

void led_hsv(float H, float S, float V);

#endif
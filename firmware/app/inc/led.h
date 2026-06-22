#pragma once

#define R_LED_MAX 800
#define G_LED_MAX 600
#define B_LED_MAX 600

namespace led{

void initialize();

void set_red_pwm(float);
void set_green_pwm(float);
void set_blue_pwm(float);

void set_hsv(float, float, float);

}
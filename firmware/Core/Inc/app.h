#ifndef APP_H
#define APP_H

#include "main.h"
#include "led.h"

extern ADC_HandleTypeDef hadc2;

enum appState {
    app_state_idle,
    app_state_active
};

extern enum appState app_state;

void app_setup();

#endif
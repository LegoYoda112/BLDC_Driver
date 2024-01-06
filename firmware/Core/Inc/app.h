#ifndef APP_H
#define APP_H

#include "main.h"

#include "cmsis_os.h"

#include "led.h"
#include "comms.h"
#include "sensors.h"
#include "drive.h"
#include "app_timers.h"

extern ADC_HandleTypeDef hadc2;

enum appState {
    app_state_idle,
    app_state_active
};

extern enum appState app_state;

void app_setup();

#endif
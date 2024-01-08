#ifndef APP_H
#define APP_H

#include "main.h"

#include "cmsis_os.h"

#include "led.h"
#include "comms.h"
#include "sensors.h"
#include "drive.h"
#include "foc.h"
#include "app_timers.h"

extern ADC_HandleTypeDef hadc2;

enum appState {
    app_state_idle,
    app_state_active
};

extern enum appState app_state;

extern int16_t current_A_mA_filtered;
extern int16_t current_B_mA_filtered;
extern int16_t current_C_mA_filtered;

extern int16_t current_Alpha_mA;
extern int16_t current_Beta_mA;

extern int16_t current_Q_mA;
extern int16_t current_D_mA;

extern int16_t angle;
extern uint8_t electrical_angle;

extern position_setpoint;

void app_setup();

#endif
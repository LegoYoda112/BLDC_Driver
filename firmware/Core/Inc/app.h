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

extern int position_setpoint;

/**
 * @brief Run during MCU init to initialise various perihperals.
 * 
 * @note Order of operations
 * - Starts ADC
 * - Enables the DRV chip and calibrates amps
 * - Start timers
 * - Sets absolute encoder offset
 * - Zero out phase PWM
 * - Enable low side gates
 * - Start FOC interrupt
 */
void app_setup();

/**s
 * @brief RTOS task to run status led
 * 
 */
void app_status_led_task();

#endif
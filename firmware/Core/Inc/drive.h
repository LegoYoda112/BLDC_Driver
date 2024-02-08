#ifndef DRIVE_H
#define DRIVE_H

#include "main.h"
#include "sensors.h"
#include "math.h"

#include "trig_luts.h"

#include "stdbool.h"

#include "cmsis_os.h"

//////////// DEFINES
#define MIN_MOTOR_RESISTANCE_OHM 0.1f
#define MAX_MOTOR_RESISTANCE_OHM 10.0f

#define MIN_SUPPLY_VOLTAGE_V 10

//////////// ENUMS
enum DriveError {
    drive_error_none,
    drive_error_high_resistance,
    drive_error_low_resistance,
    drive_error_low_voltage
};

extern enum DriveError drive_error;

enum DriveState {
    drive_state_error,
    drive_state_disabled, // All phases disabled
    drive_state_resistance_estimation,
    drive_state_encoder_calibration,
    drive_state_anti_cogging_calibration,
    drive_state_idle,     // Phases active but no control running
    drive_state_position_control
};

extern enum DriveState drive_state;

// If true, FOC is allowed to control motor phases
extern bool foc_active;
extern bool anti_cogging_enabled;

//////////// VARIABLES
extern int max_motor_current_mAmps;

extern int estimated_resistance_mOhms;

extern uint8_t electrical_angle_offset;

extern int current_Q_setpoint_mA;

extern int position_setpoint;

//////////// EXTERNS
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim6;


//////////// FUNCTIONS
void start_drive_timers();

// Enable DRV chip by setting ENABLE pin
void enable_DRV();
// Disable DRV chip by setting ENABLE pin
void disable_DRV();


// Main drive state machine
void drive_state_machine();

/**
 * @brief Enable FOC loop, allowing it to apply phase voltages
 * 
 */
void enable_foc_loop();

/**
 * @brief Disable FOC loop from applying phase voltages 
 * and zero out phases.
 * 
 */
void disable_foc_loop();

/**
 * @brief Estimate phase resistance by applying
 * a known voltage and measuring resulting phase current.
 * 
 * Throws a fault if any resistance is out of line with expected.
 * (0.1 to 10 ohms)
 * 
 * @param voltage voltage to apply during estimation
 * 
 * @note
 * This will disable FOC during the duration of the test as it requires
 * direct access to phases voltages.
 */
void estimate_phase_resistance(float voltage);
void calibrate_encoder(float voltage);

enum DriveError check_supply_voltage();

void apply_duty_at_electrical_angle_int(uint8_t angle, uint8_t magnitude);

void set_duty_phase_A(uint8_t value);
void set_duty_phase_B(uint8_t value);
void set_duty_phase_C(uint8_t value);

void set_duty_phases(uint8_t A_value, uint8_t B_value, uint8_t C_value);

#endif
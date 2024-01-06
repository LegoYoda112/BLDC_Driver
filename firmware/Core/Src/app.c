#include "app.h"

enum appState app_state = app_state_idle;

void app_setup(){

    // Start and calibrate analog 
    // (at beginning since calibration works best with )
    start_ADC();

    // Enable DRV
    enable_DRV();
    calibrate_DRV_amps();

    // Start timers
    start_app_timers();
    start_led_timers();
    start_drive_timers();

    // Start comms
    // init_and_start_can();

    // Set encoder offset
    set_encoder_absolute_offset();

    // Setup DRIVE
    set_duty_phase_A(0);
    set_duty_phase_B(0);
    set_duty_phase_C(0);

    HAL_GPIO_WritePin(INLX_GPIO_Port, INLX_Pin, 1);
}


int led_clock = 0;
void app_status_led_task(){

    estimate_phase_resistance(5.0);

    led_clock += 1;
    switch (app_state){
        case app_state_idle:
            // led_hsv(150.0f, 1.0f, sin(led_clock / 30.0f) * 0.2f + 0.2f);
            led_hsv((enc_angle_int)/(4096.0f / 360.0f) + 10000.0f, 1.0f, sin(led_clock / 20.0f) * 0.3f + 0.4f);
            break;
        case app_state_active:
            if(rx_msg_led){
                led_hsv(led_clock/5.0f, 0.9f, 1.0f);
                rx_msg_led = false;
            }else if(can_bitrate == _500Kbs){
                led_hsv(led_clock/5.0f, 0.9f, 0.5f);
            }else if(can_bitrate == _1000Kbs){
                led_hsv(led_clock/5.0f, 0.9f, 0.5f);
            }
            break;
    }

    osDelay(10);
}
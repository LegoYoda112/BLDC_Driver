#include "app.h"

enum appState app_state = app_state_idle;

uint8_t run_LED_colors[3];

void app_setup(){
    // Start and calibrate analog 
    // (at beginning since calibration works best with )
    start_ADC();

    // Start timers
    // TODO: Does this make sense
    start_led_timers();
    start_drive_timers();
    start_app_timers();

    // Enable DRV
    enable_DRV();
    calibrate_DRV_amps();

    // Start comms
    init_and_start_can();

    // Set encoder offset
    set_encoder_absolute_offset();

    // Setup DRIVE
    set_duty_phases(0, 0, 0);
    // Enable low side gates
    // TODO: Add this into a "drive enable/disable" function
    HAL_GPIO_WritePin(INLX_GPIO_Port, INLX_Pin, 1);

    // 10kHz motor commutation interrupt
    // TODO: Add this back into drive timers?
    // Can't quite remember why it's here
    disable_foc_loop();
    HAL_TIM_Base_Start_IT(&htim6);


    run_LED_colors[0] = 254; // Red
    run_LED_colors[1] = 0; // Green
    run_LED_colors[2] = 254; // Blue
}


int led_clock = 0;
char str[50];

void app_status_led_task(){

    led_clock += 1;

    // led_rgb_int(run_LED_colors[0], run_LED_colors[1], run_LED_colors[2]);

    switch(drive_state){
        case drive_state_error:
            led_rgb(1.0, 0, 0);
            osDelay(50);
            led_rgb(0, 0, 0);
            osDelay(150);
            break;
        case drive_state_disabled:
            led_hsv(182.0f, 1.0f, sin(led_clock/50.0f) * 0.2f + 0.4f);
            break;
    }
    
    // switch (app_state){
    //     case app_state_idle:
    //         // led_hsv(150.0f, 1.0f, sin(led_clock / 30.0f) * 0.2f + 0.2f);
    //         // led_hsv((enc_angle_int)/(4096.0f / 360.0f) + 10000.0f, 1.0f, sin(led_clock / 100.0f) * 0.3f + 0.4f);
    //         led_rgb_int(run_LED_colors[0], run_LED_colors[1], run_LED_colors[2]);
    //         break;
    //     case app_state_active:
    //         if(rx_msg_led){
    //             led_hsv(led_clock/5.0f, 0.9f, 1.0f);
    //             rx_msg_led = false;
    //         }else if(can_bitrate == _500Kbs){
    //             led_hsv(led_clock/5.0f, 0.9f, 0.5f);
    //         }else if(can_bitrate == _1000Kbs){
    //             led_hsv(led_clock/5.0f, 0.9f, 0.5f);
    //         }
    //         break;
    // }
    
    // enable_foc_loop();
    // disable_foc_loop();

    osDelay(10);
}
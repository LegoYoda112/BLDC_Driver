#include "app.h"
#include "comms.h"
#include "sensors.h"
#include "drive.h"
#include "cmsis_os.h"
#include "app_timers.h"

enum appState app_state = app_state_idle;

void app_setup(){
    // Start wait timer
    start_app_timers();
    start_led_timers();

    set_led_red_pwm(0.0f);
    set_led_green_pwm(0.0f);
    set_led_blue_pwm(0.0f);

    // Setup comms
    // init_and_start_can();

    // Setup sensors
    set_encoder_absolute_offset();

    start_ADC_DMA();


    // Setup DRIVE
    start_drive_PWM();

    enable_DRV();

    set_duty_phase_A(20);
    set_duty_phase_B(0);
    set_duty_phase_C(0);

    HAL_GPIO_WritePin(INLX_GPIO_Port, INLX_Pin, 1);
}


int led_clock = 0;
void app_status_led_task(){

    // volatile uint8_t a = HAL_ADC_GetValue(&hadc2);
    // volatile uint8_t b = HAL_ADC_GetValue(&hadc2);
    // volatile uint8_t c = HAL_ADC_GetValue(&hadc2);

    // led_clock += 1;
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
#include "app.h"
#include "comms.h"
#include "sensors.h"
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

    init_and_start_can();

    set_encoder_absolute_offset();
}


int led_clock = 0;
void app_status_led_task(){
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
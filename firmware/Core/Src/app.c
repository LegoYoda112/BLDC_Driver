#include "app.h"
#include "comms.h"

enum appState app_state = app_state_idle;

void app_setup(){
    start_led_timers();

    set_led_red_pwm(0.0f);
    set_led_green_pwm(0.0f);
    set_led_blue_pwm(0.0f);

    init_and_start_can();
}


int led_clock = 0;
void app_status_led_task(){
    led_clock += 1;
    switch (app_state){
        case app_state_idle:
            led_hsv(150.0f, 1.0f, sin(led_clock / 30.0f) * 0.2f + 0.2f);
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
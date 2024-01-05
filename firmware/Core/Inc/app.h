#ifndef APP_H
#define APP_H

#include "main.h"
#include "led.h"

enum appState {
    app_state_idle,
    app_state_active
};

extern enum appState app_state;

void app_setup();

#endif
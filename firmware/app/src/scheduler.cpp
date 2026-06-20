#include "scheduler.h"

using namespace scheduler;

void scheduler::initialize_scheduler(){
    HAL_TIM_Base_Start(&htim2);
}


int scheduler::period_from_seconds(float seconds){
    return seconds * CLOCK_RATE;
}

int scheduler::period_from_hertz(int hertz){
    return CLOCK_RATE / hertz;
}

bool scheduler::PeriodicTask::tick(){
    if(timer->Instance->CNT - last_run_us > period_us) {
        // TODO: This slips slightly
        last_run_us = timer->Instance->CNT;
        return true;
    } else {
        return false;
    }
}
#include "drive.h"
#include "stdbool.h"
#include "sensors.h"

float power = 10;
uint16_t target_elec_angle = 0;
bool foc_drive_enabled = false;

int motor_poles;
int counts_per_pole;
int elec_angle;
int elec_angle_offset;
uint16_t elec_angle_uint16;



void set_phase_A(uint16_t value){
    TIM1->CCR2 = value;
}

void set_phase_B(uint16_t value){
    TIM1->CCR1 = value;
}

void set_phase_C(uint16_t value){
    TIM1->CCR3 = value;
}

void apply_voltage_at_electrical_angle(float angle, uint8_t power){
    volatile float A_value = sinf(angle);
    volatile float B_value = sinf(angle + 2.0f * PI / 3.0f);
    volatile float C_value = sinf(angle + 4.0f * PI / 3.0f);

    set_phase_A( power * A_value + power );
    set_phase_B( power * B_value + power );
    set_phase_C( power * C_value + power );
}

float angle = 0.0;
void foc_interrupt(){

    // calc_elec_angle();
    // target_elec_angle = elec_angle + UINT16_MAX/2;


    elec_angle = (int)((enc_angle_uint12) % (counts_per_pole)) - elec_angle_offset;
    if(elec_angle > 0){
        elec_angle_uint16 = elec_angle;
    }else{
        elec_angle_uint16 = elec_angle + (counts_per_pole);
    }
    elec_angle_uint16 = elec_angle_uint16 * (65536.0 / (counts_per_pole));


    // angle = ((float)elec_angle_uint16 / 65536.0) * (PI * 2.0) + PI;

    angle -= 0.02;

    if(angle > 2*PI){
        angle -= 2*PI;
    }else if(angle < 0){
        angle += 2*PI;
    }

    if(foc_drive_enabled){
        // set_phase_B( sinf(i/100.0 + (0 * PI / 3.0f)) * (power/2) + power/2 );
        // set_phase_A( sinf(i/100.0 + (2 * PI / 3.0f)) * (power/2) + power/2 );
        // set_phase_C( sinf(i/100.0 + (4 * PI / 3.0f)) * (power/2) + power/2 );

        // apply_voltage_at_electrical_angle(angle, 5);
        apply_voltage_at_electrical_angle(angle, 10);
    }

    // target_elec_angle += 100;
}

void enable_foc_loop(){
    foc_drive_enabled = true;
}

void disable_foc_loop(){
    foc_drive_enabled = false;
}

void step3(int calibration_voltage){
    set_phase_A(calibration_voltage);
    osDelay(80);
    set_phase_A(0);

    set_phase_B(calibration_voltage);
    osDelay(80);
    set_phase_B(0);

    set_phase_C(calibration_voltage);
    osDelay(80);
    set_phase_C(0);
}

void spin_once(int calibration_voltage){
    step3(calibration_voltage); 
    step3(calibration_voltage); 
    step3(calibration_voltage); 

    step3(calibration_voltage);
    step3(calibration_voltage);
    step3(calibration_voltage);

    step3(calibration_voltage);
    step3(calibration_voltage);

    set_phase_A(calibration_voltage);
    osDelay(80);
    set_phase_A(0);
}

void calibrate_encoder_offset(int calibration_voltage){
    disable_foc_loop();

    // 24 phase steps takes you around once
    // Encoder is accurate (at least out to 40 rotations)

    // 14 magnets (7 poles)
    // 12 coils

    volatile int start_zero = enc_angle_int;
    // spin_once(calibration_voltage);

    // volatile int end_zero = enc_angle_int;

    // set_phase_A(calibration_voltage);
    // osDelay(80);
    // set_phase_A(0);
    

    // set_phase_B(calibration_voltage);
    // osDelay(80);
    // set_phase_B(0);

    // set_phase_C(calibration_voltage);
    // osDelay(80);
    // set_phase_C(0);

    

    // // volatile int estimated_pole_pairs = abs((int)round((4096 / ((enc_A - enc_C) / 2.0f))/3));
    // volatile int motor_poles = 14;
    // counts_per_pole = (4096 / motor_poles);

    // volatile int test = (start_zero - end_zero) / 5;
    // volatile int test2 = (end_zero + (2 * counts_per_pole));


    // volatile int average_zero = ((start_zero + (end_zero - (3 * counts_per_pole))) / 2) % (counts_per_pole / 3);
    // // volatile int test2 = (int)fmod((start_zero + 4096.0), 4096.0f/(7.0f*3.0f));

    // int offset = average_zero % (counts_per_pole);
    // if(offset < 0){
    //     offset += counts_per_pole;
    // }

    // elec_angle_offset = offset;
    // // elec_angle_offset = 96;

    enable_foc_loop();
}
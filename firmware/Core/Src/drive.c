#include "drive.h"


enum DriveError drive_error = drive_error_none;
enum DriveState drive_state = drive_state_init;

uint16_t voltage_supply_mV;


//////////// VARIABLES
float motor_kV = NAN;
float motor_kT = 1.0f;
int estimated_resistance_mOhms = -1;

// Control variables
float torque_target_Nm = 0.0f;
float torque_max_Nm = 20.0f;

float velocity_radsps = 0.0f;
float kV = 0.0f;
float velocity_target_radsps = 0.0f; 
float velocity_max_radsps = 200.0f; // ~1000 rpm

float kP = 0.5f; // for impedance control
float kPV = 0.0f; // for cascaded position control
float position_rads = 0;
float position_target_rads = 0.0f;
float position_max_rads = NAN; // no position limit
float position_min_rads = NAN;

int16_t current_offsets[256];


void start_drive_timers(){
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
}

void enable_DRV(){
    HAL_GPIO_WritePin(DRV_EN_GPIO_Port, DRV_EN_Pin, 1);
}

void disable_DRV(){
    HAL_GPIO_WritePin(DRV_EN_GPIO_Port, DRV_EN_Pin, 0);
}

void enable_foc_loop(){
    foc_active = true;
    HAL_GPIO_WritePin(INLX_GPIO_Port, INLX_Pin, 1);
}

void disable_foc_loop(){
    foc_active = false;
    set_duty_phases(0, 0, 0);
    // HAL_GPIO_WritePin(INLX_GPIO_Port, INLX_Pin, 0);
}

// State transitions
bool request_drive_state_change(enum DriveState new_state){
    // Error -> Init
    // Also clears error
    if(new_state == drive_state_init && drive_state == drive_state_error){
        drive_state = drive_state_init;
        drive_error = drive_error_none;
        return true;
    }

    // Disabled -> Encoder calibration
    if(new_state == drive_state_encoder_calibration && drive_state == drive_state_disabled){
        drive_state = drive_state_encoder_calibration;
        return true;
    }

    // Disabled -> Resistance estimation
    if(new_state == drive_state_resistance_estimation && drive_state == drive_state_disabled){
        drive_state = drive_state_resistance_estimation;
        return true;
    }

    // Disabled -> Anti cogging calibration
    if(new_state == drive_state_anti_cogging_calibration && drive_state == drive_state_disabled){
        drive_state = drive_state_anti_cogging_calibration;
        return true;
    }

    // Disabled -> Idle
    if(new_state == drive_state_idle && drive_state == drive_state_disabled){
        // TODO: Check that encoder has been calibrated
        drive_state = drive_state_idle;
        return true;
    }

    // Idle -> Disabled
    if(new_state == drive_state_disabled && drive_state == drive_state_idle){
        // TODO: Check that encoder has been calibrated
        drive_state = drive_state_disabled;
        return true;
    }

    // Idle -> Any control mode
    if((new_state == drive_state_torque_control ||
        new_state == drive_state_velocity_control ||
        new_state == drive_state_position_control ||
        new_state == drive_state_impedance_control)
       && drive_state == drive_state_idle){
        drive_state = new_state;
        return true;
    }


    // Any control mode -> Idle
    if((new_state == drive_state_idle) 
       && (drive_state == drive_state_torque_control ||
           drive_state == drive_state_velocity_control ||
           drive_state == drive_state_position_control ||
           drive_state == drive_state_impedance_control) ){
        drive_state = drive_state_idle;
        return true;
    }

    // Any control mode -> disabled
    if((new_state == drive_state_disabled) 
       && (
           drive_state == drive_state_idle ||
           drive_state == drive_state_torque_control ||
           drive_state == drive_state_velocity_control ||
           drive_state == drive_state_position_control ||
           drive_state == drive_state_impedance_control) ){
        drive_state = drive_state_disabled;
        return true;
    }

    // Else, return false
    return false;
}


// Attempt to initialise DRV chip and perform basic checks
void drive_init(){
    // Check voltage (DRV chip doesn't run without motor supply voltage)
    if(check_supply_voltage() != drive_error_none){
        return;
    }

    // Enable DRV
    enable_DRV();
    calibrate_DRV_amps();

    // Enable low side gates
    // Set all phases to 0 before enabling
    set_duty_phases(0, 0, 0);
    HAL_GPIO_WritePin(INLX_GPIO_Port, INLX_Pin, 1);

    // Start 10kHz motor commutation interrupt
    // Make sure FOC loop is disabled
    disable_foc_loop();
    HAL_TIM_Base_Start_IT(&htim6);

    // Start 1kHz control looop
    HAL_TIM_Base_Start_IT(&htim16);



    #ifdef HARDCODED_MOTOR_PARAMETERS
        electrical_mechanical_ratio = HARDCODED_MOTOR_POLES;
        electrical_angle_offset = HARDCODED_ELECTRICAL_ANGLE_OFFSET;
    #endif

    #ifdef MOTOR_REVERSED  
        motor_reversed = true;
    #endif

    #ifdef USE_MEM
        // Read in memory values
        electrical_angle_offset = mem_read_uint8(2, 0);
    #endif
 
    printf("lol lmao");
}


// Enter drive error
// This is a function since we want to do some other things automatically 
// when a fault occurs (e.g. disabling foc control)
void enter_drive_error(enum DriveError error){
    drive_error = error;
    drive_state = drive_state_error;
    disable_foc_loop();
}

void drive_state_machine(){

    switch(drive_state){

        // "Off" modes
        case drive_state_error:
            break;

        case drive_state_init:
            drive_init();
            if(drive_state != drive_state_error){
                drive_state = drive_state_disabled;
            }
            break;

        case drive_state_disabled:
            disable_foc_loop();
            break;


        // Calibration modes
        case drive_state_resistance_estimation:
            estimate_phase_resistance(5.0f);
            if(drive_state != drive_state_error){
                drive_state = drive_state_disabled;
            }
            break;

        case drive_state_encoder_calibration:
            calibrate_encoder(4.0f);
            if(drive_state != drive_state_error){
                drive_state = drive_state_disabled;
            }
            break;

        case drive_state_anti_cogging_calibration:
            anti_cogging_calibration(1.0f);
            if(drive_state != drive_state_error){
                drive_state = drive_state_disabled;
            }
            break;


        // Control modes
        case drive_state_idle:
            enable_foc_loop();
            break;

        case drive_state_torque_control:
            enable_foc_loop();
            break;

        case drive_state_velocity_control:
            enable_foc_loop();
            break;
        
        case drive_state_position_control:
            enable_foc_loop();
            break;


    }


    // Check voltage
    // TODO make this fast
    if(get_vsupply() < MIN_SUPPLY_VOLTAGE_V){
        enter_drive_error(drive_error_low_voltage);
    }
    osDelay(1);
}




void anti_cogging_calibration(float current){
    // TODO finish this
    // TODO allow current and P (I?) gains to be adjusted

    // Enable FOC, disable anti-cogging, move the motor to 0 and wait for it to settle
    // enable_foc_loop();
    // anti_cogging_enabled = false;
    
    // position_target_rads = 0; // TODO: this is bad
    // osDelay(1000);

    // // Zero out offset table
    // for(int i = 0; i < 256; i++){
    //     current_offsets[i] = 0;
    // }

    // // Move backward 4 electrical revs
    // for(int i = 0; i<256 * 4; i++){
    //     position_target_rads = -i * 2;
    //     osDelay(10);
    //     uint8_t setpoint_electrical_angle = (position_target_rads % (4096 / 8) / 2 + electrical_angle_offset);
    //     current_offsets[setpoint_electrical_angle] += current_Q_setpoint_mA;
    // }

    // // Move forward 4 electrical revs
    // for(int i = 0; i<256 * 4; i++){
    //     position_target_rads = i * 2 - 255 * 4 * 2;
    //     osDelay(10);
    //     uint8_t setpoint_electrical_angle = (position_target_rads % (4096 / 8) / 2 + electrical_angle_offset);
    //     current_offsets[setpoint_electrical_angle] += current_Q_setpoint_mA;
    // }

    // // Average samples
    // for(int i = 0; i < 256; i++){
    //     current_offsets[i] = current_offsets[i] / 8;
    // }
    
    // // Disable FOC loop and enable anti-cogging
    // disable_foc_loop();
    anti_cogging_enabled = false;
}

// Currently only checks phase A as that amp seems to be the most accurate
void estimate_phase_resistance(float voltage){

    // Verify we have motor supply voltage
    if(check_supply_voltage() != drive_error_none){
        return;
    }

    disable_foc_loop();
    HAL_GPIO_WritePin(INLX_GPIO_Port, INLX_Pin, 1);

    // Calculate required duty cycle for voltage
    float input_voltage = get_vsupply();
    float desired_voltage = voltage;
    int duty = (int)((desired_voltage / input_voltage) * 256);

    // Apply voltage
    set_duty_phases(0, duty, 0);
    osDelay(50);

    // Sample current
    int current_adc = 0;
    const int NUM_SAMPLES = 10;
    for(int i = 0; i < 10; i++){
        current_adc += adc2_dma[0];
        osDelay(1);
    }

    current_adc = current_adc / NUM_SAMPLES;

    // Reset phases
    set_duty_phases(0, 0, 0);

    float measured_shunt_voltage = ((current_adc / 4096.0 - 0.5) * 2.8f) / 40.0f;
    float measured_shunt_current = fabs(measured_shunt_voltage / 0.002f);
    
    // Calculate resistance
    estimated_resistance_mOhms = (int) (desired_voltage / measured_shunt_current * 1000);


    // Check if resistance is within expected values
    if(estimated_resistance_mOhms > MAX_MOTOR_RESISTANCE_OHM * 1000){
        drive_state = drive_state_error;
        drive_error = drive_error_high_resistance;
        return;
    }else if(estimated_resistance_mOhms < MIN_MOTOR_RESISTANCE_OHM * 1000){
        drive_state = drive_state_error;
        drive_error = drive_error_high_resistance;
        return;
    }
}

// Convert from mechanical (encoder based) to electrical angle
uint8_t convert_to_electrical_angle(int mechanical_angle, int ratio, int offset){
    volatile uint32_t temp = ((uint32_t)mechanical_angle % 4096);
    temp = temp % (4096 / ratio);
    temp = temp * (ratio * 256);
    temp = temp / 4096;
    return temp - offset;
}

// Calculate encoder offset
// TODO: Estimate hysteresis?
void calibrate_encoder(float voltage){
    disable_foc_loop();
    HAL_GPIO_WritePin(INLX_GPIO_Port, INLX_Pin, 1);
    
    // Calculate required duty cycle for voltage
    float input_voltage = get_vsupply();
    float desired_voltage = voltage;
    int duty = (int)((desired_voltage / input_voltage) * 256);
    duty = 30;

    volatile int offset_average = 0;

    // int pole_count = 8;

    apply_duty_at_electrical_angle_int(0, duty);
    osDelay(100);

    volatile int16_t starting_angle = enc_angle_int;

    // Calculate ratio
    // int electrical_turns = 21;
    // for(int i = 0; i<=256 * electrical_turns; i++){
    //     uint8_t applied_electrical_angle = (uint8_t) (int) (i) % 256;
    //     apply_duty_at_electrical_angle_int(applied_electrical_angle, duty);
        
    //     osDelay(2);
    // }
    // set_duty_phases(0,0,0);
    // int16_t finishing_angle = enc_angle_int;
    // float ratio_estimate = (4096.0 / ((float)finishing_angle - (float)starting_angle) * electrical_turns);
    // electrical_mechanical_ratio = roundf(ratio_estimate);

    // if(USE_MEM){
    //     // TODO: Calculate this based on motor movement
    // } else {
    //     electrical_mechanical_ratio = HARDCODED_ELECTRICAL_ANGLE_OFFSET;
    // }

    // Spin forward
    for(int i = 0; i<=256 * 2; i++){
        uint8_t applied_electrical_angle = (uint8_t) (int) (i) % 256;
        apply_duty_at_electrical_angle_int(applied_electrical_angle, duty);
        osDelay(5);

        volatile uint8_t measured_electrical_angle = convert_to_electrical_angle(enc_angle_int, electrical_mechanical_ratio, 0);
        
        int offset = measured_electrical_angle - applied_electrical_angle;
        if(offset < 0){
            offset = 256 + offset;
        }

        offset_average += offset;
    }

    // Spin backward
    for(int i = 0; i<=256 * 2; i++){
        uint8_t applied_electrical_angle = (uint8_t) (int) (-i) % 256;
        apply_duty_at_electrical_angle_int(applied_electrical_angle, duty);
        osDelay(5);

        volatile uint8_t measured_electrical_angle = convert_to_electrical_angle(enc_angle_int, electrical_mechanical_ratio, 0);
        
        int offset = measured_electrical_angle - applied_electrical_angle;
        if(offset < 0){
            offset = 256 + offset;
        }

        offset_average += offset;
    }

    set_duty_phases(0,0,0);
    


    electrical_angle_offset = offset_average / (256 * 4);

    mem_write_uint8(electrical_angle_offset, 2, 0);
    printf("lmao");
}


void apply_duty_at_electrical_angle_int(uint8_t angle, uint8_t magnitude){
    angle = (angle + (1.0 / 4.0) * 256);

    uint8_t A_value = mult_sin_lut_uint8((uint8_t) (angle + (0.0 / 3.0) * 256), magnitude);
    uint8_t B_value = mult_sin_lut_uint8((uint8_t) (angle + (1.0 / 3.0) * 256), magnitude);
    uint8_t C_value = mult_sin_lut_uint8((uint8_t) (angle + (2.0 / 3.0) * 256), magnitude);

    set_duty_phase_A(A_value);
    set_duty_phase_B(B_value);
    set_duty_phase_C(C_value);
}


void control_loop() {
    // Update voltage (used in foc loop)
    v_supply_mv = get_vsupply_mv_fast();

    // Convert position to physical units
    position_rads = (adjusted_enc_angle / 4096.0f) * 2.0f * M_PI;

    // Convert velocity to physical units
    velocity_radsps = (encoder_velocity / 4096.0f) * 2.0f * M_PI;

    if(drive_state == drive_state_idle){
        set_torque_setpoint(0.0f);

    } else if(drive_state == drive_state_torque_control){ // Pure torque control
        set_torque_setpoint(torque_target_Nm);

    } else if(drive_state == drive_state_velocity_control){ // Proportional velocity control
        float velocity_error = velocity_radsps - fbound_sym(velocity_target_radsps, velocity_max_radsps);
        set_torque_setpoint(
            -kV * velocity_error
        );

    } else if(drive_state == drive_state_position_control){ // Cascaded position -> velocity control
        float position_error = position_rads - fbound(position_target_rads, position_min_rads, position_max_rads);
        float velocity_target = -kPV * position_error;

        float velocity_error = velocity_radsps - fbound_sym(velocity_target, velocity_max_radsps);
        set_torque_setpoint(
            -kV * velocity_error
        );

    } else if(drive_state == drive_state_impedance_control){ // Combined proportional position and velocity control
        float position_error = position_rads - fbound(position_target_rads, position_min_rads, position_max_rads);
        float velocity_error = velocity_radsps - fbound_sym(velocity_target_radsps, velocity_max_radsps);

        set_torque_setpoint(
            -kP * position_error +
            -kV * velocity_error
        );
    }
}

void set_torque_setpoint(float torque_target_Nm){
    // Bound target torque and convert to target current using motor kT
    // TODO: Solve this upstream somewhere
    // if(motor_reversed){
    // torque_target_Nm = -torque_target_Nm;
    // }
    set_current_setpoints(0, motor_kT * fbound_sym(torque_target_Nm, torque_max_Nm) * 1000);
};


enum DriveError check_supply_voltage() {
    if(get_vsupply() < MIN_SUPPLY_VOLTAGE_V){
        enter_drive_error(drive_error_low_voltage);
    }

    return drive_error;
}

void set_duty_phase_A(uint8_t value){
    TIM1->CCR3 = value;
}

void set_duty_phase_B(uint8_t value){
    TIM1->CCR2 = value;
}

void set_duty_phase_C(uint8_t value){
    TIM1->CCR1 = value;
}

void set_duty_phases(uint8_t A_value, uint8_t B_value, uint8_t C_value){
    set_duty_phase_A(A_value);
    set_duty_phase_B(B_value);
    set_duty_phase_C(C_value);
}

void set_V_phases_mv(uint16_t A_value_mv, uint16_t B_value_mv, uint16_t C_value_mv){
    set_duty_phase_A((A_value_mv * 256) / v_supply_mv);
    set_duty_phase_B((B_value_mv * 256) / v_supply_mv);
    set_duty_phase_C((C_value_mv * 256) / v_supply_mv);
}

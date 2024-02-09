
#define DEVICE_TYPE_ID 50

////////////////////////////////// DIAGNOSTIC (0, 19)

// MESSAGE Device UID
// LAYOUT [type, uid, uid, uid, uid]
// UNIT [device type, unique chip id]
#define DIAGNOSTIC_UID_ID 5

#define DIAGNOSTIC_HARD_RESET_ID 10

//////////// ACTION

#define ACTION_REQUEST_STATE_CHANGE 20
// DEF [new state]

#define ACTION_MOTOR_ENABLE_POSITION_CONTROL 50
#define ACTION_MOTOR_POSITION_SETPOINT 51



//////////// TELEMETRY
#define TELEM_DRIVE_STATE 100
// DEF [drive state, drive error]

#define TELEM_MOTOR_POSITION 101

#define TELEM_MOTOR_PHASE_RESISTANCE 110
// Motor parameters
// DEF [R, R]
// NAME Motor phase resistance
// UNIT 

#define TELEM_MOTOR_TORQUE 111
// DEF [Q, Q]
// NAME Motor currents read through shunt
// UNIT mAmp
// SIGNED

#define TELEM_MOTOR_VOLTAGE 112
// DEF [V, V]
// NAME Supply voltage reading
// UNIT mV


//////////// PARAMETERS
#define PARAM_LED_COLOR 150
// DEF [R, G, B]
// NAME Motor run LED color

#define PARAM_PHASE_RESISTANCE 160
// DEF [resistance, resistance]
// UNIT mOhms

#define PARAM_ANTI_COGGING 170



////////////////////////////////// SYSTEM (msgs sent on ID 0)
#define SYS_STOP_ALL_ID 0
#define SYS_HARD_RESET_ALL_ID 1

#define SYS_LIST_ALL_ID 5

#define SYS_LOCATE_ID 11 // Blinks LEDs to locate a device based on it's UID

#define SYS_SET_CAN_ID_ID 12
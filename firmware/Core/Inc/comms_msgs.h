
//////////// ACTION
#define ACTION_MOTOR_ENABLE_POSITION_CONTROL 50
#define ACTION_MOTOR_POSITION_SETPOINT 51



//////////// TELEMETRY
#define TELEM_DRIVE_STATE_ID 100
// DEF [drive state, drive error]

#define TELEM_MOTOR_POSITION 101

#define TELEM_MOTOR_PHASE_RESISTANCE 110
// Motor parameters
// DEF [R, R]
// NAME Motor phase resistance
// UNIT 

#define TELEM_MOTOR_CURRENTS 111
// DEF [A, A, B, B, C, C]
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
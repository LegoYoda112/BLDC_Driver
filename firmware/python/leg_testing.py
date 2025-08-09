from FxFDrive import FxFDrive, make_can_bus, list_available_drives, bytes_to_int, int_to_bytes
from FxFDrive import DriveError, DriveState
import time
import numpy as np
from matplotlib import pyplot as plt

import serial 
import re
import math
import threading

imu_target = ""
ser = serial.Serial('/dev/cu.usbserial-0001',
        baudrate=115200,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=1,
        xonxoff=0,
        rtscts=0)

ser.flush()

rpy = [0, 0, 0]
gyro = [0,0,0]

# Shout out https://discuss.luxonis.com/d/5453-how-to-convert-quaternions-to-pitchrollyaw
def quaternion_to_euler(quat):
    q_w = quat[0]
    q_x = quat[1]
    q_y = quat[2]
    q_z = quat[3]
    # Roll (x-axis rotation)
    roll = math.atan2(2 * (q_w * q_x + q_y * q_z), 1 - 2 * (q_x**2 + q_y**2))
    
    # Pitch (y-axis rotation)
    pitch = math.asin(2 * (q_w * q_y - q_z * q_x))
    
    # Yaw (z-axis rotation)
    yaw = math.atan2(2 * (q_w * q_z + q_x * q_y), 1 - 2 * (q_y**2 + q_z**2))
    
    return roll, pitch, yaw

def imu_thread_func():
    global rpy
    global gyro
    while(True):
        line = ser.readline()
        try:
            data = np.array(re.findall(r"[-0-9.]+", str(line)), float)
        except ValueError:
            data = [1,0,0,0]
        if(len(data) != 10):
            data = [1,0,0,0]
        quat = data[0:4]
        rpy = quaternion_to_euler(quat)

        gyro = data[4:7]

imu_thread = threading.Thread(target=imu_thread_func, args=(), daemon=True)

imu_thread.start()


print("Starting can bus")
bus = make_can_bus()
motor_1 = FxFDrive(bus, 10)
motor_2 = FxFDrive(bus, 11)
wheel_1 = FxFDrive(bus, 12)
wheel_2 = FxFDrive(bus, 13)
print("Started...")

list_available_drives(bus)

max_current = 7_000

max_wheel_current = 12_000


wheel_1.set_parameter_int(wheel_1.parameters.PARAM_KV, 18_000, 3)
wheel_1.set_parameter_int(wheel_1.parameters.PARAM_MAXIMUM_MOTOR_CURRENT, max_wheel_current, 2)

wheel_2.set_parameter_int(wheel_1.parameters.PARAM_KV, 18_000, 3)
wheel_2.set_parameter_int(wheel_1.parameters.PARAM_MAXIMUM_MOTOR_CURRENT, max_wheel_current, 2)


motor_1.set_parameter_int(motor_1.parameters.PARAM_KPV, 40_000, 3)
motor_1.set_parameter_int(motor_1.parameters.PARAM_KP, 10_000, 3)
motor_1.set_parameter_int(motor_1.parameters.PARAM_KV, 270, 3)
motor_1.set_parameter_int(motor_1.parameters.PARAM_MAXIMUM_MOTOR_CURRENT, max_current, 2)

motor_2.set_parameter_int(motor_2.parameters.PARAM_KPV, 40_000, 3)
motor_2.set_parameter_int(motor_2.parameters.PARAM_KP, 10_000, 3)
motor_2.set_parameter_int(motor_2.parameters.PARAM_KV, 270, 3)
motor_2.set_parameter_int(motor_2.parameters.PARAM_MAXIMUM_MOTOR_CURRENT, max_current, 2)



input("Transition to idle")
time.sleep(0.01)
motor_1.action.request_state_change(DriveState.drive_state_idle.value)
time.sleep(0.01)
motor_2.action.request_state_change(DriveState.drive_state_idle.value)
time.sleep(0.01)
wheel_1.action.request_state_change(DriveState.drive_state_idle.value)
time.sleep(0.01)
wheel_2.action.request_state_change(DriveState.drive_state_idle.value)
# time.sleep(0.1)
input("RUN")
time.sleep(0.01)
motor_1.action.request_state_change(DriveState.drive_state_position_control.value)
time.sleep(0.01)
motor_2.action.request_state_change(DriveState.drive_state_position_control.value)
time.sleep(0.01)
wheel_1.action.request_state_change(DriveState.drive_state_velocity_control.value)
time.sleep(0.01)
wheel_2.action.request_state_change(DriveState.drive_state_velocity_control.value)

motor_1_zero_offset = -0.0
motor_2_zero_offset = -0.0

target = 30.0
roll_offset = 0.0
starting_wheel_position = (wheel_1.telemetry.get_position_rads() - wheel_2.telemetry.get_position_rads())/2.0
wheel_velocity_target = 0.0

try:
   i = 0
   while(True):
        wheel_position = (wheel_1.telemetry.get_position_rads() - wheel_2.telemetry.get_position_rads())/2.0
        wheel_velocity = (wheel_1.telemetry.get_velocity_radsps() - wheel_2.telemetry.get_velocity_radsps())/2.0
        

        
        wheel_velocity_target = -(rpy[1] + 0.03) * 120.0 - gyro[1] * 1.0 + (wheel_position-starting_wheel_position) * 0.000 + wheel_velocity * 0.0
        

        wheel_1.action.send_velocity_target(wheel_velocity_target)
        wheel_2.action.send_velocity_target(-wheel_velocity_target)
        
        # roll_offset += rpy[0] * 1.0 - gyro[0] * 0.5
        motor_1.action.send_position_target(-target + motor_1_zero_offset + max(roll_offset, 0))
        motor_2.action.send_position_target(target + motor_2_zero_offset + min(roll_offset, 0))
        # print(motor_1.telemetry.get_position_rads(), motor_2.telemetry.get_position_rads(), rpy[0])
        print(wheel_position-starting_wheel_position)
        time.sleep(0.01)
        
except KeyboardInterrupt:
    time.sleep(0.01)
    motor_1.action.request_state_change(DriveState.drive_state_disabled.value)
    time.sleep(0.01)
    motor_2.action.request_state_change(DriveState.drive_state_disabled.value)
    time.sleep(0.01)
    wheel_1.action.request_state_change(DriveState.drive_state_disabled.value)
    time.sleep(0.01)
    wheel_2.action.request_state_change(DriveState.drive_state_disabled.value)


# input("Run?")
# motor.action.request_state_change(DriveState.drive_state_idle.value)
# time.sleep(0.05)
# motor.action.request_state_change(DriveState.drive_state_position_control.value)
# motor.action.send_position_target(10.0)


# input()
# try:
#     i = 0
#     while(True):
#         i += 1
#         motor.action.send_position_target(23.0 + 16.0 * np.sin(i / 30.0))
#         time.sleep(0.01)

# except KeyboardInterrupt:
#     motor.action.request_state_change(DriveState.drive_state_disabled.value)

# motor.action.request_state_change(DriveState.drive_state_disabled.value)
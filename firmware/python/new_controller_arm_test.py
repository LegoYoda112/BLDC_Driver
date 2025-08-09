from FxFDrive import FxFDrive, make_can_bus, list_available_drives, bytes_to_int, int_to_bytes
from FxFDrive import DriveError, DriveState
import time
import numpy as np
from matplotlib import pyplot as plt

print("Starting can bus")
bus = make_can_bus()
motor = FxFDrive(bus, 11)
print("Started...")

# drive_state_num, drive_error_num = motor.telemetry.get_drive_state()
# print("Drive state", DriveState(drive_state_num).name)

motor.action.request_state_change(DriveState.drive_state_idle.value)

motor.set_parameter_int(motor.parameters.PARAM_KPV, 5_000, 3)
motor.set_parameter_int(motor.parameters.PARAM_KP, 3_000, 3)
motor.set_parameter_int(motor.parameters.PARAM_KV, 400, 3)
print(motor.get_parameter_int(motor.parameters.PARAM_KV))
motor.set_parameter_int(motor.parameters.PARAM_MAXIMUM_MOTOR_CURRENT, 3_000, 2)

input("Enable control?")
# motor.action.request_state_change(DriveState.drive_state_impedance_control.value)
motor.action.request_state_change(DriveState.drive_state_position_control.value)
motor.action.send_position_target(0.0)
motor.action.send_velocity_target(0.0)

x = []

for i in range(500):
    target = np.sin(i/10.0) * 5.0
    motor.action.send_position_target(target)
    current_position = motor.telemetry.get_position_rads()
    x.append([target, current_position, target - current_position])
    print(i)
    time.sleep(0.01)

# input("shutdown")
motor.action.request_state_change(DriveState.drive_state_idle.value)
bus.shutdown()

x = np.array(x[:, 2])
plt.plot(x)
plt.show()
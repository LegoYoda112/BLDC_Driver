import numpy as np
import time
from FxFDrive import FxFDrive, make_can_bus, list_available_drives

bus = make_can_bus()
print("Bus running")
list_available_drives(bus)

motor = FxFDrive(bus, 11)

# while(True):
#     print(motor.telemetry.get_motor_voltage_V())
#     time.sleep(0.01)

i = 0
while(True):
    motor.parameters.run_led_colors = [10, 0, 10]

    # print("Writing")
    motor.write_parameters()
    # print("Written")

    # print("Reading")
    motor.read_parameters()
    # print("Read")

    # print(motor.parameters)

    i += 1
    # print(i)

# print(motor.parameters.run_led_colors)

# print(motor.get_parameter(150))
# print(motor.set_parameter(150, [0, 0, 255]))

bus.shutdown()
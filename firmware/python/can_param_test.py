import numpy as np
import time
from FxFDrive import FxFDrive, make_can_bus

bus = make_can_bus()
print("Bus running")

motor = FxFDrive(bus, 420)


i = 0
while(True):
    motor.parameters.run_led_colors = [10, 0, 10]

    motor.write_parameters()
    motor.read_parameters()

    # print(motor.parameters)

    i += 1
    print(i)

# print(motor.parameters.run_led_colors)

# print(motor.get_parameter(150))
# print(motor.set_parameter(150, [0, 0, 255]))

bus.shutdown()
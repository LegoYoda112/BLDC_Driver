from FxFDrive import FxFDrive, make_can_bus, list_available_drives, bytes_to_int, int_to_bytes
from FxFDrive import DriveError, DriveState
import time

from matplotlib import pyplot as plt

elbow_anti_cogging = [-142, -150, -165, -187, -187, -195, -202, -187, -217, -210, -232, -240, -240, -270, -247, -270, -277, -292, -277, -300, -285, -300, -307, -300, -315, -307, -300, -330, -307, -307, -322, -307, -300, -292, -292, -270, -270, -277, -270, -262, -270, -255, -240, -247, -255, -277, -285, -187, -270, -187, -180, -165, -165, -45, -52, -7, -30, -22, 0, 0, -15, -7, 30, 60, 75, 105, 112, 127, 135, 135, 157, 165, 157, 172, 165, 195, 195, 217, 225, 225, 225, 225, 232, 247, 247, 255, 255, 270, 255, 247, 262, 240, 232, 247, 247, 240, 225, 225, 210, 202, 202, 202, 187, 180, 172, 165, 150, 157, 135, 112, 112, 105, 105, 82, 90, 67, 52, 67, 52, 52, 22, 0, -7, -7, -15, -37, -37, -30, -45, -37, -60, -22, -37, -45, -67, -60, -75, -82, -105, -97, -90, -112, -120, -105, -120, -97, -142, -135, -105, -120, -120, -127, -127, -150, -142, -135, -150, -135, -150, -142, -150, -135, -142, -135, -120, -142, -120, -120, -90, -90, -67, -60, -37, -22, -45, -15, 0, 15, 37, 7, 30, 30, 60, 60, 45, 52, 60, 67, 75, 75, 90, 97, 135, 157, 180, 195, 195, 210, 225, 217, 240, 247, 247, 240, 270, 232, 262, 240, 240, 247, 217, 232, 232, 217, 225, 240, 217, 217, 202, 202, 202, 187, 187, 195, 187, 187, 187, 187, 165, 165, 157, 150, 135, 127, 105, 90, 67, 60, 37, 30, 22, -7, -7, -30, -52, -30, -67, -60, -82, -75, -97, -105, -120, -127, -127, -157]
shoulder_anti_cogging = [202, 180, 195, 195, 195, 187, 187, 180, 180, 172, 172, 187, 187, 195, 187, 187, 202, 187, 165, 187, 180, 157, 150, 157, 142, 142, 127, 97, 97, 97, 90, 90, 82, 75, 67, 52, 37, 22, 22, 37, 22, 15, 0, -7, 0, 7, -15, -15, -22, -30, -30, -67, -67, -45, -82, -90, -82, -105, -112, -105, -142, -142, -157, -142, -142, -157, -180, -172, -187, -195, -195, -187, -195, -210, -202, -202, -202, -195, -210, -195, -202, -195, -202, -210, -202, -180, -172, -180, -172, -172, -165, -172, -157, -142, -142, -142, -120, -97, -75, -67, -45, -30, -15, 7, 37, 45, 52, 90, 127, 150, 157, 187, 202, 195, 202, 217, 217, 217, 210, 202, 225, 225, 232, 247, 262, 285, 285, 292, 315, 315, 315, 330, 315, 315, 337, 337, 345, 367, 352, 352, 345, 337, 360, 352, 330, 337, 322, 322, 307, 300, 292, 270, 247, 240, 217, 247, 240, 217, 195, 210, 210, 210, 240, 217, 247, 232, 225, 202, 195, 187, 172, 165, 142, 135, 142, 120, 112, 105, 82, 75, 52, 37, 22, 15, 0, -15, -22, -30, -30, -30, -45, -82, -60, -52, -75, -82, -90, -97, -105, -120, -120, -135, -142, -135, -142, -165, -157, -142, -165, -165, -157, -172, -180, -172, -157, -142, -157, -157, -135, -120, -120, -105, -105, -90, -75, -67, -60, -52, -45, -22, -7, 0, 7, 15, 52, 52, 52, 82, 90, 105, 135, 127, 150, 157, 180, 165, 180, 195, 195, 187, 187, 202, 202, 195, 210, 210]

bus = make_can_bus()
print("Bus running")

shoulder = FxFDrive(bus, 10)
elbow = FxFDrive(bus, 11)
time.sleep(0.1)

startup_write = True

if(startup_write):
    print("Writing encoder offsets")
    shoulder.set_parameter_int(shoulder.parameters.PARAM_ENCODER_OFFSET, -98, 1)
    elbow.set_parameter_int(elbow.parameters.PARAM_ENCODER_OFFSET, 49, 1)

    print("Writing anti-cogging tables")
    for i, value in enumerate(elbow_anti_cogging):
        value = [i] + int_to_bytes(2, value)
        elbow.set_parameter(elbow.parameters.PARAM_ANTI_COGGING_TABLE, value)

    for i, value in enumerate(shoulder_anti_cogging):
        value = [i] + int_to_bytes(2, value)
        shoulder.set_parameter(shoulder.parameters.PARAM_ANTI_COGGING_TABLE, value)
    print("Done")

time.sleep(0.1)

print("Setting parameters")
shoulder.set_parameter_int(shoulder.parameters.PARAM_CURRENT_LIMIT, 9000, 2)
elbow.set_parameter_int(elbow.parameters.PARAM_CURRENT_LIMIT, 9000, 2)

shoulder.set_parameter_int(shoulder.parameters.PARAM_ANTI_COGGING, 1, 1)
elbow.set_parameter_int(elbow.parameters.PARAM_ANTI_COGGING, 1, 1)

input("Idle drivers?")
shoulder.action.request_state_change(DriveState.drive_state_idle.value)
elbow.action.request_state_change(DriveState.drive_state_idle.value)

input("Enable position?")
shoulder.action.request_state_change(DriveState.drive_state_position_control.value)
elbow.action.request_state_change(DriveState.drive_state_position_control.value)

def lerp(a, b, factor):
    return a * (1 - factor) + b * factor

def trapazoidal_target(start, end, max_vel, max_acc, t):
    distance = abs(end - start)
    t_a = max_vel / max_acc
    t_c = (distance - t_a * max_vel) / max_vel

    sign = 1
    if(start > end):
        sign = -1

    if(t < 0):
        return start
    elif(t < t_a):
        return start + sign * (0.5 * t**2 * max_acc)
    elif(t < t_a + t_c):
        return start + sign * (0.5 * t_a**2 * max_acc + max_vel * (t - t_a))
    elif(t < t_a*2 + t_c):
        return start + sign * (0.5 * t_a**2 * max_acc + max_vel * t_c + 0.5 * (t_a**2 * max_acc - (t_c + 2 * t_a - t)**2 * max_acc))
    else:
        return end
# def trapazoidal_target_time(distance, max_vel, max_acc):
#     t_a = max_vel / max_acc
#     t_c = (distance - t_a * max_vel) / max_vel

#     return 2*t_a + t_c



# TRAP on each axis
# timestep = 0.002
# for t in range(2000):
#     shoulder_target = int(trapazoidal_target(shoulder_starting_position, shoulder_target_position, 10_000, 20_000, t * timestep))
#     shoulder.action.send_position_target(shoulder_target)

#     elbow_target = int(trapazoidal_target(elbow_starting_position, elbow_target_position, 10_000, 20_000, t * timestep))
#     elbow.action.send_position_target(elbow_target)
#     time.sleep(0.001)
    



shoulder_starting_position = shoulder.telemetry.get_motor_position_encoder_raw()
elbow_starting_position = elbow.telemetry.get_motor_position_encoder_raw()


def trap_to_position(shoulder_target_position, elbow_target_position, samples):
    global shoulder_starting_position
    global elbow_starting_position

    elbow_position = []
    # shoulder_starting_position = shoulder.telemetry.get_motor_position_encoder_raw()
    # elbow_starting_position = elbow.telemetry.get_motor_position_encoder_raw()

    timestep = 0.001
    for t in range(samples):
        factor = trapazoidal_target(0, 1, 4.0, 50.0, t * timestep)
        elbow_target = lerp(elbow_starting_position, elbow_target_position, factor)
        shoulder.action.send_position_target(lerp(shoulder_starting_position, shoulder_target_position, factor))
        elbow.action.send_position_target(elbow_target)
        time.sleep(0.001)
        elbow_position.append([elbow.telemetry.get_motor_position_encoder_raw(), elbow_target])

    shoulder_starting_position = shoulder_target_position
    elbow_starting_position = elbow_target_position

    plt.plot(elbow_position)
    plt.show()

#     shoulder_target_position = -1770
# elbow_target_position = 5000


# input("Move")
# trap_to_position(-9911, -11669, 800)
input("Move")
trap_to_position(-13500, 8000, 1000)
input("Move")
trap_to_position(-17500, -500, 1000)
input("Move")
trap_to_position(-13500, 8000, 1000)




# steps = 500
# for i in range(steps):
#     factor = i / steps
#     shoulder.action.send_position_target(lerp(shoulder_starting_position, shoulder_target_position, factor))
#     elbow.action.send_position_target(lerp(elbow_starting_position, elbow_target_position, factor))
#     time.sleep(0.002)


# shoulder -1769
# elbow 4858


input("Disable drivers?")
shoulder.action.request_state_change(DriveState.drive_state_disabled.value)
elbow.action.request_state_change(DriveState.drive_state_disabled.value)
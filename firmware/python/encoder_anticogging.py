from FxFDrive import FxFDrive, make_can_bus, list_available_drives, bytes_to_int, int_to_bytes
from FxFDrive import DriveError, DriveState
import time

bus = make_can_bus()
print("Bus running")

drive = FxFDrive(bus, 10)
time.sleep(0.1)


print("Encoder calibration")
drive.action.request_state_change(DriveState.drive_state_encoder_calibration.value)

# drive.action.request_state_change(DriveState.drive_state_anti_cogging_calibration.value)

while(drive.telemetry.get_drive_state()[0] != DriveState.drive_state_disabled.value):
    time.sleep(1.0)
    print("waiting")
print("done")

drive.read_parameters()

print(drive.parameters.encoder_offset)
print(drive.get_parameter(drive.parameters.PARAM_ENCODER_OFFSET))
import serial
from serial.tools import list_ports
import time

COM_PORT = None

for port in list_ports.comports():
    if(port.description == "50x50-Drive USB2CAN"):
        COM_PORT = port.device

if(COM_PORT == None):
    raise Exception("No interface found")

print("Interface found on", COM_PORT)

ser = serial.Serial(COM_PORT, 115200)
# time.sleep(0.1)

######## Command a flash erase
print("Entering bootloader")
ser.write(bytes('BTL\r', 'utf-8'))
time.sleep(0.5)

COM_PORT = None

while(COM_PORT == None):
    time.sleep(0.1)
    print("Checking for USB interface")
    for port in list_ports.comports():
        if(port.description == "50x50-Drive USB2CAN"):
            COM_PORT = port.device
            break

print("Interface found on", COM_PORT)

# time.sleep(0.1)
ser = serial.Serial(COM_PORT, 115200)

print("Entering app")
ser.write(bytes('APP\r', 'utf-8'))
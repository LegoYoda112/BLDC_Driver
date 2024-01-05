import serial
from serial.tools import list_ports
import time

for port in list_ports.comports():
    if(port.description == "50x50-Drive USB2CAN"):
        COM_PORT = port.device

if(COM_PORT == None):
    raise Exception("No interface found")

print("Interface found on", COM_PORT)

ser = serial.Serial(COM_PORT, 115200)
time.sleep(0.1)

######## Command a flash erase
print("Entering program flash")
ser.write(bytes('BTL\r', 'utf-8'))
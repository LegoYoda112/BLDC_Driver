import can
from serial.tools import list_ports
import numpy as np
import time

COM_PORT = None
for port in list_ports.comports():
    if(port.description == "50x50-Drive"):
        COM_PORT = port.device

if(COM_PORT == None):
    raise Exception("No interface found")

bus = can.Bus(interface="slcan", channel=COM_PORT, ttyBaudrate = 115200)
bus.set_bitrate(500_000)

def send_array(bus, id, data):
    data = [int(v) for v in data]
    send_msg = can.Message(arbitration_id=id, data=data, is_extended_id=False, dlc=len(data))
    bus.send(send_msg) 

for i in range(1000):
    send_array(bus, 420, [150, 100 * np.sin(i / 10.0) + 100, 100 * np.sin(i / 10.0 + 3.5) + 100, 100 * np.sin(i / 10.0 + 7) + 100])
    print(bus.recv())
    time.sleep(0.01)

bus.shutdown()
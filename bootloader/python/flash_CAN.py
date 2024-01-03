import can
import serial
from serial.tools import list_ports
import time
import stm32bootloader
from read_hex import convert_to_16bit_aligned

convert_to_16bit_aligned("../../firmware/build/50x50_DRIVE.hex")

COM_PORT = stm32bootloader.find_port()

bus = can.Bus(interface="slcan", channel=COM_PORT, ttyBaudrate = 115200)
bus.set_bitrate(500_000)

def build_byte_msg(byte_array):
    return can.Message(
        arbitration_id=69,
        data=byte_array,
        is_extended_id=False
    )

print("Entering bootloader")
bus.send(build_byte_msg(bytearray("BTL", "utf-8")))
time.sleep(0.5)

# print("Clearing flash")
# bus.send(build_byte_msg(bytearray("CLR", "utf-8")))
# bus.recv()

# file = open("16_bit_aligned.hex", "r")

# for line in file:
#     # print(line)
#     record_type = line[7:9]
#     address = line[3:7]
#     print(address)
#     if(record_type == '00'):
#         data_length = line[1:3]
#         data_length_int = int(data_length, 16)

#         data = line[9:9 + 2 * data_length_int]
#         data_list = list(bytearray.fromhex(data))
#         print(data_list)

#         print(data_list[0:8])
#         bus.send(build_byte_msg(data_list[0:8]))
#         bus.recv()

#         time.sleep(0.001)

#         if(data_length_int == 16):
#             # print(build_byte_msg(data_list[8:16]))
#             bus.send(build_byte_msg(data_list[8:16]))
#             bus.recv()
#             time.sleep(0.001)

#     # time.sleep(0.01)

# bus.send(build_byte_msg([]))
# bus.recv()

# time.sleep(0.2)

# print("Entering app")
# bus.send(build_byte_msg(bytearray("APP", "utf-8")))
# time.sleep(0.2)

# print("Done")

bus.shutdown()

# bus.shutdown()


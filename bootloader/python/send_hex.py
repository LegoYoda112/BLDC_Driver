import serial
from serial.tools import list_ports
import time

def num_lines(file):
    num_lines = 0
    for line in file:
        num_lines += 1

    return num_lines

######## Find port
COM_PORT = None

for port in list_ports.comports():
    if(port.description == "50x50-Drive USB2CAN"):
        COM_PORT = port.device

if(COM_PORT == None):
    raise Exception("No interface found")

print("Board found on", COM_PORT)

ser = serial.Serial(COM_PORT, 115200)
time.sleep(0.1)

######## Command a flash erase
print("> Erasing flash")
ser.write(bytes('CLR\r', 'utf-8'))
ret = ser.read_until(b'\r')
print("DONE")

######## Open file
# file = open("/Users/thomasg/Documents/github/50x50_Bootloader/build/50x50_Bootloader.hex", "r")
# total_lines = num_lines(file)
# file.close()


# file = open("/Users/thomasg/Documents/github/50x50_Bootloader/build/50x50_Bootloader.hex", "r")
file = open("16_bit_aligned.hex", "r")

# ELAR = file.readline()[1:-1]
# print(ELAR)

def print_hex_list(data_list):
    print_string = ""
    for byte in data_list:
        print_string += hex(byte) + " "
    print(print_string)


print("> Flashing new firmware")
line_num = 0
for line in file:
    # print(line)

    record_type = line[7:9]
    address = line[3:7]
    if(record_type == '00'):
        data_length = line[1:3]
        data_length_int = int(data_length, 16)

        data = line[9:9 + 2 * data_length_int]
        data_list = list(bytearray.fromhex(data))
        
        # print(address)
        # print_hex_list(data_list)

        ser.write(bytes(line[:-1] + '\r', 'utf-8'))

        ret = ser.read_until(b'\r')
        # time.sleep(0.01)
        line_num += 1
    else:
        # print(record_type)
        # print(line)
        # print()
        pass

print("DONE")

ser.close()



    
import time
import serial
from serial.tools import list_ports
import progressbar

def find_port(dev_name = "50x50-Drive"):
    COM_PORT = None

    attempts = 0

    while(COM_PORT == None):
        attempts+=1
        time.sleep(0.1)

        for port in list_ports.comports():
            if(port.description == dev_name):
                COM_PORT = port.device
                break
        if(attempts > 100):
            raise Exception("Timeout")
        
    return COM_PORT

def enter_bootloader():
    COM_PORT = find_port()
    ser = serial.Serial(COM_PORT, 115200)
    ser.write(bytes('BTL\r', 'utf-8'))
    ser.close()


def enter_app():
    COM_PORT = find_port("50x50-Drive-Bootloader")
    ser = serial.Serial(COM_PORT, 115200)
    ser.write(bytes('APP\r', 'utf-8'))
    ser.close()

def flash_hex_USB():
    COM_PORT = find_port("50x50-Drive-Bootloader")
    ser = serial.Serial(COM_PORT, 115200)

    ######## Command a flash erase
    print("   Erasing flash")
    ser.write(bytes('CLR\r', 'utf-8'))
    ret = ser.read_until(b'\r')
    print("DONE")

    ######## Flash new firmware
    file = open("16_bit_aligned.hex", "r")
    num_lines = 0
    for line in file:
        num_lines += 1
    file.close()


    bar = progressbar.ProgressBar(max_value=num_lines)

    file = open("16_bit_aligned.hex", "r")

    print("  Flashing new firmware")
    line_num = 0
    for line in file:
        record_type = line[7:9]
        address = line[3:7]
        if(record_type == '00'):
            data_length = line[1:3]
            data_length_int = int(data_length, 16)

            data = line[9:9 + 2 * data_length_int]
            data_list = list(bytearray.fromhex(data))

            ser.write(bytes(line[:-1] + '\r', 'utf-8'))

            ret = ser.read_until(b'\r')
            line_num += 1
            bar.update(line_num)
        else:

            pass
        
    ser.close()


def print_hex_list(data_list):
    print_string = ""
    for byte in data_list:
        print_string += hex(byte) + " "
    print(print_string)
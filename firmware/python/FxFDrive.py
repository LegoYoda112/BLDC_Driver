import can
from serial.tools import list_ports
import numpy as np
import time
from enum import Enum

RECV_TIMEOUT = 3
RETRY_TIMEOUT = 5

SYS_LIST_ALL_MSG_ID = 5


class DriveState(Enum):
    drive_state_error = 0
    drive_state_disabled = 1
    drive_state_resistance_estimation = 2
    drive_state_encoder_calibration = 3
    drive_state_anti_cogging_calibration = 4
    drive_state_idle = 5
    drive_state_position_control = 6

class DriveError(Enum):
    drive_error_none = 0
    drive_error_high_resistance = 1
    drive_error_low_resistance = 2
    drive_error_low_voltage = 3


def int_to_bytes(num_bytes, value, reversed = False):
    byte_array = []

    value = int(value)

    for i in range(num_bytes):
        byte = (value >> i * 8) & 0xFF
        byte_array.append(byte)

    byte_array.reverse()
    return byte_array

def bytes_to_int(byte_array, reversed = False):
    value = 0

    # byte ordering reverse is untested
    if(reversed):
        for i in range(len(byte_array)):
            value += (byte_array[i] << 8*(i-len(byte_array)))
    else:
        for i in range(len(byte_array)):
            value += (byte_array[i] << 8*i)
    
    max_int = 2**(len(byte_array) * 8)
    if(value > max_int / 2):
        value = value - max_int

    return value


def make_can_bus():
    """ Finds attached 50x50-Drive and opens it as a SLCAN interface """
    COM_PORT = None
    for port in list_ports.comports():
        if(port.description == "50x50-Drive"):
            COM_PORT = port.device

    if(COM_PORT == None):
        raise Exception("No interface found")

    bus = can.Bus(interface="slcan", channel=COM_PORT, ttyBaudrate = 115200)
    bus.set_bitrate(500_000)

    return bus

def list_available_drives(bus):
    send_msg = can.Message(arbitration_id=0, data=[SYS_LIST_ALL_MSG_ID], is_extended_id=False, dlc=1)
    bus.send(send_msg)

    messages = []

    iterations = 0

    while(True):
        msg = bus.recv(0.01)
        # print(">>" + str(msg))
        iterations += 1
        if(msg == None):
            break
        elif(msg.data[0] == SYS_LIST_ALL_MSG_ID):
            messages.append(msg)
        else:
            pass

        if(iterations > 100):
            break

    for msg in messages:
        print("ID", msg.arbitration_id, "  TYPE", msg.data[1])
    
    return messages

class Parameters:
    PARAM_LED_COLOR = 150
    PARAM_PHASE_RESISTANCE = 160
    PARAM_ANTI_COGGING = 170

    def __init__(self):
        self.run_led_colors = [0, 255, 0]
        self.phase_resistance = 0 
        self.anti_cogging = 0
        pass

    def __str__(self):
        output_string = "Motor parameters: \n"
        output_string += f"Run led colors - {self.run_led_colors}"
        return output_string

class Telemetry:
    TELEM_DRIVE_STATE = 100
    TELEM_MOTOR_POSITION = 101
    TELEM_MOTOR_PHASE_RESISTANCE = 110
    TELM_MOTOR_TORQUE = 111
    TELEM_MOTOR_VOLTAGE = 112

    def __init__(self, motor):
        self.motor = motor
        # self.TELEM_VALUE_TEST = 0
        pass

    def get_motor_voltage_V(self):
        data = self.motor.read_data(self.TELEM_MOTOR_VOLTAGE)
        voltage_mv = bytes_to_int(data)
        return round(voltage_mv / 1000.0, 1)
    
    def get_motor_position_encoder_raw(self):
        data = self.motor.read_data(self.TELEM_MOTOR_POSITION)
        position = bytes_to_int(data)
        return position

    def get_motor_position_revs(self):
        return self.get_motor_position_encoder_raw() / 4096
    
    def get_motor_position_rads(self):
        return self.get_motor_position_revs() * 2 * np.pi
    
    def get_drive_state(self):
        return self.motor.read_data(self.TELEM_DRIVE_STATE)

    def get_motor_torque(self):
        return bytes_to_int(self.motor.read_data(self.TELM_MOTOR_TORQUE))

class Action:
    ACTION_REQUEST_STATE_CHANGE = 20

    def __init__(self, motor):
        self.motor = motor

    def request_state_change(self, new_state):
        self.motor.send_array([self.ACTION_REQUEST_STATE_CHANGE, new_state])

class FxFDrive:
    def __init__(self, can_bus, can_id, uid = None, name = "50x50Drive"):
        self.can_bus = can_bus
        self.can_id = can_id
        self.uid = uid
        self.name = name

        self.parameters = Parameters()
        self.telemetry = Telemetry(self)
        self.action = Action(self)

    def send_array(self, data):
        """ Sends array to the device ID """
        data = [int(v) for v in data]
        send_msg = can.Message(arbitration_id=self.can_id, data=data, is_extended_id=False, dlc=len(data))
        self.can_bus.send(send_msg) 

    def read_data(self, value_id):
        """ Gets value based on provided id """
        # This is a separate function as it can be used to read
        # both parameters and telemetry

        ret = None

        # ret = can.Message(arbitration_id=self.can_id, data=[value_id,2,3,4], is_extended_id=False, dlc=4)
        
        for x in range(RETRY_TIMEOUT):
            self.send_array([value_id])
            done = False

            for i in range(RECV_TIMEOUT):
                ret = self.can_bus.recv(0.1)
                if(ret == None):
                    print("Retrying read")
                    break
                elif(ret.arbitration_id == self.can_id and
                ret.data[0] == value_id and 
                ret.dlc > 1):
                    done = True
                    break

            if(done == True):
                break
        
        if(ret == None):
            return None
        return [int(v) for v in ret.data][1:]
    
        
    def get_telemetry(self, telemetry_id):
        """ Gets telemetry data based on provided id """
        return self.read_data(telemetry_id)

    def default_parameters(self):
        """ Sets up default parameters """
        self.parameters = Parameters()

    def get_parameter(self, parameter_id):
        """ Gets parameter data based on provided id """
        return self.read_data(parameter_id)
    
    def set_parameter(self, parameter_id, parameter_value):
        """ Sets parameter based on provided id and value"""
        for x in range(RETRY_TIMEOUT):
            if(type(parameter_value) == list):
                self.send_array([parameter_id] + parameter_value)
            else:
                self.send_array([parameter_id] + [int(parameter_value)])

            done = False

            ret = None
            for i in range(RECV_TIMEOUT):
                ret = self.can_bus.recv(0.1)

                if(ret == None):
                    print("Retrying")
                    break
                # print(ret)
                if(ret.arbitration_id == self.can_id and
                ret.data[0] == parameter_id and 
                ret.dlc == 2):
                    done = True
                    break

            if(done == True):
                break

        if(not ret.data[0]):
            raise Exception()

        return bool(ret.data[0])
    
    def read_parameters(self):
        """ Reads out all parameters and updates values """
        self.parameters.run_led_colors = self.get_parameter(self.parameters.PARAM_LED_COLOR)
        self.parameters.phase_resistance = bytes_to_int(self.get_parameter(self.parameters.PARAM_PHASE_RESISTANCE)) / 1000.0
        self.parameters.anti_cogging = bytes_to_int(self.get_parameter(self.parameters.PARAM_ANTI_COGGING))
        return self.parameters

    def write_parameters(self, parameters = None):
        """ Writes all parameters """
        if(parameters == None):
            parameters = self.parameters
        
        self.set_parameter(self.parameters.PARAM_LED_COLOR, parameters.run_led_colors)

    def save_parameters(self):
        """ Updates parameters in device EEPROM """
        # TODO: implement when eeprom works
        pass
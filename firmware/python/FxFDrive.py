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
    drive_state_init = 1
    drive_state_disabled = 2

    drive_state_resistance_estimation = 3
    drive_state_encoder_calibration = 4
    drive_state_full_calibration = 5
    drive_state_anti_cogging_calibration = 6

    drive_state_idle = 7
    drive_state_torque_control = 8
    drive_state_velocity_control = 9
    drive_state_position_control = 10
    drive_state_impedance_control = 11

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

    if(reversed):
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
        COM_PORT = port.device

    if(COM_PORT == None):
        raise Exception("No interface found")

    bus = can.Bus(interface="slcan", channel="COM4", ttyBaudrate = 115200)
    bus.set_bitrate(500_000)

    print(bus)
    print("bus") 

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
    PARAM_ENCODER_OFFSET = 150# [offset]
    PARAM_RATIO = 151 # [ratio] SIGNED

    PARAM_CURRENT_P_GAIN = 155 # [p_gain p_gain p_gain p_gain] gain / 1000
    PARAM_CURRENT_I_GAIN = 156 # [i_gain i_gain i_gain i_gain] gain / 1000

    PARAM_ANTI_COGGING = 160
    PARAM_ANTI_COGGING_TABLE = 161

    PARAM_MAXIMUM_MOTOR_VOLTAGE = 170 # [V V] mv
    PARAM_MAXIMUM_MOTOR_CURRENT = 171 # [A A] mA

    PARAM_PHASE_RESISTANCE = 180 # [resistance, resistance] mOhms
    PARAM_MOTOR_KV = 181 # [kv kv kv] rpm/V / 1000
    PARAM_MOTOR_KT = 182 # [kt kt kt] Nm / 1000

    PARAM_KP = 191 # [kp kp] Nm / rad error / 1000
    PARAM_KPV = 192 # [kpv kpv] Nm / radps / 1000 
    PARAM_KV = 193 # [kv kv] Nm / radps error / 1000

    PARAM_P_MAX = 194 # [p_max p_max p_max p_max] rads/1000 SIGNED
    PARAM_P_MIN = 195 # [p_max p_max p_max p_max] rads/1000 SIGNED

    PARAM_V_MAX = 196 # [v_max v_max v_max v_max] radsps/1000 SIGNED



    def __init__(self, motor):
        self.phase_resistance = 0
        self.encoder_offset = 0
        self.anti_cogging = 0

        self.current_gain = 0
        self.current_limit = 0
        self.position_gain = 0
        pass

    def __str__(self):
        output_string = "Motor parameters: \n"
        return output_string

class Telemetry:
    TELEM_DRIVE_STATE = 100 # [drive state, drive error]

    TELEM_MOTOR_VOLTAGE = 101
    TELEM_MOTOR_CURRENT = 102
    TELEM_MOTOR_TORQUE = 103
    TELEM_MOTOR_POSITION = 104
    TELEM_MOTOR_VELOCITY = 105

    TELEM_FET_TEMP = 110
    TELEM_MOTOR_TEMP = 111

    def __init__(self, motor):
        self.motor = motor
        # self.TELEM_VALUE_TEST = 0
        pass

    def get_motor_voltage_V(self):
        data = self.motor.read_data(self.TELEM_MOTOR_VOLTAGE)
        voltage_mv = bytes_to_int(data)
        return round(voltage_mv / 1000.0, 1)
    
    def get_position_rads(self):
        return  bytes_to_int(self.motor.read_data(self.TELEM_MOTOR_POSITION)) / 1000.0
    
    def get_velocity_radsps(self):
        return  bytes_to_int(self.motor.read_data(self.TELEM_MOTOR_VELOCITY)) / 1000.0
    
    def get_drive_state(self):
        return self.motor.read_data(self.TELEM_DRIVE_STATE)

    def get_motor_torque(self):
        return bytes_to_int(self.motor.read_data(self.TELEM_MOTOR_TORQUE))

class Action:
    ACTION_REQUEST_STATE_CHANGE = 20 # [new state]

    ACTION_TORQUE_SETPOINT = 50 # [torque torque torque] Nm / 1000
    ACTION_POSITION_SETPOINT = 51 # [pos pos pos] rads / 1000 # signed
    ACTION_VELOCITY_SETPOINT = 52 # [vel vel vel] rads / 1000

    def __init__(self, motor):
        self.motor = motor

    def request_state_change(self, new_state):
        self.motor.send_array([self.ACTION_REQUEST_STATE_CHANGE, new_state])

    def send_position_target(self, target):
        self.motor.send_array([self.ACTION_TORQUE_SETPOINT] + int_to_bytes(3, target * 1000))
    
    def send_position_target(self, target):
        self.motor.send_array([self.ACTION_POSITION_SETPOINT] + int_to_bytes(3, target * 1000))

    def send_velocity_target(self, target):
        self.motor.send_array([self.ACTION_VELOCITY_SETPOINT] + int_to_bytes(3, target * 1000))

class FxFDrive:
    def __init__(self, can_bus, can_id, uid = None, name = "50x50Drive"):
        self.can_bus = can_bus
        self.can_id = can_id
        self.uid = uid
        self.name = name

        self.parameters = Parameters(self)
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
    
    def read_data_two(self, value_a, value_b):
        """ Gets value based on two provided keys """

        ret = None
        for x in range(RETRY_TIMEOUT):
            self.send_array([value_a, value_b])
            done = False

            for i in range(RECV_TIMEOUT):
                ret = self.can_bus.recv(0.1)
                if(ret == None):
                    print("Retrying read")
                    break
                elif(ret.arbitration_id == self.can_id and
                ret.data[0] == value_a and 
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
    
    def get_parameter_int(self, parameter_id):
        """ Gets parameter as an int based on provided id """
        return bytes_to_int(self.read_data(parameter_id))
    
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
    
    def set_parameter_int(self, parameter_id, parameter_value, num_bytes):
        self.set_parameter(parameter_id, int_to_bytes(num_bytes, parameter_value))



    def read_parameters(self):
        """ Reads out all parameters and updates values """
        self.parameters.phase_resistance = bytes_to_int(self.get_parameter(self.parameters.PARAM_PHASE_RESISTANCE)) / 1000.0
        self.parameters.anti_cogging = bytes_to_int(self.get_parameter(self.parameters.PARAM_ANTI_COGGING))
        self.parameters.encoder_offset = bytes_to_int(self.get_parameter(self.parameters.PARAM_ENCODER_OFFSET))
        # self.parameters.current_gain = bytes_to_int(self.get_parameter(self.parameters.PARAM_CURRENT_GAIN))
        self.parameters.current_limit = bytes_to_int(self.get_parameter(self.parameters.PARAM_MAXIMUM_MOTOR_CURRENT))
        # self.parameters.position_gain = bytes_to_int(self.get_parameter(self.parameters.PARAM_POSITION_GAIN))
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
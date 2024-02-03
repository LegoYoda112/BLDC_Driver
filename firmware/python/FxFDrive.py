import can
from serial.tools import list_ports

RECV_TIMEOUT = 10

TELEM_DRIVE_STATE_ID = 100
TELEM_MOTOR_POSITION = 101
TELEM_MOTOR_PHASE_RESISTANCE = 110
TELEM_MOTOR_CURRENTS = 111
TELEM_MOTOR_VOLTAGE = 112

PARAM_LED_COLOR = 150


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

class Parameters:
    def __init__(self):
        self.run_led_colors = [0, 255, 0]
        pass

    def __str__(self):
        output_string = "Motor parameters: \n"
        output_string += f"Run led colors - {self.run_led_colors}"
        return output_string


class FxFDrive:
    def __init__(self, can_bus, can_id, name = "50x50Drive"):
        self.can_bus = can_bus
        self.can_id = can_id
        self.name = name

        self.parameters = Parameters()

    def default_parameters(self):
        """ Sets up default parameters """
        self.parameters = Parameters()

    def send_array(self, data):
        """ Sends array to the device ID """
        data = [int(v) for v in data]
        send_msg = can.Message(arbitration_id=self.can_id, data=data, is_extended_id=False, dlc=len(data))
        self.can_bus.send(send_msg) 

    def get_parameter(self, parameter_id):
        """ Gets parameter based on provided id """
        self.send_array([parameter_id])

        ret = None
        for i in range(RECV_TIMEOUT):
            ret = self.can_bus.recv()
            if(ret.arbitration_id == self.can_id and
               ret.data[0] == parameter_id and 
               ret.dlc > 1):
                break
        
        if(ret == None):
            return None
        return [int(v) for v in ret.data][1:]
    
    def set_parameter(self, parameter_id, parameter_value):
        """ Sets parameter based on provided id and value"""
        if(type(parameter_value) == list):
            self.send_array([parameter_id] + parameter_value)
        else:
            self.send_array([parameter_id] + [int(parameter_value)])

        ret = None
        for i in range(RECV_TIMEOUT):
            ret = self.can_bus.recv()
            # print(ret)
            if(ret.arbitration_id == self.can_id and
               ret.data[0] == parameter_id and 
               ret.dlc == 2):
                break

        return bool(ret.data[0])
    
    def read_parameters(self):
        """ Reads out all parameters and updates values """
        self.parameters.run_led_colors = self.get_parameter(PARAM_LED_COLOR)

        return self.parameters

    def write_parameters(self, parameters = None):
        """ Writes all parameters """
        if(parameters == None):
            parameters = self.parameters
        
        self.set_parameter(PARAM_LED_COLOR, parameters.run_led_colors)

    def save_parameters(self):
        """ Updates parameters in device EEPROM """
        # TODO: implement when eeprom works
        pass
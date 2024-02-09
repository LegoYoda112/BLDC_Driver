#pip install imgui[glfw]
import imgui
import glfw
import OpenGL.GL as gl
from imgui.integrations.glfw import GlfwRenderer

import numpy as np
import time
from FxFDrive import FxFDrive, make_can_bus, list_available_drives, bytes_to_int
from FxFDrive import DriveError, DriveState

bus = make_can_bus()
print("Bus running")


available_drives = list_available_drives(bus)

drives = []

for drive in available_drives:


    uid = ''.join(format(x, '02x') for x in drive.data[2:])

    print(drive.arbitration_id, uid)
    drives.append(FxFDrive(bus, drive.arbitration_id, uid))

motor = drives[0]


def impl_glfw_init(window_name="minimal ImGui/GLFW3 example", width=1280, height=720):
    if not glfw.init():
        print("Could not initialize OpenGL context")
        exit(1)

    # OS X supports only forward-compatible core profiles from 3.2
    glfw.window_hint(glfw.CONTEXT_VERSION_MAJOR, 3)
    glfw.window_hint(glfw.CONTEXT_VERSION_MINOR, 3)
    glfw.window_hint(glfw.OPENGL_PROFILE, glfw.OPENGL_CORE_PROFILE)

    glfw.window_hint(glfw.OPENGL_FORWARD_COMPAT, gl.GL_TRUE)

    # Create a windowed mode window and its OpenGL context
    window = glfw.create_window(int(width), int(height), window_name, None, None)
    glfw.make_context_current(window)

    if not window:
        glfw.terminate()
        print("Could not initialize Window")
        exit(1)

    return window


class GUI(object):
    def __init__(self):
        super().__init__()
        self.backgroundColor = (0, 0, 0, 1)
        self.window = impl_glfw_init()
        gl.glClearColor(*self.backgroundColor)
        imgui.create_context()
        self.impl = GlfwRenderer(self.window)

        self.drive = drives[0]
        self.color = [0,0,0]

        self.plot_voltage = False

        # motor.read_parameters()

        self.loop()

    def loop(self):
        while not glfw.window_should_close(self.window):
            glfw.poll_events()
            self.impl.process_inputs()
            imgui.new_frame()

            imgui.begin("Drive GUI", True)

            # imgui.push_item_width(imgui.get_window_width() * 0.5)
            imgui.push_item_width(300)
            # imgui.push_item_width(imgui.get_font_size() * -12)

            imgui.set_window_font_scale(1.0)

            with imgui.begin_list_box("Available\nDrives") as list_box:
                if list_box.opened:
                    for drive in drives:
                        clicked, _ = imgui.selectable(
                            label= "CAN ID "+ str(drive.can_id) + " UID " + str(drive.uid), selected=(self.drive.can_id == drive.can_id)
                        )
                        if clicked:
                            self.drive = drive
                            self.drive.read_parameters()

                            self.color = [self.drive.parameters.run_led_colors[0] / 255.0,
                                self.drive.parameters.run_led_colors[1] / 255.0,
                                self.drive.parameters.run_led_colors[2] / 255.0]

            drive_voltage = self.drive.telemetry.get_motor_voltage_V()
            motor_position = self.drive.telemetry.get_motor_position_revs()
            drive_state, drive_error = self.drive.telemetry.get_drive_state()
            motor_torque = self.drive.telemetry.get_motor_torque()

            imgui.text("Action")
            imgui.text("Request state change")
            with imgui.begin_list_box("State") as list_box:
                if(list_box.opened):
                    for state in DriveState:
                        clicked, _ = imgui.selectable(state.name, selected = (state.value == drive_state))

                        if clicked:
                            print(state.value)
                            self.drive.action.request_state_change(state.value)

  
            imgui.text("Telemetry")
            imgui.drag_float(
                "Motor voltage", drive_voltage, 0.1, 0.0, 0.0, "%.01f V"
            )

            imgui.drag_float(
                "Rotor position", motor_position, 0.1, 0.0, 0.0, "%.02f revs"
            )

            imgui.drag_int(
                "Rotor position", motor_position * 4096, 0.1, 0.0, 0.0, "%f ticks"
            )

            imgui.slider_float(
                "Torque", motor_torque / 1000.0, -3.0, 3.0, "%.02f"
            )

            # imgui.drag_int(
            #     "Drive state int", drive_state, 0.1, 0.0, 0.0, "%f"
            # )

            imgui.input_text("Drive state enum", DriveState(drive_state).name)

            # imgui.drag_int(
            #     "Drive error", drive_error, 0.1, 0.0, 0.0, "%f"
            # )

            imgui.input_text("Drive error enum", DriveError(drive_error).name)

            # show, _ = imgui.collapsing_header("Telemetry")
            imgui.text("Parameters")
            changed, self.color = imgui.color_edit3("LED Color", *self.color)

            if(changed):
                self.drive.parameters.run_led_colors = [self.color[0] * 255,
                                                self.color[1] * 255,
                                                self.color[2] * 255]
                self.drive.write_parameters()

            imgui.drag_float(
                "Phase resistance", self.drive.parameters.phase_resistance, 0.1, 0.0, 0.0, "%.02f Ohms"
            )

            changed, self.drive.parameters.anti_cogging = imgui.checkbox("Anti-Cogging", self.drive.parameters.anti_cogging)

            if(changed):
                # print(self.drive.parameters.anti_cogging)
                self.drive.set_parameter(drive.parameters.PARAM_ANTI_COGGING, self.drive.parameters.anti_cogging)



            # if imgui.tree_node("Configuration"):
            #     imgui.text("lol")

            #     imgui.tree_pop()

            imgui.show_test_window()

            imgui.end()

            imgui.render()

            gl.glClearColor(*self.backgroundColor)
            gl.glClear(gl.GL_COLOR_BUFFER_BIT)

            self.impl.render(imgui.get_draw_data())
            glfw.swap_buffers(self.window)

        self.impl.shutdown()
        glfw.terminate()


if __name__ == "__main__":
    print("hello?")
    gui = GUI()
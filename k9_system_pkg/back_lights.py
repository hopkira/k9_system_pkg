#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from k9_interfaces_pkg.srv import LightsControl, SwitchState, StringCommand
import serial
import ast


class BackLightsNode(Node):
    def __init__(self):
        super().__init__('back_lights_node')
        try:
            self.ser = serial.Serial(
                port='/dev/backpanel',
                baudrate=115200,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                bytesize=serial.EIGHTBITS,
                timeout=1
            )
            self.get_logger().info("Connected to back panel on /dev/backpanel")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to connect to back panel: {e}")
            self.ser = None

        # Services
        self.create_service(Trigger, 'back_lights_on', self.on_handler)
        self.create_service(Trigger, 'back_lights_off', self.off_handler)
        self.create_service(LightsControl, 'back_lights_turn_on', self.turn_on_handler)
        self.create_service(LightsControl, 'back_lights_turn_off', self.turn_off_handler)
        self.create_service(LightsControl, 'back_lights_toggle', self.toggle_handler)
        self.create_service(Trigger, 'tv_on', self.tv_on_handler)
        self.create_service(Trigger, 'tv_off', self.tv_off_handler)
        self.create_service(SwitchState, 'back_lights_get_switch_state', self.get_switch_state_handler)
        self.create_service(StringCommand, 'back_lights_cmd', self.cmd_service_handler)  # NEW

        self.get_logger().info("Back Lights Node is running.")

    # ---------------- Helpers ---------------- #
    def __write(self, text: str) -> bool:
        """Send a command to the serial device if available."""
        if not self.ser or not self.ser.is_open:
            self.get_logger().error("Serial port not available. Command not sent.")
            return False
        try:
            self.ser.write((text + "\n").encode())
            return True
        except serial.SerialException as e:
            self.get_logger().error(f"Serial write failed: {e}")
            return False

    def __sw_light(self, cmd: str, lights: list) -> bool:
        ok = True
        for light in lights:
            if not self.__write(f"light {light} {cmd}"):
                ok = False
        return ok

    # ---------------- Service Handlers ---------------- #
    def on_handler(self, request, response):
        success = self.__write("original")
        response.success = success
        response.message = "Lights turned on" if success else "Failed to turn on lights"
        self.get_logger().info(response.message) if success else self.get_logger().error(response.message)
        return response

    def off_handler(self, request, response):
        success = self.__write("off")
        response.success = success
        response.message = "Lights turned off" if success else "Failed to turn off lights"
        self.get_logger().info(response.message) if success else self.get_logger().error(response.message)
        return response

    def turn_on_handler(self, request, response):
        success = self.__sw_light("on", request.lights)
        response.success = success
        response.message = "Lights turned on" if success else "Failed to turn on some lights"
        self.get_logger().info(response.message) if success else self.get_logger().error(response.message)
        return response

    def turn_off_handler(self, request, response):
        success = self.__sw_light("off", request.lights)
        response.success = success
        response.message = "Lights turned off" if success else "Failed to turn off some lights"
        self.get_logger().info(response.message) if success else self.get_logger().error(response.message)
        return response

    def toggle_handler(self, request, response):
        success = self.__sw_light("toggle", request.lights)
        response.success = success
        response.message = "Lights toggled" if success else "Failed to toggle some lights"
        self.get_logger().info(response.message) if success else self.get_logger().error(response.message)
        return response

    def tv_on_handler(self, request, response):
        success = self.__write("tvon")
        response.success = success
        response.message = "Side screen on" if success else "Failed to turn on side screen"
        self.get_logger().info(response.message) if success else self.get_logger().error(response.message)
        return response

    def tv_off_handler(self, request, response):
        success = self.__write("tvoff")
        response.success = success
        response.message = "Side screen off" if success else "Failed to turn off side screen"
        self.get_logger().info(response.message) if success else self.get_logger().error(response.message)
        return response

    def get_switch_state_handler(self, request, response):
        if not self.__write("switchstate"):
            response.success = False
            response.message = "Failed to request switch state (serial unavailable)"
            response.states = []
            return response

        try:
            line = self.ser.readline().decode().strip()
            if not line:
                response.success = False
                response.message = "No response from device"
                response.states = []
                self.get_logger().error(response.message)
                return response

            if not line.startswith("switchstate:"):
                response.success = False
                response.message = f"Unexpected response: {line}"
                response.states = []
                self.get_logger().error(response.message)
                return response

            input_str = line[len("switchstate:"):]
            switchstate_list = ast.literal_eval(input_str)
            bool_list = [bool(x) for x in switchstate_list]

            response.success = True
            response.message = "Switch states received"
            response.states = bool_list
            self.get_logger().info(response.message)
            self.get_logger().debug(f"Switch states: {bool_list}")
        except Exception as e:
            response.success = False
            response.message = f"Error: {e}"
            response.states = []
            self.get_logger().error(response.message)

        return response

    def cmd_service_handler(self, request, response):
        """Handle arbitrary string command requests via service."""
        success = self.__write(request.data)
        response.success = success
        response.message = "Command sent" if success else "Failed to send command"
        self.get_logger().info(response.message) if success else self.get_logger().error(response.message)
        return response

    # ---------------- Shutdown ---------------- #
    def destroy_node(self):
        if self.ser and self.ser.is_open:
            try:
                self.ser.close()
                self.get_logger().info("Serial port closed cleanly.")
            except serial.SerialException as e:
                self.get_logger().warn(f"Error closing serial port: {e}")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = BackLightsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
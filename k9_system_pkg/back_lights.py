#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger
from k9_interfaces_pkg.srv import LightsControl, SwitchState  # Custom interfaces
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
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to connect to back panel: {e}")
            self.ser = None

        self.create_service(Trigger, 'back_lights_on', self.on_handler)
        self.create_service(Trigger, 'back_lights_off', self.off_handler)
        self.create_service(LightsControl, 'back_lights_turn_on', self.turn_on_handler)
        self.create_service(LightsControl, 'back_lights_turn_off', self.turn_off_handler)
        self.create_service(LightsControl, 'back_lights_toggle', self.toggle_handler)
        self.create_service(Trigger, 'tv_on', self.tv_on_handler)
        self.create_service(Trigger, 'tv_off', self.tv_off_handler)
        self.create_service(SwitchState, 'back_lights_get_switch_state', self.get_switch_state_handler)
        self.create_subscription(String, 'back_lights_cmd', self.cmd_callback, 10)

        self.get_logger().info("Back Lights Node is running.")

    def __write(self, text: str) -> None:
        self.ser.write(str.encode(text + "\n"))

    def __sw_light(self, cmd: str, lights: list) -> None:
        for light in lights:
            text = f"light {light} {cmd}"
            self.__write(text)

    def on_handler(self, request, response):
        self.__write("original")
        response.success = True
        message = "Lights turned on"
        response.message = message
        self.get_logger().info(message)
        return response

    def off_handler(self, request, response):
        self.__write("off")
        response.success = True
        message = "Lights turned off"
        response.message = message
        self.get_logger().info(message)
        return response

    def turn_on_handler(self, request, response):
        self.__sw_light("on", request.lights)
        response.success = True
        message = "Lights turned on"
        response.message = message
        self.get_logger().info(message)
        return response

    def turn_off_handler(self, request, response):
        self.__sw_light("off", request.lights)
        response.success = True
        message = "Lights turned off"
        response.message = message
        self.get_logger().info(message)
        return response

    def toggle_handler(self, request, response):
        self.__sw_light("toggle", request.lights)
        response.success = True
        message = "Lights toggled"
        response.message = message
        self.get_logger().info(message)
        return response

    def tv_on_handler(self, request, response):
        self.__write("tvon")
        response.success = True
        message = "Side screen on"
        response.message = message
        self.get_logger().info(message)
        return response

    def tv_off_handler(self, request, response):
        self.__write("tvoff")
        response.success = True
        message = "Side screen off"
        response.message = message
        self.get_logger().info(message)
        return response

    def get_switch_state_handler(self, request, response):
        self.__write("switchstate")
        try:
            lines = self.ser.readlines()
            if not lines:
                response.success = False
                message = "No response from device"
                response.message = message
                self.get_logger().info(message)
                response.states = []
                return response

            input_str = lines[0].decode().strip()
            input_str = input_str[len('switchstate:'):]
            switchstate_list = ast.literal_eval(input_str)
            bool_list = [bool(x) for x in switchstate_list]

            response.success = True
            message = "Switch states received"
            response.message = message
            self.get_logger().info(message)
            self.get_logger().debug("Switch states:", bool_list)
            response.states = bool_list
        except Exception as e:
            response.success = False
            response.message = f"Error: {e}"
            response.states = []

        return response

    def cmd_callback(self, msg):
        self.__write(msg.data)


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

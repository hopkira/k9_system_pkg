#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from std_msgs.msg import Bool
from k9_interfaces_pkg.srv import SetBrightness, GetBrightness

import time
import board
import busio
from adafruit_pca9685 import PCA9685

class EyesTail:
    def __init__(self):
        try:
            self.i2c = busio.I2C(board.SCL, board.SDA)
            self.pca = PCA9685(self.i2c, address=0x40)
        except Exception as e:
            self.get_logger().error(f"Failed to initialize servo controller: {e}")
        self.pca.frequency = 60
        self._level = 0.0
        self.set_level(0.0)
        self.centre()
        time.sleep(0.25)
        self.pca.channels[5].duty_cycle = 0
        self.pca.channels[4].duty_cycle = 0

    def set_level(self, level: float) -> None:
        value = int(min(max(level, 0.0), 1.0) * 65535)
        self.pca.channels[0].duty_cycle = value
        self._level = level

    def off(self) -> None:
        self.set_level(0.0)

    def on(self) -> None:
        self.set_level(1.0)

    def get_level(self) -> float:
        return self._level
    
    def wag_h(self):
        self.pca.channels[4].duty_cycle = 5121
        for _ in range(4):
            self.pca.channels[5].duty_cycle = 5201
            time.sleep(0.25)
            self.pca.channels[5].duty_cycle = 7042
            time.sleep(0.25)
        self.pca.channels[5].duty_cycle = 5601
        time.sleep(0.25)
        self.pca.channels[5].duty_cycle = 0
        self.pca.channels[4].duty_cycle = 0

    def wag_v(self):
        self.pca.channels[5].duty_cycle = 5601
        for _ in range(4):
            self.pca.channels[4].duty_cycle = 5921
            time.sleep(0.25)
            self.pca.channels[4].duty_cycle = 4321
            time.sleep(0.25)
        self.pca.channels[4].duty_cycle = 5121
        time.sleep(0.25)
        self.pca.channels[5].duty_cycle = 0
        self.pca.channels[4].duty_cycle = 0

    def centre(self):
        self.pca.channels[4].duty_cycle = 5121
        self.pca.channels[5].duty_cycle = 5601
        time.sleep(0.25)
        self.pca.channels[5].duty_cycle = 0
        self.pca.channels[4].duty_cycle = 0

    def up(self):
        self.pca.channels[5].duty_cycle = 5601
        self.pca.channels[4].duty_cycle = 4321
        time.sleep(0.25)
        self.pca.channels[5].duty_cycle = 0
        self.pca.channels[4].duty_cycle = 0

    def down(self):
        self.pca.channels[5].duty_cycle = 5601
        self.pca.channels[4].duty_cycle = 5921
        time.sleep(0.25)
        self.pca.channels[5].duty_cycle = 0
        self.pca.channels[4].duty_cycle = 0


class EyesTailServiceNode(Node):
    def __init__(self):
        super().__init__('eyes_tail_service_node')
        self.eyestail = EyesTail()
        self.get_logger().info("Eyes and Tail Node is running.")

        self._is_talking = False
        self._stored_level = 0.0  # Saved level before talking began

        # Subscribers
        self.create_subscription(Bool, 'is_talking', self.talking_cb, 10)

        # Services
        self.create_service(SetBrightness, 'eyes_set_level', self.set_level_cb)
        self.create_service(GetBrightness, 'eyes_get_level', self.get_level_cb)
        self.create_service(Trigger, 'eyes_on', self.on_cb)
        self.create_service(Trigger, 'eyes_off', self.off_cb)
        self.create_service(Trigger, 'tail_wag_h', self.wag_h_cb)
        self.create_service(Trigger, 'tail_wag_v', self.wag_v_cb)
        self.create_service(Trigger, 'tail_centre', self.centre_cb)
        self.create_service(Trigger, 'tail_up', self.up_cb)
        self.create_service(Trigger, 'tail_down', self.down_cb)
        self.get_logger().info('Eyes and Tail node ready to go!')

    def talking_cb(self, msg: Bool):
        if msg.data and not self._is_talking:
            # Start talking: store previous level, set full brightness
            self._stored_level = self.eyestail.get_level()
            self.eyestail.on()
            self._is_talking = True
            self.get_logger().debug("Talking detected: eyes set to 100%")
        elif not msg.data and self._is_talking:
            # Stop talking: restore previous level
            self.eyestail.set_level(self._stored_level)
            self._is_talking = False
            self.get_logger().debug(f"Stopped talking: eyes restored to {self._stored_level:.2f}")

    # Service Callbacks
    def set_level_cb(self, request, response):
        if self._is_talking:
            response.success = False
            message = "Ignored: eyes are in talking mode"
            response.message = message
            self.get_logger().info(message)
            return response
        self.eyestail.set_level(request.level)
        response.success = True
        message = f"Brightness set to {request.level:.2f}"
        response.message = message
        self.get_logger().info(message)
        return response

    def get_level_cb(self, request, response):
        response.level = self.eyestail.get_level()
        return response

    def on_cb(self, request, response):
        if self._is_talking:
            response.success = False
            message = "Ignored: eyes are in talking mode"
            response.message = message
            self.get_logger().info(message)
            return response
        self.eyestail.on()
        response.success = True
        message = "Eyes turned on."
        response.message = message
        self.get_logger().info(message)
        return response

    def off_cb(self, request, response):
        if self._is_talking:
            response.success = False
            message = "Ignored: eyes are in talking mode"
            response.message = message
            self.get_logger().info(message)
            return response
        self.eyestail.off()
        response.success = True
        message = "Eyes turned off."
        response.message = message
        self.get_logger().info(message)
        return response
    
    def wag_h_cb(self, request, response):
        self.eyestail.wag_h()
        response.success = True
        message = "Tail wagged horizontally."
        response.message = message
        self.get_logger().info(message)
        return response

    def wag_v_cb(self, request, response):
        self.eyestail.wag_v()
        response.success = True
        message = "Tail wagged vertically."
        response.message = message
        self.get_logger().info(message)
        return response

    def centre_cb(self, request, response):
        self.eyestail.centre()
        response.success = True
        message = "Tail centred."
        response.message = message
        self.get_logger().info(message)
        return response

    def up_cb(self, request, response):
        self.eyestail.up()
        response.success = True
        message = "Tail raised."
        response.message = message
        self.get_logger().info(message)
        return response

    def down_cb(self, request, response):
        self.eyestail.down()
        response.success = True
        message = "Tail lowered."
        response.message = message
        self.get_logger().info(message)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = EyesTailServiceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from std_msgs.msg import Bool
from k9_interfaces_pkg.srv import SetBrightness, GetBrightness

import board
import busio
import adafruit_pca9685


class Eyes:
    def __init__(self):
        try:
            i2c = busio.I2C(board.SCL, board.SDA)
            self.pca = adafruit_pca9685.PCA9685(i2c)
        except Exception as e:
            self.get_logger().error(f"Failed to initialize servo controller: {e}")
        self.pca.frequency = 1000
        self._level = 0.0
        self.set_level(0.0)

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


class EyesServiceNode(Node):
    def __init__(self):
        super().__init__('eyes_service_node')
        self.eyes = Eyes()
        self.get_logger().info("Eyes Node is running.")

        self._is_talking = False
        self._stored_level = 0.0  # Saved level before talking began

        # Subscribers
        self.create_subscription(Bool, 'is_talking', self.talking_cb, 10)

        # Services
        self.create_service(SetBrightness, 'eyes_set_level', self.set_level_cb)
        self.create_service(GetBrightness, 'eyes_get_level', self.get_level_cb)
        self.create_service(Trigger, 'eyes_on', self.on_cb)
        self.create_service(Trigger, 'eyes_off', self.off_cb)

    def talking_cb(self, msg: Bool):
        if msg.data and not self._is_talking:
            # Start talking: store previous level, set full brightness
            self._stored_level = self.eyes.get_level()
            self.eyes.on()
            self._is_talking = True
            self.get_logger().debug("Talking detected: eyes set to 100%")
        elif not msg.data and self._is_talking:
            # Stop talking: restore previous level
            self.eyes.set_level(self._stored_level)
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
        self.eyes.set_level(request.level)
        response.success = True
        message = f"Brightness set to {request.level:.2f}"
        response.message = message
        self.get_logger().info(message)
        return response

    def get_level_cb(self, request, response):
        response.level = self.eyes.get_level()
        return response

    def on_cb(self, request, response):
        if self._is_talking:
            response.success = False
            message = "Ignored: eyes are in talking mode"
            response.message = message
            self.get_logger().info(message)
            return response
        self.eyes.on()
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
        self.eyes.off()
        response.success = True
        message = "Eyes turned off."
        response.message = message
        self.get_logger().info(message)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = EyesServiceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

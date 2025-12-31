#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
import serial
import json
import math
import time

class SimEars:
    """Class that communicates with simulated LIDAR ears"""


class Ears:
    """Class that communicates with the Espruino controlling K9's LIDAR ears"""

    def __init__(self, node) -> None:
        self.node = node
        try:
            self.ser = serial.Serial(
                port='/dev/ears',
                baudrate=115200,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                bytesize=serial.EIGHTBITS,
                timeout=0.5
            )
        except serial.SerialException as e:
            self.node.get_logger().error(f"Failed to connect to ears: {e}")
            self.ser = None
        self.following = False

    def __write(self, text: str) -> None:
        self.node.get_logger().debug(f"Ears: {text}")
        if not self.ser:
            return
        self.ser.write(str.encode(text + "()\n"))

    def stop(self) -> None:
        self.following = False
        self.__write("stop")

    def scan(self) -> None:
        self.following = False
        self.__write("scan")

    def fast(self) -> None:
        self.following = False
        self.__write("fast")

    def think(self) -> None:
        self.following = False
        self.__write("think")

    def follow_read(self) -> float:
        if not self.following:
            self.__write("follow")
            self.following = True
        if not self.ser:
            raise RuntimeError("Ears serial not connected")
        json_reading = self.ser.readline().decode("ascii", errors="ignore").strip()
        reading = json.loads(json_reading)
        return float(reading['distance'])

    '''
    Wrong place for this code, should now be at a higher level
    def safe_rotate(self) -> bool:
        safe_x = 0.3
        safe_y = 0.6
        duration = 4
        detected = False
        self.__write("fast")
        end_scan = time.time() + duration

        while time.time() < end_scan and not detected:
            if not self.ser:
                break
            line = self.ser.readline().decode("ascii", errors="ignore").strip()
            if not line:
                continue
            reading = json.loads(line)
            dist = float(reading['distance'])
            angle = float(reading['angle'])
            x = abs(dist * math.cos(angle))
            y = abs(dist * math.sin(angle))
            if x <= safe_x and y <= safe_y:
                detected = True

        self.__write("stop")
        return not detected
    '''

class EarsServiceNode(Node):
    def __init__(self):
        super().__init__('ears_service_node')
        self.ears = Ears(self)
        self.get_logger().info("Ears Node is running.")

        # Register services
        self.create_service(Trigger, 'ears_stop', self.stop_cb)
        self.create_service(Trigger, 'ears_scan', self.scan_cb)
        self.create_service(Trigger, 'ears_fast', self.fast_cb)
        self.create_service(Trigger, 'ears_think', self.think_cb)
        self.create_service(Trigger, 'ears_follow_read', self.follow_read_cb)
        # self.create_service(Trigger, 'ears_safe_rotate', self.safe_rotate_cb)

    # Callbacks
    def stop_cb(self, request, response):
        self.ears.stop()
        response.success = True
        message = "Stopped ears."
        response.message = message
        self.get_logger().info(message)
        return response

    def scan_cb(self, request, response):
        self.ears.scan()
        response.success = True
        message = "Started scan."
        response.message = message
        self.get_logger().info(message)
        return response

    def fast_cb(self, request, response):
        self.ears.fast()
        response.success = True
        message = "Set to fast mode."
        response.message = message
        self.get_logger().info(message)
        return response

    def think_cb(self, request, response):
        self.ears.think()
        response.success = True
        message = "Set to think mode."
        response.message = message
        self.get_logger().info(message)
        return response

    def follow_read_cb(self, request, response):
        try:
            dist = self.ears.follow_read()
            response.success = True
            message = f"Distance: {dist:.2f} m"
            response.message = message
            self.get_logger().info(message)
        except Exception as e:
            response.success = False
            message = f"Error reading distance: {str(e)}"
            response.message = message
            self.get_logger().info(message)
        return response

    def safe_rotate_cb(self, request, response):
        try:
            safe = self.ears.safe_rotate()
            response.success = safe
            message = "Safe to rotate." if safe else "Obstacle detected."
            response.message = message
            self.get_logger().info(message)
        except Exception as e:
            response.success = False
            message = f"Error during rotation check: {str(e)}"
            response.message = message
            self.get_logger().info(message)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = EarsServiceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
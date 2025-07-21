#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
import serial
import json
import math
import time

class Ears:
    """Class that communicates with the Espruino controlling K9's LIDAR ears"""

    def __init__(self) -> None:
        self.ser = serial.Serial(
            port='/dev/ears',
            baudrate=115200,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=10
        )
        self.following = False

    def __write(self, text: str) -> None:
        print("Ears:", text)
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
        json_reading = self.ser.readline().decode("ascii")
        reading = json.loads(json_reading)
        return reading['distance']

    def safe_rotate(self) -> bool:
        safe_x = 0.3
        safe_y = 0.6
        duration = 4
        detected = False
        self.__write("fast")
        end_scan = time.time() + duration

        while time.time() < end_scan and not detected:
            json_reading = self.ser.readline().decode("ascii")
            reading = json.loads(json_reading)
            dist = reading['distance']
            angle = reading['angle']
            x = abs(dist * math.cos(angle))
            y = abs(dist * math.sin(angle))
            if x <= safe_x and y <= safe_y:
                detected = True

        self.__write("stop")
        return not detected


class EarsServiceNode(Node):
    def __init__(self):
        super().__init__('ears_service_node')
        self.ears = Ears()

        # Register services
        self.create_service(Trigger, 'ears_stop', self.stop_cb)
        self.create_service(Trigger, 'ears_scan', self.scan_cb)
        self.create_service(Trigger, 'ears_fast', self.fast_cb)
        self.create_service(Trigger, 'ears_think', self.think_cb)
        self.create_service(Trigger, 'ears_follow_read', self.follow_read_cb)
        self.create_service(Trigger, 'ears_safe_rotate', self.safe_rotate_cb)

    # Callbacks
    def stop_cb(self, request, response):
        self.ears.stop()
        response.success = True
        response.message = "Stopped ears."
        return response

    def scan_cb(self, request, response):
        self.ears.scan()
        response.success = True
        response.message = "Started scan."
        return response

    def fast_cb(self, request, response):
        self.ears.fast()
        response.success = True
        response.message = "Set to fast mode."
        return response

    def think_cb(self, request, response):
        self.ears.think()
        response.success = True
        response.message = "Set to think mode."
        return response

    def follow_read_cb(self, request, response):
        try:
            dist = self.ears.follow_read()
            response.success = True
            response.message = f"Distance: {dist:.2f} m"
        except Exception as e:
            response.success = False
            response.message = f"Error reading distance: {str(e)}"
        return response

    def safe_rotate_cb(self, request, response):
        try:
            safe = self.ears.safe_rotate()
            response.success = safe
            response.message = "Safe to rotate." if safe else "Obstacle detected."
        except Exception as e:
            response.success = False
            response.message = f"Error during rotation check: {str(e)}"
        return response


def main(args=None):
    rclpy.init(args=args)
    node = EarsServiceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

import time
import board
import busio
from adafruit_pca9685 import PCA9685


class Tail:
    def __init__(self):
        self.i2c = busio.I2C(board.SCL, board.SDA)
        self.pca = PCA9685(self.i2c, address=0x40)
        self.pca.frequency = 60
        self.centre()

    def wag_h(self):
        self.pca.channels[4].duty_cycle = 5121
        for _ in range(4):
            self.pca.channels[5].duty_cycle = 5201
            time.sleep(0.25)
            self.pca.channels[5].duty_cycle = 7042
            time.sleep(0.25)
        self.pca.channels[5].duty_cycle = 5601

    def wag_v(self):
        self.pca.channels[5].duty_cycle = 5601
        for _ in range(4):
            self.pca.channels[4].duty_cycle = 5921
            time.sleep(0.25)
            self.pca.channels[4].duty_cycle = 4321
            time.sleep(0.25)
        self.pca.channels[4].duty_cycle = 5121

    def centre(self):
        self.pca.channels[4].duty_cycle = 5121
        self.pca.channels[5].duty_cycle = 5601

    def up(self):
        self.pca.channels[5].duty_cycle = 5601
        self.pca.channels[4].duty_cycle = 4321

    def down(self):
        self.pca.channels[5].duty_cycle = 5601
        self.pca.channels[4].duty_cycle = 5921


class TailServiceNode(Node):
    def __init__(self):
        super().__init__('tail_service_node')
        self.tail = Tail()

        self.create_service(Trigger, 'tail_wag_h', self.wag_h_cb)
        self.create_service(Trigger, 'tail_wag_v', self.wag_v_cb)
        self.create_service(Trigger, 'tail_centre', self.centre_cb)
        self.create_service(Trigger, 'tail_up', self.up_cb)
        self.create_service(Trigger, 'tail_down', self.down_cb)
        self.get_logger().info('Tail node ready to wag!')

    def wag_h_cb(self, request, response):
        self.tail.wag_h()
        response.success = True
        message = "Tail wagged horizontally."
        response.message = message
        self.get_logger().info(message)
        return response

    def wag_v_cb(self, request, response):
        self.tail.wag_v()
        response.success = True
        message = "Tail wagged vertically."
        response.message = message
        self.get_logger().info(message)
        return response

    def centre_cb(self, request, response):
        self.tail.centre()
        response.success = True
        message = "Tail centred."
        response.message = message
        self.get_logger().info(message)
        return response

    def up_cb(self, request, response):
        self.tail.up()
        response.success = True
        message = "Tail raised."
        response.message = message
        self.get_logger().info(message)
        return response

    def down_cb(self, request, response):
        self.tail.down()
        response.success = True
        message = "Tail lowered."
        response.message = message
        self.get_logger().info(message)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = TailServiceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

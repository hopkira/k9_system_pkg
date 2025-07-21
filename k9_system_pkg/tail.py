#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

import time
import board
import busio
import adafruit_pca9685

class Tail:
    def __init__(self):
        i2c = busio.I2C(board.SCL, board.SDA)
        self.pca = adafruit_pca9685.PCA9685(i2c)
        self.pca.frequency = 60
        self.center()

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

    def center(self):
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
        self.create_service(Trigger, 'tail_center', self.center_cb)
        self.create_service(Trigger, 'tail_up', self.up_cb)
        self.create_service(Trigger, 'tail_down', self.down_cb)

    def wag_h_cb(self, request, response):
        self.tail.wag_h()
        response.success = True
        response.message = "Tail wagged horizontally."
        return response

    def wag_v_cb(self, request, response):
        self.tail.wag_v()
        response.success = True
        response.message = "Tail wagged vertically."
        return response

    def center_cb(self, request, response):
        self.tail.center()
        response.success = True
        response.message = "Tail centered."
        return response

    def up_cb(self, request, response):
        self.tail.up()
        response.success = True
        response.message = "Tail raised."
        return response

    def down_cb(self, request, response):
        self.tail.down()
        response.success = True
        response.message = "Tail lowered."
        return response


def main(args=None):
    rclpy.init(args=args)
    node = TailServiceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
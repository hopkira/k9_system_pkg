#!/usr/bin/env python3
import socket

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import Bool

# Example usage in another node:
#
# from std_msgs.msg import Bool
# from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
# self.internet_available = False
#
# qos_latched = QoSProfile(
#     depth=1,
#     reliability=ReliabilityPolicy.RELIABLE,
#     durability=DurabilityPolicy.TRANSIENT_LOCAL,
# )
#
# self.create_subscription(Bool, 
#   "/system/internet_available",
#   _internet_cb,
#   qos_latched
#   )   
#
#
# def _internet_cb(self, msg: Bool):
#      self.internet_available = bool(msg.data)
#      self.internet_known = True

class InternetMonitor(Node):
    def __init__(self):
        super().__init__("internet_monitor")

        # Parameters (optional but useful)
        self.declare_parameter("check_period", 5.0)   # seconds
        self.declare_parameter("host", "8.8.8.8")
        self.declare_parameter("port", 53)
        self.declare_parameter("timeout", 1.0)

        self.check_period = self.get_parameter("check_period").value
        self.host = self.get_parameter("host").value
        self.port = self.get_parameter("port").value
        self.timeout = self.get_parameter("timeout").value

        # "Latched" QoS in ROS 2 = transient local durability
        qos_latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.publisher = self.create_publisher(
            Bool,
            "/system/internet_available",
            10
        )

        self.timer = self.create_timer(
            self.check_period,
            self.check_internet
        )

        self.last_state = None  # Unknown at start

        self.get_logger().info("Internet monitor started (latched QoS enabled)")

    def check_internet(self):
        online = self._can_connect()

        # Only log on change (important!)
        if online != self.last_state:
            state = "ONLINE" if online else "OFFLINE"
            self.get_logger().warn(f"Internet state changed: {state}")

        msg = Bool()
        msg.data = online
        self.publisher.publish(msg)

        self.last_state = online

    def _can_connect(self) -> bool:
        try:
            socket.setdefaulttimeout(self.timeout)
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
                sock.connect((self.host, self.port))
            return True
        except Exception:
            return False


def main():
    rclpy.init()
    node = InternetMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

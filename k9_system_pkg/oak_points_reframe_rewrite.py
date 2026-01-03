#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2


class OakPointsFrameRewrite(Node):
    def __init__(self):
        super().__init__('oak_points_frame_rewrite')

        self.declare_parameter('in_topic', '/oak/points')
        self.declare_parameter('out_topic', '/oak/points_oakd_link')
        self.declare_parameter('frame_id', 'oakd_link')

        in_topic = self.get_parameter('in_topic').value
        out_topic = self.get_parameter('out_topic').value
        self.frame_id = self.get_parameter('frame_id').value

        # Sensor QoS avoids “RViz sees nothing” surprises
        self.pub = self.create_publisher(PointCloud2, out_topic, qos_profile_sensor_data)
        self.sub = self.create_subscription(PointCloud2, in_topic, self.cb, qos_profile_sensor_data)

        self.get_logger().info(f"Rewriting {in_topic} frame_id -> '{self.frame_id}' and republishing on {out_topic}")

    def cb(self, msg: PointCloud2):
        msg.header.frame_id = self.frame_id
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = OakPointsFrameRewrite()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

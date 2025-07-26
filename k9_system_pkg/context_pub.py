#!/usr/bin/env python3

import rclpy
from k9_interfaces_pkg.msg import ContextLine
from rclpy.node import Node


class ContextLinePublisher:
    def __init__(self, node: Node, topic_name: str):
        """
        Initialize a publisher for a ContextLine message.

        :param node: A rclpy Node object to attach the publisher to.
        :param topic_name: The topic to publish ContextLine messages on.
        """
        self.node = node
        self.publisher = node.create_publisher(ContextLine, topic_name, 10)

    def publish(self, summary: str):
        """
        Publish a ContextLine message with the given summary.

        :param summary: The summary text to send as context.
        """
        msg = ContextLine()
        msg.summary = summary
        self.publisher.publish(msg)
        self.node.get_logger().debug(f"Published ContextLine: {summary}")

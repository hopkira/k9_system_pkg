import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from k9_interfaces_pkg.msg import ContextLine
from k9_interfaces_pkg.srv import GenerateUtterance

import yaml
import os
import time
from math import exp

class ContextAggregatorNode(Node):
    '''Simple description'''

    def __init__(self):
        super().__init__('context_aggregator')

        self.declare_parameter('config_path', 'context_order.yaml')
        self.declare_parameter('min_interval', 30.0)
        self.declare_parameter('max_interval', 1800.0)
        self.declare_parameter('k', 0.1)

        self.min_interval = self.get_parameter('min_interval').value
        self.max_interval = self.get_parameter('max_interval').value
        self.k = self.get_parameter('k').value

        self.config_path = self.get_parameter('config_path').value
        self.load_context_order()

        self.subscribers = {}
        self.context_lines = {}
        self.context_activity_log = {}

        self.voice_pub = self.create_publisher(String, '/voice/speak', 10)
        self.cli = self.create_client(GenerateUtterance, 'generate_utterance')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for LLM service...')

        self.scan_timer = self.create_timer(10.0, self.scan_topics)
        self.speak_timer = self.create_timer(60.0, self.speak_timer_callback)

    def load_context_order(self):
        '''
        Retrieve a list of ordered topics to enable the
        context string to be formed
        '''

        if os.path.exists(self.config_path):
            with open(self.config_path, 'r') as f:
                self.config = yaml.safe_load(f)
        else:
            self.config = {'order': []}
        self.ordered_topics = self.config.get('order', [])

    def save_context_order(self):
        '''Save the list of ordered topics'''
        self.config['order'] = self.ordered_topics
        with open(self.config_path, 'w') as f:
            yaml.safe_dump(self.config, f)

    def scan_topics(self):
        '''
        Find, subscribe and add Context topics to robot context.

        Find topics in the ROS2 workspace that are using the
        ContextLine type.  Subscribe to those topics
        if we're not already subscribed. Add any new topics not
        in the YAML order list to the bottom of that list, so
        they will be added to the context.
        '''
        topics = dict(self.get_topic_names_and_types())
        for topic, types in topics.items():
            if 'robot_context_msgs/msg/ContextLine' in types and topic not in self.subscribers:
                self.get_logger().info(f"Discovered context topic: {topic}")
                self.subscribers[topic] = self.create_subscription(
                    ContextLine, topic, self.make_callback(topic), 10)
                self.context_activity_log[topic] = []
                if topic not in self.ordered_topics:
                    self.ordered_topics.append(topic)
                    self.save_context_order()
                    self.get_logger().info(f"Added new context topic to YAML: {topic}")

    def make_callback(self, topic_name):
        def callback(msg):
            self.context_lines[topic_name] = msg.summary
            now = time.time()
            self.context_activity_log[topic_name].append(now)
        return callback

    def compute_activity_score(self):
        '''Work out activity level from the activity log.'''

        now = time.time()
        window = 300  # 5 minutes
        return sum(
            1 for timestamps in self.context_activity_log.values()
            for t in timestamps if now - t < window
        )

    def speak_timer_callback(self):
        '''Work out when to next speak based on score'''

        score = self.compute_activity_score()
        interval = self.min_interval + (self.max_interval - self.min_interval) * exp(-self.k * score)

        self.get_logger().info(f"Activity score: {score}, next utterance in {interval:.1f} sec")
        self.trigger_llm()

        self.speak_timer.cancel() # remove old timer
        self.speak_timer = self.create_timer(interval, self.speak_timer_callback)

    def trigger_llm(self):
        '''Aggregate the prompt with all the context sentences and call GenerateUtterance'''

        lines = [self.context_lines.get(t) for t in self.ordered_topics if self.context_lines.get(t)]
        if not lines:
            self.get_logger().info("No context lines available yet.")
            return

        prompt = (
            "Given this context of what is going on around you, "
            "formulate a witty and pertinent and in character "
            "sentence that might delight, amuse or surprise your audience.\n\n"
        ) + "\n".join(lines)

        req = GenerateUtterance.Request()
        req.input = prompt
        future = self.cli.call_async(req)
        future.add_done_callback(self.handle_llm_response)

    def handle_llm_response(self, future):
        '''Receive the Utterance and then publish to voice'''
        try:
            result = future.result()
            self.voice_pub.publish(String(data=result.output))
            self.get_logger().info(f"K9 says: {result.output}")
        except Exception as e:
            self.get_logger().error(f"LLM call failed: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ContextAggregatorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
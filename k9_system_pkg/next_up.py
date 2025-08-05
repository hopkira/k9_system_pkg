# next_up.py
# This file is part of a ROS 2 package that interacts with Google Calendar.
# It provides a service to retrieve upcoming calendar events.
# Ensure you have the necessary credentials and token.pickle file set up.
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from std_msgs.msg import String
from calendar_helper import get_upcoming_events

class CalendarInfoNode(Node):
    def __init__(self):
        super().__init__('calendar_info_node')

        # Service for on-demand request
        self.srv = self.create_service(
            Trigger,
            'get_calendar_events',
            self.handle_calendar_request
        )

        # Publisher for periodic updates (e.g. to robot voice system)
        self.publisher_ = self.create_publisher(
            String,
            '/robot_dog/calendar_summary',
            10
        )

        # Timer for periodic polling (every 30 minutes)
        self.timer = self.create_timer(
            1800.0,  # 1800 seconds = 30 minutes
            self.check_calendar_periodically
        )

        self.get_logger().info('Calendar Info Node ready (service + periodic mode).')

    def handle_calendar_request(self, request, response):
        try:
            events = get_upcoming_events()
            if not events:
                message = "No upcoming events."
            else:
                message = "\n".join([f"{title} at {time}" for title, time in events])

            response.success = True
            response.message = message
            self.get_logger().info('[Manual] Calendar queried.')

        except Exception as e:
            response.success = False
            response.message = str(e)
            self.get_logger().error(f'[Manual] Error accessing calendar: {e}')

        return response

    def check_calendar_periodically(self):
        try:
            events = get_upcoming_events()
            if not events:
                message = "No upcoming events."
            else:
                message = "Upcoming events: " + "; ".join(
                    [f"{title} at {time}" for title, time in events])

            msg = String()
            msg.data = message
            self.publisher_.publish(msg)
            self.get_logger().info('[Auto] Calendar updated and published.')

        except Exception as e:
            self.get_logger().error(f"[Auto] Error accessing calendar: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = CalendarInfoNode()
    rclpy.spin(node)
    rclpy.shutdown()

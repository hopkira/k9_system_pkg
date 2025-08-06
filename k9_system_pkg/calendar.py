import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger
from k9_system_pkg import calendar_helper as cal
import json
from datetime import datetime, timedelta
import dateutil.parser
import pytz

# ROS2 Calendar Node
#
# Start node and speak reminder every 60 seconds:
# ros2 run k9_system_pkg calendar_node --ros-args -p check_interval:=60
#
# Speak today's appointments:
# ros2 service call /get_today_appointments std_srvs/srv/Trigger
#
# Speak the next appointment:
# ros2 service call /get_next_appointment std_srvs/srv/Trigger
#

class CalendarNode(Node):
    def __init__(self):
        super().__init__('calendar_node')

        # Declare configurable parameter for interval
        self.declare_parameter('check_interval', 60)
        interval = self.get_parameter('check_interval').get_parameter_value().integer_value

        # Publisher to TTS topic
        self.tts_pub = self.create_publisher(String, 'tts_input', 10)

        # Timer to check calendar
        self.timer = self.create_timer(interval, self.check_calendar)

        # Services
        self.srv_today = self.create_service(Trigger, 'get_today_appointments', self.get_today_appointments)
        self.srv_next = self.create_service(Trigger, 'get_next_appointment', self.get_next_appointment)

        self.warned_events = set()
        self.timezone = pytz.utc  # Update this if needed
        self.get_logger().info('Calendar node started.')

    def check_calendar(self):
        try:
            events = cal.get_upcoming_events(max_results=10)
            now = datetime.now(tz=self.timezone)

            for summary, start_str in events:
                start_time = dateutil.parser.parse(start_str)
                time_to_event = (start_time - now).total_seconds()

                if 295 < time_to_event < 305 and start_str not in self.warned_events:
                    self.warned_events.add(start_str)
                    message = f"You have a meeting in five minutes: {summary}."
                    self.publish_tts(message)
        except Exception as e:
            self.get_logger().error(f"Error checking calendar: {e}")

    def publish_tts(self, text):
        msg = String()
        msg.data = text
        self.tts_pub.publish(msg)
        self.get_logger().info(f"Published to tts_input: {text}")

    def get_today_appointments(self, request, response):
        try:
            today = datetime.now(tz=self.timezone)
            end_of_day = today.replace(hour=23, minute=59, second=59)

            events = cal.get_upcoming_events(max_results=20)
            events_today = [(s, t) for s, t in events if dateutil.parser.parse(t) <= end_of_day]

            if not events_today:
                response.success = True
                response.message = "You have no meetings for the rest of today."
            else:
                sentence = self.format_event_list(events_today)
                response.success = True
                response.message = sentence
                self.publish_tts(sentence)

        except Exception as e:
            response.success = False
            response.message = f"Error fetching today’s appointments: {e}"

        return response

    def get_next_appointment(self, request, response):
        try:
            events = cal.get_upcoming_events(max_results=1)
            if not events:
                sentence = "You have no upcoming meetings."
                response.success = True
                response.message = sentence
            else:
                summary, start = events[0]
                sentence = self.format_event(summary, start)
                response.success = True
                response.message = sentence

            self.publish_tts(response.message)
        except Exception as e:
            response.success = False
            response.message = f"Error fetching next appointment: {e}"

        return response

    def format_event(self, summary, start):
        dt = dateutil.parser.parse(start).astimezone(self.timezone)
        spoken_time = dt.strftime('%I:%M %p').lstrip("0").replace(':00', '')  # e.g., '10:00 AM' → '10 a.m.'
        spoken_time = spoken_time.replace('AM', 'a.m.').replace('PM', 'p.m.')
        return f"You have a meeting at {spoken_time} with {summary}."

    def format_event_list(self, events):
        sentences = []
        for summary, start in events:
            sentences.append(self.format_event(summary, start))
        return " ".join(sentences)


def main(args=None):
    rclpy.init(args=args)
    node = CalendarNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

# voice_input_manager/voice_state_manager.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from k9_interfaces_pkg.srv import SetVoiceState

class VoiceStateManager(Node):
    def __init__(self):
        super().__init__('voice_state_manager')
        self.state_pub = self.create_publisher(String, 'voice_state', 10)
        self.srv = self.create_service(SetVoiceState, 'set_voice_input_state', self.set_state_callback)
        self.current_state = 'NotListening'
        self.publish_state()

    def set_state_callback(self, request, response):
        if request.state not in ['NotListening', 'WaitingForHotword', 'Listening']:
            response.success = False
            response.message = f"Invalid state: {request.state}"
            return response

        self.current_state = request.state
        self.publish_state()
        response.success = True
        response.message = f"State set to {request.state}"
        self.get_logger().info(response.message)
        return response

    def publish_state(self):
        msg = String()
        msg.data = self.current_state
        self.state_pub.publish(msg)


def main():
    rclpy.init()
    node = VoiceStateManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
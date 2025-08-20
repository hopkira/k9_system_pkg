import rclpy
import threading
import time
import os

from rclpy.node import Node
from std_msgs.msg import Bool, Empty
from std_srvs.srv import SetBool
from ament_index_python.packages import get_package_share_directory

import pvporcupine
from pvrecorder import PvRecorder
from secrets import ACCESS_KEY

package_path = get_package_share_directory('k9_system_pkg')
ppn_path = os.path.join(package_path, 'assets', 'canine_en_raspberry-pi_v2_1_0.ppn')


class HotwordNode(Node):
    def __init__(self):
        super().__init__('hotword_node')

        # Runtime state
        self.listening = False
        self.lock = threading.Lock()

        # ROS interfaces
        self.create_service(SetBool, 'enable_hotword', self.handle_enable_hotword)
        self.publisher = self.create_publisher(Empty, 'hotword_detected', 10)

        # Background listening thread
        self.thread = threading.Thread(target=self.listen_loop, daemon=True)
        self.thread.start()

        # Setup placeholders
        self.porcupine = None
        self.recorder = None

        self.get_logger().info("HotwordNode ready.")

    def handle_enable_hotword(self, request, response):
        with self.lock:
            if request.data and not self.listening:
                try:
                    self.porcupine = pvporcupine.create(
                        access_key=ACCESS_KEY,
                        keyword_paths=[ppn_path]
                    )
                    self.recorder = PvRecorder(device_index=-1, frame_length=self.porcupine.frame_length)
                    self.recorder.start()
                    self.listening = True
                    self.get_logger().info("Hotword detection ENABLED.")
                    response.success = True
                    response.message = "Listening started."
                except Exception as e:
                    self.get_logger().error(f"Failed to start Porcupine: {e}")
                    response.success = False
                    response.message = str(e)
            elif not request.data and self.listening:
                self.stop_listening()
                self.get_logger().info("Hotword detection DISABLED.")
                response.success = True
                response.message = "Listening stopped."
            else:
                response.success = True
                response.message = "No change."
        return response

    def stop_listening(self):
        if self.recorder:
            self.recorder.stop()
            self.recorder.delete()
            self.recorder = None
        if self.porcupine:
            self.porcupine.delete()
            self.porcupine = None
        self.listening = False

    def listen_loop(self):
        while rclpy.ok():
            with self.lock:
                if self.listening and self.recorder and self.porcupine:
                    try:
                        pcm = self.recorder.read()
                        result = self.porcupine.process(pcm)
                        if result >= 0:
                            self.get_logger().info("Hotword detected!")
                            self.publisher.publish(Empty())
                            self.stop_listening()  # stop immediately
                    except Exception as e:
                        self.get_logger().error(f"Detection error: {e}")
                        self.stop_listening()
                else:
                    time.sleep(0.1)  # idle wait


def main(args=None):
    rclpy.init(args=args)
    node = HotwordNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_listening()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
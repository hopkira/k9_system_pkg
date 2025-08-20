import rclpy
import threading
import time
import os

from rclpy.node import Node
from std_msgs.msg import Empty
from audio_common_msgs.msg import AudioData
from ament_index_python.packages import get_package_share_directory

import pvporcupine
from pvrecorder import PvRecorder
from secrets import ACCESS_KEY

package_path = get_package_share_directory('k9_system_pkg')
ppn_path = os.path.join(package_path, 'assets', 'canine_en_raspberry-pi_v2_1_0.ppn')


class HotwordNode(Node):
    def __init__(self):
        super().__init__('hotword_node')

        # ROS interfaces
        self.hotword_pub = self.create_publisher(Empty, 'hotword_detected', 10)
        self.audio_pub = self.create_publisher(AudioData, 'mic_audio', 10)

        # Setup Porcupine + recorder
        self.porcupine = pvporcupine.create(
            access_key=ACCESS_KEY,
            keyword_paths=[ppn_path]
        )
        self.recorder = PvRecorder(device_index=-1, frame_length=self.porcupine.frame_length)
        self.recorder.start()

        # Background thread
        self.thread = threading.Thread(target=self.listen_loop, daemon=True)
        self.thread.start()

        self.get_logger().info("HotwordNode ready (mic open, streaming audio).")

    def listen_loop(self):
        while rclpy.ok():
            try:
                pcm = self.recorder.read()  # list[int16]
                # Publish audio for STT
                msg = AudioData()
                msg.data = bytearray(pcm)  # convert to bytes
                self.audio_pub.publish(msg)

                # Run hotword detection
                result = self.porcupine.process(pcm)
                if result >= 0:
                    self.get_logger().info("Hotword detected!")
                    self.hotword_pub.publish(Empty())
                    # NOTE: mic stays open; STT node will react to voice_state
            except Exception as e:
                self.get_logger().error(f"Detection error: {e}")
                time.sleep(0.1)

    def destroy_node(self):
        # Cleanup when node is shut down
        if self.recorder:
            self.recorder.stop()
            self.recorder.delete()
        if self.porcupine:
            self.porcupine.delete()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = HotwordNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
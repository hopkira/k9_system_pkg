# hotword_listener/hotword_listener.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Empty
import pvporcupine
import sounddevice as sd
import numpy as np

class HotwordListener(Node):
    def __init__(self):
        super().__init__('hotword_listener')
        self.state = 'NotListening'
        self.sub = self.create_subscription(String, 'voice_state', self.state_callback, 10)
        self.pub = self.create_publisher(Empty, 'hotword_detected', 10)
        self.timer = self.create_timer(0.1, self.detect_loop)

        self.porcupine = pvporcupine.create(keywords=["hey pico"])
        self.audio_stream = sd.InputStream(
            samplerate=self.porcupine.sample_rate,
            blocksize=self.porcupine.frame_length,
            channels=1,
            dtype='int16',
            callback=self.audio_callback
        )
        self.buffer = []

    def state_callback(self, msg):
        self.state = msg.data

    def detect_loop(self):
        if self.state == 'WaitingForHotword':
            if not self.audio_stream.active:
                self.audio_stream.start()
        else:
            if self.audio_stream.active:
                self.audio_stream.stop()

    def audio_callback(self, indata, frames, time, status):
        if self.state != 'WaitingForHotword':
            return
        pcm = np.frombuffer(indata, dtype=np.int16)
        result = self.porcupine.process(pcm)
        if result >= 0:
            self.get_logger().info("Hotword detected!")
            self.pub.publish(Empty())


def main():
    rclpy.init()
    node = HotwordListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
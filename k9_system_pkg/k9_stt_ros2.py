# speech_to_text/speech_to_text.py
import rclpy
import threading
from rclpy.node import Node
from std_msgs.msg import String, Empty
from audio_common_msgs.msg import AudioData  # standard ROS audio message
import numpy as np
import queue

class SpeechToText(Node):
    def __init__(self):
        super().__init__('speech_to_text')
        self.get_logger().info("STT node starting...")
        self.state = 'NotListening'

        # Subscribe to hotword events
        self.sub_hotword = self.create_subscription(
            Empty,
            '/hotword_detected',
            self.hotword_callback,
            10
        )
        self.get_logger().info("Subscribed to hotword_detected")

        self.sub_audio = self.create_subscription(AudioData, 'mic_audio', self.audio_callback, 10)
        self.pub = self.create_publisher(String, 'heard', 10)

        self.audio_queue = queue.Queue()

        # Timer for periodic transcription
        self.timer = self.create_timer(0.5, self.process_audio)

        threading.Thread(target=self.load_whisper_model, daemon=True).start()

    def load_whisper_model(self):
        self.get_logger().info("Loading Whisper model in background...")
        from faster_whisper import WhisperModel
        self.model = WhisperModel("base", compute_type="int8")
        self.get_logger().info("Whisper model loaded")


    def hotword_callback(self, msg: Empty):
        self.state = 'Listening'
        self.get_logger().info("Hotword detected -> Listening...")

    def audio_callback(self, msg: AudioData):
        """Receives audio chunks from mic node (always running)."""
        if self.state != 'Listening':
            return
        # msg.data is a byte array → convert to np.int16
        audio_np = np.frombuffer(msg.data, dtype=np.int16)
        self.audio_queue.put(audio_np)

    def process_audio(self):
        if self.state != 'Listening' or self.audio_queue.empty():
            return

        # Gather all queued audio
        audio_data = []
        while not self.audio_queue.empty():
            audio_data.append(self.audio_queue.get())
        audio_np = np.concatenate(audio_data).astype(np.float32) / 32768.0

        # Run Whisper
        segments, _ = self.model.transcribe(audio_np, language="en")
        for segment in segments:
            transcript = segment.text.strip()
            if transcript:
                self.get_logger().info(f"Heard: {transcript}")
                msg = String()
                msg.data = transcript
                self.pub.publish(msg)


def main():
    rclpy.init()
    node = SpeechToText()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

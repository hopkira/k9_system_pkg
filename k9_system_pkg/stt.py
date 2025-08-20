# speech_to_text/speech_to_text.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from audio_common_msgs.msg import AudioData  # standard ROS audio message
from faster_whisper import WhisperModel
import numpy as np
import queue

class SpeechToText(Node):
    def __init__(self):
        super().__init__('speech_to_text')
        self.state = 'NotListening'
        self.sub_state = self.create_subscription(String, 'voice_state', self.state_callback, 10)
        self.sub_audio = self.create_subscription(AudioData, 'mic_audio', self.audio_callback, 10)
        self.pub = self.create_publisher(String, 'heard', 10)

        self.audio_queue = queue.Queue()

        # Load whisper model once
        self.model = WhisperModel("base", compute_type="int8")

        # Timer for periodic transcription
        self.timer = self.create_timer(0.5, self.process_audio)

    def state_callback(self, msg):
        self.state = msg.data
        if self.state == 'Listening':
            self.get_logger().info("Listening for speech...")
        else:
            self.get_logger().info(f"Voice state: {self.state}")

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
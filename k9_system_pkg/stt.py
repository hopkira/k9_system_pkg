# speech_to_text/speech_to_text.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from faster_whisper import WhisperModel
import sounddevice as sd
import numpy as np
import queue

class SpeechToText(Node):
    def __init__(self):
        super().__init__('speech_to_text')
        self.state = 'NotListening'
        self.sub = self.create_subscription(String, 'voice_state', self.state_callback, 10)
        self.pub = self.create_publisher(String, 'heard', 10)
        self.audio_queue = queue.Queue()
        self.recording = False

        self.model = WhisperModel("base", compute_type="int8")
        self.stream = sd.InputStream(samplerate=16000, channels=1, dtype='int16', callback=self.audio_callback)
        self.timer = self.create_timer(0.5, self.process_audio)

    def state_callback(self, msg):
        self.state = msg.data
        if self.state == 'Listening':
            self.get_logger().info("Listening for speech...")
            if not self.stream.active:
                self.stream.start()
        else:
            if self.stream.active:
                self.stream.stop()

    def audio_callback(self, indata, frames, time, status):
        if self.state != 'Listening':
            return
        self.audio_queue.put(indata.copy())

    def process_audio(self):
        if self.state != 'Listening' or self.audio_queue.empty():
            return

        audio_data = []
        while not self.audio_queue.empty():
            audio_data.append(self.audio_queue.get())
        audio_np = np.concatenate(audio_data, axis=0).flatten().astype(np.float32) / 32768.0

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
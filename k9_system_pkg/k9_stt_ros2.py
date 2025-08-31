#!/usr/bin/env python3
import numpy as np
import webrtcvad
from faster_whisper import WhisperModel
import queue
import threading
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Empty
from audio_common_msgs.msg import AudioData
from k9_interfaces_pkg.srv import EmptySrv

class SpeechToTextNode(Node):
    def __init__(self):
        super().__init__("speech_to_text")

        # --- Parameters ---
        self.declare_parameter("vad_aggressiveness", 2)
        self.declare_parameter("silence_timeout", 1.0)
        self.declare_parameter("model_size", "tiny")
        self.declare_parameter("model_compute_type", "int8")
        self.declare_parameter("mic_audio_topic", "/mic_audio")
        # self.declare_parameter("max_listen_duration", 30.0)  # Now handled by BT

        self.vad_aggressiveness = self.get_parameter("vad_aggressiveness").value
        self.silence_timeout = self.get_parameter("silence_timeout").value
        model_size = self.get_parameter("model_size").value
        compute_type = self.get_parameter("model_compute_type").value
        self.mic_audio_topic = self.get_parameter("mic_audio_topic").value
        # self.max_listen_duration = self.get_parameter("max_listen_duration").value

        # --- Audio parameters ---
        self.SAMPLE_RATE = 16000
        self.FRAME_DURATION_MS = 30
        self.FRAME_SIZE = int(self.SAMPLE_RATE * self.FRAME_DURATION_MS / 1000)

        # --- VAD + Whisper ---
        self.vad = webrtcvad.Vad(self.vad_aggressiveness)
        self.model = WhisperModel(model_size, compute_type=compute_type)

        # --- Buffers and state ---
        self.speech_buffer = []
        self.speech_queue = queue.Queue()
        self.last_voice_time = None
        self.lock = threading.Lock()
        self.pre_buffer = []
        self.pre_buffer_frames = int(self.SAMPLE_RATE * 0.5)
        self.is_listening = False
        self.listen_start_time = None

        # --- ROS interfaces ---
        self.text_pub = self.create_publisher(String, "/speech_to_text/text", 10)
        self.state_pub = self.create_publisher(String, "/speech_to_text/state", 10)

        self.start_service = self.create_service(EmptySrv, "start_listening", self.start_listening)
        self.stop_service = self.create_service(EmptySrv, "stop_listening", self.stop_listening)

        self.audio_sub = self.create_subscription(
            AudioData,
            self.mic_audio_topic,
            self.audio_callback,
            10
        )

        threading.Thread(target=self.transcription_loop, daemon=True).start()
        self.get_logger().info(f"SpeechToText node initialized (listening to {self.mic_audio_topic})")

    # --- Services ---
    def start_listening(self, request, response):
        self.is_listening = True
        self.listen_start_time = time.time()
        self.state_pub.publish(String(data="listening"))
        self.get_logger().info("Listening started")
        return response

    def stop_listening(self, request, response):
        self.is_listening = False
        self.state_pub.publish(String(data="not_listening"))
        self.get_logger().info("Listening stopped")
        return response

    # --- Audio callback ---
    def audio_callback(self, msg: AudioData):
        if not self.is_listening:
            return

        pcm16 = np.frombuffer(msg.data, dtype=np.int16)

        for start in range(0, len(pcm16), self.FRAME_SIZE):
            frame = pcm16[start:start + self.FRAME_SIZE]
            if len(frame) < self.FRAME_SIZE:
                continue

            self.pre_buffer.extend(frame.tolist())
            if len(self.pre_buffer) > self.pre_buffer_frames:
                self.pre_buffer = self.pre_buffer[-self.pre_buffer_frames:]

            is_speech = self.vad.is_speech(frame.tobytes(), self.SAMPLE_RATE)

            with self.lock:
                if is_speech:
                    if not self.speech_buffer:
                        self.speech_buffer.extend(self.pre_buffer)
                    self.speech_buffer.extend(frame.tolist())
                    self.last_voice_time = time.time()
                else:
                    if self.speech_buffer and self.last_voice_time:
                        if time.time() - self.last_voice_time >= self.silence_timeout:
                            self.speech_queue.put(np.array(self.speech_buffer, dtype=np.int16))
                            self.speech_buffer.clear()
        ''' - Now handled by BT
        # Absolute timeout
        if self.listen_start_time and (time.time() - self.listen_start_time > self.max_listen_duration):
            self.get_logger().info("Max listening duration reached, stopping.")
            self.stop_listening(None, None)
        '''

    # --- Transcription loop ---
    def transcription_loop(self):
        while rclpy.ok():
            audio_np = self.speech_queue.get()
            if audio_np is None:
                break
            audio_f32 = audio_np.astype(np.float32) / 32768.0
            segments, _ = self.model.transcribe(audio_f32, language="en")
            for segment in segments:
                text = segment.text.strip()
                if not text or text == ".":
                    continue
                if len(text) < 3 or not any(c.isalpha() for c in text):
                    continue
                msg = String()
                msg.data = text
                self.text_pub.publish(msg)
                self.get_logger().info(f"Heard: {text}")

    def destroy_node(self):
        self.speech_queue.put(None)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SpeechToTextNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

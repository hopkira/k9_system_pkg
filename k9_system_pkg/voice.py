#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from k9_interfaces_pkg.srv import Speak
from k9_interfaces_pkg.srv import CancelSpeech  # Updated to k9_voice package
import threading
import numpy as np
import sounddevice as sd
from piper.voice import PiperVoice
from queue import Queue, Empty

class K9TTSNode(Node):
    def __init__(self):
        super().__init__('k9_tts_node')

        model_path = "./k9_2449_model.onnx"  # Update with your model path
        self.voice = PiperVoice.load(model_path)

        # Subscribe to the topic for regular speech
        self.subscription = self.create_subscription(
            String,
            'tts_input',
            self.tts_callback,
            10)

        # Publisher for is_talking status
        self.publisher = self.create_publisher(Bool, 'is_talking', 10)

        # Create services for interrupting and canceling speech
        self.create_service(Speak, 'speak_now', self.speak_now_callback)
        self.create_service(CancelSpeech, 'cancel_speech', self.cancel_speech_callback)

        # Queue and thread for speech processing
        self.text_queue = Queue()
        self.interrupt_event = threading.Event()
        self.queue_lock = threading.Lock()

        self.worker_thread = threading.Thread(target=self.process_queue, daemon=True)
        self.worker_thread.start()

        self.get_logger().info('K9TTSNode with interrupt and cancel speech services is running.')

    def tts_callback(self, msg):
        """Callback for the tts_input topic"""
        text = msg.data.strip()
        if not text:
            self.get_logger().warn("Received empty text, ignoring.")
            return
        self.text_queue.put(text)
        self.get_logger().info(f"Received text for TTS: '{text}'")

    def process_queue(self):
        """Worker thread that processes the speech queue"""
        try:
            stream = sd.OutputStream(
                samplerate=self.voice.config.sample_rate,
                channels=1,
                dtype='int16'
            )
            stream.start()
        except Exception as e:
            self.get_logger().error(f"Failed to start audio stream: {e}")
            return

        while rclpy.ok():
            text = self.text_queue.get()
            self.publish_status(True)
            self.interrupt_event.clear()

            try:
                for audio_bytes in self.voice.synthesize_stream_raw(text):
                    if self.interrupt_event.is_set():
                        self.get_logger().info("Speech interrupted.")
                        break
                    int_data = np.frombuffer(audio_bytes, dtype=np.int16)
                    stream.write(int_data)
            except Exception as e:
                self.get_logger().error(f"TTS error: {e}")

            self.publish_status(False)
            self.text_queue.task_done()

        stream.stop()
        stream.close()

    def publish_status(self, is_talking):
        """Publish the is_talking status"""
        msg = Bool()
        msg.data = is_talking
        self.publisher.publish(msg)
        self.get_logger().info(f"Talking: {is_talking}")

    def speak_now_callback(self, request, response):
        """Service callback to interrupt and immediately speak"""
        text = request.text.strip()
        if not text:
            response.success = False
            response.message = "Empty text"
            self.get_logger().warn("Received empty text for speak_now service.")
            return response

        self.get_logger().info(f"Interrupt received via service: '{text}'")

        # Clear the queue and signal interruption
        with self.queue_lock:
            self.interrupt_event.set()
            while not self.text_queue.empty():
                try:
                    self.text_queue.get_nowait()
                    self.text_queue.task_done()
                except Empty:
                    break
            self.text_queue.put(text)

        response.success = True
        response.message = "Speech interrupt queued"
        self.get_logger().info("Speech interrupt queued.")
        return response

    def cancel_speech_callback(self, request, response):
        """Service callback to cancel speech"""
        self.get_logger().info("Cancel speech request received.")
        with self.queue_lock:
            self.interrupt_event.set()
            # Clear the queue
            while not self.text_queue.empty():
                try:
                    self.text_queue.get_nowait()
                    self.text_queue.task_done()
                except Empty:
                    break
        response.success = True
        response.message = "Speech queue cleared and current speech interrupted."
        return response

def main(args=None):
    rclpy.init(args=args)
    node = K9TTSNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

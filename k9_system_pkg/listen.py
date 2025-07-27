import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

import numpy as np
import deepspeech
from audio_tools import VADAudio
import threading
import time

# TODO Ensure the model path is correct
# TODO Handle model loading errors gracefully
# TODO Add error handling for audio input issues
# TODO Make the model path into a ROS 2 resource parameter

class ListenNode(Node):
    def __init__(self):
        super().__init__('listen_node')

        # Load DeepSpeech
        self.model = deepspeech.Model("/home/pi/k9/deepspeech-0.9.3-models.tflite")
        self.model.enableExternalScorer("/home/pi/k9/deepspeech-0.9.3-models.scorer")

        # ROS interfaces
        self.command_pub = self.create_publisher(String, 'heard_command', 10)
        self.start_service = self.create_service(Trigger, 'start_listening', self.handle_start_listening)
        self.stop_service = self.create_service(Trigger, 'stop_listening', self.handle_stop_listening)

        # State
        self.listening = False
        self.lock = threading.Lock()
        self.stop_event = threading.Event()

        self.get_logger().info("ListenNode ready: waiting for start_listening service.")

    def handle_start_listening(self, request, response):
        with self.lock:
            if not self.listening:
                self.get_logger().info("Listening started...")
                self.listening = True
                self.stop_event.clear()
                threading.Thread(target=self.listen_for_command, daemon=True).start()
                response.success = True
                response.message = "Listening started."
            else:
                response.success = False
                response.message = "Already listening."
        return response

    def handle_stop_listening(self, request, response):
        with self.lock:
            if self.listening:
                self.get_logger().info("Listening stopped by request.")
                self.stop_event.set()
                response.success = True
                response.message = "Listening stopped."
            else:
                response.success = False
                response.message = "Not currently listening."
        return response

    def listen_for_command(self):
        vad_audio = None
        stream_context = None
        try:
            vad_audio = VADAudio(aggressiveness=1, device=None, input_rate=16000, file=None)
            stream_context = self.model.createStream()

            start_time = time.time()
            MAX_LISTEN_TIME = 10.0  # Timeout for silence or cancellation

            for frame in vad_audio.vad_collector():
                if self.stop_event.is_set():
                    self.get_logger().warn("Listening cancelled.")
                    break

                if time.time() - start_time > MAX_LISTEN_TIME:
                    self.get_logger().warn("Listening timed out.")
                    break

                if frame is not None:
                    stream_context.feedAudioContent(np.frombuffer(frame, np.int16))
                else:
                    command = stream_context.finishStream()
                    if command:
                        self.get_logger().info(f"Recognized: {command}")
                        self.command_pub.publish(String(data=command))
                    else:
                        self.get_logger().info("No speech recognized.")
                    break

        except Exception as e:
            self.get_logger().error(f"Error while listening: {e}")

        finally:
            with self.lock:
                self.listening = False
            if vad_audio:
                vad_audio.destroy()
            if stream_context:
                del stream_context
            self.get_logger().info("Listening complete. Node is idle.")


def main(args=None):
    rclpy.init(args=args)
    node = ListenNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
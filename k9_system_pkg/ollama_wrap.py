#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from k9_interfaces_pkg.srv import GenerateUtterance
import threading
import ollama

MODEL_NAME = 'granite3.3:2b'

PROMPT_TEMPLATE = (
    "You are K9, the robotic dog from Doctor Who. "
    "User says: {input}. "
    "Reply in a single line of at most two short, precise sentences. "
    "Use a mechanical, formal tone. "
    "Say 'affirmative' instead of yes and 'negative' instead of no. "
    "Be literal, logical, and occasionally dryly humorous. "
    "Do not use contractions, idioms, or emotional language."
)

class OllamaLLMNode(Node):
    def __init__(self):
        super().__init__('ollama_llm_node')

        # ROS interfaces
        self.voice_pub = self.create_publisher(String, '/voice/tts_input', 10)
        self.srv = self.create_service(
            GenerateUtterance,
            'generate_utterance',
            self.handle_service_request
        )

        # Subscribe to STT output for automatic responses
        self.create_subscription(String, '/speech_to_text/text', self.handle_stt_text, 10)

        self.get_logger().info("Ollama LLM Node ready (service + subscriber).")

    def handle_service_request(self, request, response):
        """Service callback: generate utterance for a given input string."""
        threading.Thread(target=self._generate_and_publish, args=(request.input, response), daemon=True).start()
        return response  # Return immediately; response will be filled asynchronously

    def handle_stt_text(self, msg: String):
        """Subscriber callback: automatically respond to STT text."""
        threading.Thread(target=self._generate_and_publish, args=(msg.data, None), daemon=True).start()

    def _generate_and_publish(self, user_input: str, response=None):
        """Internal method: call the LLM, publish to TTS, and optionally fill service response."""
        try:
            prompt = PROMPT_TEMPLATE.format(input=user_input)
            result = ollama.generate(model=MODEL_NAME, prompt=prompt)
            output_text = getattr(result, 'response', '').strip()  # adjust if dict

            if output_text:
                self.publish_to_voice(output_text)
                self.get_logger().info(f"Generated: {output_text}")
                if response is not None:
                    response.output = output_text
            else:
                fallback = "Apologies, my cognitive faculties are temporarily impaired."
                self.publish_to_voice(fallback)
                if response is not None:
                    response.output = fallback
        except Exception as e:
            self.get_logger().error(f"LLM failure: {e}")
            fallback = "Apologies, my cognitive faculties are temporarily impaired."
            self.publish_to_voice(fallback)
            if response is not None:
                response.output = fallback

    def publish_to_voice(self, text: str):
        """Publish the generated text to the TTS topic."""
        msg = String()
        msg.data = text
        self.voice_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = OllamaLLMNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()

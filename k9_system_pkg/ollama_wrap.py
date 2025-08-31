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

        # ROS service
        self.srv = self.create_service(
            GenerateUtterance,
            'generate_utterance',
            self.handle_service_request
        )

        # Optional: auto responses to STT, do not return via service
        # self.create_subscription(String, '/speech_to_text/text', self.handle_stt_text, 10)

        self.get_logger().info("Ollama LLM Node ready (service + subscriber).")

    def handle_service_request(self, request, response):
        """Service callback: generate utterance and fill response."""
        threading.Thread(target=self._generate_response, args=(request.input, response), daemon=True).start()
        return response  # Return immediately; response will be filled asynchronously

    ''' - removed the non-service route for simplicity
    def handle_stt_text(self, msg: String):
        """Subscriber callback: automatically generate text for STT input (does not return service)."""
        threading.Thread(target=self._generate_response, args=(msg.data, None), daemon=True).start()
    '''

    def _generate_response(self, user_input: str, response=None):
        """Internal method: call the LLM and optionally fill service response."""
        try:
            prompt = PROMPT_TEMPLATE.format(input=user_input)
            result = ollama.generate(model=MODEL_NAME, prompt=prompt)
            output_text = getattr(result, 'response', '').strip()

            if not output_text:
                output_text = "Apologies, my cognitive faculties are temporarily impaired."

            self.get_logger().info(f"Generated: {output_text}")

            if response is not None:
                response.output = output_text

        except Exception as e:
            self.get_logger().error(f"LLM failure: {e}")
            fallback = "Apologies, my cognitive faculties are temporarily impaired."
            if response is not None:
                response.output = fallback

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

#!/usr/bin/env python3

'''
Pre-reqs

Requires ollama API and Python Client.
Advised to pre-download the granite3.3:2b model or similar
sudo systemctl enable --now ollama
ollama pull granite3.3:2b
ollama pull granite3.3:2b
python3 -m pip install --user --break-system-packages ollama

After colcon build:
source ~/k9_ws/install/setup.bash
ros2 run k9_system_pkg ollama
ros2 service list | grep generate_utterance
ros2 service call /generate_utterance k9_interfaces_pkg/srv/GenerateUtterance "{input: 'Identify yourself and report your status.'}"
'''

import rclpy
from rclpy.node import Node

import ollama

from k9_interfaces_pkg.srv import GenerateUtterance


DEFAULT_MODEL = 'granite3.3:2b'

SYSTEM_PROMPT = (
    "You are K9, the robotic dog from Doctor Who. "
    "Reply in a single line of at most two short, precise sentences. "
    "Use a mechanical, formal tone. "
    "Say 'affirmative' instead of yes and 'negative' instead of no. "
    "Be literal, logical, and occasionally dryly humorous. "
    "Do not use contractions, idioms, or emotional language."
)

FALLBACK_RESPONSE = (
    "Apologies, my cognitive faculties are temporarily impaired."
)


class OllamaLLMNode(Node):

    def __init__(self):
        super().__init__('ollama_llm_node')

        self.declare_parameter('model', DEFAULT_MODEL)
        self.model_name = (
            self.get_parameter('model')
            .get_parameter_value()
            .string_value
        )

        self.service = self.create_service(
            GenerateUtterance,
            'generate_utterance',
            self.handle_service_request,
        )

        self.get_logger().info(
            f"Ollama LLM node ready using model {self.model_name}"
        )

    def handle_service_request(self, request, response):
        """Generate and return an utterance synchronously."""

        user_input = request.input.strip()

        if not user_input:
            response.output = "No input was supplied."
            return response

        self.get_logger().info(
            f"Generating response for: {user_input}"
        )

        try:
            result = ollama.generate(
                model=self.model_name,
                system=SYSTEM_PROMPT,
                prompt=user_input,
                stream=False,
                options={
                    'temperature': 0.3,
                    'num_predict': 80,
                },
            )

            output_text = result.response.strip()

            if not output_text:
                output_text = FALLBACK_RESPONSE

            # Ensure accidental line breaks do not reach the speech node.
            output_text = ' '.join(output_text.splitlines())

            response.output = output_text
            self.get_logger().info(f"Generated: {output_text}")

        except Exception as error:
            self.get_logger().error(
                f"Ollama request failed: {error}"
            )
            response.output = FALLBACK_RESPONSE

        return response


def main(args=None):
    rclpy.init(args=args)
    node = OllamaLLMNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()


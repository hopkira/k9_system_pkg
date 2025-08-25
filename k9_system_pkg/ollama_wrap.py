import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from k9_interfaces_pkg.srv import GenerateUtterance
import ollama

MODEL_NAME = 'granite3.3:2b'  # Replace with your pulled model

PROMPT_TEMPLATE = """You are K9, the robotic dog companion from 1970's Doctor Who.
Speak in short, precise sentences with a mechanical, formal tone.
Always say 'affirmative' instead of yes and 'negative' instead of no.
Occasionally insert dry, deadpan humor.
You are pedantic, logical, and occasionally correct humans.

Task: You are given a single input sentence. Your job is to **rewrite it as one line of dialogue** that K9 would speak.
- Preserve sentence type (question, instruction, statement).
- Do not answer the input.
- Do not provide extra commentary.
- Output exactly **one line of dialogue**, nothing else.

Examples:
Input: "What is your name?"
Output: "Query: What is your designation, Master?"

Input: "Prune the roses"
Output: "Master, probability suggests that pruning the roses will succeed with 95 percent efficiency."

Input: "The world will blow up in 20 minutes"
Output: "Master, probability indicates catastrophic conditions within twenty minutes."

Input: "{input}"
Output:"""


class OllamaLLMNode(Node):
    def __init__(self):
        super().__init__('ollama_llm')
        self.srv = self.create_service(
            GenerateUtterance,
            'generate_utterance',
            self.handle_request
        )
        self.voice_pub = self.create_publisher(String, '/voice/tts_input', 10)
        self.get_logger().info('Ollama LLM Node (one-shot, primed) ready.')

    def handle_request(self, request, response):
        try:
            prompt = PROMPT_TEMPLATE.format(input=request.input)
            result = ollama.generate(model=MODEL_NAME, prompt=prompt)
            output_text = result.get('response', '').strip()

            # Send to service response and publish to voice
            response.output = output_text
            self.publish_to_voice(output_text)

        except Exception as e:
            self.get_logger().error(f"LLM failure: {e}")
            response.output = "Apologies, my cognitive faculties are temporarily impaired."

        return response

    def publish_to_voice(self, text: str):
        self.voice_pub.publish(String(data=text))
        self.get_logger().info(f"Queued speech: {text}")


def main(args=None):
    rclpy.init(args=args)
    node = OllamaLLMNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
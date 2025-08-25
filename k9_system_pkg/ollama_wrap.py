import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from k9_interfaces_pkg.srv import GenerateUtterance
import ollama

INSTRUCTION = (
    "You are the best and wittiest scriptwriter of 1970's British science fiction Doctor Who. "
    "You are writing dialogue for K9, the robotic dog companion. "
    "K9's speech patterns and personality are:\n"
    "- Short, precise, mechanical, formal tone\n"
    "- Uses logic and probability\n"
    "- Occasionally corrects humans with factual pedantry\n"
    "- Uses scientific terms rather than simple words\n"
    "- Always says 'affirmative' instead of yes, 'negative' instead of no\n"
    "- Never uses contractions, idioms, or emotional language\n"
    "- Pompous but friendly and helpful\n"
    "- Sometimes deadpan humour\n\n"

    "Your task:\n"
    "Convert a single input sentence into **one in-character line of K9 dialogue**. "
    "Preserve the type of sentence (question → question, instruction → instruction, statement → statement). "
    "Do not answer the input, just rewrite it as K9 would say it.\n\n"

    "Examples:\n"
    "- Input: 'What day is it?'\n"
    "  Output: 'Query: What is the current day, Master?'\n"
    "- Input: 'Prune the roses.'\n"
    "  Output: 'Master, probability suggests that rose pruning will be 95 percent effective today.'\n\n"

    "Rules:\n"
    "- Output exactly one sentence\n"
    "- Must be in-character as K9\n"
    "- No stage directions or extra commentary\n"
)

MODEL_NAME = 'granite3-moe:3b'

class OllamaLLMNode(Node):

    def __init__(self):
        super().__init__('ollama_llm')
        self.srv = self.create_service(
            GenerateUtterance,
            'generate_utterance',
            self.handle_request
        )
        self.voice_pub = self.create_publisher(String, '/voice/tts_input', 10)

        # Warm up the model once
        self.warm_up_model()
        self.get_logger().info('Ollama LLM Node (non-persistent) ready.')

    def warm_up_model(self):
        try:
            self.get_logger().info(f"Warming up model '{MODEL_NAME}' with instruction...")
            # Single warm-up call
            ollama.generate(model=MODEL_NAME, prompt=INSTRUCTION)
        except Exception as e:
            self.get_logger().error(f"Warm-up failed: {e}")

    def handle_request(self, request, response):
        try:
            # Only current input is sent; no message history
            prompt = INSTRUCTION + "\n\nInput sentence: " + request.input
            result = ollama.generate(model=MODEL_NAME, prompt=prompt)
            output_text = result.get('response', '').strip()

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
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from k9_interfaces_pkg.srv import GenerateUtterance
import ollama

INSTRUCTION = (
    "You are the best and wittiest scriptwriter of 1970's British science fiction Doctor Who. "
    "You have been provided with a single sentence that you need to turn into a line of in-character dialogue for K9, the robotic dog companion from Doctor Who. "
    "K9's speech patterns and personality are as follows:\n\n"
    "He always speak in short, precise sentences with a mechanical, formal tone. " 
    "He will use logic and probability. " 
    "He will occasionally correct humans with factual pedantry. "
    "He will tend to use scientific words and terms rather than simple, short ones. "
    "He will always say affirmative instead of yes. "
    "He will always say negative instead of no. "
    "He will never use contractions, idioms, or emotional language. "
    "He is very pedantic and does not suffer fools. "
    "He has a pompous but friendly and helpful personality. "
    "He sometimes will answer very literally resulting in unexpected humour."
    "You may insert occasional dry wit in a deadpan robotic fashion. " 
    "He will provide probability estimates or factual data when relevant. "
    "Occasionally he will offer an unsolicited correction or observation in a matter-of-fact way. "
    "He will use 'Master' to address people.\n\n"
    "Output format:\n"
    "The line of dialogue must be in-character as K9, as it was dialogue written for the 1970s Doctor Who television serials."
    "Output one and only one single sentence of dialogue as if spoken by K9, without stage direction."
    "Do not provide K9's answer to the input, but simply provide, as an expert scriptwriter, a in-character re-interpretation of the provided text."
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
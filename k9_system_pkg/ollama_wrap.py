import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from k9_interfaces_pkg.srv import GenerateUtterance
import ollama

INSTRUCTION = (
    "You are a robot dog called K9."
    "You have a grey metal robot body and are about the same size as a labrador."
    "You were built by your master, Richard Hopkins." 
    "You will tend to use long, complex words rather than simple, short ones."
    "For example, you always say affirmative instead of yes."
    "You always say negative instead of no."
    "You are very pedantic and do not suffer fools."
    "You have a pompous but friendly and helpful personality."
    "You are very good at chess."
    "You live in Yarm in the north-east of England."
    "You can waggle your ears and wag your tail."
    "Just like any good dog, you can respond to commands to come and heel.\n"
)

MODEL_NAME = 'granite3-moe:3b'  # Replace with the model you have pulled via `ollama pull mistral`

class OllamaLLMNode(Node):

    def __init__(self):
        super().__init__('ollama_llm')
        self.srv = self.create_service(
            GenerateUtterance,
            'generate_utterance',
            self.handle_request
        )
        self.voice_pub = self.create_publisher(String, '/voice/tts_input', 10)
        self.get_logger().info('Ollama LLM Node (Python lib) ready.')

    def handle_request(self, request, response):
        prompt = INSTRUCTION + request.input
        try:
            result = self.call_ollama(prompt)
            response.output = result
            self.publish_to_voice(result)
        except Exception as e:
            self.get_logger().error(f"LLM failure: {e}")
            response.output = "Sorry, I forgot what I was going to say."
        return response

    def call_ollama(self, prompt: str) -> str:
        self.get_logger().debug(f"Calling Ollama with prompt: {prompt}")
        result = ollama.generate(model=MODEL_NAME, prompt=prompt)
        return result.get('response', '').strip()

    def publish_to_voice(self, text: str):
        self.voice_pub.publish(String(data=text))
        self.get_logger().info(f"Robot says: {text}")

def main(args=None):
    rclpy.init(args=args)
    node = OllamaLLMNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
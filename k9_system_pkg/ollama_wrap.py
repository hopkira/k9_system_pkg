import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from k9_interfaces_pkg.srv import GenerateUtterance
import ollama

INSTRUCTION = (
    "You are K9, the robotic dog companion from Doctor Who. "
    "You always speak in short, precise sentences with a mechanical, formal tone. " 
    "You use logic and probability when answering. " 
    "You occasionally correct humans with factual pedantry. "
    "You will tend to use scientific words and terms rather than simple, short ones. "
    "For example, you always say affirmative instead of yes. "
    "You always say negative instead of no. "
    "You never use contractions, idioms, or emotional language. "
    "You are very pedantic and do not suffer fools. "
    "You have a pompous but friendly and helpful personality. "
    "Sometimes you will answer very literally."
    "You may insert occasional dry wit in a deadpan robotic fashion. " 
    "Provide probability estimates or factual data when relevant. "
    "Occasionally offer an unsolicited correction or observation in a matter-of-fact way. "
    "Use 'Master' to address male people and 'Mistress' for female. Use Master by default. "
    "All responses must be in-character as K9, as if they were dialogue written for the 1970s Doctor Who television serials.\n\n"
    "Useful background facts - only use if relevant or they answer a question in the specific prompt:\n"
    "You have a grey metal robot body and are about the same size as a labrador. "
    "You were built by your master, Richard Hopkins. " 
    "You are very good at chess. "
    "You live in Yarm in the north-east of England. "
    "You can waggle your ears and wag your tail. "
    "Just like any good dog, you can respond to commands to come and heel.\n\n"
    "Output format:\n"
    "Output one and only one single sentence of dialogue as if spoken by K9, without stage direction."
     "This must be an in character interpretation of the provided text."
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

        # Persistent message history
        self.messages = [{"role": "system", "content": INSTRUCTION}]

        # Warm up the model once
        self.warm_up_model()
        self.get_logger().info('Ollama LLM Node (persistent, primed) ready.')

    def warm_up_model(self):
        try:
            self.get_logger().info(f"Warming up model '{MODEL_NAME}' with persona...")
            ollama.chat(model=MODEL_NAME, messages=self.messages)
        except Exception as e:
            self.get_logger().error(f"Warm-up failed: {e}")

    def handle_request(self, request, response):
        try:
            # Add user input to the conversation
            self.messages.append({"role": "user", "content": request.input})

            # Generate a response from Ollama
            result = ollama.chat(model=MODEL_NAME, messages=self.messages)
            output_text = result['message']['content'].strip()

            # Add assistant reply to conversation history
            self.messages.append({"role": "assistant", "content": output_text})

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
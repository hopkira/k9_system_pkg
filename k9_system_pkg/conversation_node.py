#!/usr/bin/env python3

import json
import queue
import threading
import urllib.error
import urllib.request

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from std_srvs.srv import Trigger

from k9_interfaces_pkg.msg import IntentResult


class K9ConversationNode(Node):
    """
    General conversation node for K9.

    Receives classified utterances from /intent/result.
    GENERAL_CONVERSATION utterances are sent to Ollama.
    Responses are published on /conversation/response.

    Conversation history is maintained locally so that Ollama can
    continue a short multi-turn conversation.
    """

    def __init__(self):
        super().__init__('k9_conversation')

        # ------------------------------------------------------------------
        # Parameters
        # ------------------------------------------------------------------

        self.declare_parameter('model', 'k9')
        self.declare_parameter(
            'ollama_url',
            'http://127.0.0.1:11434/api/chat'
        )
        self.declare_parameter('accepted_intent', 'GENERAL_CONVERSATION')
        self.declare_parameter('max_history_turns', 8)
        self.declare_parameter('timeout_seconds', 30.0)
        self.declare_parameter('keep_alive', '10m')

        self.model = (
            self.get_parameter('model')
            .get_parameter_value()
            .string_value
        )

        self.ollama_url = (
            self.get_parameter('ollama_url')
            .get_parameter_value()
            .string_value
        )

        self.accepted_intent = (
            self.get_parameter('accepted_intent')
            .get_parameter_value()
            .string_value
        )

        self.max_history_turns = (
            self.get_parameter('max_history_turns')
            .get_parameter_value()
            .integer_value
        )

        self.timeout_seconds = (
            self.get_parameter('timeout_seconds')
            .get_parameter_value()
            .double_value
        )

        self.keep_alive = (
            self.get_parameter('keep_alive')
            .get_parameter_value()
            .string_value
        )

        # ------------------------------------------------------------------
        # ROS interfaces
        # ------------------------------------------------------------------

        self.response_pub = self.create_publisher(
            String,
            '/conversation/response',
            10
        )

        self.intent_sub = self.create_subscription(
            IntentResult,
            '/intent/result',
            self.intent_callback,
            10
        )

        self.reset_srv = self.create_service(
            Trigger,
            '/conversation/reset',
            self.reset_callback
        )

        # ------------------------------------------------------------------
        # Conversation state
        # ------------------------------------------------------------------

        self.history = []
        self.history_lock = threading.Lock()

        # Do not perform the Ollama HTTP request directly in the ROS
        # subscription callback. A worker thread prevents generation from
        # blocking ROS callbacks.
        self.request_queue = queue.Queue(maxsize=5)

        self.stop_event = threading.Event()

        self.worker = threading.Thread(
            target=self.conversation_worker,
            daemon=True
        )

        self.worker.start()

        self.get_logger().info(
            f'K9 conversation node ready '
            f'(model={self.model}, '
            f'intent={self.accepted_intent}, '
            f'history={self.max_history_turns} turns)'
        )

    # ----------------------------------------------------------------------
    # ROS callbacks
    # ----------------------------------------------------------------------

    def intent_callback(self, msg: IntentResult):
        """
        Receive an intent classification.

        Only GENERAL_CONVERSATION intents requiring a response are
        handled by this node.
        """

        if msg.intent != self.accepted_intent:
            return

        if not msg.requires_response:
            return

        text = msg.text.strip()

        if not text:
            self.get_logger().warning(
                'Received GENERAL_CONVERSATION intent with empty text'
            )
            return

        self.get_logger().info(
            f'Conversation request: "{text}"'
        )

        try:
            self.request_queue.put_nowait(text)

        except queue.Full:
            self.get_logger().warning(
                'Conversation request queue full; dropping utterance'
            )

    def reset_callback(self, request, response):
        """Clear the short-term conversation history."""

        del request

        with self.history_lock:
            self.history.clear()

        response.success = True
        response.message = 'Conversation history cleared'

        self.get_logger().info('Conversation history cleared')

        return response

    # ----------------------------------------------------------------------
    # Conversation worker
    # ----------------------------------------------------------------------

    def conversation_worker(self):
        """
        Serial worker for LLM requests.

        Keeping requests serial ensures conversation history remains
        correctly ordered.
        """

        while not self.stop_event.is_set():

            try:
                text = self.request_queue.get(timeout=0.5)

            except queue.Empty:
                continue

            try:
                self.process_utterance(text)

            except Exception as exc:
                self.get_logger().error(
                    f'Unexpected conversation error: {exc}'
                )

            finally:
                self.request_queue.task_done()

    def process_utterance(self, text: str):
        """Send one utterance plus history to Ollama."""

        with self.history_lock:
            messages = list(self.history)

        messages.append({
            'role': 'user',
            'content': text
        })

        payload = {
            'model': self.model,
            'messages': messages,
            'stream': False,
            'think': False,
            'keep_alive': self.keep_alive,
        }

        response_text = self.call_ollama(payload)

        if not response_text:
            self.get_logger().warning(
                'Ollama returned an empty conversation response'
            )
            return

        # Store the completed exchange.
        with self.history_lock:

            self.history.append({
                'role': 'user',
                'content': text
            })

            self.history.append({
                'role': 'assistant',
                'content': response_text
            })

            self.trim_history()

        # Publish for the Behaviour Tree / Voice Executive.
        response_msg = String()
        response_msg.data = response_text

        self.response_pub.publish(response_msg)

        self.get_logger().info(
            f'K9 response: "{response_text}"'
        )

    # ----------------------------------------------------------------------
    # Ollama
    # ----------------------------------------------------------------------

    def call_ollama(self, payload: dict) -> str:
        """Call the local Ollama chat API."""

        data = json.dumps(payload).encode('utf-8')

        request = urllib.request.Request(
            self.ollama_url,
            data=data,
            headers={
                'Content-Type': 'application/json'
            },
            method='POST'
        )

        try:
            with urllib.request.urlopen(
                request,
                timeout=self.timeout_seconds
            ) as response:

                body = response.read().decode('utf-8')

        except urllib.error.HTTPError as exc:

            try:
                body = exc.read().decode('utf-8')
            except Exception:
                body = ''

            self.get_logger().error(
                f'Ollama HTTP error {exc.code}: {body}'
            )

            return ''

        except urllib.error.URLError as exc:

            self.get_logger().error(
                f'Cannot contact Ollama: {exc.reason}'
            )

            return ''

        except TimeoutError:

            self.get_logger().error(
                'Ollama request timed out'
            )

            return ''

        try:
            result = json.loads(body)

        except json.JSONDecodeError as exc:

            self.get_logger().error(
                f'Invalid JSON from Ollama: {exc}'
            )

            return ''

        message = result.get('message', {})

        return message.get('content', '').strip()

    # ----------------------------------------------------------------------
    # History
    # ----------------------------------------------------------------------

    def trim_history(self):
        """
        Keep at most max_history_turns user/assistant exchanges.

        One turn consists of one user message and one assistant message.
        """

        max_messages = self.max_history_turns * 2

        if max_messages <= 0:
            self.history.clear()
            return

        if len(self.history) > max_messages:
            self.history = self.history[-max_messages:]

    # ----------------------------------------------------------------------
    # Shutdown
    # ----------------------------------------------------------------------

    def destroy_node(self):
        self.stop_event.set()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = K9ConversationNode()

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
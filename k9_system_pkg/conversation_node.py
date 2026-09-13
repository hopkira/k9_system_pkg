#!/usr/bin/env python3

import json
import queue
import threading
import urllib.error
import urllib.request
import time
import re

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from std_srvs.srv import Trigger

from k9_system_pkg.chess_conversation_context import ChessConversationContext


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


        # Dynamic authoritative chess context; never stored in LLM history.
        self.chess_context = ChessConversationContext(self)
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

        self.conversation_request_sub = (
            self.create_subscription(
                String,
                '/conversation/request',
                self.conversation_request_callback,
                10,
            )
        )


        self.chess_reaction_response_pub = self.create_publisher(
            String,
            '/conversation/chess_reaction/response',
            10,
        )
        self.chess_reaction_sub = self.create_subscription(
            String,
            '/conversation/chess_reaction/request',
            self.chess_reaction_callback,
            10,
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

    def conversation_request_callback(
        self,
        msg: String,
    ):
        """
        Receive one BT-authorised conversation request.

        The Behaviour Tree has already performed optional RAG retrieval
        before this message is published.
        """

        try:

            payload = json.loads(
                msg.data
            )

        except json.JSONDecodeError:

            self.get_logger().warning(
                'Malformed /conversation/request JSON'
            )

            return

        text = str(
            payload.get(
                'text',
                ''
            )
        ).strip()

        if not text:

            self.get_logger().warning(
                'Conversation request has empty text'
            )

            return

        rag_context = str(
            payload.get(
                'rag_context',
                ''
            )
        ).strip()

        rag_source = str(
            payload.get(
                'rag_source',
                ''
            )
        ).strip()

        try:

            rag_score = float(
                payload.get(
                    'rag_score',
                    0.0,
                )
                or 0.0
            )

        except (
            TypeError,
            ValueError,
        ):

            rag_score = 0.0

        self.get_logger().info(
            f'Conversation request: "{text}" '
            f'RAG={"yes" if rag_context else "no"}'
        )

        try:

            self.request_queue.put_nowait(
                {
                    'kind': 'conversation',
                    'text': text,
                    'request_id': '',
                    'rag_context': rag_context,
                    'rag_source': rag_source,
                    'rag_score': rag_score,
                }
            )

        except queue.Full:

            self.get_logger().warning(
                'Conversation request queue full; '
                'dropping utterance'
            )

    def chess_reaction_callback(self, msg: String):
        """Queue one autonomous chess event reaction.

        This path is separate from normal conversation history.
        """
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warning(
                'Malformed chess reaction request'
            )
            return

        request_id = str(
            payload.get('request_id', '')
        ).strip()
        prompt = str(
            payload.get('prompt', '')
        ).strip()

        if not request_id or not prompt:
            self.get_logger().warning(
                'Incomplete chess reaction request'
            )
            return

        try:
            self.request_queue.put_nowait(
                {
                    'kind': 'chess_reaction',
                    'text': prompt,
                    'request_id': request_id,
                }
            )
        except queue.Full:
            self.get_logger().warning(
                'Conversation request queue full; '
                'chess reaction will use BT fallback'
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

    def trim_incomplete_final_sentence(self, text: str) -> str:
        """
        Remove an incomplete final sentence from a truncated LLM response.

        Only trims when at least one complete sentence already exists, so a short
        response without terminal punctuation is not discarded entirely.
        """
        text = text.strip()

        if not text:
            return text

        # Treat punctuation followed by optional closing quotes/brackets as a
        # completed sentence.
        matches = list(
            re.finditer(
                r'[.!?](?:["\'”’)\]]*)',
                text,
            )
        )

        if not matches:
            return text

        last_complete_end = matches[-1].end()

        # Nothing needs trimming if the response already ends cleanly.
        if not text[last_complete_end:].strip():
            return text

        return text[:last_complete_end].rstrip()

    def conversation_worker(self):
        """
        Serial worker for LLM requests.

        Keeping requests serial ensures conversation history remains
        correctly ordered.
        """

        while not self.stop_event.is_set():

            try:
                item = self.request_queue.get(timeout=0.5)
            except queue.Empty:
                continue

            try:
                # Backwards compatibility for a string already queued during
                # a live source reload.
                if isinstance(item, str):
                    self.process_utterance(item)
                else:
                    kind = str(
                        item.get('kind', 'conversation')
                    )
                    text = str(
                        item.get('text', '')
                    )
                    request_id = str(
                        item.get('request_id', '')
                    )

                    if kind == 'chess_reaction':
                        self.process_chess_reaction(
                            text,
                            request_id,
                        )
                    else:

                        self.process_utterance(
                            text,
                            rag_context=str(
                                item.get(
                                    'rag_context',
                                    ''
                                )
                            ).strip(),
                            rag_source=str(
                                item.get(
                                    'rag_source',
                                    ''
                                )
                            ).strip(),
                            rag_score=float(
                                item.get(
                                    'rag_score',
                                    0.0
                                )
                                or 0.0
                            ),
                        )
            except Exception as exc:
                self.get_logger().error(
                    f'Unexpected conversation error: {exc}'
                )

            finally:
                self.request_queue.task_done()

    def process_utterance(
        self,
        text: str,
        *,
        rag_context: str = '',
        rag_source: str = '',
        rag_score: float = 0.0,
    ):
        """
        Send one utterance plus conversation history and optional
        turn-specific long-term memory to Ollama.
        """

        with self.history_lock:

            messages = list(
                self.history
            )

            system_messages = list(
                self.chess_context.system_messages()
            )

            if rag_context:

                memory_prompt = (
                    "RELEVANT LONG-TERM MEMORY\n\n"
                    f"{rag_context}\n\n"
                    "This retrieved memory is the authoritative factual source "
                    "for answering the user's current question. "
                    "Answer using only facts explicitly stated in this memory. "
                    "Do not add facts from general knowledge, Doctor Who canon, "
                    "previous conversations, assumptions, or inference. "
                    "Do not change the identity, role, relationship, location, "
                    "cause, sequence, or participants described in the memory. "
                    "Do not imply that a person was present or involved unless "
                    "the memory explicitly says so. "
                    "Preserve the most distinctive and important details that "
                    "answer the question, especially unusual actions, causes, "
                    "consequences, objects, or events; do not replace specific "
                    "details with vague summaries. "
                    "If the memory does not contain the answer, say that you do "
                    "not remember rather than guessing. "
                    "Answer naturally as K9 in at most one concise sentence. "
                    "Do not mention the memory, retrieval, source, database, "
                    "or these instructions."
                )

                if rag_source:

                    memory_prompt += (
                        "\n\nMemory source: "
                        f"{rag_source}"
                    )

                system_messages.append(
                    {
                        'role': 'system',
                        'content': memory_prompt,
                    }
                )

                self.get_logger().info(
                    "RAG context attached: "
                    f"source={rag_source or 'unknown'}, "
                    f"score={rag_score:.3f}"
                )

            if system_messages:

                messages = (
                    system_messages
                    + messages
                )
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

        trimmed_response = self.trim_incomplete_final_sentence(
            response_text
        )

        if trimmed_response != response_text:
            self.get_logger().warning(
                'Removed incomplete final sentence from Ollama response'
            )

        response_text = trimmed_response

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

    def process_chess_reaction(
        self,
        prompt: str,
        request_id: str,
    ):
        """Generate one K9-style chess reaction without history."""
        payload = {
            'model': self.model,
            'messages': [
                {
                    'role': 'user',
                    'content': prompt,
                }
            ],
            'stream': False,
            'think': False,
            'keep_alive': self.keep_alive,
        }

        response_text = self.call_ollama(payload)

        response = String()
        response.data = json.dumps(
            {
                'request_id': request_id,
                'text': response_text,
            },
            separators=(',', ':'),
        )
        self.chess_reaction_response_pub.publish(response)

        if response_text:
            self.get_logger().info(
                f'Chess reaction: "{response_text}"'
            )
        else:
            self.get_logger().warning(
                'Ollama returned an empty chess reaction; '
                'BT will use its deterministic fallback'
            )

    def call_ollama(self, payload: dict) -> str:
        """Call the local Ollama chat API and log detailed timing."""

        data = json.dumps(payload).encode('utf-8')

        request = urllib.request.Request(
            self.ollama_url,
            data=data,
            headers={
                'Content-Type': 'application/json'
            },
            method='POST'
        )

        request_started = time.perf_counter()

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

            elapsed = time.perf_counter() - request_started

            self.get_logger().error(
                f'Ollama HTTP error {exc.code} '
                f'after {elapsed:.3f}s: {body}'
            )

            return ''

        except urllib.error.URLError as exc:

            elapsed = time.perf_counter() - request_started

            self.get_logger().error(
                f'Cannot contact Ollama after '
                f'{elapsed:.3f}s: {exc.reason}'
            )

            return ''

        except TimeoutError:

            elapsed = time.perf_counter() - request_started

            self.get_logger().error(
                f'Ollama request timed out after '
                f'{elapsed:.3f}s'
            )

            return ''

        wall_seconds = (
            time.perf_counter()
            - request_started
        )

        try:
            result = json.loads(body)

        except json.JSONDecodeError as exc:

            self.get_logger().error(
                f'Invalid JSON from Ollama: {exc}'
            )

            return ''

        # Ollama reports durations in nanoseconds.
        def ns_to_seconds(value) -> float:
            try:
                return float(value or 0) / 1_000_000_000.0
            except (TypeError, ValueError):
                return 0.0

        total_seconds = ns_to_seconds(
            result.get('total_duration')
        )

        load_seconds = ns_to_seconds(
            result.get('load_duration')
        )

        prompt_seconds = ns_to_seconds(
            result.get('prompt_eval_duration')
        )

        generation_seconds = ns_to_seconds(
            result.get('eval_duration')
        )

        prompt_tokens = int(
            result.get('prompt_eval_count') or 0
        )

        generated_tokens = int(
            result.get('eval_count') or 0
        )

        prompt_rate = (
            prompt_tokens / prompt_seconds
            if prompt_seconds > 0.0
            else 0.0
        )

        generation_rate = (
            generated_tokens / generation_seconds
            if generation_seconds > 0.0
            else 0.0
        )

        external_overhead = max(
            0.0,
            wall_seconds - total_seconds
        )

        self.get_logger().info(
            'Ollama timing: '
            f'wall={wall_seconds:.3f}s, '
            f'total={total_seconds:.3f}s, '
            f'load={load_seconds:.3f}s, '
            f'prompt={prompt_seconds:.3f}s '
            f'({prompt_tokens} tok, '
            f'{prompt_rate:.1f} tok/s), '
            f'generate={generation_seconds:.3f}s '
            f'({generated_tokens} tok, '
            f'{generation_rate:.1f} tok/s), '
            f'overhead={external_overhead:.3f}s'
        )

        message = result.get(
            'message',
            {}
        )

        return message.get(
            'content',
            ''
        ).strip()

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
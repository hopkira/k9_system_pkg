#!/usr/bin/env python3

import json
import re
from dataclasses import dataclass
from threading import Lock
from typing import Dict, Optional, Tuple

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from k9_interfaces_pkg.msg import IntentResult


@dataclass(frozen=True)
class Classification:
    intent: str
    confidence: float
    requires_response: bool
    parameters: Dict[str, object]
    source: str


class K9IntentNode(Node):
    """Classify STT text into K9 executive intents or general conversation."""

    EXECUTIVE_INTENTS = {
        "STOP_LISTENING",
        "PLAY_CHESS",
        "FOLLOW_ME",
        "COME_HERE",
        "STAY",
        "TURN_ABOUT",
        "SHOW_OFF",
        "CHESS_SETUP_ANSWER",
    }

    def __init__(self) -> None:
        super().__init__("k9_intent")

        self.input_topic = self.declare_parameter(
            "input_topic", "/speech_to_text/text"
        ).value
        self.context_topic = self.declare_parameter(
            "context_topic", "/intent/context"
        ).value
        self.result_topic = self.declare_parameter(
            "result_topic", "/intent/result"
        ).value
        self.name_topic = self.declare_parameter(
            "name_topic", "/intent/name"
        ).value
        self.log_classifications = bool(
            self.declare_parameter("log_classifications", True).value
        )

        self._context_lock = Lock()
        self._context: Dict[str, object] = {}

        self._result_pub = self.create_publisher(
            IntentResult, self.result_topic, 10
        )
        self._name_pub = self.create_publisher(
            String, self.name_topic, 10
        )

        self._text_sub = self.create_subscription(
            String, self.input_topic, self._on_text, 10
        )
        self._context_sub = self.create_subscription(
            String, self.context_topic, self._on_context, 10
        )

        # Rules are deliberately ordered from the most specific / safety
        # relevant to the more general commands.
        self._rules = [
            (
                "STOP_LISTENING",
                [
                    r"\bstop listening\b",
                    r"\bstop hearing\b",
                    r"\bpay no attention\b",
                    r"\bclose (?:your )?ears\b",
                    r"\bbe quiet\b",
                    r"\bquiet k9\b",
                    r"\bsilence k9\b",
                    r"\bhush(?: now)?\b",
                    r"\btime to sleep\b",
                    r"\bgo to sleep\b",
                ],
            ),
            (
                "PLAY_CHESS",
                [
                    r"\bplay chess\b",
                    r"\bgame of chess\b",
                    r"\bplay a game\b.*\bchess\b",
                    r"\bchess game\b",
                ],
            ),
            (
                "SHOW_OFF",
                [
                    r"\bshow off\b",
                    r"\bquick demo\b",
                    r"\bdemonstrat(?:e|ion)\b",
                    r"\bshow me (?:a|your) trick",
                    r"\bwhat tricks can you do\b",
                ],
            ),
            (
                "COME_HERE",
                [
                    r"\bcome here\b",
                    r"\bcome to me\b",
                    r"\bcome over here\b",
                    r"\bget over here\b",
                    r"\bmove over here\b",
                    r"\bk9 come\b",
                ],
            ),
            (
                "FOLLOW_ME",
                [
                    r"\bfollow me\b",
                    r"\bfollow\b",
                    r"\bheel\b",
                    r"\bwalk behind me\b",
                    r"\bcome along\b",
                    r"\bwalkies\b",
                    r"\btime for a walk\b",
                    r"\bgo for a walk\b",
                    r"\bcome on\b",
                ],
            ),
            (
                "TURN_ABOUT",
                [
                    r"\bturn around\b",
                    r"\bturnabout\b",
                    r"\bturn about\b",
                    r"\breverse\b",
                    r"\bdouble back\b",
                    r"\bback the way we came\b",
                    r"\b(?:lets|let us) go back\b",
                    r"\bchange direction\b",
                    r"\bu turn\b",
                    r"\byou turn\b",
                ],
            ),
            (
                "STAY",
                [
                    r"\bstay there\b",
                    r"\bstay put\b",
                    r"\bremain there\b",
                    r"\bhold on\b",
                    r"\bhang on\b",
                    r"\bhalt\b",
                    r"\bpause\b",
                ],
            ),
        ]

        self.get_logger().info(
            "K9 intent classifier ready: "
            f"{self.input_topic} -> {self.result_topic}"
        )

    def _on_context(self, msg: String) -> None:
        text = msg.data.strip()
        if not text:
            context: Dict[str, object] = {}
        else:
            try:
                parsed = json.loads(text)
                if not isinstance(parsed, dict):
                    raise ValueError("context JSON must be an object")
                context = parsed
            except (json.JSONDecodeError, ValueError) as exc:
                self.get_logger().warning(
                    f"Ignoring invalid intent context: {exc}"
                )
                return

        with self._context_lock:
            self._context = context

        self.get_logger().debug(
            f"Intent context updated: {json.dumps(context, sort_keys=True)}"
        )

    def _on_text(self, msg: String) -> None:
        text = msg.data.strip()
        if not text:
            return

        with self._context_lock:
            context = dict(self._context)

        result = self.classify(text, context)

        out = IntentResult()
        out.text = text
        out.intent = result.intent
        out.confidence = float(result.confidence)
        out.requires_response = result.requires_response
        out.parameters_json = json.dumps(
            result.parameters, separators=(",", ":"), sort_keys=True
        )
        out.source = result.source
        self._result_pub.publish(out)

        name = String()
        name.data = result.intent
        self._name_pub.publish(name)

        if self.log_classifications:
            self.get_logger().info(
                "Intent: %s confidence=%.2f source=%s response=%s params=%s | %s"
                % (
                    result.intent,
                    result.confidence,
                    result.source,
                    str(result.requires_response).lower(),
                    out.parameters_json,
                    text,
                )
            )

    def classify(
        self, text: str, context: Optional[Dict[str, object]] = None
    ) -> Classification:
        context = context or {}
        normal = self._normalise(text)

        contextual = self._classify_contextual(text, normal, context)
        if contextual is not None:
            return contextual

        command = self._classify_command(normal)
        if command is not None:
            return command

        # Deliberately conservative: anything we do not confidently recognise
        # as an executive command remains conversation. This prevents a vague
        # sentence from unexpectedly moving the robot.
        return Classification(
            intent="GENERAL_CONVERSATION",
            confidence=0.70,
            requires_response=True,
            parameters={},
            source="fallback",
        )

    def _classify_contextual(
        self,
        original: str,
        normal: str,
        context: Dict[str, object],
    ) -> Optional[Classification]:
        chess_state = str(
            context.get("chess_state", context.get("chess/state", ""))
        ).upper()
        setup_step = str(
            context.get(
                "chess_setup_step",
                context.get("chess/setup_step", ""),
            )
        ).upper()

        in_setup = chess_state in {"SETUP", "SETTING_UP"} or bool(setup_step)
        if not in_setup:
            return None

        if setup_step in {
            "WAIT_COLOUR",
            "WAIT_COLOR",
            "COLOUR",
            "COLOR",
        }:
            colour = self._extract_colour(normal)
            if colour is not None:
                return Classification(
                    intent="CHESS_SETUP_ANSWER",
                    confidence=0.99,
                    requires_response=False,
                    parameters={
                        "field": "colour",
                        "colour": colour,
                    },
                    source="context",
                )

            # Do not consume unrelated speech merely because chess happens to
            # be waiting for a colour. The user may still converse with K9.
            return None

        if setup_step in {"WAIT_NAME", "NAME"}:
            name = self._extract_name(original)
            if name is not None:
                return Classification(
                    intent="CHESS_SETUP_ANSWER",
                    confidence=0.95,
                    requires_response=False,
                    parameters={
                        "field": "name",
                        "name": name,
                    },
                    source="context",
                )
            return None

        return None

    def _classify_command(
        self, normal: str
    ) -> Optional[Classification]:
        # Exact short commands are handled separately to avoid matching common
        # words inside conversational sentences such as "why did you stop?".
        exact = {
            "stop": "STAY",
            "stay": "STAY",
            "follow": "FOLLOW_ME",
            "heel": "FOLLOW_ME",
            "reverse": "TURN_ABOUT",
            "quiet": "STOP_LISTENING",
            "silence": "STOP_LISTENING",
        }

        canonical = self._strip_polite_wrapper(normal)

        if canonical in exact:
            return Classification(
                intent=exact[canonical],
                confidence=0.99,
                requires_response=False,
                parameters={},
                source="exact",
            )

        for intent, patterns in self._rules:
            for pattern in patterns:
                if re.search(pattern, canonical):
                    return Classification(
                        intent=intent,
                        confidence=0.97,
                        requires_response=False,
                        parameters={},
                        source="rule",
                    )

        return None

    @staticmethod
    def _normalise(text: str) -> str:
        text = text.lower().replace("’", "'")
        text = re.sub(r"[^a-z0-9'\s-]", " ", text)
        text = text.replace("-", " ")
        text = re.sub(r"\s+", " ", text).strip()
        return text

    @staticmethod
    def _strip_polite_wrapper(text: str) -> str:
        text = re.sub(r"^k9\s+", "", text)
        text = re.sub(
            r"^(?:please\s+)?"
            r"(?:could you|would you|can you|will you|"
            r"would you like to|i want you to|i'd like you to)\s+",
            "",
            text,
        )
        text = re.sub(r"^please\s+", "", text)
        text = re.sub(r"\s+please$", "", text)
        return text.strip()

    @staticmethod
    def _extract_colour(text: str) -> Optional[str]:
        has_white = re.search(r"\bwhite\b", text) is not None
        has_black = re.search(r"\bblack\b", text) is not None

        if has_white and not has_black:
            return "WHITE"
        if has_black and not has_white:
            return "BLACK"
        return None

    @staticmethod
    def _extract_name(text: str) -> Optional[str]:
        cleaned = text.strip(" \t\r\n.,!?")

        patterns = [
            r"(?i)\bmy name is\s+([a-z][a-z' -]{0,40})$",
            r"(?i)\bi am\s+([a-z][a-z' -]{0,40})$",
            r"(?i)\bi'm\s+([a-z][a-z' -]{0,40})$",
            r"(?i)\bcall me\s+([a-z][a-z' -]{0,40})$",
        ]

        for pattern in patterns:
            match = re.search(pattern, cleaned)
            if match:
                return K9IntentNode._title_name(match.group(1))

        # A bare short answer such as "Richard" is sensible when K9 has
        # explicitly just asked for a name. Reject question-like answers.
        if "?" not in text:
            words = re.findall(r"[A-Za-z][A-Za-z'-]*", cleaned)
            if 1 <= len(words) <= 3 and len(cleaned) <= 40:
                return K9IntentNode._title_name(" ".join(words))

        return None

    @staticmethod
    def _title_name(name: str) -> str:
        return " ".join(part.capitalize() for part in name.split())


def main(args=None) -> None:
    rclpy.init(args=args)
    node = K9IntentNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

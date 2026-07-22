#!/usr/bin/env python3
"""Cancellable, priority-aware Piper TTS action server for K9.

Primary interface:
    /voice/speak              k9_interfaces_pkg/action/SpeakText

State/animation topics:
    /voice/is_talking         std_msgs/msg/Bool
    /voice/rms_level          std_msgs/msg/Float32

Compatibility interfaces retained during migration:
    /voice/tts_input          std_msgs/msg/String
    /speak_now                k9_interfaces_pkg/srv/Speak
    /cancel_speech            k9_interfaces_pkg/srv/CancelSpeech

The action server is the preferred interface for behaviour-tree leaves. It
provides completion results, feedback, cancellation and controlled pre-emption.
"""

from __future__ import annotations

import heapq
import threading
from dataclasses import dataclass, field
from enum import Enum, auto
from typing import Hashable, Optional

import numpy as np
import rclpy
import sounddevice as sd
from piper.voice import PiperVoice
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, String

from k9_interfaces_pkg.action import SpeakText
from k9_interfaces_pkg.srv import CancelSpeech, Speak


class JobState(Enum):
    QUEUED = auto()
    ACTIVE = auto()
    DONE = auto()


class JobOutcome(Enum):
    SUCCEEDED = auto()
    CLIENT_CANCELLED = auto()
    PREEMPTED = auto()
    FAILED = auto()


@dataclass
class SpeechJob:
    key: Hashable
    text: str
    owner: str
    priority: int
    interrupt_lower_priority: bool
    clear_lower_priority: bool
    sequence: int
    goal_handle: Optional[ServerGoalHandle] = None
    state: JobState = JobState.QUEUED
    outcome: Optional[JobOutcome] = None
    message: str = ""
    chunks_played: int = 0
    done_event: threading.Event = field(default_factory=threading.Event)
    cancel_event: threading.Event = field(default_factory=threading.Event)
    preempt_event: threading.Event = field(default_factory=threading.Event)


class K9TTSActionNode(Node):
    """Piper TTS server with one speaker, priority queue and cancellation."""

    def __init__(self) -> None:
        super().__init__("k9_tts_node")

        self.declare_parameter(
            "model_path",
            "/home/hopkira/GitHub/k9_piper_voice/k9_2449_model.onnx",
        )
        self.declare_parameter("audio_device", "")
        self.declare_parameter("topic_priority", 20)
        self.declare_parameter("speak_now_priority", 100)
        self.declare_parameter("max_text_length", 4000)
        self.declare_parameter("idle_poll_seconds", 0.1)
        self.declare_parameter("publish_legacy_is_talking", True)

        model_path = str(self.get_parameter("model_path").value)
        self._audio_device = str(self.get_parameter("audio_device").value).strip()
        self._topic_priority = int(self.get_parameter("topic_priority").value)
        self._speak_now_priority = int(
            self.get_parameter("speak_now_priority").value
        )
        self._max_text_length = int(self.get_parameter("max_text_length").value)
        self._idle_poll_seconds = float(
            self.get_parameter("idle_poll_seconds").value
        )
        self._publish_legacy_is_talking = bool(
            self.get_parameter("publish_legacy_is_talking").value
        )

        self.get_logger().info(f"Loading Piper voice: {model_path}")
        self.voice = PiperVoice.load(model_path)

        self.talking_pub = self.create_publisher(
            Bool,
            "/voice/is_talking",
            10,
        )
        self.rms_pub = self.create_publisher(
            Float32,
            "/voice/rms_level",
            10,
        )
        self.legacy_talking_pub = None
        if self._publish_legacy_is_talking:
            self.legacy_talking_pub = self.create_publisher(
                Bool,
                "/is_talking",
                10,
            )

        self.callback_group = ReentrantCallbackGroup()

        self.action_server = ActionServer(
            self,
            SpeakText,
            "/voice/speak",
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            handle_accepted_callback=self.handle_accepted_callback,
            callback_group=self.callback_group,
        )

        # Compatibility interfaces. New BT code should use /voice/speak.
        self.tts_subscription = self.create_subscription(
            String,
            "/voice/tts_input",
            self.tts_callback,
            10,
            callback_group=self.callback_group,
        )
        self.create_service(
            Speak,
            "/speak_now",
            self.speak_now_callback,
            callback_group=self.callback_group,
        )
        self.create_service(
            CancelSpeech,
            "/cancel_speech",
            self.cancel_speech_callback,
            callback_group=self.callback_group,
        )

        self._condition = threading.Condition(threading.RLock())
        self._queue: list[tuple[int, int, Hashable]] = []
        self._jobs: dict[Hashable, SpeechJob] = {}
        self._active_job: Optional[SpeechJob] = None
        self._sequence = 0
        self._shutdown_event = threading.Event()
        self._talking: Optional[bool] = None

        self._publish_rms(0.0)
        self._set_talking(False)

        self._worker = threading.Thread(
            target=self._worker_loop,
            name="k9-tts-worker",
            daemon=True,
        )
        self._worker.start()

        self.get_logger().info(
            "K9 TTS action server ready on /voice/speak; "
            "compatibility topic/services are also enabled"
        )

    # ------------------------------------------------------------------
    # Action server callbacks
    # ------------------------------------------------------------------

    def goal_callback(self, goal_request: SpeakText.Goal) -> GoalResponse:
        text = goal_request.text.strip()
        if not text:
            self.get_logger().warning("Rejecting empty speech goal")
            return GoalResponse.REJECT
        if len(text) > self._max_text_length:
            self.get_logger().warning(
                f"Rejecting speech goal longer than {self._max_text_length} characters"
            )
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def handle_accepted_callback(
        self,
        goal_handle: ServerGoalHandle,
    ) -> None:
        request = goal_handle.request
        key = self._goal_key(goal_handle)

        job = self._new_job(
            key=key,
            text=request.text.strip(),
            owner=request.owner.strip() or "unspecified",
            priority=int(request.priority),
            interrupt_lower_priority=bool(request.interrupt_lower_priority),
            clear_lower_priority=bool(request.clear_lower_priority),
            goal_handle=goal_handle,
        )

        self._enqueue_job(job)

        # This schedules execute_callback in the executor. The callback waits
        # for the dedicated playback worker to complete this particular job.
        goal_handle.execute()

    def execute_callback(
        self,
        goal_handle: ServerGoalHandle,
    ) -> SpeakText.Result:
        key = self._goal_key(goal_handle)

        with self._condition:
            job = self._jobs.get(key)

        result = SpeakText.Result()

        if job is None:
            goal_handle.abort()
            result.success = False
            result.interrupted = False
            result.message = "Internal error: accepted speech job was not found"
            return result

        while rclpy.ok() and not job.done_event.wait(timeout=0.025):
            if goal_handle.is_cancel_requested:
                self._request_client_cancel(job)

        if not job.done_event.is_set():
            self._request_client_cancel(job)
            job.done_event.wait(timeout=1.0)

        outcome = job.outcome or JobOutcome.FAILED

        if outcome is JobOutcome.SUCCEEDED:
            goal_handle.succeed()
            result.success = True
            result.interrupted = False
        elif outcome is JobOutcome.CLIENT_CANCELLED:
            goal_handle.canceled()
            result.success = False
            result.interrupted = True
        else:
            # Server-side pre-emption and playback errors are aborted goals.
            goal_handle.abort()
            result.success = False
            result.interrupted = outcome is JobOutcome.PREEMPTED

        result.message = job.message

        with self._condition:
            self._jobs.pop(key, None)

        return result

    def cancel_callback(
        self,
        goal_handle: ServerGoalHandle,
    ) -> CancelResponse:
        key = self._goal_key(goal_handle)
        with self._condition:
            job = self._jobs.get(key)
            if job is None or job.state is JobState.DONE:
                return CancelResponse.REJECT

            job.cancel_event.set()

            if job.state is JobState.QUEUED:
                self._finish_job_locked(
                    job,
                    JobOutcome.CLIENT_CANCELLED,
                    "Speech goal cancelled before playback",
                )

            self._condition.notify_all()

        return CancelResponse.ACCEPT

    # ------------------------------------------------------------------
    # Compatibility callbacks
    # ------------------------------------------------------------------

    def tts_callback(self, msg: String) -> None:
        text = msg.data.strip()
        if not text:
            self.get_logger().warning("Ignoring empty /voice/tts_input message")
            return

        job = self._new_job(
            key=self._next_legacy_key("topic"),
            text=text,
            owner="legacy_topic",
            priority=self._topic_priority,
            interrupt_lower_priority=False,
            clear_lower_priority=False,
        )
        self._enqueue_job(job)

    def speak_now_callback(self, request: Speak.Request, response: Speak.Response):
        text = request.text.strip()
        if not text:
            response.success = False
            response.message = "Empty text"
            return response

        job = self._new_job(
            key=self._next_legacy_key("service"),
            text=text,
            owner="legacy_speak_now",
            priority=self._speak_now_priority,
            interrupt_lower_priority=True,
            clear_lower_priority=True,
        )
        self._enqueue_job(job)

        response.success = True
        response.message = "Speech accepted; latest high-priority request wins"
        return response

    def cancel_speech_callback(
        self,
        request: CancelSpeech.Request,
        response: CancelSpeech.Response,
    ):
        del request
        cancelled = self.cancel_all_speech("Cancelled through /cancel_speech")
        response.success = True
        response.message = f"Cancellation requested for {cancelled} speech job(s)"
        return response

    # ------------------------------------------------------------------
    # Queue and pre-emption
    # ------------------------------------------------------------------

    def _new_job(
        self,
        *,
        key: Hashable,
        text: str,
        owner: str,
        priority: int,
        interrupt_lower_priority: bool,
        clear_lower_priority: bool,
        goal_handle: Optional[ServerGoalHandle] = None,
    ) -> SpeechJob:
        with self._condition:
            self._sequence += 1
            sequence = self._sequence

        return SpeechJob(
            key=key,
            text=text,
            owner=owner,
            priority=max(0, min(255, int(priority))),
            interrupt_lower_priority=interrupt_lower_priority,
            clear_lower_priority=clear_lower_priority,
            sequence=sequence,
            goal_handle=goal_handle,
        )

    def _enqueue_job(self, job: SpeechJob) -> None:
        with self._condition:
            if job.clear_lower_priority:
                for queued_job in list(self._jobs.values()):
                    if (
                        queued_job.state is JobState.QUEUED
                        and queued_job.priority <= job.priority
                    ):
                        self._finish_job_locked(
                            queued_job,
                            JobOutcome.PREEMPTED,
                            f"Replaced by {job.owner} speech request",
                        )

            active = self._active_job
            if (
                active is not None
                and job.interrupt_lower_priority
                and job.priority >= active.priority
            ):
                active.preempt_event.set()
                self.get_logger().info(
                    f"Pre-empting speech owned by {active.owner} "
                    f"with priority {active.priority}; new owner={job.owner}, "
                    f"priority={job.priority}"
                )

            self._jobs[job.key] = job
            heapq.heappush(
                self._queue,
                (-job.priority, job.sequence, job.key),
            )
            self._condition.notify_all()

        self.get_logger().info(
            f"Queued speech owner={job.owner!r}, priority={job.priority}: "
            f"{job.text!r}"
        )

    def _request_client_cancel(self, job: SpeechJob) -> None:
        with self._condition:
            if job.state is JobState.DONE:
                return
            job.cancel_event.set()
            if job.state is JobState.QUEUED:
                self._finish_job_locked(
                    job,
                    JobOutcome.CLIENT_CANCELLED,
                    "Speech goal cancelled before playback",
                )
            self._condition.notify_all()

    def cancel_all_speech(self, reason: str) -> int:
        with self._condition:
            affected = 0
            for job in list(self._jobs.values()):
                if job.state is JobState.DONE:
                    continue
                affected += 1
                if job.state is JobState.ACTIVE:
                    job.cancel_event.set()
                else:
                    self._finish_job_locked(
                        job,
                        JobOutcome.CLIENT_CANCELLED,
                        reason,
                    )
            self._condition.notify_all()
            return affected

    def _finish_job_locked(
        self,
        job: SpeechJob,
        outcome: JobOutcome,
        message: str,
    ) -> None:
        if job.state is JobState.DONE:
            return
        job.outcome = outcome
        job.message = message
        job.state = JobState.DONE
        job.done_event.set()

        # Legacy jobs have no execute callback to remove them.
        if job.goal_handle is None:
            self._jobs.pop(job.key, None)

    def _pop_next_job_locked(self) -> Optional[SpeechJob]:
        while self._queue:
            _, _, key = heapq.heappop(self._queue)
            job = self._jobs.get(key)
            if job is None:
                continue
            if job.state is not JobState.QUEUED or job.done_event.is_set():
                continue
            return job
        return None

    def _queue_has_runnable_job_locked(self) -> bool:
        return any(
            job.state is JobState.QUEUED and not job.done_event.is_set()
            for job in self._jobs.values()
        )

    # ------------------------------------------------------------------
    # Playback worker
    # ------------------------------------------------------------------

    def _worker_loop(self) -> None:
        while rclpy.ok() and not self._shutdown_event.is_set():
            with self._condition:
                job = self._pop_next_job_locked()
                while job is None and not self._shutdown_event.is_set():
                    # No playable work: this is the authoritative transition
                    # to idle. During pre-emption, the replacement is already
                    # queued, so talking remains true between utterances.
                    self._publish_rms(0.0)
                    self._set_talking(False)
                    self._condition.wait(timeout=self._idle_poll_seconds)
                    job = self._pop_next_job_locked()

                if self._shutdown_event.is_set():
                    break

                job.state = JobState.ACTIVE
                self._active_job = job

            self._set_talking(True)
            self._publish_feedback(job, "starting", 0.0)

            outcome, message = self._play_job(job)

            with self._condition:
                self._active_job = None
                self._finish_job_locked(job, outcome, message)
                more_work = self._queue_has_runnable_job_locked()
                self._condition.notify_all()

            self._publish_rms(0.0)
            if not more_work:
                self._set_talking(False)

        self._publish_rms(0.0)
        self._set_talking(False)

    def _play_job(self, job: SpeechJob) -> tuple[JobOutcome, str]:
        stream: Optional[sd.OutputStream] = None
        aborted = False

        try:
            stream_kwargs = {
                "samplerate": self.voice.config.sample_rate,
                "channels": 1,
                "dtype": "int16",
            }
            if self._audio_device:
                stream_kwargs["device"] = self._audio_device

            stream = sd.OutputStream(**stream_kwargs)
            stream.start()

            for audio_bytes in self.voice.synthesize_stream_raw(job.text):
                if job.cancel_event.is_set():
                    aborted = True
                    stream.abort()
                    return (
                        JobOutcome.CLIENT_CANCELLED,
                        "Speech cancelled",
                    )

                if job.preempt_event.is_set():
                    aborted = True
                    stream.abort()
                    return (
                        JobOutcome.PREEMPTED,
                        "Speech pre-empted by a newer request",
                    )

                int_data = np.frombuffer(audio_bytes, dtype=np.int16)
                if int_data.size == 0:
                    continue

                # Use float64 for the square to avoid integer overflow.
                samples = int_data.astype(np.float64)
                rms = float(np.sqrt(np.mean(samples * samples)) / 32768.0)
                rms = max(0.0, min(1.0, rms))

                self._publish_rms(rms)
                job.chunks_played += 1
                self._publish_feedback(job, "speaking", rms)

                # This write blocks only for the current Piper chunk. The
                # cancellation/pre-emption checks run before every chunk.
                stream.write(int_data)

            stream.stop()
            self._publish_feedback(job, "completed", 0.0)
            return JobOutcome.SUCCEEDED, "Speech completed"

        except Exception as error:  # noqa: BLE001 - preserve playback details
            self.get_logger().error(f"TTS playback failed: {error}")
            if stream is not None and not aborted:
                try:
                    stream.abort()
                except Exception:
                    pass
            return JobOutcome.FAILED, f"TTS playback failed: {error}"

        finally:
            if stream is not None:
                try:
                    stream.close()
                except Exception as error:  # noqa: BLE001
                    self.get_logger().warning(
                        f"Failed to close audio stream cleanly: {error}"
                    )

    # ------------------------------------------------------------------
    # Publications and utility methods
    # ------------------------------------------------------------------

    def _publish_feedback(
        self,
        job: SpeechJob,
        state: str,
        rms: float,
    ) -> None:
        if job.goal_handle is None:
            return
        if job.done_event.is_set():
            return

        feedback = SpeakText.Feedback()
        feedback.state = state
        feedback.rms_level = float(rms)
        feedback.chunks_played = int(job.chunks_played)

        try:
            job.goal_handle.publish_feedback(feedback)
        except Exception as error:  # noqa: BLE001
            self.get_logger().debug(f"Unable to publish speech feedback: {error}")

    def _publish_rms(self, value: float) -> None:
        self.rms_pub.publish(Float32(data=float(value)))

    def _set_talking(self, value: bool) -> None:
        if self._talking == value:
            return
        self._talking = value
        message = Bool(data=value)
        self.talking_pub.publish(message)
        if self.legacy_talking_pub is not None:
            self.legacy_talking_pub.publish(message)
        self.get_logger().debug(f"Talking: {value}")

    @staticmethod
    def _goal_key(goal_handle: ServerGoalHandle) -> bytes:
        return bytes(goal_handle.goal_id.uuid)

    def _next_legacy_key(self, source: str) -> str:
        with self._condition:
            self._sequence += 1
            return f"legacy:{source}:{self._sequence}"

    def destroy_node(self) -> bool:
        self._shutdown_event.set()
        self.cancel_all_speech("Node shutting down")
        with self._condition:
            self._condition.notify_all()

        if self._worker.is_alive():
            self._worker.join(timeout=2.0)

        self.action_server.destroy()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = K9TTSActionNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

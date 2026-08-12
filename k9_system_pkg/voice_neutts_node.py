#!/usr/bin/env python3
"""K9 NeuTTS-Air streaming voice node.

Primary ROS interface:
    /voice/speak              k9_interfaces_pkg/action/SpeakText

State/animation topics:
    /voice/is_talking         std_msgs/msg/Bool
    /voice/rms_level          std_msgs/msg/Float32

Compatibility interfaces:
    /voice/tts_input          std_msgs/msg/String
    /speak_now                k9_interfaces_pkg/srv/Speak
    /cancel_speech            k9_interfaces_pkg/srv/CancelSpeech

Voice selection:
    voice_id: 0..999

For voice_id N, the node loads these files from voice_dir:
    NNN.txt
    k9_NNN.pt

where NNN is zero-padded to three digits.

NeuTTS is kept resident for the lifetime of the node. Audio is synthesised in
streaming chunks and queued to one persistent PipeWire pw-cat process per
utterance. Synthesis and playback therefore overlap.
"""

from __future__ import annotations

import heapq
import os
import queue
import subprocess
import threading
import time
from dataclasses import dataclass, field
from enum import Enum, auto
from pathlib import Path
from typing import Any, Hashable, Optional

import numpy as np
import rclpy
import torch
from neutts import NeuTTS
from rcl_interfaces.msg import SetParametersResult
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
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


@dataclass
class VoiceReference:
    voice_id: int
    codes_path: Path
    text_path: Path
    codes_mtime_ns: int
    text_mtime_ns: int
    codes: Any
    text: str


class K9NeuTTSVoiceNode(Node):
    """Priority-aware, cancellable NeuTTS-Air streaming voice server."""

    def __init__(self) -> None:
        super().__init__("k9_tts_node")

        # ------------------------------------------------------------------
        # Parameters
        # ------------------------------------------------------------------

        self.declare_parameter("voice_id", 0)
        self.declare_parameter("voice_dir", "~/tts_env/neutts/samples")

        self.declare_parameter(
            "backbone_repo",
            "neuphonic/neutts-air-q4-gguf",
        )
        self.declare_parameter("backbone_device", "cuda")
        self.declare_parameter(
            "codec_repo",
            "neuphonic/neucodec-onnx-decoder-int8",
        )
        self.declare_parameter("codec_device", "cpu")

        # Empty means use the model's own default language mapping.
        self.declare_parameter("language", "")
        # Negative means choose a fresh random seed for each utterance.
        self.declare_parameter("seed", -1)
        self.declare_parameter("temperature", 1.0)
        self.declare_parameter("top_k", 50)

        # 25 frames = about 500 ms with NeuTTS' 20 ms hop.
        self.declare_parameter("streaming_frames_per_chunk", 25)

        # Quiet startup warm-up eliminates most first-use CUDA graph overhead.
        self.declare_parameter("warmup_enabled", True)
        self.declare_parameter(
            "warmup_text",
            "Affirmative. Systems are functioning normally.",
        )

        # PipeWire output. Empty target means automatic default sink selection.
        self.declare_parameter("pipewire_executable", "pw-cat")
        self.declare_parameter("pipewire_target", "")
        self.declare_parameter("pipewire_latency_ms", 50)
        self.declare_parameter("pipewire_volume", 1.0)
        self.declare_parameter("tail_padding_ms", 250)
        self.declare_parameter("playback_queue_chunks", 4)

        # Queue/action compatibility parameters retained from the Piper node.
        self.declare_parameter("topic_priority", 20)
        self.declare_parameter("speak_now_priority", 100)
        self.declare_parameter("max_text_length", 4000)
        self.declare_parameter("idle_poll_seconds", 0.1)
        self.declare_parameter("publish_legacy_is_talking", True)

        self._voice_dir = self._expand_path(
            str(self.get_parameter("voice_dir").value)
        )
        self._backbone_repo = str(
            self.get_parameter("backbone_repo").value
        )
        self._backbone_device = str(
            self.get_parameter("backbone_device").value
        )
        self._codec_repo = str(self.get_parameter("codec_repo").value)
        self._codec_device = str(self.get_parameter("codec_device").value)
        self._language = str(self.get_parameter("language").value).strip()

        seed_value = int(self.get_parameter("seed").value)
        self._seed: Optional[int] = None if seed_value < 0 else seed_value

        self._temperature = float(self.get_parameter("temperature").value)
        self._top_k = int(self.get_parameter("top_k").value)
        self._streaming_frames_per_chunk = int(
            self.get_parameter("streaming_frames_per_chunk").value
        )

        self._warmup_enabled = bool(
            self.get_parameter("warmup_enabled").value
        )
        self._warmup_text = str(
            self.get_parameter("warmup_text").value
        ).strip()

        self._pipewire_executable = str(
            self.get_parameter("pipewire_executable").value
        )
        self._pipewire_target = str(
            self.get_parameter("pipewire_target").value
        ).strip()
        self._pipewire_latency_ms = int(
            self.get_parameter("pipewire_latency_ms").value
        )
        self._pipewire_volume = float(
            self.get_parameter("pipewire_volume").value
        )
        self._tail_padding_ms = int(
            self.get_parameter("tail_padding_ms").value
        )
        self._playback_queue_chunks = int(
            self.get_parameter("playback_queue_chunks").value
        )

        self._topic_priority = int(
            self.get_parameter("topic_priority").value
        )
        self._speak_now_priority = int(
            self.get_parameter("speak_now_priority").value
        )
        self._max_text_length = int(
            self.get_parameter("max_text_length").value
        )
        self._idle_poll_seconds = float(
            self.get_parameter("idle_poll_seconds").value
        )
        self._publish_legacy_is_talking = bool(
            self.get_parameter("publish_legacy_is_talking").value
        )

        self._validate_static_parameters()

        initial_voice_id = int(self.get_parameter("voice_id").value)
        self._validate_voice_id(initial_voice_id, require_files=True)

        # ------------------------------------------------------------------
        # Internal state
        # ------------------------------------------------------------------

        self._condition = threading.Condition(threading.RLock())
        self._queue: list[tuple[int, int, Hashable]] = []
        self._jobs: dict[Hashable, SpeechJob] = {}
        self._active_job: Optional[SpeechJob] = None
        self._sequence = 0
        self._legacy_sequence = 0
        self._shutdown_event = threading.Event()
        self._talking: Optional[bool] = None

        self._voice_lock = threading.RLock()
        self._voice_cache: dict[int, VoiceReference] = {}

        # The player is exposed to action/service callbacks only so an active
        # utterance can be stopped immediately on cancel/pre-emption.
        self._player_lock = threading.RLock()
        self._active_player: Optional[subprocess.Popen] = None
        self._active_player_job_key: Optional[Hashable] = None

        # ------------------------------------------------------------------
        # Load the resident NeuTTS model
        # ------------------------------------------------------------------

        self.get_logger().info(
            f"Loading NeuTTS backbone={self._backbone_repo} "
            f"on {self._backbone_device}; codec={self._codec_repo} "
            f"on {self._codec_device}"
        )

        tts_kwargs: dict[str, Any] = {
            "backbone_repo": self._backbone_repo,
            "backbone_device": self._backbone_device,
            "codec_repo": self._codec_repo,
            "codec_device": self._codec_device,
            "seed": self._seed,
        }
        if self._language:
            tts_kwargs["language"] = self._language

        self.tts = NeuTTS(**tts_kwargs)

        # NeuTTS derives stride from frames_per_chunk during construction, so
        # keep both values in sync if the ROS parameter overrides the default.
        self.tts.streaming_frames_per_chunk = (
            self._streaming_frames_per_chunk
        )
        self.tts.streaming_stride_samples = (
            self.tts.streaming_frames_per_chunk * self.tts.hop_length
        )
        self._sample_rate = int(self.tts.sample_rate)

        # Load the initial voice now so configuration errors fail at startup.
        self._load_voice_reference(initial_voice_id)

        if self._warmup_enabled and self._warmup_text:
            self._warm_up(initial_voice_id)

        # ------------------------------------------------------------------
        # ROS interfaces
        # ------------------------------------------------------------------

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

        self.add_on_set_parameters_callback(self._parameter_callback)

        self._publish_rms(0.0)
        self._set_talking(False)

        self._worker = threading.Thread(
            target=self._worker_loop,
            name="k9-neutts-worker",
            daemon=True,
        )
        self._worker.start()

        self.get_logger().info(
            f"K9 NeuTTS voice ready: voice_id={initial_voice_id:03d}, "
            f"sample_rate={self._sample_rate} Hz, action=/voice/speak"
        )

    # ------------------------------------------------------------------
    # Parameter and voice handling
    # ------------------------------------------------------------------

    @staticmethod
    def _expand_path(value: str) -> Path:
        return Path(
            os.path.expandvars(os.path.expanduser(value))
        ).resolve()

    def _validate_static_parameters(self) -> None:
        if self._streaming_frames_per_chunk <= 0:
            raise ValueError("streaming_frames_per_chunk must be > 0")
        if self._top_k <= 0:
            raise ValueError("top_k must be > 0")
        if self._temperature <= 0:
            raise ValueError("temperature must be > 0")
        if self._pipewire_latency_ms < 0:
            raise ValueError("pipewire_latency_ms must be >= 0")
        if self._pipewire_volume < 0:
            raise ValueError("pipewire_volume must be >= 0")
        if self._tail_padding_ms < 0:
            raise ValueError("tail_padding_ms must be >= 0")
        if self._playback_queue_chunks < 1:
            raise ValueError("playback_queue_chunks must be >= 1")

    def _voice_paths(self, voice_id: int) -> tuple[Path, Path]:
        number = f"{voice_id:03d}"
        codes_path = self._voice_dir / f"k9_{number}.pt"
        text_path = self._voice_dir / f"{number}.txt"
        return codes_path, text_path

    def _validate_voice_id(
        self,
        voice_id: int,
        *,
        require_files: bool,
    ) -> None:
        if not 0 <= voice_id <= 999:
            raise ValueError("voice_id must be in the range 0..999")

        if require_files:
            codes_path, text_path = self._voice_paths(voice_id)
            missing = [
                str(path)
                for path in (codes_path, text_path)
                if not path.is_file()
            ]
            if missing:
                raise FileNotFoundError(
                    "Missing voice reference file(s): "
                    + ", ".join(missing)
                )

    def _load_voice_reference(self, voice_id: int) -> VoiceReference:
        self._validate_voice_id(voice_id, require_files=True)
        codes_path, text_path = self._voice_paths(voice_id)

        codes_stat = codes_path.stat()
        text_stat = text_path.stat()

        with self._voice_lock:
            cached = self._voice_cache.get(voice_id)
            if (
                cached is not None
                and cached.codes_mtime_ns == codes_stat.st_mtime_ns
                and cached.text_mtime_ns == text_stat.st_mtime_ns
            ):
                return cached

            self.get_logger().info(
                f"Loading voice {voice_id:03d}: "
                f"{codes_path} + {text_path}"
            )

            codes = torch.load(
                str(codes_path),
                map_location="cpu",
            )
            text = text_path.read_text(encoding="utf-8").strip()

            if not text:
                raise ValueError(
                    f"Voice transcript is empty: {text_path}"
                )

            reference = VoiceReference(
                voice_id=voice_id,
                codes_path=codes_path,
                text_path=text_path,
                codes_mtime_ns=codes_stat.st_mtime_ns,
                text_mtime_ns=text_stat.st_mtime_ns,
                codes=codes,
                text=text,
            )
            self._voice_cache[voice_id] = reference
            return reference

    def _parameter_callback(self, params) -> SetParametersResult:
        # Only voice_id is intentionally mutable at runtime. Model/backend
        # changes require a restart because NeuTTS is resident on the GPU.
        static_names = {
            "voice_dir",
            "backbone_repo",
            "backbone_device",
            "codec_repo",
            "codec_device",
            "language",
            "seed",
            "temperature",
            "top_k",
            "streaming_frames_per_chunk",
            "warmup_enabled",
            "warmup_text",
            "pipewire_executable",
            "pipewire_target",
            "pipewire_latency_ms",
            "pipewire_volume",
            "tail_padding_ms",
            "playback_queue_chunks",
            "topic_priority",
            "speak_now_priority",
            "max_text_length",
            "idle_poll_seconds",
            "publish_legacy_is_talking",
        }

        try:
            for param in params:
                if param.name == "voice_id":
                    if param.type_ != Parameter.Type.INTEGER:
                        return SetParametersResult(
                            successful=False,
                            reason="voice_id must be an integer 0..999",
                        )
                    self._validate_voice_id(
                        int(param.value),
                        require_files=True,
                    )
                elif param.name in static_names:
                    return SetParametersResult(
                        successful=False,
                        reason=(
                            f"{param.name} is startup-only; restart the "
                            "voice node to change it"
                        ),
                    )
        except Exception as error:
            return SetParametersResult(
                successful=False,
                reason=str(error),
            )

        return SetParametersResult(successful=True)

    def _warm_up(self, voice_id: int) -> None:
        reference = self._load_voice_reference(voice_id)
        self.get_logger().info("Warming NeuTTS without playback...")
        started = time.perf_counter()
        chunks = 0

        for _ in self.tts.infer_stream(
            self._warmup_text,
            reference.codes,
            reference.text,
            temperature=self._temperature,
            top_k=self._top_k,
        ):
            chunks += 1

        elapsed = time.perf_counter() - started
        self.get_logger().info(
            f"NeuTTS warm-up complete in {elapsed:.2f}s "
            f"({chunks} chunks)"
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
                f"Rejecting speech goal longer than "
                f"{self._max_text_length} characters"
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
            interrupt_lower_priority=bool(
                request.interrupt_lower_priority
            ),
            clear_lower_priority=bool(request.clear_lower_priority),
            goal_handle=goal_handle,
        )

        self._enqueue_job(job)
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
            result.message = (
                "Internal error: accepted speech job was not found"
            )
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
        active_to_stop: Optional[SpeechJob] = None

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
            elif job.state is JobState.ACTIVE:
                active_to_stop = job

            self._condition.notify_all()

        if active_to_stop is not None:
            self._terminate_player_for_job(active_to_stop)

        return CancelResponse.ACCEPT

    # ------------------------------------------------------------------
    # Compatibility callbacks
    # ------------------------------------------------------------------

    def tts_callback(self, msg: String) -> None:
        text = msg.data.strip()
        if not text:
            self.get_logger().warning(
                "Ignoring empty /voice/tts_input message"
            )
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

    def speak_now_callback(
        self,
        request: Speak.Request,
        response: Speak.Response,
    ):
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
        response.message = (
            "Speech accepted; latest high-priority request wins"
        )
        return response

    def cancel_speech_callback(
        self,
        request: CancelSpeech.Request,
        response: CancelSpeech.Response,
    ):
        del request
        cancelled = self.cancel_all_speech(
            "Cancelled through /cancel_speech"
        )
        response.success = True
        response.message = (
            f"Cancellation requested for {cancelled} speech job(s)"
        )
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
        active_to_stop: Optional[SpeechJob] = None

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
                active_to_stop = active
                self.get_logger().info(
                    f"Pre-empting speech owner={active.owner} "
                    f"priority={active.priority} with owner={job.owner} "
                    f"priority={job.priority}"
                )

            self._jobs[job.key] = job
            heapq.heappush(
                self._queue,
                (-job.priority, job.sequence, job.key),
            )
            self._condition.notify_all()

        if active_to_stop is not None:
            self._terminate_player_for_job(active_to_stop)

        self.get_logger().info(
            f"Queued speech owner={job.owner!r} "
            f"priority={job.priority}: {job.text!r}"
        )

    def _request_client_cancel(self, job: SpeechJob) -> None:
        active = False

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
            elif job.state is JobState.ACTIVE:
                active = True
            self._condition.notify_all()

        if active:
            self._terminate_player_for_job(job)

    def cancel_all_speech(self, reason: str) -> int:
        active_to_stop: list[SpeechJob] = []

        with self._condition:
            affected = 0
            for job in list(self._jobs.values()):
                if job.state is JobState.DONE:
                    continue

                affected += 1
                if job.state is JobState.ACTIVE:
                    job.cancel_event.set()
                    active_to_stop.append(job)
                else:
                    self._finish_job_locked(
                        job,
                        JobOutcome.CLIENT_CANCELLED,
                        reason,
                    )

            self._condition.notify_all()

        for job in active_to_stop:
            self._terminate_player_for_job(job)

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

        # Legacy jobs have no action execute callback to remove them later.
        if job.goal_handle is None:
            self._jobs.pop(job.key, None)

    def _pop_next_job_locked(self) -> Optional[SpeechJob]:
        while self._queue:
            _, _, key = heapq.heappop(self._queue)
            job = self._jobs.get(key)
            if job is None:
                continue
            if job.state is not JobState.QUEUED:
                continue
            if job.done_event.is_set():
                continue
            return job
        return None

    def _queue_has_runnable_job_locked(self) -> bool:
        return any(
            job.state is JobState.QUEUED
            and not job.done_event.is_set()
            for job in self._jobs.values()
        )

    # ------------------------------------------------------------------
    # Playback worker
    # ------------------------------------------------------------------

    def _worker_loop(self) -> None:
        while rclpy.ok() and not self._shutdown_event.is_set():
            with self._condition:
                job = self._pop_next_job_locked()

                while (
                    job is None
                    and not self._shutdown_event.is_set()
                ):
                    self._publish_rms(0.0)
                    self._set_talking(False)
                    self._condition.wait(
                        timeout=self._idle_poll_seconds
                    )
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

    def _current_voice_id(self) -> int:
        return int(self.get_parameter("voice_id").value)

    def _pipewire_command(self) -> list[str]:
        command = [
            self._pipewire_executable,
            "--playback",
            "--raw",
            f"--rate={self._sample_rate}",
            "--channels=1",
            "--channel-map=mono",
            "--format=s16",
            f"--latency={self._pipewire_latency_ms}ms",
            f"--volume={self._pipewire_volume}",
            "--media-role=Communication",
        ]

        if self._pipewire_target:
            command.append(f"--target={self._pipewire_target}")

        command.append("-")
        return command

    def _set_active_player(
        self,
        job: SpeechJob,
        process: subprocess.Popen,
    ) -> None:
        with self._player_lock:
            self._active_player = process
            self._active_player_job_key = job.key

    def _clear_active_player(
        self,
        job: SpeechJob,
        process: subprocess.Popen,
    ) -> None:
        with self._player_lock:
            if (
                self._active_player is process
                and self._active_player_job_key == job.key
            ):
                self._active_player = None
                self._active_player_job_key = None

    def _terminate_player_for_job(self, job: SpeechJob) -> None:
        process: Optional[subprocess.Popen] = None

        with self._player_lock:
            if self._active_player_job_key == job.key:
                process = self._active_player

        if process is not None and process.poll() is None:
            try:
                process.terminate()
            except ProcessLookupError:
                pass
            except Exception as error:  # noqa: BLE001
                self.get_logger().debug(
                    f"Unable to terminate PipeWire player: {error}"
                )

    @staticmethod
    def _job_interrupt_outcome(
        job: SpeechJob,
    ) -> Optional[tuple[JobOutcome, str]]:
        if job.cancel_event.is_set():
            return (
                JobOutcome.CLIENT_CANCELLED,
                "Speech cancelled",
            )
        if job.preempt_event.is_set():
            return (
                JobOutcome.PREEMPTED,
                "Speech pre-empted by a newer request",
            )
        return None

    def _queue_audio_with_interrupts(
        self,
        audio_queue: queue.Queue,
        item,
        job: SpeechJob,
        stop_event: threading.Event,
    ) -> bool:
        while True:
            if self._job_interrupt_outcome(job) is not None:
                return False
            if stop_event.is_set():
                return False
            try:
                audio_queue.put(item, timeout=0.05)
                return True
            except queue.Full:
                continue

    def _play_job(self, job: SpeechJob) -> tuple[JobOutcome, str]:
        process: Optional[subprocess.Popen] = None
        feeder: Optional[threading.Thread] = None
        feeder_stop = threading.Event()
        feeder_errors: list[BaseException] = []

        try:
            voice_id = self._current_voice_id()
            reference = self._load_voice_reference(voice_id)

            command = self._pipewire_command()
            self.get_logger().debug(
                f"Starting PipeWire player: {' '.join(command)}"
            )

            process = subprocess.Popen(
                command,
                stdin=subprocess.PIPE,
                stdout=subprocess.DEVNULL,
                stderr=None,
                bufsize=0,
            )

            if process.stdin is None:
                raise RuntimeError(
                    "pw-cat started without a writable stdin"
                )

            self._set_active_player(job, process)

            audio_queue: queue.Queue = queue.Queue(
                maxsize=self._playback_queue_chunks
            )

            def feed_pipewire() -> None:
                try:
                    while not feeder_stop.is_set():
                        item = audio_queue.get()
                        try:
                            if item is None:
                                break

                            pcm, rms, publish_feedback = item

                            interrupted = self._job_interrupt_outcome(job)
                            if interrupted is not None:
                                break

                            process.stdin.write(pcm.tobytes())

                            if publish_feedback:
                                self._publish_rms(rms)
                                job.chunks_played += 1
                                self._publish_feedback(
                                    job,
                                    "speaking",
                                    rms,
                                )
                        finally:
                            audio_queue.task_done()
                except (BrokenPipeError, OSError) as error:
                    if self._job_interrupt_outcome(job) is None:
                        feeder_errors.append(error)
                    feeder_stop.set()
                except BaseException as error:  # noqa: BLE001
                    feeder_errors.append(error)
                    feeder_stop.set()
                finally:
                    try:
                        process.stdin.close()
                    except Exception:
                        pass

            feeder = threading.Thread(
                target=feed_pipewire,
                name="k9-pipewire-feeder",
                daemon=True,
            )
            feeder.start()

            started = time.perf_counter()
            last_yield = started
            first_chunk_seconds: Optional[float] = None
            generated_audio_samples = 0
            generated_chunks = 0
            generation_seconds = 0.0

            self._publish_feedback(job, "generating", 0.0)

            for chunk in self.tts.infer_stream(
                job.text,
                reference.codes,
                reference.text,
                temperature=self._temperature,
                top_k=self._top_k,
            ):
                interrupted = self._job_interrupt_outcome(job)
                if interrupted is not None:
                    feeder_stop.set()
                    self._terminate_player_for_job(job)
                    return interrupted

                now = time.perf_counter()
                interval = now - last_yield
                last_yield = now
                generation_seconds += interval

                if first_chunk_seconds is None:
                    first_chunk_seconds = now - started

                float_audio = np.asarray(
                    chunk,
                    dtype=np.float32,
                ).reshape(-1)

                if float_audio.size == 0:
                    continue

                float_audio = np.clip(float_audio, -1.0, 1.0)
                rms = float(
                    np.sqrt(
                        np.mean(
                            float_audio.astype(np.float64) ** 2
                        )
                    )
                )
                rms = max(0.0, min(1.0, rms))

                pcm = (
                    float_audio * 32767.0
                ).astype(np.int16)

                generated_audio_samples += pcm.size
                generated_chunks += 1

                if not self._queue_audio_with_interrupts(
                    audio_queue,
                    (pcm, rms, True),
                    job,
                    feeder_stop,
                ):
                    interrupted = self._job_interrupt_outcome(job)
                    if interrupted is not None:
                        self._terminate_player_for_job(job)
                        return interrupted
                    raise RuntimeError(
                        "PipeWire playback feeder stopped unexpectedly"
                    )

            interrupted = self._job_interrupt_outcome(job)
            if interrupted is not None:
                feeder_stop.set()
                self._terminate_player_for_job(job)
                return interrupted

            # A short silent tail lets PipeWire drain the final generated
            # samples without clipping the end of the utterance.
            if self._tail_padding_ms > 0:
                tail_samples = int(
                    self._sample_rate
                    * self._tail_padding_ms
                    / 1000
                )
                tail = np.zeros(tail_samples, dtype=np.int16)
                if not self._queue_audio_with_interrupts(
                    audio_queue,
                    (tail, 0.0, False),
                    job,
                    feeder_stop,
                ):
                    interrupted = self._job_interrupt_outcome(job)
                    if interrupted is not None:
                        self._terminate_player_for_job(job)
                        return interrupted
                    raise RuntimeError(
                        "PipeWire playback feeder stopped unexpectedly"
                    )

            self._queue_audio_with_interrupts(
                audio_queue,
                None,
                job,
                feeder_stop,
            )

            if feeder is not None:
                feeder.join()

            if feeder_errors:
                raise RuntimeError(
                    f"PipeWire feeder failed: {feeder_errors[0]}"
                )

            if process.poll() is None:
                try:
                    process.wait(timeout=5.0)
                except subprocess.TimeoutExpired:
                    process.terminate()
                    process.wait(timeout=1.0)

            if process.returncode not in (0, None):
                interrupted = self._job_interrupt_outcome(job)
                if interrupted is not None:
                    return interrupted
                raise RuntimeError(
                    f"pw-cat exited with status {process.returncode}"
                )

            audio_seconds = (
                generated_audio_samples / self._sample_rate
                if generated_audio_samples
                else 0.0
            )
            synthesis_rtf = (
                generation_seconds / audio_seconds
                if audio_seconds > 0.0
                else 0.0
            )
            ttfa_ms = (
                first_chunk_seconds * 1000.0
                if first_chunk_seconds is not None
                else 0.0
            )

            self.get_logger().info(
                f"Speech complete voice={voice_id:03d} "
                f"TTFA={ttfa_ms:.1f}ms "
                f"synthesis_RTF={synthesis_rtf:.3f} "
                f"audio={audio_seconds:.2f}s "
                f"chunks={generated_chunks}"
            )

            self._publish_feedback(job, "completed", 0.0)
            return JobOutcome.SUCCEEDED, "Speech completed"

        except Exception as error:  # noqa: BLE001
            interrupted = self._job_interrupt_outcome(job)
            if interrupted is not None:
                return interrupted

            self.get_logger().error(
                f"NeuTTS playback failed: {error}"
            )
            return (
                JobOutcome.FAILED,
                f"NeuTTS playback failed: {error}",
            )

        finally:
            feeder_stop.set()

            if process is not None:
                self._clear_active_player(job, process)

                if process.poll() is None:
                    try:
                        process.terminate()
                        process.wait(timeout=1.0)
                    except Exception:
                        try:
                            process.kill()
                        except Exception:
                            pass

            if feeder is not None and feeder.is_alive():
                feeder.join(timeout=1.0)

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
            self.get_logger().debug(
                f"Unable to publish speech feedback: {error}"
            )

    def _publish_rms(self, value: float) -> None:
        if hasattr(self, "rms_pub"):
            self.rms_pub.publish(Float32(data=float(value)))

    def _set_talking(self, value: bool) -> None:
        if self._talking == value:
            return

        self._talking = value

        if not hasattr(self, "talking_pub"):
            return

        message = Bool(data=value)
        self.talking_pub.publish(message)

        if self.legacy_talking_pub is not None:
            self.legacy_talking_pub.publish(message)

    @staticmethod
    def _goal_key(goal_handle: ServerGoalHandle) -> bytes:
        return bytes(goal_handle.goal_id.uuid)

    def _next_legacy_key(self, source: str) -> str:
        with self._condition:
            self._legacy_sequence += 1
            return f"legacy:{source}:{self._legacy_sequence}"

    def destroy_node(self) -> bool:
        self._shutdown_event.set()
        self.cancel_all_speech("Node shutting down")

        with self._condition:
            self._condition.notify_all()

        if hasattr(self, "_worker") and self._worker.is_alive():
            self._worker.join(timeout=2.0)

        if hasattr(self, "action_server"):
            self.action_server.destroy()

        # Force llama.cpp objects to be destroyed before interpreter teardown;
        # this avoids the harmless Llama.__del__ shutdown exception sometimes
        # seen when module globals have already been cleared.
        try:
            del self.tts
        except Exception:
            pass

        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node: Optional[K9NeuTTSVoiceNode] = None

    try:
        node = K9NeuTTSVoiceNode()
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            try:
                executor.shutdown()
            except Exception:
                pass
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

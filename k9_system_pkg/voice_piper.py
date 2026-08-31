#!/usr/bin/env python3
"""Sentence-driven Piper replacement for K9's established SpeakText action.

External contract
-----------------
Action server:
    /voice/speak    k9_interfaces_pkg/action/SpeakText

The SpeakText goal already owns interruption and queue semantics:
    text
    owner
    priority
    interrupt_lower_priority
    clear_lower_priority

Cancellation uses normal ROS 2 action goal cancellation.

Implementation
--------------
* K9's Piper voice is loaded once and kept resident on CPU.
* Each action goal remains one speech goal from the BT's perspective.
* The goal text is split internally at sentence boundaries.
* Sentence N+1 is synthesised while sentence N is playing.
* Higher-priority goals can interrupt/clear lower-priority speech according to
  the goal flags.
"""

from __future__ import annotations

from array import array
from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass, field
import heapq
import itertools
import math
import os
from pathlib import Path
import re
import subprocess
import sys
import tempfile
import threading
import time
import wave


# Piper remains in its own venv.  The ROS node itself runs in K9's ROS Python
# environment and imports Piper/ONNX Runtime from this site-packages directory.
_PIPER_SITE_PACKAGES = os.environ.get(
    "K9_PIPER_SITE_PACKAGES",
    "/home/hopkira/tts_env/piper/.venv/lib/python3.12/site-packages",
)
if _PIPER_SITE_PACKAGES not in sys.path:
    sys.path.insert(0, _PIPER_SITE_PACKAGES)

from piper import PiperVoice  # noqa: E402

import rclpy  # noqa: E402
from rclpy.action import ActionServer, CancelResponse, GoalResponse  # noqa: E402
from rclpy.callback_groups import ReentrantCallbackGroup  # noqa: E402
from rclpy.executors import MultiThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402
from std_msgs.msg import Bool, Float32, String  # noqa: E402

from k9_interfaces_pkg.action import SpeakText  # noqa: E402


_SENTENCE_SPLIT_RE = re.compile(r"(?<=[.!?])\s+")


@dataclass
class GoalContext:
    """Runtime state for one accepted SpeakText action goal."""

    goal_handle: object
    text: str
    owner: str
    priority: int
    interrupt_lower_priority: bool
    clear_lower_priority: bool
    sequence: int

    cancel_event: threading.Event = field(default_factory=threading.Event)
    done_event: threading.Event = field(default_factory=threading.Event)

    success: bool = False
    cancelled: bool = False
    preempted: bool = False
    error_message: str = ""


@dataclass(frozen=True)
class SentenceAudio:
    """PCM produced by Piper for one sentence."""

    text: str
    pcm_bytes: bytes
    sample_rate: int
    channels: int


class VoicePiperNode(Node):
    """Priority-aware SpeakText action server backed by Piper CPU TTS."""

    def __init__(self) -> None:
        super().__init__("voice_piper")

        # ------------------------------------------------------------------
        # Voice model
        # ------------------------------------------------------------------
        self.declare_parameter(
            "model_path",
            "/home/hopkira/k9_piper_voice/k9_model.onnx",
        )
        self.declare_parameter("config_path", "")
        self.declare_parameter("use_cuda", False)
        self.declare_parameter("warmup_enabled", True)
        self.declare_parameter("warmup_text", "Affirmative.")

        # ------------------------------------------------------------------
        # Established ROS contract
        # ------------------------------------------------------------------
        self.declare_parameter("action_name", "/voice/speak")
        self.declare_parameter("is_talking_topic", "/voice/is_talking")
        self.declare_parameter("rms_topic", "/voice/rms_level")
        self.declare_parameter("state_topic", "/voice/state")
        self.declare_parameter(
            "activity_topic",
            "/interaction/activity",
        )

        # ------------------------------------------------------------------
        # Playback
        # ------------------------------------------------------------------
        self.declare_parameter("pipewire_executable", "pw-play")
        self.declare_parameter("pipewire_volume", 1.0)
        self.declare_parameter("rms_frame_ms", 50)

        self.declare_parameter(
            "leading_padding_ms",
            150,
        )

        self._leading_padding_ms = max(
            0,
            int(
                self.get_parameter(
                    "leading_padding_ms"
                ).value
            ),
        )

        self._model_path = Path(
            str(self.get_parameter("model_path").value)
        ).expanduser()

        config_text = str(
            self.get_parameter("config_path").value
        ).strip()
        self._config_path = (
            Path(config_text).expanduser()
            if config_text
            else None
        )

        self._use_cuda = bool(
            self.get_parameter("use_cuda").value
        )
        self._action_name = str(
            self.get_parameter("action_name").value
        )
        self._is_talking_topic = str(
            self.get_parameter("is_talking_topic").value
        )
        self._rms_topic = str(
            self.get_parameter("rms_topic").value
        )
        self._state_topic = str(
            self.get_parameter("state_topic").value
        )
        self._activity_topic = str(
            self.get_parameter("activity_topic").value
        )
        self._pw_play = str(
            self.get_parameter("pipewire_executable").value
        )
        self._volume = float(
            self.get_parameter("pipewire_volume").value
        )
        self._rms_frame_ms = max(
            10,
            int(self.get_parameter("rms_frame_ms").value),
        )

        if not self._model_path.is_file():
            raise FileNotFoundError(
                f"Piper model not found: {self._model_path}"
            )
        if (
            self._config_path is not None
            and not self._config_path.is_file()
        ):
            raise FileNotFoundError(
                f"Piper config not found: {self._config_path}"
            )

        # ------------------------------------------------------------------
        # Status publications used elsewhere in K9.
        # ------------------------------------------------------------------

        self._activity_pub = self.create_publisher(
            String,
            self._activity_topic,
            10,
        )
        self._talking_pub = self.create_publisher(
            Bool,
            self._is_talking_topic,
            10,
        )
        self._rms_pub = self.create_publisher(
            Float32,
            self._rms_topic,
            10,
        )
        self._state_pub = self.create_publisher(
            String,
            self._state_topic,
            10,
        )

        # Track the last published values so repeated sentence boundaries do
        # not generate unnecessary activity/talking transitions.
        self._status_lock = threading.Lock()
        self._current_activity = "IDLE"
        self._is_talking = False

        self._publish_talking(False, force=True)
        self._publish_rms(0.0)
        self._publish_state("LOADING")

        # ------------------------------------------------------------------
        # Load K9's model exactly once.
        # ------------------------------------------------------------------
        start = time.perf_counter()

        if self._config_path is None:
            self._voice = PiperVoice.load(
                str(self._model_path),
                use_cuda=self._use_cuda,
            )
        else:
            self._voice = PiperVoice.load(
                str(self._model_path),
                config_path=str(self._config_path),
                use_cuda=self._use_cuda,
            )

        load_seconds = time.perf_counter() - start
        processor = "CUDA" if self._use_cuda else "CPU"
        self.get_logger().info(
            f"Loaded Piper voice in {load_seconds:.3f}s "
            f"using {processor}: {self._model_path}"
        )

        if bool(
            self.get_parameter("warmup_enabled").value
        ):
            warmup_text = str(
                self.get_parameter("warmup_text").value
            ).strip()
            if warmup_text:
                warm_start = time.perf_counter()
                list(self._voice.synthesize(warmup_text))
                warm_seconds = (
                    time.perf_counter() - warm_start
                )
                self.get_logger().info(
                    f"Piper warm-up completed in "
                    f"{warm_seconds:.3f}s"
                )

        # ------------------------------------------------------------------
        # Goal scheduler
        # ------------------------------------------------------------------
        self._sequence = itertools.count()

        # Entries are (-priority, sequence, GoalContext), therefore the
        # numerically highest priority is selected first and equal-priority
        # goals remain FIFO.
        self._goal_heap = []
        self._contexts = {}

        self._condition = threading.Condition()
        self._active_context = None
        self._shutdown_requested = False

        # Playback can be terminated by action cancellation or pre-emption.
        self._playback_lock = threading.Lock()
        self._play_process = None

        # Piper synthesis is serialised through one executor because one
        # persistent PiperVoice/ONNX session is shared by all goals.
        self._synth_executor = ThreadPoolExecutor(
            max_workers=1,
            thread_name_prefix="k9-piper-synth",
        )

        self._scheduler_thread = threading.Thread(
            target=self._scheduler_loop,
            name="k9-piper-scheduler",
            daemon=True,
        )
        self._scheduler_thread.start()

        # Reentrant callbacks + a multithreaded executor are important:
        # execute_callback waits for its queued speech goal to finish, while
        # new higher-priority goals and cancellation requests must still be
        # processed immediately.
        self._callback_group = ReentrantCallbackGroup()

        self._action_server = ActionServer(
            self,
            SpeakText,
            self._action_name,
            execute_callback=self._execute_callback,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self._callback_group,
        )

        self._publish_state("IDLE")
        self._publish_activity("IDLE", force=True)
        self.get_logger().info(
            f"Piper SpeakText action ready: "
            f"{self._action_name}"
        )

    # ======================================================================
    # Activity publication
    # ======================================================================

    def _publish_activity(
        self,
        activity: str,
        *,
        force: bool = False,
    ) -> None:
        """Publish K9's transient conversational activity.

        PROCESSING is normally published by the STT/VAD side as soon as an
        utterance closes.  This node takes over at the instant audible speech
        begins by publishing SPEAKING, then returns to IDLE only when the
        complete queued speech workload has finished.
        """
        activity = str(activity).strip().upper()

        with self._status_lock:
            if (
                not force
                and activity == self._current_activity
            ):
                return

            self._current_activity = activity

        self._activity_pub.publish(
            String(data=activity)
        )

    # ======================================================================
    # Action callbacks
    # ======================================================================

    def _goal_callback(self, goal_request):
        """Validate a SpeakText request without changing its semantics."""
        text = str(
            getattr(goal_request, "text", "")
        ).strip()

        if not text:
            self.get_logger().warning(
                "Rejecting empty SpeakText goal"
            )
            return GoalResponse.REJECT

        return GoalResponse.ACCEPT

    def _execute_callback(self, goal_handle):
        """Queue an accepted action goal and wait for its speech result."""
        request = goal_handle.request

        context = GoalContext(
            goal_handle=goal_handle,
            text=str(
                getattr(request, "text", "")
            ).strip(),
            owner=str(
                getattr(request, "owner", "unknown")
            ).strip() or "unknown",
            priority=int(
                getattr(request, "priority", 0)
            ),
            interrupt_lower_priority=bool(
                getattr(
                    request,
                    "interrupt_lower_priority",
                    False,
                )
            ),
            clear_lower_priority=bool(
                getattr(
                    request,
                    "clear_lower_priority",
                    False,
                )
            ),
            sequence=next(self._sequence),
        )

        self._enqueue_goal(context)

        # The scheduler/playback workers do the actual work.  This callback
        # waits so the action result still represents completion of the whole
        # original SpeakText goal, not merely queue admission.
        while (
            not context.done_event.wait(timeout=0.05)
            and rclpy.ok()
        ):
            if goal_handle.is_cancel_requested:
                self._request_cancel(
                    context,
                    "Cancelled by action client",
                )

        result = self._make_result(
            success=context.success,
            error_message=context.error_message,
        )

        if (
            context.cancelled
            or goal_handle.is_cancel_requested
        ):
            goal_handle.canceled()
        elif context.success:
            goal_handle.succeed()
        else:
            goal_handle.abort()

        with self._condition:
            self._contexts.pop(
                id(goal_handle),
                None,
            )

        return result

    def _cancel_callback(self, goal_handle):
        """Use normal ROS action cancellation for speech cancellation."""
        with self._condition:
            context = self._contexts.get(
                id(goal_handle)
            )

        if context is not None:
            self._request_cancel(
                context,
                "Cancelled by action client",
            )

        return CancelResponse.ACCEPT

    # ======================================================================
    # Priority scheduler
    # ======================================================================

    def _enqueue_goal(self, context: GoalContext) -> None:
        """Apply clear/pre-empt rules and add the new goal to the heap."""
        cleared = []
        active_to_interrupt = None

        with self._condition:
            self._contexts[
                id(context.goal_handle)
            ] = context

            # clear_lower_priority removes queued work below this goal's
            # priority.  Equal/higher priority work is preserved.
            if context.clear_lower_priority:
                kept = []

                for entry in self._goal_heap:
                    queued = entry[2]

                    if (
                        queued.priority
                        < context.priority
                        and not queued.done_event.is_set()
                    ):
                        queued.preempted = True
                        queued.error_message = (
                            "Cleared by higher-priority "
                            f"speech from {context.owner}"
                        )
                        cleared.append(queued)
                    else:
                        kept.append(entry)

                self._goal_heap = kept
                heapq.heapify(self._goal_heap)

            # interrupt_lower_priority applies only to speech that is already
            # active and strictly lower priority.
            if (
                context.interrupt_lower_priority
                and self._active_context is not None
                and self._active_context.priority
                    < context.priority
                and not self._active_context.done_event.is_set()
            ):
                active_to_interrupt = (
                    self._active_context
                )
                active_to_interrupt.preempted = True
                active_to_interrupt.error_message = (
                    "Pre-empted by higher-priority "
                    f"speech from {context.owner}"
                )
                active_to_interrupt.cancel_event.set()

            heapq.heappush(
                self._goal_heap,
                (
                    -context.priority,
                    context.sequence,
                    context,
                ),
            )

            self._condition.notify_all()

        for queued in cleared:
            queued.success = False
            queued.done_event.set()

        if active_to_interrupt is not None:
            self._terminate_playback()

        self._publish_state("QUEUED")

        self.get_logger().info(
            f"Queued speech owner={context.owner} "
            f"priority={context.priority}: "
            f"{context.text}"
        )

    def _request_cancel(
        self,
        context: GoalContext,
        reason: str,
    ) -> None:
        """Cancel an active or queued SpeakText goal."""
        context.cancelled = True
        context.success = False
        context.error_message = reason
        context.cancel_event.set()

        with self._condition:
            is_active = (
                self._active_context is context
            )
            self._condition.notify_all()

        if is_active:
            self._terminate_playback()
        else:
            # A queued cancelled context does not need to wait for the
            # scheduler to reach it.
            context.done_event.set()

    def _scheduler_loop(self) -> None:
        """Run one speech goal at a time in priority order."""
        while True:
            with self._condition:
                self._condition.wait_for(
                    lambda: (
                        self._shutdown_requested
                        or bool(self._goal_heap)
                    )
                )

                if self._shutdown_requested:
                    return

                _, _, context = heapq.heappop(
                    self._goal_heap
                )

                if context.done_event.is_set():
                    # A queued goal may have been cancelled/cleared before the
                    # scheduler reached it.  If that was the last outstanding
                    # goal, restore the presentation to IDLE rather than
                    # leaving it stuck in SPEAKING/PROCESSING.
                    idle = not any(
                        not entry[2].done_event.is_set()
                        for entry in self._goal_heap
                    )

                    if idle:
                        self._publish_idle_status()

                    continue

                self._active_context = context

            try:
                self._process_goal(context)
            except Exception as exc:
                context.success = False
                context.error_message = str(exc)
                self.get_logger().error(
                    f"Speech goal failed: {exc}"
                )
                self._publish_state("ERROR")
            finally:
                with self._condition:
                    if self._active_context is context:
                        self._active_context = None

                if not context.done_event.is_set():
                    context.done_event.set()

                with self._condition:
                    idle = (
                        self._active_context is None
                        and not self._goal_heap
                    )

                if idle:
                    self._publish_idle_status()

    def _publish_idle_status(self) -> None:
        """Restore voice/activity status once no speech work remains."""
        self._publish_talking(False)
        self._publish_rms(0.0)
        self._publish_state("IDLE")
        self._publish_activity("IDLE")

    # ======================================================================
    # Sentence pipeline
    # ======================================================================

    @staticmethod
    def _split_sentences(
        text: str,
    ) -> list[str]:
        """Split internally at strong sentence boundaries only."""
        normalised = " ".join(text.split())

        if not normalised:
            return []

        return [
            sentence.strip()
            for sentence in _SENTENCE_SPLIT_RE.split(
                normalised
            )
            if sentence.strip()
        ]

    def _process_goal(
        self,
        context: GoalContext,
    ) -> None:
        """Speak all sentences belonging to one SpeakText action goal."""
        sentences = self._split_sentences(
            context.text
        )

        if not sentences:
            context.success = False
            context.error_message = "No speakable text"
            return

        self._publish_state("SYNTHESISING")

        # Generate sentence 1 before talking starts.
        current_future = self._synth_executor.submit(
            self._synthesise_sentence,
            sentences[0],
        )

        try:
            for index, sentence in enumerate(
                sentences
            ):
                if context.cancel_event.is_set():
                    break

                audio = current_future.result()

                # Cancellation/pre-emption may have happened while ONNX was
                # finishing the current sentence.
                if context.cancel_event.is_set():
                    break

                # Start sentence N+1 synthesis before sentence N playback.
                next_future = None

                if index + 1 < len(sentences):
                    next_future = (
                        self._synth_executor.submit(
                            self._synthesise_sentence,
                            sentences[index + 1],
                        )
                    )

                self._play_sentence(
                    audio,
                    context.cancel_event,
                    announce_speaking=(index == 0),
                )

                if context.cancel_event.is_set():
                    break

                completed = index + 1
                progress = completed / len(sentences)
                self._publish_feedback(
                    context.goal_handle,
                    progress,
                )

                current_future = next_future

            if context.cancelled:
                context.success = False

            elif context.preempted:
                context.success = False

            elif context.cancel_event.is_set():
                context.success = False
                if not context.error_message:
                    context.error_message = (
                        "Speech interrupted"
                    )

            else:
                context.success = True
                context.error_message = ""
                self._publish_feedback(
                    context.goal_handle,
                    1.0,
                )

        finally:
            # Keep /interaction/activity as SPEAKING until the scheduler knows
            # that there are no further queued goals.  This prevents a brief
            # IDLE/green-panel flicker between consecutive SpeakText goals.
            self._publish_talking(False)
            self._publish_rms(0.0)

    def _synthesise_sentence(
        self,
        sentence: str,
    ) -> SentenceAudio:
        """Run one complete sentence through the resident Piper model."""
        start = time.perf_counter()
        chunks = list(
            self._voice.synthesize(sentence)
        )

        if not chunks:
            raise RuntimeError(
                "Piper returned no audio"
            )

        first = chunks[0]
        sample_rate = int(first.sample_rate)
        channels = int(first.sample_channels)
        pcm_parts = []

        for chunk in chunks:
            if int(chunk.sample_rate) != sample_rate:
                raise RuntimeError(
                    "Piper changed sample rate "
                    "within a sentence"
                )

            if int(
                chunk.sample_channels
            ) != channels:
                raise RuntimeError(
                    "Piper changed channel count "
                    "within a sentence"
                )

            if int(chunk.sample_width) != 2:
                raise RuntimeError(
                    "Piper did not return "
                    "16-bit PCM"
                )

            pcm_parts.append(
                chunk.audio_int16_bytes
            )

        pcm_bytes = b"".join(pcm_parts)

        audio_seconds = (
            len(pcm_bytes)
            / 2.0
            / channels
            / sample_rate
        )
        synth_seconds = (
            time.perf_counter() - start
        )
        rtf = (
            synth_seconds / audio_seconds
            if audio_seconds > 0.0
            else 0.0
        )

        self.get_logger().info(
            f"Synthesised {audio_seconds:.2f}s "
            f"in {synth_seconds:.3f}s "
            f"(RTF {rtf:.3f}): {sentence}"
        )

        return SentenceAudio(
            text=sentence,
            pcm_bytes=pcm_bytes,
            sample_rate=sample_rate,
            channels=channels,
        )

    # ======================================================================
    # PipeWire playback
    # ======================================================================

    def _play_sentence(
        self,
        audio: SentenceAudio,
        cancel_event: threading.Event,
        *,
        announce_speaking: bool,
    ) -> None:
        """Play one in-memory Piper sentence through pw-play.

        For the first sentence in a SpeakText goal, PROCESSING remains visible
        during the leading PipeWire padding.  SPEAKING and /voice/is_talking
        are asserted only when the real Piper samples are about to begin.
        Subsequent sentences keep the existing SPEAKING activity continuously.
        """

        temp_path = None

        try:
            # --------------------------------------------------------------
            # Generate a short period of digital silence before the speech.
            #
            # This gives PipeWire / ALSA time to open and stabilise the
            # playback stream before the first actual Piper samples arrive.
            # It prevents the first phoneme of an utterance being clipped.
            # --------------------------------------------------------------

            leading_padding_seconds = (
                self._leading_padding_ms / 1000.0
            )

            leading_padding_frames = int(
                audio.sample_rate
                * leading_padding_seconds
            )

            # Piper PCM is signed 16-bit audio, so each sample is two bytes.
            # One silent frame contains one zero-valued sample per channel.
            leading_silence = (
                b"\x00"
                * leading_padding_frames
                * audio.channels
                * 2
            )

            # --------------------------------------------------------------
            # Write the padded utterance to a temporary WAV file.
            # --------------------------------------------------------------

            with tempfile.NamedTemporaryFile(
                prefix="k9_piper_",
                suffix=".wav",
                delete=False,
            ) as temp_file:
                temp_path = temp_file.name

            with wave.open(
                temp_path,
                "wb",
            ) as wav_file:
                wav_file.setnchannels(
                    audio.channels
                )
                wav_file.setsampwidth(2)
                wav_file.setframerate(
                    audio.sample_rate
                )

                # Write silence first so that pw-play can establish
                # the audio stream before K9 actually starts speaking.
                wav_file.writeframes(
                    leading_silence
                )

                # Follow immediately with the actual Piper audio.
                wav_file.writeframes(
                    audio.pcm_bytes
                )

            # --------------------------------------------------------------
            # Start PipeWire playback.
            # --------------------------------------------------------------

            process = subprocess.Popen(
                [
                    self._pw_play,
                    f"--volume={self._volume}",
                    temp_path,
                ],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.PIPE,
            )

            with self._playback_lock:
                if cancel_event.is_set():
                    process.terminate()

                self._play_process = process

            # --------------------------------------------------------------
            # Align visual/talking state with the first real speech sample.
            #
            # pw-play is already running, but the WAV begins with digital
            # silence.  Keeping PROCESSING active through this short wait means
            # the panel changes to SPEAKING at the point K9 is actually about
            # to make sound, rather than when the playback process merely opens.
            # --------------------------------------------------------------

            padding_cancelled = False

            if leading_padding_seconds > 0.0:
                padding_cancelled = cancel_event.wait(
                    leading_padding_seconds
                )

            playback_alive = (
                process.poll() is None
            )

            if (
                not padding_cancelled
                and not cancel_event.is_set()
                and playback_alive
            ):
                if announce_speaking:
                    self._publish_talking(True)
                    self._publish_activity("SPEAKING")
                    self._publish_state("SPEAKING")

                # RMS now starts at sample zero of the unpadded Piper PCM,
                # therefore it remains aligned with the audible speech.
                rms_stop = threading.Event()
                rms_thread = threading.Thread(
                    target=self._rms_loop,
                    args=(
                        audio,
                        rms_stop,
                        cancel_event,
                    ),
                    name="k9-piper-rms",
                    daemon=True,
                )
                rms_thread.start()
            else:
                rms_stop = None
                rms_thread = None

            # --------------------------------------------------------------
            # Wait for pw-play to finish.
            # --------------------------------------------------------------

            _, stderr = process.communicate()

            if rms_stop is not None:
                rms_stop.set()

            if rms_thread is not None:
                rms_thread.join(
                    timeout=0.5
                )

            with self._playback_lock:
                if self._play_process is process:
                    self._play_process = None

            self._publish_rms(0.0)

            if (
                process.returncode not in (
                    0,
                    -15,
                    -9,
                )
                and not cancel_event.is_set()
            ):
                detail = stderr.decode(
                    "utf-8",
                    errors="replace",
                ).strip()

                self.get_logger().warning(
                    f"pw-play failed "
                    f"(returncode={process.returncode}): "
                    f"{detail}"
                )

        finally:
            # --------------------------------------------------------------
            # Always remove the temporary WAV file.
            # --------------------------------------------------------------

            if temp_path is not None:
                try:
                    os.unlink(
                        temp_path
                    )
                except OSError:
                    pass

    def _terminate_playback(self) -> None:
        """Immediately terminate the current PipeWire player."""
        with self._playback_lock:
            process = self._play_process

            if (
                process is None
                or process.poll() is not None
            ):
                return

            try:
                process.terminate()
            except ProcessLookupError:
                pass

    # ======================================================================
    # Feedback / status
    # ======================================================================

    @staticmethod
    def _make_result(
        *,
        success: bool,
        error_message: str,
    ):
        """Populate the established SpeakText result defensively."""
        result = SpeakText.Result()

        if hasattr(result, "success"):
            result.success = bool(success)

        if hasattr(result, "error_message"):
            result.error_message = str(
                error_message
            )

        return result

    @staticmethod
    def _publish_feedback(
        goal_handle,
        progress: float,
    ) -> None:
        """Publish SpeakText progress if the current action defines it."""
        feedback = SpeakText.Feedback()

        if hasattr(feedback, "progress"):
            feedback.progress = float(
                max(0.0, min(1.0, progress))
            )

        goal_handle.publish_feedback(
            feedback
        )

    def _rms_loop(
        self,
        audio: SentenceAudio,
        stop_event: threading.Event,
        cancel_event: threading.Event,
    ) -> None:
        """Publish approximate RMS in playback time."""
        samples = array("h")
        samples.frombytes(audio.pcm_bytes)

        if sys.byteorder != "little":
            samples.byteswap()

        samples_per_frame = max(
            1,
            int(
                audio.sample_rate
                * audio.channels
                * self._rms_frame_ms
                / 1000
            ),
        )

        frame_seconds = (
            self._rms_frame_ms / 1000.0
        )

        for start in range(
            0,
            len(samples),
            samples_per_frame,
        ):
            if (
                stop_event.is_set()
                or cancel_event.is_set()
            ):
                break

            frame = samples[
                start:
                start + samples_per_frame
            ]

            if not frame:
                break

            square_sum = sum(
                sample * sample
                for sample in frame
            )

            rms = math.sqrt(
                square_sum / len(frame)
            ) / 32768.0

            # A sqrt display curve gives useful expression movement while
            # preserving a 0..1 public level.
            level = min(
                1.0,
                math.sqrt(
                    max(0.0, rms)
                ),
            )

            self._publish_rms(level)
            stop_event.wait(
                frame_seconds
            )

    def _publish_talking(
        self,
        talking: bool,
        *,
        force: bool = False,
    ) -> None:
        talking = bool(talking)

        with self._status_lock:
            if (
                not force
                and talking == self._is_talking
            ):
                return

            self._is_talking = talking

        self._talking_pub.publish(
            Bool(data=talking)
        )

    def _publish_rms(
        self,
        level: float,
    ) -> None:
        self._rms_pub.publish(
            Float32(
                data=float(
                    max(
                        0.0,
                        min(1.0, level),
                    )
                )
            )
        )

    def _publish_state(
        self,
        state: str,
    ) -> None:
        self._state_pub.publish(
            String(data=state)
        )

    # ======================================================================
    # Shutdown
    # ======================================================================

    def close(self) -> None:
        """Stop active speech, scheduler and synthesis executor."""
        with self._condition:
            if self._shutdown_requested:
                return

            self._shutdown_requested = True

            if self._active_context is not None:
                self._active_context.cancel_event.set()

            for _, _, context in self._goal_heap:
                context.cancelled = True
                context.error_message = (
                    "Voice node shutting down"
                )
                context.cancel_event.set()
                context.done_event.set()

            self._goal_heap.clear()
            self._condition.notify_all()

        self._terminate_playback()
        self._scheduler_thread.join(
            timeout=2.0
        )

        self._synth_executor.shutdown(
            wait=False,
            cancel_futures=True,
        )

        self._publish_talking(False)
        self._publish_rms(0.0)
        self._publish_state("IDLE")
        self._publish_activity("IDLE")

        self._action_server.destroy()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VoicePiperNode()

    executor = MultiThreadedExecutor(
        num_threads=4
    )
    executor.add_node(node)

    try:
        executor.spin()

    except KeyboardInterrupt:
        pass

    finally:
        node.close()
        executor.remove_node(node)
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

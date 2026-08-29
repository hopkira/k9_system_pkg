#!/usr/bin/env python3
"""Sentence-driven Piper voice node for K9.

Piper is installed in its own virtual environment, while this ROS 2 node runs
under K9's normal ROS Python environment.  At import time we temporarily add
Piper's site-packages directory so Piper and its ONNX Runtime dependencies can
be imported without duplicating them into k9_venv.

Architecture
------------
/voice/sentence -> sentence queue -> Piper synthesis -> audio queue -> pw-play

Synthesis and playback use separate worker threads.  This allows sentence N+1
to be synthesised while sentence N is already being spoken.

Existing K9 semantics retained:
* /speak_now pre-empts current and queued ordinary speech.
* /cancel_speech stops playback and discards queued/stale synthesis.
* /voice/is_talking publishes the actual playback state.
* /voice/rms_level publishes a normalised RMS level while speech is playing.
"""

from __future__ import annotations

from array import array
from collections import deque
from dataclasses import dataclass
import heapq
import io
import itertools
import math
import os
from pathlib import Path
import re
import subprocess
import sys
import threading
import time
import wave


# ---------------------------------------------------------------------------
# Import Piper from its dedicated venv without requiring rclpy in that venv.
# Override with K9_PIPER_SITE_PACKAGES if the Piper environment moves.
# ---------------------------------------------------------------------------
_PIPER_SITE_PACKAGES = os.environ.get(
    "K9_PIPER_SITE_PACKAGES",
    "/home/hopkira/tts_env/piper/.venv/lib/python3.12/site-packages",
)

_piper_path_added = False
if _PIPER_SITE_PACKAGES not in sys.path:
    sys.path.insert(0, _PIPER_SITE_PACKAGES)
    _piper_path_added = True

try:
    from piper import PiperVoice
finally:
    # Piper has now imported its own onnxruntime/numpy dependencies.  Remove
    # the temporary path so subsequent ROS imports resolve normally.
    if _piper_path_added:
        try:
            sys.path.remove(_PIPER_SITE_PACKAGES)
        except ValueError:
            pass


import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, String

from k9_interfaces_pkg.srv import CancelSpeech, Speak


_SENTENCE_SPLIT_RE = re.compile(r"(?<=[.!?])\s+")


@dataclass(frozen=True)
class SpeechItem:
    """One complete sentence waiting for Piper synthesis."""

    text: str
    owner: str
    priority: int
    generation: int
    sequence: int


@dataclass(frozen=True)
class AudioItem:
    """A synthesised sentence waiting for PipeWire playback."""

    text: str
    owner: str
    priority: int
    generation: int
    sequence: int
    wav_bytes: bytes
    pcm_bytes: bytes
    sample_rate: int
    channels: int


class VoicePiperNode(Node):
    """Persistent CPU Piper TTS with sentence-ahead synthesis."""

    def __init__(self) -> None:
        super().__init__("voice_piper")

        # ------------------------------------------------------------------
        # Voice parameters
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
        # ROS interface parameters
        # ------------------------------------------------------------------
        self.declare_parameter("sentence_topic", "/voice/sentence")
        self.declare_parameter("is_talking_topic", "/voice/is_talking")
        self.declare_parameter("rms_topic", "/voice/rms_level")
        self.declare_parameter("state_topic", "/voice/state")
        self.declare_parameter("topic_priority", 20)
        self.declare_parameter("speak_now_priority", 100)

        # ------------------------------------------------------------------
        # Playback parameters
        # ------------------------------------------------------------------
        self.declare_parameter("pipewire_executable", "pw-play")
        self.declare_parameter("pipewire_volume", 1.0)
        self.declare_parameter("rms_frame_ms", 50)

        self._model_path = Path(
            str(self.get_parameter("model_path").value)
        ).expanduser()

        config_text = str(self.get_parameter("config_path").value).strip()
        self._config_path = (
            Path(config_text).expanduser() if config_text else None
        )

        self._use_cuda = bool(self.get_parameter("use_cuda").value)

        self._sentence_topic = str(
            self.get_parameter("sentence_topic").value
        )
        self._is_talking_topic = str(
            self.get_parameter("is_talking_topic").value
        )
        self._rms_topic = str(self.get_parameter("rms_topic").value)
        self._state_topic = str(self.get_parameter("state_topic").value)

        self._topic_priority = int(
            self.get_parameter("topic_priority").value
        )
        self._speak_now_priority = int(
            self.get_parameter("speak_now_priority").value
        )

        self._pw_play = str(
            self.get_parameter("pipewire_executable").value
        )
        self._volume = float(
            self.get_parameter("pipewire_volume").value
        )
        self._rms_frame_ms = max(
            10, int(self.get_parameter("rms_frame_ms").value)
        )

        if not self._model_path.is_file():
            raise FileNotFoundError(
                f"Piper model not found: {self._model_path}"
            )

        if self._config_path is not None and not self._config_path.is_file():
            raise FileNotFoundError(
                f"Piper config not found: {self._config_path}"
            )

        # ------------------------------------------------------------------
        # ROS publishers, subscriber and compatibility services
        # ------------------------------------------------------------------
        self._talking_pub = self.create_publisher(
            Bool, self._is_talking_topic, 10
        )
        self._rms_pub = self.create_publisher(
            Float32, self._rms_topic, 10
        )
        self._state_pub = self.create_publisher(
            String, self._state_topic, 10
        )

        self._sentence_sub = self.create_subscription(
            String,
            self._sentence_topic,
            self._sentence_callback,
            10,
        )

        self._speak_now_srv = self.create_service(
            Speak,
            "speak_now",
            self._speak_now_callback,
        )
        self._cancel_srv = self.create_service(
            CancelSpeech,
            "cancel_speech",
            self._cancel_callback,
        )

        # ------------------------------------------------------------------
        # Queues and cancellation generation
        # ------------------------------------------------------------------
        # The sentence heap is priority ordered.  Negating priority means
        # larger numeric priorities are selected first.  sequence preserves
        # FIFO ordering between sentences at equal priority.
        self._sentence_heap: list[tuple[int, int, SpeechItem]] = []

        # Synthesised audio is FIFO.  Piper synthesis is serial, so this
        # preserves sentence order while still allowing synthesis-ahead.
        self._audio_queue: deque[AudioItem] = deque()

        self._sequence = itertools.count()
        self._condition = threading.Condition()

        # Incrementing generation invalidates both queued work and an older
        # sentence that happens to finish synthesis after cancellation.
        self._generation = 0
        self._shutdown_requested = False

        # The active pw-play process is retained so cancel_speech/speak_now can
        # stop it from the ROS callback thread.
        self._play_process: subprocess.Popen[bytes] | None = None

        self._publish_talking(False)
        self._publish_rms(0.0)
        self._publish_state("LOADING")

        # ------------------------------------------------------------------
        # Load K9's Piper voice exactly once.
        # ------------------------------------------------------------------
        load_start = time.perf_counter()
        self._voice = PiperVoice.load(
            self._model_path,
            config_path=self._config_path,
            use_cuda=self._use_cuda,
        )
        self.get_logger().info(
            "Loaded Piper voice in %.3fs using %s: %s",
            time.perf_counter() - load_start,
            "CUDA" if self._use_cuda else "CPU",
            self._model_path,
        )

        # Warm ONNX Runtime before the first conversational response.
        if bool(self.get_parameter("warmup_enabled").value):
            warmup_text = str(
                self.get_parameter("warmup_text").value
            ).strip()
            if warmup_text:
                warm_start = time.perf_counter()
                list(self._voice.synthesize(warmup_text))
                self.get_logger().info(
                    "Piper warm-up completed in %.3fs",
                    time.perf_counter() - warm_start,
                )

        # ------------------------------------------------------------------
        # Separate synthesis and playback workers.
        # ------------------------------------------------------------------
        self._synth_thread = threading.Thread(
            target=self._synthesis_loop,
            name="k9-piper-synthesis",
            daemon=True,
        )
        self._play_thread = threading.Thread(
            target=self._playback_loop,
            name="k9-piper-playback",
            daemon=True,
        )
        self._synth_thread.start()
        self._play_thread.start()

        self._publish_state("IDLE")
        self.get_logger().info(
            "Sentence-driven Piper ready: %s",
            self._sentence_topic,
        )

    # ======================================================================
    # ROS callbacks
    # ======================================================================

    def _sentence_callback(self, msg: String) -> None:
        """Queue ordinary dialogue speech.

        The conversation layer should publish each completed sentence as soon
        as it is available.  A multi-sentence message is still accepted and
        split defensively.
        """
        text = msg.data.strip()
        if not text:
            return

        count = self._queue_text(
            text,
            owner="dialogue",
            priority=self._topic_priority,
            interrupt=False,
        )
        if count:
            self.get_logger().info(
                "Queued %d dialogue sentence(s): %s",
                count,
                text,
            )

    def _speak_now_callback(self, request, response):
        """Pre-empt ordinary speech and immediately queue urgent speech."""
        text = request.text.strip()

        if not text:
            response.success = False
            if hasattr(response, "message"):
                response.message = "No text supplied"
            return response

        count = self._queue_text(
            text,
            owner="speak_now",
            priority=self._speak_now_priority,
            interrupt=True,
        )

        response.success = count > 0
        if hasattr(response, "message"):
            response.message = f"Queued {count} sentence(s)"
        return response

    def _cancel_callback(self, _request, response):
        """Stop playback and discard all queued/stale speech."""
        self._cancel_all("cancel_speech")
        response.success = True
        if hasattr(response, "message"):
            response.message = "Speech cancelled"
        return response

    # ======================================================================
    # Queue management
    # ======================================================================

    @staticmethod
    def _split_sentences(text: str) -> list[str]:
        """Split only at strong sentence boundaries: full stop, ? and !."""
        normalised = " ".join(text.split())
        if not normalised:
            return []

        return [
            part.strip()
            for part in _SENTENCE_SPLIT_RE.split(normalised)
            if part.strip()
        ]

    def _queue_text(
        self,
        text: str,
        *,
        owner: str,
        priority: int,
        interrupt: bool,
    ) -> int:
        sentences = self._split_sentences(text)
        if not sentences:
            return 0

        with self._condition:
            if interrupt:
                # speak_now starts a fresh generation.  Already-synthesised
                # audio and queued sentences from the old response disappear.
                self._generation += 1
                self._sentence_heap.clear()
                self._audio_queue.clear()
                self._terminate_playback_locked()

            generation = self._generation

            for sentence in sentences:
                sequence = next(self._sequence)
                item = SpeechItem(
                    text=sentence,
                    owner=owner,
                    priority=priority,
                    generation=generation,
                    sequence=sequence,
                )
                heapq.heappush(
                    self._sentence_heap,
                    (-priority, sequence, item),
                )

            self._condition.notify_all()

        self._publish_state("QUEUED")
        return len(sentences)

    def _cancel_all(self, reason: str) -> None:
        with self._condition:
            self._generation += 1
            self._sentence_heap.clear()
            self._audio_queue.clear()
            self._terminate_playback_locked()
            self._condition.notify_all()

        self._publish_talking(False)
        self._publish_rms(0.0)
        self._publish_state("IDLE")
        self.get_logger().info("Speech cancelled: %s", reason)

    # ======================================================================
    # Piper synthesis worker
    # ======================================================================

    def _synthesis_loop(self) -> None:
        """Convert queued sentences to in-memory WAV audio."""
        while True:
            with self._condition:
                self._condition.wait_for(
                    lambda: (
                        self._shutdown_requested
                        or bool(self._sentence_heap)
                    )
                )
                if self._shutdown_requested:
                    return

                _, _, item = heapq.heappop(self._sentence_heap)

            # The sentence may have been invalidated while waiting.
            if item.generation != self._generation:
                continue

            self._publish_state("SYNTHESISING")
            synth_start = time.perf_counter()

            try:
                chunks = list(self._voice.synthesize(item.text))
                audio = self._chunks_to_audio(item, chunks)
            except Exception as exc:
                self.get_logger().error(
                    "Piper synthesis failed for %r: %s",
                    item.text,
                    exc,
                )
                self._publish_state("ERROR")
                continue

            synth_seconds = time.perf_counter() - synth_start

            # Cancellation cannot abort a blocking ONNX session.run(), but a
            # generation check prevents the resulting stale audio from playing.
            with self._condition:
                if item.generation != self._generation:
                    continue

                self._audio_queue.append(audio)
                self._condition.notify_all()

            audio_seconds = (
                len(audio.pcm_bytes)
                / 2.0
                / audio.channels
                / audio.sample_rate
            )

            self.get_logger().info(
                "Synthesised %.2fs in %.3fs (RTF %.3f): %s",
                audio_seconds,
                synth_seconds,
                (
                    synth_seconds / audio_seconds
                    if audio_seconds > 0.0
                    else 0.0
                ),
                item.text,
            )

    @staticmethod
    def _chunks_to_audio(
        item: SpeechItem,
        chunks,
    ) -> AudioItem:
        """Combine Piper chunks and wrap the PCM in an in-memory WAV."""
        if not chunks:
            raise RuntimeError("Piper returned no audio")

        first = chunks[0]
        sample_rate = int(first.sample_rate)
        channels = int(first.sample_channels)

        pcm_parts: list[bytes] = []

        for chunk in chunks:
            if int(chunk.sample_rate) != sample_rate:
                raise RuntimeError("Piper changed sample rate within sentence")
            if int(chunk.sample_channels) != channels:
                raise RuntimeError("Piper changed channel count within sentence")
            if int(chunk.sample_width) != 2:
                raise RuntimeError(
                    f"Expected 16-bit Piper audio, got "
                    f"{chunk.sample_width}-byte samples"
                )

            pcm_parts.append(chunk.audio_int16_bytes)

        pcm_bytes = b"".join(pcm_parts)

        wav_buffer = io.BytesIO()
        with wave.open(wav_buffer, "wb") as wav_file:
            wav_file.setnchannels(channels)
            wav_file.setsampwidth(2)
            wav_file.setframerate(sample_rate)
            wav_file.writeframes(pcm_bytes)

        return AudioItem(
            text=item.text,
            owner=item.owner,
            priority=item.priority,
            generation=item.generation,
            sequence=item.sequence,
            wav_bytes=wav_buffer.getvalue(),
            pcm_bytes=pcm_bytes,
            sample_rate=sample_rate,
            channels=channels,
        )

    # ======================================================================
    # PipeWire playback worker
    # ======================================================================

    def _playback_loop(self) -> None:
        """Play synthesised sentences in order while synthesis continues."""
        while True:
            with self._condition:
                self._condition.wait_for(
                    lambda: (
                        self._shutdown_requested
                        or bool(self._audio_queue)
                    )
                )
                if self._shutdown_requested:
                    return

                audio = self._audio_queue.popleft()

            if audio.generation != self._generation:
                continue

            self._play_audio(audio)

            # If nothing remains at either stage, speech really is idle.
            with self._condition:
                empty = (
                    not self._audio_queue
                    and not self._sentence_heap
                    and self._play_process is None
                )

            if empty:
                self._publish_state("IDLE")
            else:
                self._publish_state("QUEUED")

    def _play_audio(self, audio: AudioItem) -> None:
        """Feed one complete in-memory WAV to pw-play."""
        command = [
            self._pw_play,
            f"--volume={self._volume}",
            "-",
        ]

        try:
            process = subprocess.Popen(
                command,
                stdin=subprocess.PIPE,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.PIPE,
            )
        except Exception as exc:
            self.get_logger().error(
                "Unable to start PipeWire playback: %s",
                exc,
            )
            self._publish_state("ERROR")
            return

        with self._condition:
            # If cancellation raced with process creation, do not start.
            if audio.generation != self._generation:
                process.terminate()
                return
            self._play_process = process

        self._publish_state("SPEAKING")
        self._publish_talking(True)

        rms_stop = threading.Event()
        rms_thread = threading.Thread(
            target=self._rms_loop,
            args=(audio, rms_stop),
            name="k9-piper-rms",
            daemon=True,
        )
        rms_thread.start()

        stderr = b""
        try:
            _, stderr = process.communicate(input=audio.wav_bytes)
        except BrokenPipeError:
            # Expected when cancel_speech terminates pw-play.
            pass
        finally:
            rms_stop.set()
            rms_thread.join(timeout=0.5)

            with self._condition:
                if self._play_process is process:
                    self._play_process = None

            self._publish_rms(0.0)
            self._publish_talking(False)

        if (
            process.returncode not in (0, -15, -9)
            and audio.generation == self._generation
        ):
            detail = stderr.decode("utf-8", errors="replace").strip()
            self.get_logger().error(
                "pw-play failed rc=%s: %s",
                process.returncode,
                detail,
            )
            self._publish_state("ERROR")

    def _terminate_playback_locked(self) -> None:
        """Terminate active pw-play. Caller holds self._condition."""
        process = self._play_process
        if process is None or process.poll() is not None:
            return

        try:
            process.terminate()
        except ProcessLookupError:
            return

    # ======================================================================
    # RMS / diagnostics
    # ======================================================================

    def _rms_loop(
        self,
        audio: AudioItem,
        stop_event: threading.Event,
    ) -> None:
        """Publish approximate playback RMS from each 50 ms PCM window."""
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
        frame_seconds = self._rms_frame_ms / 1000.0

        for start in range(0, len(samples), samples_per_frame):
            if stop_event.is_set():
                break

            frame = samples[start : start + samples_per_frame]
            if not frame:
                break

            square_sum = sum(sample * sample for sample in frame)
            rms = math.sqrt(square_sum / len(frame)) / 32768.0

            # A square-root display curve provides useful movement for the eye
            # animation while retaining the 0..1 contract.
            level = min(1.0, math.sqrt(max(0.0, rms)))
            self._publish_rms(level)

            stop_event.wait(frame_seconds)

    def _publish_talking(self, talking: bool) -> None:
        self._talking_pub.publish(Bool(data=talking))

    def _publish_rms(self, level: float) -> None:
        self._rms_pub.publish(
            Float32(data=float(max(0.0, min(1.0, level))))
        )

    def _publish_state(self, state: str) -> None:
        self._state_pub.publish(String(data=state))

    # ======================================================================
    # Shutdown
    # ======================================================================

    def close(self) -> None:
        """Stop worker threads and playback cleanly."""
        with self._condition:
            if self._shutdown_requested:
                return

            self._shutdown_requested = True
            self._generation += 1
            self._sentence_heap.clear()
            self._audio_queue.clear()
            self._terminate_playback_locked()
            self._condition.notify_all()

        self._synth_thread.join(timeout=2.0)
        self._play_thread.join(timeout=2.0)

        self._publish_talking(False)
        self._publish_rms(0.0)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None

    try:
        node = VoicePiperNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""
K9 Hotword / Audio Front-End Node

Responsibilities
----------------
* Own the physical microphone permanently.
* Capture 16 kHz, mono, signed 16-bit PCM from ALSA.
* Publish raw PCM frames on /audio/raw.
* Run sherpa-onnx keyword spotting only while the effective audio state is
  WAITING_FOR_HOTWORD.
* Publish exactly one std_msgs/Bool(True) on /hotword_detected for each armed
  wake-word event.
* Re-arm only after the effective audio state leaves WAITING_FOR_HOTWORD and
  later returns to it.

The revised node deliberately DOES NOT start/stop STT or control conversation state.
Those decisions belong to the Behaviour Tree / Audio State Manager.
"""

from __future__ import annotations

import os
import queue
import threading
from pathlib import Path
from typing import Optional

import alsaaudio
import numpy as np
import rclpy
import sherpa_onnx
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import Bool, String

from k9_interfaces_pkg.msg import AudioFrame


WAITING_FOR_HOTWORD = "WAITING_FOR_HOTWORD"


class HotwordNode(Node):
    def __init__(self) -> None:
        super().__init__("hotword")

        # ------------------------------------------------------------------
        # ROS parameters
        # ------------------------------------------------------------------
        self.declare_parameter("audio_device", "plughw:0,0")
        self.declare_parameter("sample_rate", 16000)
        self.declare_parameter("channels", 1)
        self.declare_parameter("frame_ms", 20)

        self.declare_parameter(
            "model_dir",
            "~/k9_models/sherpa-onnx-kws-zipformer-zh-en-3M-2025-12-20",
        )
        self.declare_parameter(
            "keywords_file",
            "~/k9_models/keywords.txt",
        )
        self.declare_parameter(
            "encoder_file",
            "encoder-epoch-13-avg-2-chunk-8-left-64.int8.onnx",
        )
        self.declare_parameter(
            "decoder_file",
            "decoder-epoch-13-avg-2-chunk-8-left-64.onnx",
        )
        self.declare_parameter(
            "joiner_file",
            "joiner-epoch-13-avg-2-chunk-8-left-64.int8.onnx",
        )
        self.declare_parameter("tokens_file", "tokens.txt")

        self.declare_parameter("provider", "cpu")
        self.declare_parameter("num_threads", 2)
        self.declare_parameter("max_active_paths", 4)
        self.declare_parameter("num_trailing_blanks", 1)
        self.declare_parameter("keywords_score", 1.0)
        self.declare_parameter("keywords_threshold", 0.25)

        self.declare_parameter("audio_topic", "/audio/raw")
        self.declare_parameter("hotword_topic", "/hotword_detected")
        self.declare_parameter(
            "effective_state_topic",
            "/audio/effective_state",
        )
        self.declare_parameter(
            "initial_effective_state",
            WAITING_FOR_HOTWORD,
        )

        self._last_debug_tokens = ""

        # ------------------------------------------------------------------
        # DEBUG
        # ------------------------------------------------------------------
        #self._frames_captured = 0
        #self._frames_published = 0

        #self._debug_timer = self.create_timer(
        #    2.0,
        #    self._report_audio_stats,
        #)

        # ------------------------------------------------------------------
        # Resolve parameters
        # ------------------------------------------------------------------
        self.audio_device = self._str_param("audio_device")
        self.sample_rate = self._int_param("sample_rate")
        self.channels = self._int_param("channels")
        frame_ms = self._int_param("frame_ms")

        if self.sample_rate != 16000:
            self.get_logger().warning(
                "This sherpa KWS model expects 16 kHz audio; "
                f"sample_rate is configured as {self.sample_rate}."
            )

        if self.channels != 1:
            raise RuntimeError(
                "HotwordNode currently requires mono input (channels=1)."
            )

        self.period_size = int(self.sample_rate * frame_ms / 1000)
        if self.period_size <= 0:
            raise RuntimeError("frame_ms produced an invalid ALSA period size")

        model_dir = Path(
            os.path.expandvars(os.path.expanduser(self._str_param("model_dir")))
        )
        keywords_file = Path(
            os.path.expandvars(os.path.expanduser(self._str_param("keywords_file")))
        )

        encoder = model_dir / self._str_param("encoder_file")
        decoder = model_dir / self._str_param("decoder_file")
        joiner = model_dir / self._str_param("joiner_file")
        tokens = model_dir / self._str_param("tokens_file")

        for path in (encoder, decoder, joiner, tokens, keywords_file):
            if not path.is_file():
                raise FileNotFoundError(f"Required hotword file not found: {path}")

        # ------------------------------------------------------------------
        # ROS interfaces
        # ------------------------------------------------------------------
        audio_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.audio_pub = self.create_publisher(
            AudioFrame,
            self._str_param("audio_topic"),
            audio_qos,
        )
        self.hotword_pub = self.create_publisher(
            Bool,
            self._str_param("hotword_topic"),
            10,
        )
        self.state_sub = self.create_subscription(
            String,
            self._str_param("effective_state_topic"),
            self._state_callback,
            10,
        )

        # The audio thread never calls rclpy publishers directly. It places
        # captured frames/events into queues, and this ROS timer drains them.
        self._audio_queue: queue.Queue[np.ndarray] = queue.Queue(maxsize=25)
        self._hotword_queue: queue.Queue[str] = queue.Queue(maxsize=4)
        self._publish_timer = self.create_timer(0.01, self._drain_queues)

        # ------------------------------------------------------------------
        # sherpa-onnx KWS
        # ------------------------------------------------------------------
        self.get_logger().info("Loading sherpa-onnx keyword spotter...")
        self.kws = sherpa_onnx.KeywordSpotter(
            tokens=str(tokens),
            encoder=str(encoder),
            decoder=str(decoder),
            joiner=str(joiner),
            num_threads=self._int_param("num_threads"),
            max_active_paths=self._int_param("max_active_paths"),
            keywords_file=str(keywords_file),
            keywords_score=self._float_param("keywords_score"),
            keywords_threshold=self._float_param("keywords_threshold"),
            num_trailing_blanks=self._int_param("num_trailing_blanks"),
            provider=self._str_param("provider"),
        )
        self.kws_stream = self.kws.create_stream()

        # ------------------------------------------------------------------
        # State / latch
        # ------------------------------------------------------------------
        self._state_lock = threading.Lock()
        self._effective_state = self._normalise_state(
            self._str_param("initial_effective_state")
        )
        self._armed = self._effective_state == WAITING_FOR_HOTWORD

        # ------------------------------------------------------------------
        # ALSA capture
        # ------------------------------------------------------------------
        self._stop_event = threading.Event()
        self._pcm: Optional[alsaaudio.PCM] = None
        self._audio_thread = threading.Thread(
            target=self._audio_loop,
            name="k9-hotword-audio",
            daemon=True,
        )
        self._audio_thread.start()

        self.get_logger().info(
            "Hotword node ready: "
            f"device={self.audio_device}, "
            f"{self.sample_rate} Hz mono, "
            f"{frame_ms} ms frames, "
            f"state={self._effective_state}, "
            f"armed={self._armed}"
        )

    # ----------------------------------------------------------------------
    # Parameter helpers
    # ----------------------------------------------------------------------
    def _str_param(self, name: str) -> str:
        return str(self.get_parameter(name).value)

    def _int_param(self, name: str) -> int:
        return int(self.get_parameter(name).value)

    def _float_param(self, name: str) -> float:
        return float(self.get_parameter(name).value)

    # ----------------------------------------------------------------------
    # Audio-state handling
    # ----------------------------------------------------------------------
    @staticmethod
    def _normalise_state(value: str) -> str:
        return value.strip().upper().replace(" ", "_")

    def _state_callback(self, msg: String) -> None:
        new_state = self._normalise_state(msg.data)

        with self._state_lock:
            old_state = self._effective_state
            if new_state == old_state:
                return

            self._effective_state = new_state

            # Reset the decoder whenever responsibility changes between KWS
            # and another audio mode. This prevents old partial phonemes from
            # causing a trigger when KWS is re-enabled.
            self.kws.reset_stream(self.kws_stream)

            if new_state == WAITING_FOR_HOTWORD:
                self._armed = True
                self.get_logger().info(
                    f"Audio state {old_state} -> {new_state}: hotword ARMED"
                )
            else:
                self._armed = False
                self.get_logger().info(
                    f"Audio state {old_state} -> {new_state}: hotword DISARMED"
                )

    # ----------------------------------------------------------------------
    # ALSA
    # ----------------------------------------------------------------------
    def _open_pcm(self) -> alsaaudio.PCM:
        self.get_logger().info(f"Opening ALSA capture device {self.audio_device}")

        pcm = alsaaudio.PCM(
            type=alsaaudio.PCM_CAPTURE,
            mode=alsaaudio.PCM_NORMAL,
            rate=self.sample_rate,
            channels=self.channels,
            format=alsaaudio.PCM_FORMAT_S16_LE,
            periodsize=self.period_size,
            periods=4,
            device=self.audio_device,
        )

        info = pcm.info()
        actual_rate = int(info.get("rate", self.sample_rate))
        actual_channels = int(info.get("channels", self.channels))
        actual_period = int(info.get("period_size", self.period_size))

        self.get_logger().info(
            "ALSA capture active: "
            f"rate={actual_rate}, channels={actual_channels}, "
            f"period={actual_period} frames"
        )

        if actual_rate != self.sample_rate:
            pcm.close()
            raise RuntimeError(
                f"ALSA realised {actual_rate} Hz; expected {self.sample_rate} Hz"
            )
        if actual_channels != self.channels:
            pcm.close()
            raise RuntimeError(
                f"ALSA realised {actual_channels} channels; expected {self.channels}"
            )

        return pcm

    def _audio_loop(self) -> None:
        try:
            self._pcm = self._open_pcm()

            while not self._stop_event.is_set():
                frames, data = self._pcm.read()
                #if frames > 0:
                #    self._frames_captured += 1
                if frames <= 0 or not data:
                    continue

                samples_i16 = np.frombuffer(data, dtype="<i2").copy()

                # Always publish the microphone audio, regardless of the
                # current behaviour-tree audio state.
                self._queue_latest_audio(samples_i16)

                # KWS is state-gated but the microphone itself remains open.
                with self._state_lock:
                    armed = self._armed

                if not armed:
                    continue

                samples_f32 = samples_i16.astype(np.float32) / 32768.0

                with self._state_lock:
                    # Re-check after obtaining the lock in case a state
                    # transition happened while converting samples.
                    if not self._armed:
                        continue

                    self.kws_stream.accept_waveform(
                        self.sample_rate,
                        samples_f32,
                    )

                    while self.kws.is_ready(self.kws_stream):
                        self.kws.decode_stream(self.kws_stream)

                        tokens = self.keyword_spotter.tokens(self.stream)

                        if tokens:
                            token_text = " ".join(tokens)

                            if token_text != self._last_debug_tokens:
                                self.get_logger().info(
                                    f"KWS hearing: {token_text}"
                                )
                                self._last_debug_tokens = token_text

                    result = self.kws.get_result(self.kws_stream)

                    if result:
                        # One spoken wake word = one ROS event. The stream is
                        # reset immediately as required by sherpa-onnx.
                        self.kws.reset_stream(self.kws_stream)
                        self._armed = False
                        self._queue_hotword(result)

        except Exception as exc:
            self.get_logger().error(f"Audio capture/KWS thread failed: {exc!r}")
            # Surface failure to the ROS process. The timer will notice this
            # flag and log it; the process can then be restarted by launch/
            # systemd rather than silently running without a microphone.
            self._stop_event.set()

        finally:
            if self._pcm is not None:
                try:
                    self._pcm.close()
                except Exception:
                    pass
                self._pcm = None

    def _queue_latest_audio(self, samples: np.ndarray) -> None:
        try:
            self._audio_queue.put_nowait(samples)
        except queue.Full:
            # For live audio, stale data is worse than losing one frame.
            try:
                self._audio_queue.get_nowait()
            except queue.Empty:
                pass
            try:
                self._audio_queue.put_nowait(samples)
            except queue.Full:
                pass

    def _queue_hotword(self, result: str) -> None:
        try:
            self._hotword_queue.put_nowait(result)
        except queue.Full:
            # The latch should make this practically impossible, but never
            # block the capture thread.
            pass

    # ----------------------------------------------------------------------
    # ROS publication
    # ----------------------------------------------------------------------
    def _drain_queues(self) -> None:
        # Publish all queued hotword events first.
        while True:
            try:
                result = self._hotword_queue.get_nowait()
            except queue.Empty:
                break

            self.get_logger().info(f"Hotword detected: {result}")
            msg = Bool()
            msg.data = True
            self.hotword_pub.publish(msg)

        # Drain a bounded number of audio frames per timer invocation. Under
        # normal operation there will only be one or zero.
        for _ in range(5):
            try:
                samples = self._audio_queue.get_nowait()
            except queue.Empty:
                break

            msg = AudioFrame()
            msg.stamp = self.get_clock().now().to_msg()
            msg.sample_rate = self.sample_rate
            msg.channels = self.channels
            msg.samples = samples.tolist()
            self.audio_pub.publish(msg)
            #self._frames_published += 1


        if self._stop_event.is_set() and not self._audio_thread.is_alive():
            # Throttle manually: report once.
            if not hasattr(self, "_thread_failure_reported"):
                self._thread_failure_reported = True
                self.get_logger().error(
                    "Audio capture thread has stopped. "
                    "Restart the hotword node after correcting the audio error."
                )

    #----------------------------------------------------------------------
    #def _report_audio_stats(self):
    #    self.get_logger().info(
    #        f"Audio stats: "
    #        f"captured={self._frames_captured}, "
    #        f"published={self._frames_published}, "
    #        f"queue={self._audio_queue.qsize()}"
    #     )
    #----------------------------------------------------------------------

    # ----------------------------------------------------------------------
    # Shutdown
    # ----------------------------------------------------------------------
    def destroy_node(self) -> bool:
        self.get_logger().info("Stopping hotword/audio capture...")
        self._stop_event.set()

        if self._audio_thread.is_alive():
            self._audio_thread.join(timeout=1.0)

        return super().destroy_node()

def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = HotwordNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

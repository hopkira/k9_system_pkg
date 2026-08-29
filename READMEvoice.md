# K9 sentence-driven Piper voice node

This replaces the NeuTTS synthesis path with K9's existing Piper model running
on CPU.  The model remains resident, and synthesis runs ahead of playback one
sentence at a time.

## Copy into the workspace

```bash
cp k9_system_pkg/voice_piper.py \
  ~/k9_ws/src/k9_system_pkg/k9_system_pkg/

cp config/voice_piper.yaml \
  ~/k9_ws/src/k9_system_pkg/config/

cp launch/voice_piper.launch.py \
  ~/k9_ws/src/k9_system_pkg/launch/
```

## setup.py

Add this console script inside `entry_points["console_scripts"]`:

```python
"voice_piper = k9_system_pkg.voice_piper:main",
```

Ensure the existing `data_files` installs `config/*.yaml` and `launch/*.launch.py`.
If setup.py lists files explicitly, add:

```python
"config/voice_piper.yaml"
"launch/voice_piper.launch.py"
```

to the corresponding package share entries.

No new k9_interfaces message/service is required: this version reuses the
existing `Speak` and `CancelSpeech` services.

## Piper venv

The node normally runs under K9's ROS Python environment, but imports Piper
from:

```text
/home/hopkira/tts_env/piper/.venv/lib/python3.12/site-packages
```

If that changes, set:

```bash
export K9_PIPER_SITE_PACKAGES=/new/path/lib/python3.12/site-packages
```

before launching.

## Build

```bash
cd ~/k9_ws
source /opt/ros/jazzy/setup.bash
source ~/k9_venv/bin/activate

colcon build \
  --symlink-install \
  --packages-select k9_system_pkg

source install/setup.bash
```

## Launch

```bash
ros2 launch k9_system_pkg voice_piper.launch.py
```

Expected startup includes model load, warm-up, then:

```text
Piper voice ready
```

## First sentence test

```bash
ros2 topic pub --once \
  /voice/sentence \
  std_msgs/msg/String \
  "{data: 'Affirmative. Systems are functioning normally.'}"
```

The node defensively splits multi-sentence input, but the intended contract is
that the conversation layer publishes each completed sentence immediately.

## Observe state

```bash
ros2 topic echo /voice/state
```

```bash
ros2 topic echo /voice/is_talking
```

```bash
ros2 topic echo /voice/rms_level
```

## Existing services

Inspect the installed service definition if needed:

```bash
ros2 interface show k9_interfaces_pkg/srv/Speak
ros2 interface show k9_interfaces_pkg/srv/CancelSpeech
```

Then the existing `/speak_now` and `/cancel_speech` clients should continue to
work unchanged.

## Architecture

```text
LLM token stream
      |
sentence boundary
      |
/voice/sentence
      |
sentence priority queue
      |
Piper CPU synthesis  <---- model stays resident
      |
audio FIFO
      |
pw-play / PipeWire
      |
speaker

While sentence N is playing, sentence N+1 can be synthesised.
```

`speak_now` increments a generation counter, clears queued ordinary speech and
terminates the active `pw-play` process.  `cancel_speech` does the same without
adding replacement speech.  Piper's blocking ONNX call cannot be interrupted
mid-sentence, so an in-flight cancelled synthesis is allowed to finish and its
result is then discarded as stale.

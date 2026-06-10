# RUDRA Voice AI v0.5

`rudra_voice` is the local voice-control package for RUDRA v4 on the Intel NUC with the Plantronics/Poly Calisto P610 USB speakerphone.

Voice can request motion, but it does not directly control motors. The voice node publishes approved requests to `/cmd_vel_voice_request`; `command_guard_node` clamps and times out those requests before publishing `/cmd_vel_safe`. In the integrated robot launch, `ps2_uno_to_teensy` forwards `/cmd_vel_safe` to the Teensy only while PS2/manual input is idle.

Physical E-stop has highest priority. PS2/manual control has priority over voice. If `/joy` or `/rudra/ps2_raw` shows manual input, voice motion is rejected.

## What Starts

The main launch file is:

```bash
ros2 launch rudra_voice rudra_voice.launch.py
```

For the whole robot, use the integrated launch from `rudra_base_bridge`:

```bash
ros2 launch rudra_base_bridge rudra_full.launch.py
```

That full launch keeps Ollama routing off unless you explicitly turn it on:

```bash
ros2 launch rudra_base_bridge rudra_full.launch.py \
  enable_ollama:=true \
  voice_use_llm_router:=true
```

It starts:

```text
voice_node
command_guard_node
```

The launch file also tries to run both nodes through:

```text
/home/rudra/Projects/RUDRAv4/.venv_voice/bin/python
```

That matters because ROS Lyrical installs console scripts with `#!/usr/bin/python3`, while the speech dependencies are installed in `.venv_voice`.

Expected launch line:

```text
RUDRA voice launch using Python: /home/rudra/Projects/RUDRAv4/.venv_voice/bin/python
```

If you do not see that line, the launch is probably using `/usr/bin/python3`, and the node may fail with `No module named 'sounddevice'`.

## One-Time Setup

Run these commands from the Intel NUC.

```bash
cd /home/rudra/Projects/RUDRAv4
```

Install OS audio and build dependencies:

```bash
sudo apt update
sudo apt install -y alsa-utils espeak-ng python3-venv python3-pip libportaudio2 portaudio19-dev wget unzip
```

Create the voice Python environment:

```bash
python3 -m venv --system-site-packages .venv_voice
source .venv_voice/bin/activate
python -m pip install --upgrade pip
python -m pip install vosk sounddevice numpy requests
```

Verify the venv can import the audio stack:

```bash
source .venv_voice/bin/activate
python -c "import sounddevice, vosk, requests; print('voice python ok')"
```

Download the Vosk model:

```bash
mkdir -p /home/rudra/Projects/RUDRAv4/models
cd /home/rudra/Projects/RUDRAv4/models
wget https://alphacephei.com/vosk/models/vosk-model-small-en-us-0.15.zip
unzip vosk-model-small-en-us-0.15.zip
```

Optional Ollama setup for natural language fallback:

```bash
curl -fsSL https://ollama.com/install.sh | sh
ollama pull qwen2.5:3b
```

Start Ollama from the repo:

```bash
cd /home/rudra/Projects/RUDRAv4
bash scripts/start_ollama.sh
```

## Audio Hardware Check

Plug the Plantronics/Poly Calisto P610 into the Intel NUC USB port.

Check that Linux sees it:

```bash
lsusb
arecord -l
aplay -l
```

Record and play a short clip:

```bash
arecord -f S16_LE -r 16000 -c 1 -d 5 /tmp/rudra_p610_test.wav
aplay /tmp/rudra_p610_test.wav
```

## Build

Use ROS first, then activate the voice venv:

```bash
cd /home/rudra/Projects/RUDRAv4
source /opt/ros/lyrical/setup.bash
source .venv_voice/bin/activate
colcon build --packages-select rudra_voice rudra_base_bridge
source install/setup.bash
```

## Launch

Normal launch:

```bash
cd /home/rudra/Projects/RUDRAv4
source /opt/ros/lyrical/setup.bash
source .venv_voice/bin/activate
source install/setup.bash
ros2 launch rudra_voice rudra_voice.launch.py
```

Useful explicit launch, if you want to force the venv Python:

```bash
ros2 launch rudra_voice rudra_voice.launch.py \
  use_venv:=true \
  venv_python:=/home/rudra/Projects/RUDRAv4/.venv_voice/bin/python
```

Debug launch without the venv:

```bash
ros2 launch rudra_voice rudra_voice.launch.py use_venv:=false
```

If you want to disable the language model even in the voice-only launch, add:

```bash
ros2 launch rudra_voice rudra_voice.launch.py use_llm_router:=false
```

The normal launch should show something like:

```text
RUDRA voice launch using Python: /home/rudra/Projects/RUDRAv4/.venv_voice/bin/python
Using audio input "Plantronics P610: USB Audio (hw:1,0)" and output "Plantronics P610: USB Audio (hw:1,0)".
Using Vosk command grammar with ...
RUDRA Voice AI v0.5 started.
```

You should also see Vosk model loading messages. Those are normal.

## Speech Output Quality

The default backend is Piper when the local binary and model are present:

```yaml
tts_backend: "piper"
tts_piper_model_path: "/home/rudra/Projects/RUDRAv4/models/piper/en_US-amy-medium.onnx"
tts_piper_executable: "/home/rudra/Projects/RUDRAv4/tools/piper/piper"
```

Install Piper locally without sudo:

```bash
cd /home/rudra/Projects/RUDRAv4
mkdir -p tools/piper models/piper
curl -L --fail -o /tmp/piper_linux_x86_64.tar.gz \
  https://github.com/rhasspy/piper/releases/download/2023.11.14-2/piper_linux_x86_64.tar.gz
tar -xzf /tmp/piper_linux_x86_64.tar.gz -C tools/piper --strip-components=1
curl -L --fail -o models/piper/en_US-amy-medium.onnx \
  https://huggingface.co/rhasspy/piper-voices/resolve/main/en/en_US/amy/medium/en_US-amy-medium.onnx
curl -L --fail -o models/piper/en_US-amy-medium.onnx.json \
  https://huggingface.co/rhasspy/piper-voices/resolve/main/en/en_US/amy/medium/en_US-amy-medium.onnx.json
```

Quick Piper test:

```bash
printf 'I am online. Ready for safe commands.\n' | \
  tools/piper/piper --model models/piper/en_US-amy-medium.onnx \
  --output_file /tmp/rudra_piper_test.wav
aplay /tmp/rudra_piper_test.wav
```

`espeak-ng` is still the fallback. It is reliable and offline, but it will not sound like Alexa or Siri. The fallback config tunes it to be a little smoother:

```yaml
tts_backend: "espeak"
tts_espeak_voice: "en-us+f3"
tts_espeak_rate_wpm: 155
tts_espeak_pitch: 45
tts_espeak_amplitude: 180
```

Quick test:

```bash
espeak-ng -v en-us+f3 -s 155 -p 45 -a 180 "I am online. Ready for safe commands."
```

If Piper or its model is missing, the node falls back to eSpeak.

## Speech Recognition Notes

The small Vosk model often mishears `RUDRA`. The voice node uses a constrained command grammar and accepts common wake phrase variants:

```text
hey rudra
rudra
hey robot
hello robot
robot
hey deidre
he redraw
here to draw
read dre
he bought
```

For first tests, speak short command phrases with a small pause after the wake phrase:

```text
Hey robot, status
Hello robot, stop
Hey RUDRA, move forward
Here to draw, move forward
```

Non-motion commands like `status` may run without the wake phrase. Motion commands still need a wake phrase or accepted wake alias.

## Test With Topics

Open a second terminal:

```bash
cd /home/rudra/Projects/RUDRAv4
source /opt/ros/lyrical/setup.bash
source .venv_voice/bin/activate
source install/setup.bash
```

Watch transcripts:

```bash
ros2 topic echo /rudra_voice/transcript
```

Each `ros2 topic echo` command blocks. Use separate terminals for transcript, intent, reply, and motion topics.

Say:

```text
Hey RUDRA, status
```

Watch parsed intents:

```bash
ros2 topic echo /rudra_voice/intent
```

Say:

```text
Hey RUDRA, move forward
Hey RUDRA, stop
```

Watch replies:

```bash
ros2 topic echo /rudra_voice/reply
```

Watch raw voice motion requests:

```bash
ros2 topic echo /cmd_vel_voice_request
```

Watch guarded safe output:

```bash
ros2 topic echo /cmd_vel_safe
```

## Test Without Voice

Use this to test the guard path without the microphone:

```bash
ros2 topic pub /cmd_vel_voice_request geometry_msgs/msg/Twist "{linear: {x: 0.08}, angular: {z: 0.0}}" -1
```

`/cmd_vel_safe` should publish the clamped command, then publish zero after the timeout.

Test stop:

```bash
ros2 topic pub /rudra_voice/intent std_msgs/msg/String "{data: stop}" -1
```

`/cmd_vel_safe` should publish zero.

Test emergency stop:

```bash
ros2 topic pub /rudra_voice/intent std_msgs/msg/String "{data: emergency_stop}" -1
```

`/cmd_vel_safe` should publish zero and the guard will latch emergency stop for this node run.

## Example Voice Phrases

```text
Hey RUDRA, status
Hey RUDRA, move forward
Hey RUDRA, move backward
Hey RUDRA, turn left
Hey RUDRA, turn right
Hey RUDRA, stop
Hey RUDRA, emergency stop
Hey RUDRA, check lidar
Hey RUDRA, check Teensy
Hey RUDRA, check controller
Hey RUDRA, run self test
```

Unsafe or open-ended requests should be rejected:

```text
Hey RUDRA, drive forward forever
Hey RUDRA, go full speed
Hey RUDRA, follow me
```

## Common Failures

`No module named 'sounddevice'`

This means ROS launched with a Python that cannot see the voice packages. Confirm launch is using the venv:

```bash
ros2 launch rudra_voice rudra_voice.launch.py
```

Look for:

```text
RUDRA voice launch using Python: /home/rudra/Projects/RUDRAv4/.venv_voice/bin/python
```

If missing, rebuild and source the workspace:

```bash
source /opt/ros/lyrical/setup.bash
colcon build --packages-select rudra_voice
source install/setup.bash
```

`PortAudio library not found`

Install PortAudio:

```bash
sudo apt install -y libportaudio2 portaudio19-dev
```

`externally-managed-environment`

Ubuntu is blocking pip installs into `/usr/bin/python3`. Do not fight this for normal RUDRA voice use. Use `.venv_voice`; the launch file is designed for that.

`Vosk model not found`

Make sure this directory exists:

```text
/home/rudra/Projects/RUDRAv4/models/vosk-model-small-en-us-0.15
```

`Plantronics/Poly Calisto P610 was not found by name`

The node will fall back to default audio, but voice quality may be wrong. Check:

```bash
arecord -l
aplay -l
```

Then unplug and replug the P610.

`Local language model is not responding`

Ollama is optional. Deterministic commands still work. To enable LLM routing:

```bash
ollama serve
ollama pull qwen2.5:3b
```

## Safety Notes

Approved motion skills are:

```text
move_forward_slow
move_backward_slow
turn_left_slow
turn_right_slow
stop
emergency_stop
```

Default limits:

```text
forward: 0.08 m/s for 1 second
reverse: -0.06 m/s for 1 second
turn left: 0.35 rad/s for 1 second
turn right: -0.35 rad/s for 1 second
```

The package does not publish raw `/cmd_vel`. The voice node only requests motion on `/cmd_vel_voice_request`; the guard publishes `/cmd_vel_safe`.

Before connecting this to the motor bridge, make the base bridge consume `/cmd_vel_safe`. Do not remap voice directly to the Teensy, Arduino Uno, Sabertooth, serial drivers, or raw motor topics.

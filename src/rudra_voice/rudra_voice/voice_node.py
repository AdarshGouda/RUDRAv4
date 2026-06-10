"""RUDRA Voice AI v0.5 ROS2 node."""

from __future__ import annotations

import json
import threading
import sys
import time
from typing import Optional

from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

from .audio_devices import AudioDeviceSelection, find_preferred_audio_devices
from .intent_parser import (
    APPROVED_MOTION_SKILLS,
    APPROVED_NON_MOTION_SKILLS,
    DEFAULT_WAKE_ALIASES,
    IntentResult,
    build_vosk_grammar_phrases,
    parse_deterministic_intent,
    strip_wake_phrase,
    validate_intent_payload,
)
from .ollama_router import OllamaRouter
from .stt_vosk import VoskSpeechToText
from .tts import make_tts_backend


DEFAULT_MOTION_DURATION_SEC = 5.0


MOTION_COMMANDS = {
    'move_forward_slow': (0.95, 0.0),
    'move_backward_slow': (-0.95, 0.0),
    'turn_left_slow': (0.0, 0.95),
    'turn_right_slow': (0.0, -0.95),
    'stop': (0.0, 0.0),
    'emergency_stop': (0.0, 0.0),
}


class RudraVoiceNode(Node):
    """Listen for wake-phrase commands and publish validated RUDRA intents."""

    def __init__(self) -> None:
        super().__init__('voice_node')

        self.declare_parameter('wake_phrase', 'hey rudra')
        self.declare_parameter('wake_phrase_aliases', list(DEFAULT_WAKE_ALIASES))
        self.declare_parameter('use_vosk_command_grammar', True)
        self.declare_parameter('allow_non_motion_without_wake', True)
        self.declare_parameter('sample_rate', 16000)
        self.declare_parameter(
            'vosk_model_path',
            '~/Projects/RUDRAv4/models/vosk-model-small-en-us-0.15',
        )
        self.declare_parameter('ollama_url', 'http://localhost:11434/api/chat')
        self.declare_parameter('ollama_model', 'qwen2.5:3b')
        self.declare_parameter('ollama_timeout_sec', 15.0)
        self.declare_parameter('use_llm_router', True)
        self.declare_parameter('tts_backend', 'espeak')
        self.declare_parameter('tts_espeak_voice', 'en-us+f3')
        self.declare_parameter('tts_espeak_rate_wpm', 155)
        self.declare_parameter('tts_espeak_pitch', 45)
        self.declare_parameter('tts_espeak_amplitude', 180)
        self.declare_parameter('tts_piper_model_path', '')
        self.declare_parameter('tts_piper_executable', 'piper')
        self.declare_parameter(
            'audio_device_keywords',
            ['plantronics', 'poly', 'calisto', 'usb audio'],
        )
        self.declare_parameter('listen_while_speaking', False)
        self.declare_parameter('motion.enable_voice_motion', True)
        self.declare_parameter('motion.publish_topic', '/cmd_vel_voice_request')
        self.declare_parameter('motion.max_linear_x', 1.0)
        self.declare_parameter('motion.max_reverse_x', -1.0)
        self.declare_parameter('motion.max_angular_z', 3.0)
        self.declare_parameter(
            'motion.default_motion_duration_sec',
            DEFAULT_MOTION_DURATION_SEC,
        )

        self.wake_phrase = str(self.get_parameter('wake_phrase').value)
        self.wake_aliases = [
            str(value)
            for value in self.get_parameter('wake_phrase_aliases').value
        ]
        self.use_vosk_command_grammar = bool(
            self.get_parameter('use_vosk_command_grammar').value
        )
        self.allow_non_motion_without_wake = bool(
            self.get_parameter('allow_non_motion_without_wake').value
        )
        self.sample_rate = int(self.get_parameter('sample_rate').value)
        self.vosk_model_path = str(self.get_parameter('vosk_model_path').value)
        self.use_llm_router = bool(self.get_parameter('use_llm_router').value)
        self.ollama_timeout_sec = float(self.get_parameter('ollama_timeout_sec').value)
        self.listen_while_speaking = bool(
            self.get_parameter('listen_while_speaking').value
        )
        self.voice_motion_enabled = bool(
            self.get_parameter('motion.enable_voice_motion').value
        )
        self.motion_topic = str(self.get_parameter('motion.publish_topic').value)
        self.max_linear_x = abs(float(self.get_parameter('motion.max_linear_x').value))
        self.max_reverse_x = -abs(float(self.get_parameter('motion.max_reverse_x').value))
        self.max_angular_z = abs(float(self.get_parameter('motion.max_angular_z').value))

        self.transcript_pub = self.create_publisher(String, '/rudra_voice/transcript', 10)
        self.intent_pub = self.create_publisher(String, '/rudra_voice/intent', 10)
        self.reply_pub = self.create_publisher(String, '/rudra_voice/reply', 10)
        self.llm_raw_pub = self.create_publisher(String, '/rudra_voice/llm_raw', 10)
        self.status_pub = self.create_publisher(String, '/rudra_voice/status', 10)
        self.motion_request_pub = self.create_publisher(
            String,
            '/rudra_voice/motion_request',
            10,
        )
        self.voice_twist_pub = self.create_publisher(Twist, self.motion_topic, 10)

        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_profile_sensor_data,
        )
        self.odom_sub = self.create_subscription(
            Odometry,
            '/rudra/wheel_odom',
            self.odom_callback,
            10,
        )
        self.diagnostics_sub = self.create_subscription(
            DiagnosticArray,
            '/diagnostics',
            self.diagnostics_callback,
            10,
        )

        self.last_scan_time = 0.0
        self.last_odom_time = 0.0
        self.last_diagnostics_time = 0.0
        self.last_diagnostics_summary = 'No diagnostics received.'
        self.is_speaking = False
        self.shutdown_requested = False
        self.device_selection = self._select_audio_devices()

        self.tts = make_tts_backend(
            backend=str(self.get_parameter('tts_backend').value),
            output_device_name=self.device_selection.output_name,
            speaking_callback=self.set_speaking,
            espeak_voice=str(self.get_parameter('tts_espeak_voice').value),
            espeak_rate_wpm=int(self.get_parameter('tts_espeak_rate_wpm').value),
            espeak_pitch=int(self.get_parameter('tts_espeak_pitch').value),
            espeak_amplitude=int(self.get_parameter('tts_espeak_amplitude').value),
            piper_model_path=str(self.get_parameter('tts_piper_model_path').value),
            piper_executable=str(self.get_parameter('tts_piper_executable').value),
        )
        self.router = OllamaRouter(
            url=str(self.get_parameter('ollama_url').value),
            model=str(self.get_parameter('ollama_model').value),
            timeout_sec=self.ollama_timeout_sec,
        )

        self.status_pub.publish(String(data='RUDRA voice node starting.'))
        self.listener_thread = threading.Thread(target=self.listen_loop, daemon=True)
        self.listener_thread.start()
        self.get_logger().info('RUDRA Voice AI v0.5 started.')

    def _select_audio_devices(self) -> AudioDeviceSelection:
        keywords_param = self.get_parameter('audio_device_keywords').value
        keywords = [str(value) for value in keywords_param]
        try:
            selection = find_preferred_audio_devices(keywords)
        except Exception as exc:  # noqa: BLE001 - audio libraries are optional at launch.
            self.get_logger().warning(f'Audio device query failed: {exc}')
            return AudioDeviceSelection(None, None, 'default input', 'default output', False)

        if selection.found_preferred:
            self.get_logger().info(
                f'Using audio input "{selection.input_name}" and output '
                f'"{selection.output_name}".'
            )
        else:
            self.get_logger().warning(
                'Plantronics/Poly Calisto P610 was not found by name. '
                'Falling back to default audio devices.'
            )
        return selection

    def set_speaking(self, speaking: bool) -> None:
        self.is_speaking = speaking

    def scan_callback(self, msg: LaserScan) -> None:
        self.last_scan_time = time.monotonic()

    def odom_callback(self, msg: Odometry) -> None:
        self.last_odom_time = time.monotonic()

    def diagnostics_callback(self, msg: DiagnosticArray) -> None:
        self.last_diagnostics_time = time.monotonic()
        if msg.status:
            status = msg.status[0]
            self.last_diagnostics_summary = f'{status.name}: {status.message}'

    def listen_loop(self) -> None:
        """Initialize Vosk and process microphone transcripts."""
        try:
            grammar_phrases = None
            if self.use_vosk_command_grammar:
                grammar_phrases = build_vosk_grammar_phrases(
                    self.wake_phrase,
                    self.wake_aliases,
                )
                self.get_logger().info(
                    f'Using Vosk command grammar with {len(grammar_phrases)} phrases.'
                )
            stt = VoskSpeechToText(
                model_path=self.vosk_model_path,
                sample_rate=self.sample_rate,
                input_device=self.device_selection.input_device,
                grammar_phrases=grammar_phrases,
            )
        except Exception as exc:  # noqa: BLE001 - keep node alive without STT.
            message = (
                f'Voice STT unavailable from {sys.executable}: {exc}. '
                'Install sounddevice, vosk, and PortAudio for the Python used by ROS.'
            )
            self.get_logger().error(message)
            self.status_pub.publish(String(data=message))
            return

        for transcript in stt.transcripts():
            if self.shutdown_requested or not rclpy.ok():
                break
            if self.is_speaking and not self.listen_while_speaking:
                continue
            self.process_transcript(transcript)

    def process_transcript(self, transcript: str) -> None:
        """Process one final speech transcript."""
        self.transcript_pub.publish(String(data=transcript))
        wake_detected, command_text = strip_wake_phrase(
            transcript,
            self.wake_phrase,
            self.wake_aliases,
        )
        if not wake_detected:
            if self.allow_non_motion_without_wake:
                direct_intent = parse_deterministic_intent(transcript)
                if (
                    direct_intent is not None
                    and direct_intent.skill in APPROVED_NON_MOTION_SKILLS
                ):
                    self.execute_intent(direct_intent)
            return
        if not command_text:
            self._publish_status('wake phrase detected without command')
            return

        intent = parse_deterministic_intent(command_text)
        if intent is None:
            if self.use_llm_router:
                intent, raw_text, error = self.router.route(command_text)
                if raw_text:
                    self.llm_raw_pub.publish(String(data=raw_text))
                if error:
                    self._publish_status(f'Ollama router issue: {error}')
            else:
                intent = IntentResult(
                    skill='chat_only',
                    reply='I heard you, but I only know the safe command set right now.',
                    source='no_llm',
                )

        self.execute_intent(intent)

    def execute_intent(self, intent: IntentResult) -> None:
        """Publish intent, reply, status, and safe motion request if allowed."""
        validated = validate_intent_payload(intent.to_json_dict(), source=intent.source)
        self.intent_pub.publish(String(data=validated.skill))
        self.reply_pub.publish(String(data=validated.reply))
        self._publish_status(json.dumps(validated.to_json_dict()))

        if validated.skill in APPROVED_MOTION_SKILLS:
            self.publish_motion_request(validated)
        else:
            self.handle_non_motion_skill(validated)

        if validated.reply:
            self.tts.speak_async(validated.reply)

    def publish_motion_request(self, intent: IntentResult) -> None:
        """Publish only approved motion requests to the guarded voice topic."""
        self.motion_request_pub.publish(String(data=intent.skill))
        if not self.voice_motion_enabled:
            self._publish_status('voice motion disabled by configuration')
            return
        linear_x, angular_z = self._motion_command_for_skill(intent.skill)
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        self.voice_twist_pub.publish(msg)

    def _motion_command_for_skill(self, skill: str) -> tuple[float, float]:
        if skill == 'move_forward_slow':
            return self.max_linear_x, 0.0
        if skill == 'move_backward_slow':
            return self.max_reverse_x, 0.0
        if skill == 'turn_left_slow':
            return 0.0, self.max_angular_z
        if skill == 'turn_right_slow':
            return 0.0, -self.max_angular_z
        return MOTION_COMMANDS.get(skill, (0.0, 0.0))

    def handle_non_motion_skill(self, intent: IntentResult) -> None:
        """Publish useful status for non-motion voice skills."""
        now = time.monotonic()
        if intent.skill == 'status':
            scan_age = self._age_text(now, self.last_scan_time)
            odom_age = self._age_text(now, self.last_odom_time)
            diagnostics_age = self._age_text(now, self.last_diagnostics_time)
            self._publish_status(
                f'voice online; scan {scan_age}; odom {odom_age}; '
                f'diagnostics {diagnostics_age}'
            )
        elif intent.skill == 'check_lidar':
            self._publish_status(f'lidar scan {self._age_text(now, self.last_scan_time)}')
        elif intent.skill == 'check_teensy':
            self._publish_status(f'teensy odom {self._age_text(now, self.last_odom_time)}')
        elif intent.skill == 'check_controller':
            self._publish_status('PS2 controller status is monitored by command_guard_node.')
        elif intent.skill == 'run_self_test':
            self._publish_status(
                f'self test: scan {self._age_text(now, self.last_scan_time)}, '
                f'odom {self._age_text(now, self.last_odom_time)}, '
                f'diagnostics {self.last_diagnostics_summary}'
            )
        elif intent.skill == 'shutdown':
            self._publish_status('shutdown voice command received; manual shutdown is required')

    @staticmethod
    def _age_text(now: float, timestamp: float) -> str:
        if timestamp <= 0.0:
            return 'not seen'
        return f'{now - timestamp:.1f}s ago'

    def _publish_status(self, message: str) -> None:
        self.status_pub.publish(String(data=message))
        self.get_logger().info(message)


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = RudraVoiceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown_requested = True
        try:
            node.destroy_node()
        except (KeyboardInterrupt, RuntimeError) as exc:
            if rclpy.ok():
                node.get_logger().debug(f'Node destroy interrupted during shutdown: {exc}')
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

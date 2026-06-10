"""Guard voice motion requests before publishing safe velocity commands."""

from __future__ import annotations

import math
import time
from typing import Optional

from geometry_msgs.msg import Twist
import rclpy
from rclpy._rclpy_pybind11 import RCLError
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import String

from .intent_parser import APPROVED_MOTION_SKILLS
from .tts import make_tts_backend


class CommandGuardNode(Node):
    """Clamp and timeout voice motion before it reaches the safe motion topic."""

    def __init__(self) -> None:
        super().__init__('command_guard_node')

        self.declare_parameter('motion.publish_topic', '/cmd_vel_voice_request')
        self.declare_parameter('motion.safe_output_topic', '/cmd_vel_safe')
        self.declare_parameter('motion.max_linear_x', 0.08)
        self.declare_parameter('motion.max_reverse_x', -0.06)
        self.declare_parameter('motion.max_angular_z', 0.35)
        self.declare_parameter('motion.default_motion_duration_sec', 1.0)
        self.declare_parameter('motion.require_timeout', True)
        self.declare_parameter('motion.ps2_manual_override_enabled', True)
        self.declare_parameter('motion.manual_override_timeout_sec', 0.35)
        self.declare_parameter('motion.manual_override_deadband', 0.05)
        self.declare_parameter('joy_topic', '/joy')
        self.declare_parameter('ps2_raw_topic', '/rudra/ps2_raw')
        self.declare_parameter('ps2_raw_axis_center', 127)
        self.declare_parameter('ps2_raw_deadband', 15)

        self.voice_request_topic = str(self.get_parameter('motion.publish_topic').value)
        self.safe_output_topic = str(self.get_parameter('motion.safe_output_topic').value)
        self.max_linear_x = abs(float(self.get_parameter('motion.max_linear_x').value))
        self.max_reverse_x = -abs(float(self.get_parameter('motion.max_reverse_x').value))
        self.max_angular_z = abs(float(self.get_parameter('motion.max_angular_z').value))
        self.motion_duration_sec = float(
            self.get_parameter('motion.default_motion_duration_sec').value
        )
        self.require_timeout = bool(self.get_parameter('motion.require_timeout').value)
        self.ps2_override_enabled = bool(
            self.get_parameter('motion.ps2_manual_override_enabled').value
        )
        self.manual_override_timeout_sec = float(
            self.get_parameter('motion.manual_override_timeout_sec').value
        )
        self.manual_override_deadband = float(
            self.get_parameter('motion.manual_override_deadband').value
        )
        self.ps2_raw_axis_center = int(self.get_parameter('ps2_raw_axis_center').value)
        self.ps2_raw_deadband = int(self.get_parameter('ps2_raw_deadband').value)

        self.safe_pub = self.create_publisher(Twist, self.safe_output_topic, 10)
        self.reply_pub = self.create_publisher(String, '/rudra_voice/reply', 10)
        self.status_pub = self.create_publisher(String, '/rudra_voice/status', 10)

        self.request_sub = self.create_subscription(
            Twist,
            self.voice_request_topic,
            self.motion_request_callback,
            10,
        )
        self.intent_sub = self.create_subscription(
            String,
            '/rudra_voice/intent',
            self.intent_callback,
            10,
        )
        self.joy_sub = self.create_subscription(
            Joy,
            str(self.get_parameter('joy_topic').value),
            self.joy_callback,
            10,
        )
        self.ps2_raw_sub = self.create_subscription(
            String,
            str(self.get_parameter('ps2_raw_topic').value),
            self.ps2_raw_callback,
            10,
        )

        self.active_until = 0.0
        self.last_motion = Twist()
        self.last_intent = ''
        self.emergency_stop_latched = False
        self.last_manual_input_time = 0.0
        self.tts = make_tts_backend(backend='espeak')

        self.timer = self.create_timer(0.05, self.loop)

        self.get_logger().info(
            f'Voice guard: {self.voice_request_topic} -> {self.safe_output_topic}'
        )

    def intent_callback(self, msg: String) -> None:
        self.last_intent = msg.data.strip()
        if self.last_intent == 'emergency_stop':
            self.emergency_stop_latched = True
            self.publish_zero('emergency_stop')
        elif self.last_intent == 'stop':
            self.publish_zero('stop')
        elif self.last_intent and self.last_intent not in APPROVED_MOTION_SKILLS:
            self.active_until = 0.0

    def joy_callback(self, msg: Joy) -> None:
        if not self.ps2_override_enabled:
            return
        axes_active = any(abs(axis) > self.manual_override_deadband for axis in msg.axes)
        buttons_active = any(button != 0 for button in msg.buttons)
        if axes_active or buttons_active:
            self.last_manual_input_time = time.monotonic()

    def ps2_raw_callback(self, msg: String) -> None:
        if not self.ps2_override_enabled:
            return
        parsed = self._parse_ps2_raw(msg.data)
        if parsed is None:
            return
        select, axes = parsed
        axes_active = any(
            abs(axis - self.ps2_raw_axis_center) > self.ps2_raw_deadband
            for axis in axes
        )
        if select or axes_active:
            self.last_manual_input_time = time.monotonic()

    def _parse_ps2_raw(self, line: str) -> tuple[bool, list[int]] | None:
        parts = line.strip().split(',')
        if len(parts) not in (6, 7) or parts[0] != 'J':
            return None
        try:
            select = bool(int(parts[1]))
            axes = [int(value) for value in parts[2:6]]
        except ValueError:
            return None
        return select, axes

    def manual_override_active(self) -> bool:
        if not self.ps2_override_enabled:
            return False
        return (time.monotonic() - self.last_manual_input_time) <= self.manual_override_timeout_sec

    def motion_request_callback(self, msg: Twist) -> None:
        if self.emergency_stop_latched:
            self.publish_zero('emergency_stop_latched')
            return
        if self.manual_override_active():
            reply = 'PS2 manual control is active. Voice motion rejected.'
            self.reply_pub.publish(String(data=reply))
            self.status_pub.publish(String(data=reply))
            self.get_logger().warning(reply)
            self.tts.speak_async(reply)
            self.publish_zero('manual_override')
            return

        clamped = self.clamp_twist(msg)
        if self.is_zero(clamped):
            self.publish_zero('zero_request')
            return

        self.last_motion = clamped
        now = time.monotonic()
        self.active_until = now + max(0.0, self.motion_duration_sec)
        self.safe_pub.publish(clamped)
        self.status_pub.publish(String(data='voice motion accepted'))

    def clamp_twist(self, msg: Twist) -> Twist:
        safe = Twist()
        safe.linear.x = min(max(float(msg.linear.x), self.max_reverse_x), self.max_linear_x)
        safe.angular.z = min(
            max(float(msg.angular.z), -self.max_angular_z),
            self.max_angular_z,
        )
        return safe

    @staticmethod
    def is_zero(msg: Twist) -> bool:
        return math.isclose(msg.linear.x, 0.0, abs_tol=1e-6) and math.isclose(
            msg.angular.z,
            0.0,
            abs_tol=1e-6,
        )

    def publish_zero(self, reason: str) -> None:
        self.active_until = 0.0
        self.last_motion = Twist()
        if not rclpy.ok():
            return
        try:
            self.safe_pub.publish(Twist())
            self.status_pub.publish(String(data=f'voice guard zero: {reason}'))
        except (RCLError, RuntimeError) as exc:
            self.get_logger().debug(f'Skipping zero publish during shutdown: {exc}')

    def loop(self) -> None:
        if self.active_until <= 0.0:
            return
        if self.require_timeout and time.monotonic() >= self.active_until:
            self.publish_zero('timeout')


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = CommandGuardNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.publish_zero('shutdown')
        try:
            node.destroy_node()
        except (KeyboardInterrupt, RCLError, RuntimeError) as exc:
            if rclpy.ok():
                node.get_logger().debug(f'Node destroy interrupted during shutdown: {exc}')
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

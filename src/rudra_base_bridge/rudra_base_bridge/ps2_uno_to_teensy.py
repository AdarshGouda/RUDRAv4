"""Bridge Arduino Uno PS2 serial packets to Teensy Sabertooth drive commands."""

from __future__ import annotations

import time
from typing import NamedTuple, Optional, Tuple

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .obstacle_guard import LidarObstacleGuard
from .serial_utils import apply_deadband, clamp, clamp_int, LineSerial
from .teensy_telemetry import TeensyTelemetryBridge


class Ps2Packet(NamedTuple):
    select: bool
    ry: int
    rx: int
    ly: int
    lx: int
    obstacle_guard_enabled: Optional[bool] = None


class Ps2UnoToTeensy(Node):
    """Read PS2 packets from Uno and send D,left,right,enable to Teensy."""

    def __init__(self) -> None:
        super().__init__('ps2_uno_to_teensy')

        uno_port = '/dev/serial/by-id/usb-Arduino__www.arduino.cc__0043_959313232323510182C0-if00'
        teensy_port = '/dev/serial/by-id/usb-Teensyduino_USB_Serial_9210670-if00'

        self.declare_parameter('uno_port', uno_port)
        self.declare_parameter('teensy_port', teensy_port)
        self.declare_parameter('baud', 115200)
        self.declare_parameter('timer_period_sec', 0.02)
        self.declare_parameter('command_timeout_sec', 0.30)
        self.declare_parameter('publish_cmd_vel', True)
        self.declare_parameter('select_as_enable', True)
        self.declare_parameter('axis_center', 127)
        self.declare_parameter('axis_range', 127)
        self.declare_parameter('deadband', 15)
        self.declare_parameter('wheelbase_m', 0.29)
        self.declare_parameter('max_sabertooth_cmd', 127)
        self.declare_parameter('max_linear_speed_mps', 1.20)
        self.declare_parameter('max_angular_speed_radps', 2.50)
        self.declare_parameter('send_stop_when_disabled', True)
        self.declare_parameter('enable_voice_cmd_vel', True)
        self.declare_parameter('voice_cmd_vel_topic', '/cmd_vel_safe')
        self.declare_parameter('voice_cmd_timeout_sec', 0.30)
        self.declare_parameter('manual_override_timeout_sec', 0.35)
        self.declare_parameter('ps2_obstacle_latch_enabled', True)
        self.declare_parameter('ps2_obstacle_control_enabled', False)
        self.declare_parameter('ps2_obstacle_control_hold_sec', 0.80)
        self.declare_parameter('ps2_obstacle_control_threshold', 90)
        self.declare_parameter('ps2_obstacle_control_require_stopped', True)

        self.uno_port = str(self.get_parameter('uno_port').value)
        self.teensy_port = str(self.get_parameter('teensy_port').value)
        self.baud = int(self.get_parameter('baud').value)
        self.command_timeout_sec = float(self.get_parameter('command_timeout_sec').value)
        self.publish_cmd_vel_enabled = bool(self.get_parameter('publish_cmd_vel').value)
        self.select_as_enable = bool(self.get_parameter('select_as_enable').value)
        self.axis_center = int(self.get_parameter('axis_center').value)
        self.axis_range = max(1, int(self.get_parameter('axis_range').value))
        self.deadband = int(self.get_parameter('deadband').value)
        self.wheelbase_m = float(self.get_parameter('wheelbase_m').value)
        self.max_sabertooth_cmd = int(self.get_parameter('max_sabertooth_cmd').value)
        self.max_linear_speed_mps = float(self.get_parameter('max_linear_speed_mps').value)
        self.max_angular_speed_radps = float(self.get_parameter('max_angular_speed_radps').value)
        self.send_stop_when_disabled = bool(self.get_parameter('send_stop_when_disabled').value)
        self.enable_voice_cmd_vel = bool(self.get_parameter('enable_voice_cmd_vel').value)
        self.voice_cmd_vel_topic = str(self.get_parameter('voice_cmd_vel_topic').value)
        self.voice_cmd_timeout_sec = float(
            self.get_parameter('voice_cmd_timeout_sec').value
        )
        self.manual_override_timeout_sec = float(
            self.get_parameter('manual_override_timeout_sec').value
        )
        self.ps2_obstacle_latch_enabled = bool(
            self.get_parameter('ps2_obstacle_latch_enabled').value
        )
        self.ps2_obstacle_control_enabled = bool(
            self.get_parameter('ps2_obstacle_control_enabled').value
        )
        self.ps2_obstacle_control_hold_sec = float(
            self.get_parameter('ps2_obstacle_control_hold_sec').value
        )
        self.ps2_obstacle_control_threshold = int(
            self.get_parameter('ps2_obstacle_control_threshold').value
        )
        self.ps2_obstacle_control_require_stopped = bool(
            self.get_parameter('ps2_obstacle_control_require_stopped').value
        )

        self.uno = LineSerial(self.uno_port, self.baud)
        self.teensy = LineSerial(self.teensy_port, self.baud)
        self.obstacle_guard = LidarObstacleGuard(self)
        self.teensy_telemetry = TeensyTelemetryBridge(self)

        self.raw_pub = self.create_publisher(String, '/rudra/ps2_raw', 10)
        self.drive_pub = self.create_publisher(String, '/rudra/drive_cmd', 10)
        self.ack_pub = self.create_publisher(String, '/rudra/teensy_ack', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.voice_cmd_sub = None
        if self.enable_voice_cmd_vel:
            self.voice_cmd_sub = self.create_subscription(
                Twist,
                self.voice_cmd_vel_topic,
                self.voice_cmd_callback,
                10,
            )

        self.last_valid_packet_time = 0.0
        self.last_sent_stop = 0.0
        self.last_enable = False
        self.last_manual_input_time = 0.0
        self.last_voice_cmd_time = 0.0
        self.last_voice_cmd = Twist()
        self.obstacle_control_target: Optional[bool] = None
        self.obstacle_control_started = 0.0
        self.obstacle_control_latched = False

        period = float(self.get_parameter('timer_period_sec').value)
        self.timer = self.create_timer(period, self.loop)

        self.get_logger().info(f'Uno port: {self.uno_port}')
        self.get_logger().info(f'Teensy port: {self.teensy_port}')
        self.get_logger().info('Expected Uno line: J,select,ry,rx,ly,lx[,guard]')
        self.get_logger().info('Sending Teensy line: D,left,right,enable')
        if self.enable_voice_cmd_vel:
            self.get_logger().info(
                f'Voice safe motion enabled: {self.voice_cmd_vel_topic} -> Teensy '
                'when PS2 input is idle'
            )
        if self.ps2_obstacle_latch_enabled:
            self.get_logger().info(
                'PS2 obstacle guard latch: START toggles guard on the Uno'
            )
        if self.ps2_obstacle_control_enabled:
            self.get_logger().info(
                'PS2 obstacle guard chord: hold SELECT + right-stick up '
                'to enable, SELECT + right-stick down to disable'
            )

    def parse_uno_line(self, line: str) -> Optional[Ps2Packet]:
        line = line.strip()
        if not line.startswith('J,'):
            return None
        parts = line.split(',')
        if len(parts) not in (6, 7):
            return None
        try:
            select = bool(int(parts[1]))
            ry = int(parts[2])
            rx = int(parts[3])
            ly = int(parts[4])
            lx = int(parts[5])
            guard_enabled = bool(int(parts[6])) if len(parts) == 7 else None
        except ValueError:
            return None
        return Ps2Packet(select, ry, rx, ly, lx, guard_enabled)

    def axis_to_command(self, value: int, invert: bool = False) -> int:
        centered = value - self.axis_center
        if invert:
            centered = -centered
        scaled = int(round((centered / float(self.axis_range)) * self.max_sabertooth_cmd))
        return apply_deadband(
            clamp_int(scaled, -self.max_sabertooth_cmd, self.max_sabertooth_cmd),
            self.deadband,
        )

    def mix_drive(self, select: bool, rx: int, ly: int) -> Tuple[int, int, bool, int, int]:
        throttle = self.axis_to_command(ly, invert=True)
        steering = self.axis_to_command(rx, invert=True)

        left = throttle - steering
        right = throttle + steering
        left = clamp_int(left, -self.max_sabertooth_cmd, self.max_sabertooth_cmd)
        right = clamp_int(right, -self.max_sabertooth_cmd, self.max_sabertooth_cmd)

        enable = select if self.select_as_enable else True
        if not enable:
            left = 0
            right = 0
        return left, right, enable, throttle, steering

    def publish_cmd_vel(self, throttle: int, steering: int, enable: bool) -> None:
        msg = Twist()
        if enable:
            normalized_throttle = clamp(throttle / float(self.max_sabertooth_cmd), -1.0, 1.0)
            normalized_steering = clamp(steering / float(self.max_sabertooth_cmd), -1.0, 1.0)
            msg.linear.x = normalized_throttle * self.max_linear_speed_mps
            # Positive angular.z should correspond to left turn.
            # Tune sign in config/node if needed.
            msg.angular.z = normalized_steering * self.max_angular_speed_radps
        self.cmd_vel_pub.publish(msg)

    def voice_cmd_callback(self, msg: Twist) -> None:
        self.last_voice_cmd = msg
        self.last_voice_cmd_time = time.monotonic()

    def ps2_manual_input_active(self, packet: Ps2Packet) -> bool:
        axes = [packet.ry, packet.rx, packet.ly, packet.lx]
        axes_active = any(abs(axis - self.axis_center) > self.deadband for axis in axes)
        return packet.select or axes_active

    def manual_override_active(self, now: float) -> bool:
        return (now - self.last_manual_input_time) <= self.manual_override_timeout_sec

    def voice_cmd_active(self, now: float) -> bool:
        if not self.enable_voice_cmd_vel:
            return False
        return (now - self.last_voice_cmd_time) <= self.voice_cmd_timeout_sec

    def twist_to_drive(self, msg: Twist) -> tuple[int, int, bool]:
        v = self.obstacle_guard.filter_linear_x(float(msg.linear.x))
        wz = float(msg.angular.z)

        left_mps = v - wz * (self.wheelbase_m / 2.0)
        right_mps = v + wz * (self.wheelbase_m / 2.0)

        left_norm = clamp(left_mps / self.max_linear_speed_mps, -1.0, 1.0)
        right_norm = clamp(right_mps / self.max_linear_speed_mps, -1.0, 1.0)

        left = clamp_int(
            round(left_norm * self.max_sabertooth_cmd),
            -self.max_sabertooth_cmd,
            self.max_sabertooth_cmd,
        )
        right = clamp_int(
            round(right_norm * self.max_sabertooth_cmd),
            -self.max_sabertooth_cmd,
            self.max_sabertooth_cmd,
        )
        enable = bool(left != 0 or right != 0)
        if not enable:
            left = 0
            right = 0
        return left, right, enable

    def reset_obstacle_control_chord(self) -> None:
        self.obstacle_control_target = None
        self.obstacle_control_started = 0.0
        self.obstacle_control_latched = False

    def handle_obstacle_latch(self, guard_enabled: Optional[bool]) -> None:
        if not self.ps2_obstacle_latch_enabled:
            return
        if guard_enabled is None:
            return

        if self.obstacle_guard.enabled != guard_enabled:
            self.obstacle_guard.set_enabled(
                guard_enabled,
                source='ps2_latch',
            )

    def handle_obstacle_control_chord(
        self,
        select: bool,
        ry: int,
        throttle: int,
        steering: int,
    ) -> None:
        if not self.ps2_obstacle_control_enabled:
            return

        stopped = throttle == 0 and steering == 0
        if not select:
            self.reset_obstacle_control_chord()
            return
        if self.ps2_obstacle_control_require_stopped and not stopped:
            self.reset_obstacle_control_chord()
            return

        right_stick_y = self.axis_to_command(ry, invert=True)
        target: Optional[bool]
        if right_stick_y >= self.ps2_obstacle_control_threshold:
            target = True
        elif right_stick_y <= -self.ps2_obstacle_control_threshold:
            target = False
        else:
            self.reset_obstacle_control_chord()
            return

        now = time.monotonic()
        if target != self.obstacle_control_target:
            self.obstacle_control_target = target
            self.obstacle_control_started = now
            self.obstacle_control_latched = False
            return

        held_long_enough = (
            now - self.obstacle_control_started
        ) >= self.ps2_obstacle_control_hold_sec
        if held_long_enough and not self.obstacle_control_latched:
            self.obstacle_guard.set_enabled(target, source='ps2')
            self.obstacle_control_latched = True

    def send_drive(self, left: int, right: int, enable: bool) -> None:
        drive_line = f'D,{left},{right},{1 if enable else 0}'
        self.teensy.write_line(drive_line)
        self.drive_pub.publish(String(data=drive_line))

    def send_stop(self) -> None:
        now = time.monotonic()
        if now - self.last_sent_stop > 0.10:
            if rclpy.ok():
                try:
                    self.send_drive(0, 0, False)
                except RuntimeError:
                    pass
            self.last_sent_stop = now

    def loop(self) -> None:
        result = self.uno.readline()
        now = time.monotonic()
        ps2_sent_drive = False

        if result.error == 'not_connected':
            self.send_stop()
        elif result.error:
            self.get_logger().warning(f'Uno serial error: {result.error}')
            self.send_stop()

        if result.line:
            self.raw_pub.publish(String(data=result.line))
            parsed = self.parse_uno_line(result.line)
            if parsed is not None:
                if self.ps2_manual_input_active(parsed):
                    self.last_manual_input_time = now
                left, right, enable, throttle, steering = self.mix_drive(
                    parsed.select,
                    parsed.rx,
                    parsed.ly,
                )
                self.handle_obstacle_latch(parsed.obstacle_guard_enabled)
                self.handle_obstacle_control_chord(
                    parsed.select,
                    parsed.ry,
                    throttle,
                    steering,
                )
                if enable:
                    left, right = self.obstacle_guard.filter_tank(
                        left,
                        right,
                        self.max_sabertooth_cmd,
                    )
                    throttle = int(round((left + right) / 2.0))
                    steering = int(round((right - left) / 2.0))
                if enable or self.manual_override_active(now):
                    self.send_drive(left, right, enable)
                    ps2_sent_drive = True
                self.last_valid_packet_time = now
                self.last_enable = enable
                if self.publish_cmd_vel_enabled:
                    self.publish_cmd_vel(throttle, steering, enable)

        for ack in self.teensy.drain_available(max_lines=12):
            if ack:
                if not self.teensy_telemetry.handle_line(ack):
                    self.ack_pub.publish(String(data=ack))

        timed_out = (now - self.last_valid_packet_time) > self.command_timeout_sec
        if self.voice_cmd_active(now) and not self.manual_override_active(now):
            left, right, enable = self.twist_to_drive(self.last_voice_cmd)
            self.send_drive(left, right, enable)
            self.last_enable = enable
        elif (
            not ps2_sent_drive
            and (timed_out or (not self.last_enable and self.send_stop_when_disabled))
        ):
            self.send_stop()
            if self.publish_cmd_vel_enabled:
                self.cmd_vel_pub.publish(Twist())


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = Ps2UnoToTeensy()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.send_stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

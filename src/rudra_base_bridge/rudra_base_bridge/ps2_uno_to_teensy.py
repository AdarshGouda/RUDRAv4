"""Bridge Arduino Uno PS2 serial packets to Teensy Sabertooth drive commands."""

from __future__ import annotations

import time
from typing import Optional, Tuple

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .obstacle_guard import LidarObstacleGuard
from .serial_utils import apply_deadband, clamp, clamp_int, LineSerial


class Ps2UnoToTeensy(Node):
    """Read J,select,ry,rx,ly,lx from Uno and send D,left,right,enable to Teensy."""

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
        self.declare_parameter('max_sabertooth_cmd', 127)
        self.declare_parameter('max_linear_speed_mps', 1.20)
        self.declare_parameter('max_angular_speed_radps', 2.50)
        self.declare_parameter('send_stop_when_disabled', True)

        self.uno_port = str(self.get_parameter('uno_port').value)
        self.teensy_port = str(self.get_parameter('teensy_port').value)
        self.baud = int(self.get_parameter('baud').value)
        self.command_timeout_sec = float(self.get_parameter('command_timeout_sec').value)
        self.publish_cmd_vel_enabled = bool(self.get_parameter('publish_cmd_vel').value)
        self.select_as_enable = bool(self.get_parameter('select_as_enable').value)
        self.axis_center = int(self.get_parameter('axis_center').value)
        self.axis_range = max(1, int(self.get_parameter('axis_range').value))
        self.deadband = int(self.get_parameter('deadband').value)
        self.max_sabertooth_cmd = int(self.get_parameter('max_sabertooth_cmd').value)
        self.max_linear_speed_mps = float(self.get_parameter('max_linear_speed_mps').value)
        self.max_angular_speed_radps = float(self.get_parameter('max_angular_speed_radps').value)
        self.send_stop_when_disabled = bool(self.get_parameter('send_stop_when_disabled').value)

        self.uno = LineSerial(self.uno_port, self.baud)
        self.teensy = LineSerial(self.teensy_port, self.baud)
        self.obstacle_guard = LidarObstacleGuard(self)

        self.raw_pub = self.create_publisher(String, '/rudra/ps2_raw', 10)
        self.drive_pub = self.create_publisher(String, '/rudra/drive_cmd', 10)
        self.ack_pub = self.create_publisher(String, '/rudra/teensy_ack', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.last_valid_packet_time = 0.0
        self.last_sent_stop = 0.0
        self.last_enable = False

        period = float(self.get_parameter('timer_period_sec').value)
        self.timer = self.create_timer(period, self.loop)

        self.get_logger().info(f'Uno port: {self.uno_port}')
        self.get_logger().info(f'Teensy port: {self.teensy_port}')
        self.get_logger().info('Expected Uno line: J,select,ry,rx,ly,lx')
        self.get_logger().info('Sending Teensy line: D,left,right,enable')

    def parse_uno_line(self, line: str) -> Optional[Tuple[bool, int, int, int, int]]:
        line = line.strip()
        if not line.startswith('J,'):
            return None
        parts = line.split(',')
        if len(parts) != 6:
            return None
        try:
            select = bool(int(parts[1]))
            ry = int(parts[2])
            rx = int(parts[3])
            ly = int(parts[4])
            lx = int(parts[5])
        except ValueError:
            return None
        return select, ry, rx, ly, lx

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

    def send_stop(self) -> None:
        now = time.monotonic()
        if now - self.last_sent_stop > 0.10:
            self.teensy.write_line('D,0,0,0')
            self.drive_pub.publish(String(data='D,0,0,0'))
            self.last_sent_stop = now

    def loop(self) -> None:
        result = self.uno.readline()
        now = time.monotonic()

        if result.error == 'not_connected':
            self.send_stop()
        elif result.error:
            self.get_logger().warning(f'Uno serial error: {result.error}')
            self.send_stop()

        if result.line:
            self.raw_pub.publish(String(data=result.line))
            parsed = self.parse_uno_line(result.line)
            if parsed is not None:
                select, ry, rx, ly, lx = parsed
                left, right, enable, throttle, steering = self.mix_drive(select, rx, ly)
                if enable:
                    left, right = self.obstacle_guard.filter_tank(
                        left,
                        right,
                        self.max_sabertooth_cmd,
                    )
                    throttle = int(round((left + right) / 2.0))
                    steering = int(round((right - left) / 2.0))
                drive_line = f'D,{left},{right},{1 if enable else 0}'
                self.teensy.write_line(drive_line)
                self.drive_pub.publish(String(data=drive_line))
                self.last_valid_packet_time = now
                self.last_enable = enable
                if self.publish_cmd_vel_enabled:
                    self.publish_cmd_vel(throttle, steering, enable)

        for ack in self.teensy.drain_available(max_lines=5):
            if ack:
                self.ack_pub.publish(String(data=ack))

        timed_out = (now - self.last_valid_packet_time) > self.command_timeout_sec
        if timed_out or (not self.last_enable and self.send_stop_when_disabled):
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
        node.send_stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

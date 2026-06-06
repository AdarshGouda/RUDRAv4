"""Bridge ROS2 /cmd_vel to Teensy Sabertooth serial commands."""

from __future__ import annotations

import time
from typing import Optional

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .obstacle_guard import LidarObstacleGuard
from .serial_utils import clamp, clamp_int, LineSerial


class CmdVelToTeensy(Node):
    """Convert /cmd_vel into D,left,right,enable serial lines for Teensy."""

    def __init__(self) -> None:
        super().__init__('cmd_vel_to_teensy')

        teensy_port = '/dev/serial/by-id/usb-Teensyduino_USB_Serial_9210670-if00'

        self.declare_parameter('teensy_port', teensy_port)
        self.declare_parameter('baud', 115200)
        self.declare_parameter('wheelbase_m', 0.29)
        self.declare_parameter('max_linear_speed_mps', 1.20)
        self.declare_parameter('max_sabertooth_cmd', 127)
        self.declare_parameter('cmd_timeout_sec', 0.30)
        self.declare_parameter('timer_period_sec', 0.02)
        self.declare_parameter('enable_on_nonzero_cmd', True)

        self.teensy_port = str(self.get_parameter('teensy_port').value)
        self.baud = int(self.get_parameter('baud').value)
        self.wheelbase_m = float(self.get_parameter('wheelbase_m').value)
        self.max_linear_speed_mps = float(self.get_parameter('max_linear_speed_mps').value)
        self.max_sabertooth_cmd = int(self.get_parameter('max_sabertooth_cmd').value)
        self.cmd_timeout_sec = float(self.get_parameter('cmd_timeout_sec').value)
        self.enable_on_nonzero_cmd = bool(self.get_parameter('enable_on_nonzero_cmd').value)

        self.teensy = LineSerial(self.teensy_port, self.baud)
        self.obstacle_guard = LidarObstacleGuard(self)
        self.last_cmd_time = 0.0
        self.last_cmd = Twist()
        self.last_sent_stop = 0.0

        self.drive_pub = self.create_publisher(String, '/rudra/drive_cmd', 10)
        self.ack_pub = self.create_publisher(String, '/rudra/teensy_ack', 10)
        self.sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)

        period = float(self.get_parameter('timer_period_sec').value)
        self.timer = self.create_timer(period, self.loop)

        self.get_logger().info(f'Teensy port: {self.teensy_port}')
        self.get_logger().info('Subscribing to /cmd_vel; sending D,left,right,enable')

    def cmd_callback(self, msg: Twist) -> None:
        self.last_cmd = msg
        self.last_cmd_time = time.monotonic()

    def cmd_to_sabertooth(self, msg: Twist) -> tuple[int, int, bool]:
        v = self.obstacle_guard.filter_linear_x(msg.linear.x)
        wz = msg.angular.z

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

        enable = True
        if self.enable_on_nonzero_cmd:
            enable = bool(left != 0 or right != 0)
        if not enable:
            left = 0
            right = 0
        return left, right, enable

    def send_drive(self, left: int, right: int, enable: bool) -> None:
        line = f'D,{left},{right},{1 if enable else 0}'
        self.teensy.write_line(line)
        self.drive_pub.publish(String(data=line))

    def send_stop(self) -> None:
        now = time.monotonic()
        if now - self.last_sent_stop > 0.10:
            self.send_drive(0, 0, False)
            self.last_sent_stop = now

    def loop(self) -> None:
        now = time.monotonic()
        if (now - self.last_cmd_time) > self.cmd_timeout_sec:
            self.send_stop()
        else:
            left, right, enable = self.cmd_to_sabertooth(self.last_cmd)
            self.send_drive(left, right, enable)

        for ack in self.teensy.drain_available(max_lines=5):
            if ack:
                self.ack_pub.publish(String(data=ack))


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = CmdVelToTeensy()
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

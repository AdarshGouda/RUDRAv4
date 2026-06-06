"""LiDAR obstacle guard for final motor commands."""

from __future__ import annotations

import math
import time
from typing import Optional, Tuple

from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

from .serial_utils import clamp, clamp_int


class LidarObstacleGuard:
    """Scale forward commands when a LaserScan sees an obstacle ahead."""

    def __init__(self, node: Node) -> None:
        self.node = node

        node.declare_parameter('obstacle_avoidance_enabled', True)
        node.declare_parameter('scan_topic', '/scan')
        node.declare_parameter('obstacle_front_angle_deg', 70.0)
        node.declare_parameter('obstacle_stop_distance_m', 0.45)
        node.declare_parameter('obstacle_slow_distance_m', 1.00)
        node.declare_parameter('obstacle_scan_timeout_sec', 0.75)
        node.declare_parameter('obstacle_status_period_sec', 0.25)

        self.enabled = bool(node.get_parameter('obstacle_avoidance_enabled').value)
        self.scan_topic = str(node.get_parameter('scan_topic').value)
        self.front_angle_deg = float(node.get_parameter('obstacle_front_angle_deg').value)
        self.stop_distance_m = float(node.get_parameter('obstacle_stop_distance_m').value)
        self.slow_distance_m = float(node.get_parameter('obstacle_slow_distance_m').value)
        self.scan_timeout_sec = float(node.get_parameter('obstacle_scan_timeout_sec').value)
        self.status_period_sec = float(node.get_parameter('obstacle_status_period_sec').value)

        if self.slow_distance_m < self.stop_distance_m:
            self.slow_distance_m = self.stop_distance_m

        self.closest_front_m: Optional[float] = None
        self.last_scan_time = 0.0
        self.last_status_time = 0.0

        self.status_pub = node.create_publisher(String, '/rudra/obstacle_guard', 10)
        self.scan_sub = node.create_subscription(
            LaserScan,
            self.scan_topic,
            self.scan_callback,
            10,
        )

        if self.enabled:
            node.get_logger().info(
                'LiDAR obstacle guard enabled on '
                f'{self.scan_topic}: stop <= {self.stop_distance_m:.2f} m, '
                f'slow <= {self.slow_distance_m:.2f} m',
            )
        else:
            node.get_logger().info('LiDAR obstacle guard disabled')

    def scan_callback(self, msg: LaserScan) -> None:
        half_angle_rad = math.radians(max(0.0, self.front_angle_deg) / 2.0)
        closest: Optional[float] = None

        for index, distance in enumerate(msg.ranges):
            if not math.isfinite(distance):
                continue
            if msg.range_min and distance < msg.range_min:
                continue
            if msg.range_max and distance > msg.range_max:
                continue

            angle = msg.angle_min + index * msg.angle_increment
            if abs(angle) > half_angle_rad:
                continue

            if closest is None or distance < closest:
                closest = distance

        self.closest_front_m = closest
        self.last_scan_time = time.monotonic()

    def has_fresh_scan(self) -> bool:
        if self.last_scan_time <= 0.0:
            return False
        return (time.monotonic() - self.last_scan_time) <= self.scan_timeout_sec

    def forward_scale(self) -> Tuple[float, str]:
        if not self.enabled:
            return 1.0, 'disabled'
        if not self.has_fresh_scan():
            return 1.0, 'no_fresh_scan'
        if self.closest_front_m is None:
            return 1.0, 'no_front_points'

        closest = self.closest_front_m
        if closest <= self.stop_distance_m:
            return 0.0, 'blocked'
        if closest >= self.slow_distance_m:
            return 1.0, 'clear'
        if self.slow_distance_m == self.stop_distance_m:
            return 0.0, 'blocked'

        scale = (closest - self.stop_distance_m) / (
            self.slow_distance_m - self.stop_distance_m
        )
        return clamp(scale, 0.0, 1.0), 'slowing'

    def filter_linear_x(self, linear_x: float) -> float:
        if linear_x <= 0.0:
            self.publish_status(1.0, 'reverse_or_turn')
            return linear_x

        scale, reason = self.forward_scale()
        self.publish_status(scale, reason)
        return linear_x * scale

    def filter_tank(self, left: int, right: int, max_cmd: int) -> Tuple[int, int]:
        forward = (left + right) / 2.0
        if forward <= 0.0:
            self.publish_status(1.0, 'reverse_or_turn')
            return left, right

        turn = (right - left) / 2.0
        scale, reason = self.forward_scale()
        self.publish_status(scale, reason)

        filtered_forward = forward * scale
        filtered_left = int(round(filtered_forward - turn))
        filtered_right = int(round(filtered_forward + turn))
        return (
            clamp_int(filtered_left, -max_cmd, max_cmd),
            clamp_int(filtered_right, -max_cmd, max_cmd),
        )

    def publish_status(self, scale: float, reason: str) -> None:
        now = time.monotonic()
        if now - self.last_status_time < self.status_period_sec:
            return
        self.last_status_time = now

        closest = 'none'
        if self.closest_front_m is not None and self.has_fresh_scan():
            closest = f'{self.closest_front_m:.2f}'

        self.status_pub.publish(
            String(data=f'{reason},closest_front_m={closest},forward_scale={scale:.2f}')
        )

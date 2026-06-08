"""LiDAR obstacle guard for final motor commands."""

from __future__ import annotations

import math
import time
from typing import Optional, Tuple

from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from std_msgs.msg import String
from visualization_msgs.msg import Marker

from .serial_utils import clamp, clamp_int


class LidarObstacleGuard:
    """Scale forward commands when a LaserScan sees an obstacle ahead."""

    def __init__(self, node: Node) -> None:
        self.node = node

        node.declare_parameter('obstacle_avoidance_enabled', True)
        node.declare_parameter('obstacle_enable_topic', '/rudra/obstacle_guard_enable')
        node.declare_parameter('scan_topic', '/scan')
        node.declare_parameter('obstacle_front_angle_deg', 70.0)
        node.declare_parameter('obstacle_stop_distance_m', 0.45)
        node.declare_parameter('obstacle_slow_distance_m', 1.00)
        node.declare_parameter('obstacle_scan_timeout_sec', 0.75)
        node.declare_parameter('obstacle_status_period_sec', 0.25)
        node.declare_parameter('obstacle_marker_topic', '/rudra/obstacle_guard_marker')
        node.declare_parameter('obstacle_marker_frame', 'laser')

        self.enabled = bool(node.get_parameter('obstacle_avoidance_enabled').value)
        self.enable_topic = str(node.get_parameter('obstacle_enable_topic').value)
        self.scan_topic = str(node.get_parameter('scan_topic').value)
        self.front_angle_deg = float(node.get_parameter('obstacle_front_angle_deg').value)
        self.stop_distance_m = float(node.get_parameter('obstacle_stop_distance_m').value)
        self.slow_distance_m = float(node.get_parameter('obstacle_slow_distance_m').value)
        self.scan_timeout_sec = float(node.get_parameter('obstacle_scan_timeout_sec').value)
        self.status_period_sec = float(node.get_parameter('obstacle_status_period_sec').value)
        self.marker_topic = str(node.get_parameter('obstacle_marker_topic').value)
        self.marker_frame = str(node.get_parameter('obstacle_marker_frame').value)

        if self.slow_distance_m < self.stop_distance_m:
            self.slow_distance_m = self.stop_distance_m

        self.closest_front_m: Optional[float] = None
        self.closest_front_angle_rad: Optional[float] = None
        self.last_scan_time = 0.0
        self.last_status_time = 0.0

        self.status_pub = node.create_publisher(String, '/rudra/obstacle_guard', 10)
        self.marker_pub = node.create_publisher(Marker, self.marker_topic, 10)
        self.enable_sub = node.create_subscription(
            Bool,
            self.enable_topic,
            self.enable_callback,
            10,
        )
        self.scan_sub = node.create_subscription(
            LaserScan,
            self.scan_topic,
            self.scan_callback,
            qos_profile_sensor_data,
        )

        if self.enabled:
            node.get_logger().info(
                'LiDAR obstacle guard enabled on '
                f'{self.scan_topic}: stop <= {self.stop_distance_m:.2f} m, '
                f'slow <= {self.slow_distance_m:.2f} m',
            )
        else:
            node.get_logger().info('LiDAR obstacle guard disabled')

        node.get_logger().info(
            f'LiDAR obstacle guard runtime enable topic: {self.enable_topic}'
        )
        node.get_logger().info(
            f'LiDAR obstacle guard visualization marker: {self.marker_topic}'
        )

    def enable_callback(self, msg: Bool) -> None:
        self.set_enabled(bool(msg.data), source='topic')

    def set_enabled(self, enabled: bool, source: str = 'local') -> None:
        if self.enabled == enabled:
            self.publish_status(1.0, 'enabled' if enabled else 'disabled', force=True)
            return

        self.enabled = enabled
        state = 'enabled' if enabled else 'disabled'
        self.node.get_logger().info(
            f'LiDAR obstacle guard {state} by {source}'
        )
        self.publish_status(1.0, state, force=True)

    def scan_callback(self, msg: LaserScan) -> None:
        half_angle_rad = math.radians(max(0.0, self.front_angle_deg) / 2.0)
        closest: Optional[float] = None
        closest_angle: Optional[float] = None

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
                closest_angle = angle

        self.closest_front_m = closest
        self.closest_front_angle_rad = closest_angle
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

    def publish_status(self, scale: float, reason: str, force: bool = False) -> None:
        now = time.monotonic()
        if not force and now - self.last_status_time < self.status_period_sec:
            return
        self.last_status_time = now

        closest = 'none'
        if self.closest_front_m is not None and self.has_fresh_scan():
            closest = f'{self.closest_front_m:.2f}'

        self.status_pub.publish(
            String(
                data=(
                    f'{reason},enabled={int(self.enabled)},'
                    f'closest_front_m={closest},forward_scale={scale:.2f}'
                )
            )
        )
        self.publish_marker(scale, reason)

    def publish_marker(self, scale: float, reason: str) -> None:
        marker = Marker()
        marker.header.stamp = self.node.get_clock().now().to_msg()
        marker.header.frame_id = self.marker_frame
        marker.ns = 'rudra_obstacle_guard'
        marker.id = 0

        if (
            self.closest_front_m is None
            or self.closest_front_angle_rad is None
            or not self.has_fresh_scan()
        ):
            marker.action = Marker.DELETE
            self.marker_pub.publish(marker)
            return

        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = self.closest_front_m * math.cos(
            self.closest_front_angle_rad
        )
        marker.pose.position.y = self.closest_front_m * math.sin(
            self.closest_front_angle_rad
        )
        marker.pose.position.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.18
        marker.scale.y = 0.18
        marker.scale.z = 0.18

        marker.color.a = 0.9
        if not self.enabled or reason == 'disabled':
            marker.color.r = 0.45
            marker.color.g = 0.45
            marker.color.b = 0.45
        elif scale <= 0.0 or reason == 'blocked':
            marker.color.r = 1.0
            marker.color.g = 0.05
            marker.color.b = 0.02
        elif scale < 1.0 or reason == 'slowing':
            marker.color.r = 1.0
            marker.color.g = 0.65
            marker.color.b = 0.0
        else:
            marker.color.r = 0.0
            marker.color.g = 0.85
            marker.color.b = 0.2

        self.marker_pub.publish(marker)

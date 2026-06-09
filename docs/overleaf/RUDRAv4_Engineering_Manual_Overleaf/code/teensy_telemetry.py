"""Helpers for Teensy IMU and wheel-odometry telemetry lines."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Optional

from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu


@dataclass(frozen=True)
class ImuTelemetry:
    """One IMU sample from the Teensy plain-serial stream."""

    ax: float
    ay: float
    az: float
    gx: float
    gy: float
    gz: float


@dataclass(frozen=True)
class OdomTelemetry:
    """One wheel-odometry sample from the Teensy plain-serial stream."""

    x: float
    y: float
    theta: float
    linear_x: float
    angular_z: float
    left_velocity: float
    right_velocity: float


def _yaw_to_quaternion(yaw: float) -> Quaternion:
    quat = Quaternion()
    quat.z = math.sin(yaw * 0.5)
    quat.w = math.cos(yaw * 0.5)
    return quat


def parse_imu_line(line: str) -> Optional[ImuTelemetry]:
    """Parse IMU,ax,ay,az,gx,gy,gz from the Teensy serial stream."""
    parts = line.strip().split(',')
    if len(parts) != 7 or parts[0] != 'IMU':
        return None
    try:
        values = [float(value) for value in parts[1:]]
    except ValueError:
        return None
    return ImuTelemetry(*values)


def parse_odom_line(line: str) -> Optional[OdomTelemetry]:
    """Parse ODOM,x,y,theta,linear_x,angular_z,left_vel,right_vel."""
    parts = line.strip().split(',')
    if len(parts) != 8 or parts[0] != 'ODOM':
        return None
    try:
        values = [float(value) for value in parts[1:]]
    except ValueError:
        return None
    return OdomTelemetry(*values)


class TeensyTelemetryBridge:
    """Publish ROS topics from optional Teensy telemetry lines."""

    def __init__(self, node: Node) -> None:
        self.node = node

        node.declare_parameter('publish_teensy_telemetry', True)
        node.declare_parameter('imu_topic', '/rudra/imu/raw')
        node.declare_parameter('wheel_odom_topic', '/rudra/wheel_odom')
        node.declare_parameter('imu_frame_id', 'imu_link')
        node.declare_parameter('wheel_odom_frame_id', 'odom')
        node.declare_parameter('wheel_odom_child_frame_id', 'base_link')

        self.enabled = bool(node.get_parameter('publish_teensy_telemetry').value)
        self.imu_frame_id = str(node.get_parameter('imu_frame_id').value)
        self.wheel_odom_frame_id = str(node.get_parameter('wheel_odom_frame_id').value)
        self.wheel_odom_child_frame_id = str(
            node.get_parameter('wheel_odom_child_frame_id').value
        )

        if not self.enabled:
            self.imu_pub = None
            self.odom_pub = None
            return

        self.imu_pub = node.create_publisher(
            Imu,
            str(node.get_parameter('imu_topic').value),
            10,
        )
        self.odom_pub = node.create_publisher(
            Odometry,
            str(node.get_parameter('wheel_odom_topic').value),
            10,
        )

    def handle_line(self, line: str) -> bool:
        """Publish IMU or odom topics if the line carries telemetry."""
        if not self.enabled:
            return False

        imu_sample = parse_imu_line(line)
        if imu_sample is not None:
            self._publish_imu(imu_sample)
            return True

        odom_sample = parse_odom_line(line)
        if odom_sample is not None:
            self._publish_odom(odom_sample)
            return True

        return False

    def _publish_imu(self, sample: ImuTelemetry) -> None:
        if self.imu_pub is None:
            return

        msg = Imu()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = self.imu_frame_id

        # The Teensy publishes raw gyro/accel only. Orientation is intentionally unset.
        msg.orientation_covariance[0] = -1.0
        msg.angular_velocity.x = sample.gx
        msg.angular_velocity.y = sample.gy
        msg.angular_velocity.z = sample.gz
        msg.linear_acceleration.x = sample.ax
        msg.linear_acceleration.y = sample.ay
        msg.linear_acceleration.z = sample.az

        msg.angular_velocity_covariance[0] = 0.02
        msg.angular_velocity_covariance[4] = 0.02
        msg.angular_velocity_covariance[8] = 0.02
        msg.linear_acceleration_covariance[0] = 0.10
        msg.linear_acceleration_covariance[4] = 0.10
        msg.linear_acceleration_covariance[8] = 0.10

        self.imu_pub.publish(msg)

    def _publish_odom(self, sample: OdomTelemetry) -> None:
        if self.odom_pub is None:
            return

        msg = Odometry()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = self.wheel_odom_frame_id
        msg.child_frame_id = self.wheel_odom_child_frame_id

        msg.pose.pose.position.x = sample.x
        msg.pose.pose.position.y = sample.y
        msg.pose.pose.orientation = _yaw_to_quaternion(sample.theta)
        msg.twist.twist.linear.x = sample.linear_x
        msg.twist.twist.angular.z = sample.angular_z

        msg.pose.covariance[0] = 0.05
        msg.pose.covariance[7] = 0.05
        msg.pose.covariance[35] = 0.10
        msg.twist.covariance[0] = 0.05
        msg.twist.covariance[35] = 0.10

        self.odom_pub.publish(msg)

"""Broadcast odom -> base_link from an Odometry topic."""

from __future__ import annotations

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


class OdomTfBroadcaster(Node):
    """Republish an Odometry pose as a TF transform."""

    def __init__(self) -> None:
        super().__init__('odom_tf_broadcaster')
        self.declare_parameter('odom_topic', '/odometry/filtered')
        odom_topic = str(self.get_parameter('odom_topic').value)

        self.tf_broadcaster = TransformBroadcaster(self)
        self.subscription = self.create_subscription(
            Odometry,
            odom_topic,
            self.broadcast_odom_tf,
            10,
        )
        self.get_logger().info(f'Publishing TF from {odom_topic} messages.')

    def broadcast_odom_tf(self, msg: Odometry) -> None:
        transform = TransformStamped()
        transform.header = msg.header
        transform.child_frame_id = msg.child_frame_id or 'base_link'
        transform.transform.translation.x = msg.pose.pose.position.x
        transform.transform.translation.y = msg.pose.pose.position.y
        transform.transform.translation.z = msg.pose.pose.position.z
        transform.transform.rotation = msg.pose.pose.orientation
        self.tf_broadcaster.sendTransform(transform)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = OdomTfBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

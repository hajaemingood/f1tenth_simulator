#!/usr/bin/env python3
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import Pose2D
from tf2_ros import Buffer, TransformException, TransformListener
from tf_transformations import euler_from_quaternion


class BaseLinkPosePublisher(Node):
    def __init__(self) -> None:
        super().__init__("base_link_pose_publisher")
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.publisher = self.create_publisher(Pose2D, "/base_link_pose", 10)
        self.timer = self.create_timer(0.05, self._publish_pose)

    def _publish_pose(self) -> None:
        try:
            transform = self.tf_buffer.lookup_transform(
                "map",
                "base_link",
                Time(),
                timeout=Duration(seconds=0.1),
            )
        except TransformException as exc:
            # Use debug level so occasional lookup failures do not spam logs.
            self.get_logger().debug(f"TF lookup failed: {exc}")
            return

        translation = transform.transform.translation
        rotation = transform.transform.rotation
        yaw = euler_from_quaternion([rotation.x, rotation.y, rotation.z, rotation.w])[2]

        msg = Pose2D()
        msg.x = translation.x
        msg.y = translation.y
        msg.theta = yaw
        self.publisher.publish(msg)


def main() -> None:
    rclpy.init()
    node = BaseLinkPosePublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

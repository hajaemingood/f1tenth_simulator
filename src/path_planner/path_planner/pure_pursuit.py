#!/usr/bin/env python3
import csv
from math import cos, sin, sqrt, tan
from typing import List, Tuple

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Float64


class PurePursuitNode(Node):
    def __init__(self) -> None:
        super().__init__("pure_pursuit_node")

        self.declare_parameter(
            "waypoints_csv",
            "/root/f1_sim/src/path_planner/path_planner/waypoints.csv",
        )
        self.declare_parameter("lookahead_distance", 0.8)

        self.waypoints_path = self.get_parameter("waypoints_csv").get_parameter_value().string_value
        self.lookahead_distance = (
            self.get_parameter("lookahead_distance").get_parameter_value().double_value
        )

        self.waypoints = self._load_waypoints(self.waypoints_path)

        self.motor_pub = self.create_publisher(Float64, "/commands/motor/speed", 10)
        self.servo_pub = self.create_publisher(Float64, "/commands/servo/position", 10)

        self.create_subscription(Pose2D, "/base_link_pose", self._base_callback, 10)

        self.speed_msg = Float64()
        self.steer_msg = Float64()

    def _base_callback(self, pose_msg: Pose2D) -> None:
        goal_point = self._find_goal_point(pose_msg)
        if not goal_point:
            self.get_logger().debug("No waypoint satisfies lookahead constraint.")
            return

        steering_angle = self._calculate_steering_angle(goal_point)
        steering_angle = -(steering_angle - 0.5)
        self.get_logger().debug(f"Computed steering angle: {steering_angle:.3f}")

        self._publish_drive_message(steering_angle)

    def _find_goal_point(self, pose_msg: Pose2D) -> Tuple[float, float, float, float, float]:
        car_x = pose_msg.x
        car_y = pose_msg.y
        yaw = pose_msg.theta

        max_distance = -1.0
        goal_point: Tuple[float, float, float, float, float] = ()

        for x, y in self.waypoints:
            dx = x - car_x
            dy = y - car_y
            distance = sqrt(dx**2 + dy**2)

            if distance <= self.lookahead_distance:
                rotated_x = cos(-yaw) * dx - sin(-yaw) * dy
                rotated_y = sin(-yaw) * dx + cos(-yaw) * dy

                if rotated_x > 0.0 and distance > max_distance:
                    max_distance = distance
                    goal_point = (x, y, rotated_x, rotated_y, distance)

        return goal_point

    def _calculate_steering_angle(self, goal_point: Tuple[float, float, float, float, float]) -> float:
        L = self.lookahead_distance
        y = goal_point[3]
        curvature = 2.0 * y / (L**2)
        curvature = max(min(curvature, 0.5), -0.5)
        return curvature * 0.9

    def _publish_drive_message(self, steering_angle: float) -> None:
        lr = 0.1514

        try:
            v = np.sqrt(11.2815 * np.sqrt((0.325 / tan(steering_angle)) ** 2 + lr))
            _ = v  # Value retained for parity with legacy implementation.
        except ZeroDivisionError:
            self.get_logger().warn("Steering angle too close to zero; skipping velocity calculation.")

        if steering_angle > 0.65 or steering_angle < 0.35:
            velocity = 5000.0
        elif steering_angle > 0.875 or steering_angle < 0.175:
            velocity = 3000.0
        else:
            velocity = 7000.0

        self.speed_msg.data = float(velocity)
        self.steer_msg.data = steering_angle

        self.motor_pub.publish(self.speed_msg)
        self.servo_pub.publish(self.steer_msg)

    def _load_waypoints(self, csv_file: str) -> List[Tuple[float, float]]:
        waypoints: List[Tuple[float, float]] = []
        try:
            with open(csv_file, "r") as file:
                reader = csv.reader(file)
                next(reader, None)
                for row in reader:
                    if len(row) < 2:
                        continue
                    x = float(row[0])
                    y = float(row[1])
                    waypoints.append((x, y))
        except FileNotFoundError:
            self.get_logger().error(f"Waypoint CSV not found: {csv_file}")
        except ValueError as exc:
            self.get_logger().error(f"Invalid waypoint data: {exc}")
        return waypoints


def main() -> None:
    rclpy.init()
    node = PurePursuitNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

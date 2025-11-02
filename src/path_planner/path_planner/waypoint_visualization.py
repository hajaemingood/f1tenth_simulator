#!/usr/bin/env python3
import csv
from typing import List

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker


class WaypointVisualizer(Node):
    def __init__(self) -> None:
        super().__init__("waypoint_visualizer")

        self.declare_parameter(
            "waypoints_csv",
            "/root/f1_sim/src/ferrari/ferrari/path_controller/waypoints.csv",
        )
        self.declare_parameter("frame_id", "map")

        self.marker_pub = self.create_publisher(Marker, "/waypoint_marker", 10)

        self.marker = Marker()
        self.marker.ns = "waypoints"
        self.marker.id = 0
        self.marker.type = Marker.POINTS
        self.marker.action = Marker.ADD
        self.marker.scale.x = 0.1
        self.marker.scale.y = 0.1
        self.marker.color.a = 1.0
        self.marker.color.r = 0.0
        self.marker.color.g = 0.0
        self.marker.color.b = 1.0
        self.marker.header.frame_id = (
            self.get_parameter("frame_id").get_parameter_value().string_value
        )

        csv_path = self.get_parameter("waypoints_csv").get_parameter_value().string_value
        self.marker.points = self._load_waypoints(csv_path)

        self.timer = self.create_timer(0.1, self._publish_marker)

    def _load_waypoints(self, csv_file: str) -> List[Point]:
        points: List[Point] = []
        try:
            with open(csv_file, "r") as file:
                reader = csv.reader(file)
                next(reader, None)
                for row in reader:
                    if len(row) < 2:
                        continue
                    x = float(row[0])
                    y = float(row[1])
                    point = Point()
                    point.x = x
                    point.y = y
                    point.z = 0.0
                    points.append(point)
        except FileNotFoundError:
            self.get_logger().error(f"Waypoint CSV not found: {csv_file}")
        except ValueError as exc:
            self.get_logger().error(f"Invalid waypoint data: {exc}")
        return points

    def _publish_marker(self) -> None:
        self.marker.header.stamp = self.get_clock().now().to_msg()
        self.marker_pub.publish(self.marker)


def main() -> None:
    rclpy.init()
    node = WaypointVisualizer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

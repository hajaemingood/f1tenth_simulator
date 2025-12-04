#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
정적 장애물(csv)을 costmap으로 시각화하는 노드.

- isStatic_coord.csv에 기록된 Gazebo world(=odom) 좌표를 map 프레임으로 변환
- 각 박스 크기 + 허용 오차 만큼 OccupancyGrid에 100으로 채움
"""

import csv
import math
from dataclasses import dataclass
from typing import List

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from nav_msgs.msg import OccupancyGrid


DEFAULT_STATIC_CSV = "/root/f1tenth_simulator/src/machine_learning/config/isStatic_coord.csv"
HALF_LENGTH = 1.259320 / 2.0
HALF_WIDTH = 1.007450 / 2.0
DEFAULT_MARGIN = 0.02


@dataclass
class StaticObstacle:
    x: float
    y: float
    yaw: float


def load_static_obstacles(csv_path: str) -> List[StaticObstacle]:
    obstacles: List[StaticObstacle] = []
    with open(csv_path, newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            try:
                x = float(row["x"])
                y = float(row["y"])
                yaw = float(row.get("yaw", "0.0"))
            except (KeyError, ValueError):
                continue
            obstacles.append(StaticObstacle(x, y, yaw))
    return obstacles


class StaticObstacleVisualizer(Node):
    def __init__(self):
        super().__init__("static_obstacle_visualizer")
        self.declare_parameter("static_csv", DEFAULT_STATIC_CSV)
        self.declare_parameter("resolution", 0.02)
        self.declare_parameter("margin", DEFAULT_MARGIN)
        # Gazebo world(=odom) → map 프레임 고정 변환 파라미터.
        # 기본값은 ferrari_spawn_pose ↔ AMCL initial pose에서 계산한 값.
        self.declare_parameter("world_to_map_yaw", 0.016341999999999857)
        self.declare_parameter("world_to_map_tx", 0.43860305302501423)
        self.declare_parameter("world_to_map_ty", -1.4371280282927352)

        csv_path = self.get_parameter("static_csv").value
        self.resolution = float(self.get_parameter("resolution").value)
        self.margin = float(self.get_parameter("margin").value)
        self.world_to_map_yaw = float(self.get_parameter("world_to_map_yaw").value)
        self.world_to_map_tx = float(self.get_parameter("world_to_map_tx").value)
        self.world_to_map_ty = float(self.get_parameter("world_to_map_ty").value)

        self.obstacles_world = load_static_obstacles(csv_path)
        if not self.obstacles_world:
            self.get_logger().error(f"{csv_path}에서 장애물을 읽지 못했습니다.")

        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.costmap_pub = self.create_publisher(OccupancyGrid, "/static_obstacle_costmap", qos)

        self.timer = self.create_timer(1.0, self.publish_once)
        self.published = False

    def publish_once(self):
        if self.published or not self.obstacles_world:
            return

        obstacles_map = [self.transform_obstacle(obs) for obs in self.obstacles_world]
        costmap = self.build_costmap(obstacles_map)
        self.costmap_pub.publish(costmap)
        self.get_logger().info("Static obstacle costmap published.")
        self.published = True

    def transform_obstacle(self, obs: StaticObstacle) -> StaticObstacle:
        c = math.cos(self.world_to_map_yaw)
        s = math.sin(self.world_to_map_yaw)
        map_x = c * obs.x - s * obs.y + self.world_to_map_tx
        map_y = s * obs.x + c * obs.y + self.world_to_map_ty
        return StaticObstacle(map_x, map_y, obs.yaw + self.world_to_map_yaw)

    def build_costmap(self, obstacles: List[StaticObstacle]) -> OccupancyGrid:
        half_x = HALF_LENGTH + self.margin
        half_y = HALF_WIDTH + self.margin
        min_x = min(obs.x - half_x for obs in obstacles)
        max_x = max(obs.x + half_x for obs in obstacles)
        min_y = min(obs.y - half_y for obs in obstacles)
        max_y = max(obs.y + half_y for obs in obstacles)

        width = max(1, math.ceil((max_x - min_x) / self.resolution))
        height = max(1, math.ceil((max_y - min_y) / self.resolution))
        data = [0] * (width * height)

        for oy in range(height):
            y = min_y + (oy + 0.5) * self.resolution
            for ox in range(width):
                x = min_x + (ox + 0.5) * self.resolution
                if self.point_inside(x, y, obstacles):
                    data[oy * width + ox] = 100

        grid = OccupancyGrid()
        grid.header.frame_id = "map"
        grid.header.stamp = self.get_clock().now().to_msg()
        grid.info.resolution = self.resolution
        grid.info.width = width
        grid.info.height = height
        grid.info.origin.position.x = min_x
        grid.info.origin.position.y = min_y
        grid.info.origin.orientation.w = 1.0
        grid.data = data
        return grid

    def point_inside(self, x: float, y: float, obstacles: List[StaticObstacle]) -> bool:
        half_x = HALF_LENGTH + self.margin
        half_y = HALF_WIDTH + self.margin
        for obs in obstacles:
            dx = x - obs.x
            dy = y - obs.y
            cos_yaw = math.cos(obs.yaw)
            sin_yaw = math.sin(obs.yaw)
            local_x = cos_yaw * dx + sin_yaw * dy
            local_y = -sin_yaw * dx + cos_yaw * dy
            if abs(local_x) <= half_x and abs(local_y) <= half_y:
                return True
        return False


def main():
    rclpy.init()
    node = StaticObstacleVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

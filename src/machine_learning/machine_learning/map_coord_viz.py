#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import csv
import os

import rclpy
from rclpy.node import Node

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy


class MapCoordVisualizer(Node):
    def __init__(self):
        super().__init__('map_coord_visualizer')

        # Parameters
        default_csv = '/root/f1tenth_simulator/src/machine_learning/config/map_coord.csv'
        self.declare_parameter('csv_path', default_csv)
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('point_size', 0.05)    # RViz 점 크기[m]
        self.declare_parameter('z_offset', 0.02)      # 맵 위로 띄울 높이[m]
        self.declare_parameter('publish_rate', 1.0)   # Marker 퍼블리시 주기[Hz]

        self.csv_path = (
            self.get_parameter('csv_path')
            .get_parameter_value().string_value
        )
        self.frame_id = (
            self.get_parameter('frame_id')
            .get_parameter_value().string_value
        )
        self.point_size = (
            self.get_parameter('point_size')
            .get_parameter_value().double_value
        )
        self.z_offset = (
            self.get_parameter('z_offset')
            .get_parameter_value().double_value
        )
        self.publish_rate = (
            self.get_parameter('publish_rate')
            .get_parameter_value().double_value
        )

        # Marker Publisher (TRANSIENT_LOCAL so RViz가 나중에 켜져도 수신 가능)
        marker_qos = QoSProfile(depth=1)
        marker_qos.reliability = ReliabilityPolicy.RELIABLE
        marker_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.marker_pub = self.create_publisher(
            Marker,
            '/map_wall_points',
            marker_qos
        )

        self.points = self._load_points(self.csv_path)
        if not self.points:
            self.get_logger().error('CSV에 유효한 좌표가 없어 시각화를 중단합니다.')
            raise SystemExit(1)

        self.get_logger().info(
            f"MapCoordVisualizer started\n"
            f"csv_path={self.csv_path}, frame_id={self.frame_id}, "
            f"point_size={self.point_size}, z_offset={self.z_offset}, "
            f"points={len(self.points)}"
        )

        period = 1.0 / self.publish_rate if self.publish_rate > 0 else 1.0
        self.timer = self.create_timer(period, self.publish_marker)
        self._first_publish = True

    def _load_points(self, csv_path):
        if not os.path.isfile(csv_path):
            self.get_logger().error(f'CSV 파일을 찾을 수 없습니다: {csv_path}')
            return []

        points = []
        with open(csv_path, newline='') as f:
            reader = csv.DictReader(f)
            for idx, row in enumerate(reader, start=2):  # header considered line 1
                try:
                    x = float(row['x'])
                    y = float(row['y'])
                except (KeyError, ValueError) as exc:
                    self.get_logger().warn(
                        f'{csv_path}:{idx} 행을 파싱할 수 없어 건너뜁니다 ({exc})'
                    )
                    continue
                points.append((x, y))

        if not points:
            self.get_logger().error('CSV에서 좌표를 하나도 읽지 못했습니다.')

        return points

    def publish_marker(self):
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'map_walls'
        marker.id = 0
        marker.type = Marker.POINTS
        marker.action = Marker.ADD

        marker.scale.x = self.point_size
        marker.scale.y = self.point_size

        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        marker.points = []

        for x, y in self.points:
            p = Point()
            p.x = x
            p.y = y
            p.z = self.z_offset
            marker.points.append(p)

        self.marker_pub.publish(marker)

        if self._first_publish:
            self.get_logger().info(
                f"CSV 좌표 {len(marker.points)}개를 Marker로 퍼블리시했습니다 "
                f"(토픽: /map_wall_points)"
            )
            self._first_publish = False


def main(args=None):
    rclpy.init(args=args)
    node = MapCoordVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

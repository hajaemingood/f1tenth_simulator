#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
bar_op TF를 읽어 실시간 위치를 OccupancyGrid(코스트맵)과 Marker로 퍼블리시하는 노드.

LaserScan 메시지 타임스탬프를 기준으로 map→bar_op 변환을 조회해
  - 실제 bar 크기
  - opponent margin이 포함된 박스
를 costmap 스타일(OccupancyGrid)과 RViz Marker로 시각화한다.
"""

import math
from dataclasses import dataclass
from typing import Dict, List, Optional, Set, Tuple

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage
from visualization_msgs.msg import Marker


def sec_nsec_to_ns(sec: int, nsec: int) -> int:
    return sec * 1_000_000_000 + nsec


IDENTITY_ROT = (0.0, 0.0, 0.0, 1.0)


@dataclass(frozen=True)
class RectangularExtent:
    front: float
    rear: float
    left: float
    right: float

    def expand(self, margin_x: float, margin_y: float) -> "RectangularExtent":
        return RectangularExtent(
            front=self.front + margin_x,
            rear=self.rear + margin_x,
            left=self.left + margin_y,
            right=self.right + margin_y,
        )

    def contains(self, dx: float, dy: float) -> bool:
        return (-self.rear <= dx <= self.front) and (-self.right <= dy <= self.left)

    @property
    def width(self) -> float:
        return self.front + self.rear

    @property
    def height(self) -> float:
        return self.left + self.right

    @property
    def offset_x(self) -> float:
        return (self.front - self.rear) / 2.0

    @property
    def offset_y(self) -> float:
        return (self.left - self.right) / 2.0


def apply_xy_offset(position: Tuple[float, float, float], orientation: Tuple[float, float, float, float], offset_x: float, offset_y: float) -> Tuple[float, float, float]:
    offset_world = rotate_vector(orientation, (offset_x, offset_y, 0.0))
    return (
        position[0] + offset_world[0],
        position[1] + offset_world[1],
        position[2] + offset_world[2],
    )


# ferrari_op.xacro 기준 차체 치수 (앞 +0.105m, 뒤 -0.15m, 폭 ±0.115m)와 추가 허용 마진
CAR_EXTENT_DEFAULT = RectangularExtent(front=0.105, rear=0.15, left=0.115, right=0.115)
DEFAULT_MARGIN_X = 0.4
DEFAULT_MARGIN_Y = 0.2


@dataclass
class TransformData:
    translation: Tuple[float, float, float]
    rotation: Tuple[float, float, float, float]


class TransformHistory:
    __slots__ = ("times", "transforms")

    def __init__(self):
        self.times: List[int] = []
        self.transforms: List[TransformData] = []

    def add(self, stamp: int, transform: TransformData):
        self.times.append(stamp)
        self.transforms.append(transform)

    def lookup(self, stamp: int) -> TransformData:
        # 최신 시각 이전의 마지막 변환 사용
        left = 0
        right = len(self.times) - 1
        if right < 0:
            raise KeyError("transform history empty")
        if stamp < self.times[0]:
            raise KeyError("transform not yet available for requested time")
        while left <= right:
            mid = (left + right) // 2
            if self.times[mid] == stamp:
                return self.transforms[mid]
            if self.times[mid] < stamp:
                left = mid + 1
            else:
                right = mid - 1
        return self.transforms[right]


class TransformStore:
    def __init__(self):
        self.static: Dict[Tuple[str, str], TransformData] = {}
        self.dynamic: Dict[Tuple[str, str], TransformHistory] = {}
        self.children: Dict[str, Set[str]] = {}
        self.cached_paths: Dict[Tuple[str, str], List[str]] = {}

    def _record_edge(self, parent: str, child: str):
        self.children.setdefault(parent, set()).add(child)
        self.cached_paths.clear()

    def add_transform(
        self,
        parent: str,
        child: str,
        stamp_ns: int,
        transform: TransformData,
        is_static: bool,
    ):
        key = (parent, child)
        self._record_edge(parent, child)
        if is_static:
            self.static[key] = transform
            return

        history = self.dynamic.setdefault(key, TransformHistory())
        history.add(stamp_ns, transform)

    def lookup(self, parent: str, child: str, stamp_ns: int) -> TransformData:
        key = (parent, child)
        if key in self.static:
            return self.static[key]
        history = self.dynamic.get(key)
        if history is None:
            raise KeyError(f"{parent}->{child} transform not found")
        return history.lookup(stamp_ns)

    def get_frame_chain(self, start: str, end: str) -> Optional[List[str]]:
        if start == end:
            return [start]
        key = (start, end)
        cached = self.cached_paths.get(key)
        if cached is not None:
            return cached

        queue: List[Tuple[str, List[str]]] = [(start, [start])]
        visited = {start}

        while queue:
            current, path = queue.pop(0)
            for child in self.children.get(current, ()):
                if child in visited:
                    continue
                new_path = path + [child]
                if child == end:
                    self.cached_paths[key] = new_path
                    return new_path
                visited.add(child)
                queue.append((child, new_path))
        return None


def normalize_quaternion(q: Tuple[float, float, float, float]) -> Tuple[float, float, float, float]:
    norm = math.sqrt(sum(v * v for v in q))
    if norm < 1e-12:
        return IDENTITY_ROT
    return tuple(v / norm for v in q)


def quaternion_multiply(q1, q2):
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return (
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    )


def rotate_vector(q: Tuple[float, float, float, float], v: Tuple[float, float, float]):
    qx, qy, qz, qw = q
    vx, vy, vz = v
    tx = 2.0 * (qy * vz - qz * vy)
    ty = 2.0 * (qz * vx - qx * vz)
    tz = 2.0 * (qx * vy - qy * vx)
    rx = vx + qw * tx + (qy * tz - qz * ty)
    ry = vy + qw * ty + (qz * tx - qx * tz)
    rz = vz + qw * tz + (qx * ty - qy * tx)
    return (rx, ry, rz)


def transform_point(transform: TransformData, point: Tuple[float, float, float]):
    rotated = rotate_vector(transform.rotation, point)
    tx, ty, tz = transform.translation
    return (rotated[0] + tx, rotated[1] + ty, rotated[2] + tz)


def compose_transform(t1: TransformData, t2: TransformData) -> TransformData:
    translated = transform_point(t1, t2.translation)
    rotated = quaternion_multiply(t1.rotation, t2.rotation)
    return TransformData(translated, normalize_quaternion(rotated))


def compose_transform_chain(
    store: TransformStore,
    frames: List[str],
    stamp_ns: int,
) -> TransformData:
    transform = TransformData((0.0, 0.0, 0.0), IDENTITY_ROT)
    for parent, child in zip(frames, frames[1:]):
        next_tf = store.lookup(parent, child, stamp_ns)
        transform = compose_transform(transform, next_tf)
    return transform


def warn_once(seen: Dict[str, bool], logger, key: str, message: str):
    if not seen.get(key):
        logger.warn(message)
        seen[key] = True


def resolve_bar_transform(
    store: TransformStore,
    base_frame: str,
    bar_frame: str,
    bar_chain: Optional[List[str]],
    stamp_ns: int,
    warned: Dict[str, bool],
    logger,
) -> Optional[TransformData]:
    if bar_chain:
        frames = [base_frame, *bar_chain, bar_frame]
    else:
        frames = store.get_frame_chain(base_frame, bar_frame)
        if frames is None:
            warn_once(
                warned,
                logger,
                "bar_chain_missing",
                (
                    f"{base_frame}에서 {bar_frame}까지 이어지는 TF 체인을 아직 확보하지 못했습니다. "
                    "필요하다면 --ros-args -p bar_chain:='[\"body_op\"]' 같이 중간 프레임을 지정하세요."
                ),
            )
            return None

    try:
        return compose_transform_chain(store, frames, stamp_ns)
    except KeyError as exc:
        warn_once(
            warned,
            logger,
            f"bar_chain_lookup_{exc}",
            f"{frames} 체인 중 {exc} 변환을 {stamp_ns}ns 시각에서 찾지 못했습니다.",
        )
        return None


class BarOpCostmapVisualizer(Node):
    def __init__(self):
        super().__init__("bar_op_costmap_visualizer")

        self.declare_parameter("scan_topic", "/ferrari/scan")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("odom_frame", "odom_op")
        self.declare_parameter("base_frame", "base_link_op")
        self.declare_parameter("bar_frame", "bar_op")
        self.declare_parameter("car_front_extent", CAR_EXTENT_DEFAULT.front)
        self.declare_parameter("car_rear_extent", CAR_EXTENT_DEFAULT.rear)
        self.declare_parameter("car_side_extent", CAR_EXTENT_DEFAULT.left)
        self.declare_parameter("margin_x", DEFAULT_MARGIN_X)
        self.declare_parameter("margin_y", DEFAULT_MARGIN_Y)
        self.declare_parameter("resolution", 0.02)  # OccupancyGrid resolution
        self.declare_parameter("bar_cost", 100)
        self.declare_parameter("margin_cost", 75)
        self.declare_parameter("free_cost", 0)
        self.declare_parameter("publish_markers", True)
        self.declare_parameter("marker_height", 0.05)
        self.declare_parameter("marker_center_size", 0.03)
        self.declare_parameter("bar_chain", [])

        self.scan_topic = (
            self.get_parameter("scan_topic").get_parameter_value().string_value
        )
        self.map_frame = (
            self.get_parameter("map_frame").get_parameter_value().string_value
        )
        self.odom_frame = (
            self.get_parameter("odom_frame").get_parameter_value().string_value
        )
        self.base_frame = (
            self.get_parameter("base_frame").get_parameter_value().string_value
        )
        self.bar_frame = (
            self.get_parameter("bar_frame").get_parameter_value().string_value
        )
        self.car_front_extent = self.get_parameter("car_front_extent").value
        self.car_rear_extent = self.get_parameter("car_rear_extent").value
        self.car_side_extent = self.get_parameter("car_side_extent").value
        self.margin_x = self.get_parameter("margin_x").value
        self.margin_y = self.get_parameter("margin_y").value
        self.resolution = self.get_parameter("resolution").value
        self.bar_cost = int(self.get_parameter("bar_cost").value)
        self.margin_cost = int(self.get_parameter("margin_cost").value)
        self.free_cost = int(self.get_parameter("free_cost").value)
        publish_markers_param = (
            self.get_parameter("publish_markers").get_parameter_value().bool_value
        )
        self.publish_markers_enabled = publish_markers_param
        self.marker_height = self.get_parameter("marker_height").value
        self.marker_center = self.get_parameter("marker_center_size").value
        bar_chain_param = self.get_parameter("bar_chain")
        if bar_chain_param.type_ == Parameter.Type.STRING_ARRAY:
            self.bar_chain = [s.strip() for s in bar_chain_param.value if s.strip()]
        else:
            self.bar_chain = []

        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.RELIABLE
        self.scan_sub = self.create_subscription(
            LaserScan,
            self.scan_topic,
            self.scan_callback,
            qos,
        )
        self.grid_pub = self.create_publisher(
            OccupancyGrid,
            "/bar_op_costmap",
            qos,
        )

        self.marker_pub = None
        if self.publish_markers_enabled:
            self.marker_pub = self.create_publisher(
                Marker,
                "/bar_op_marker",
                qos,
            )

        tf_qos = QoSProfile(depth=100)
        tf_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        self.tf_sub = self.create_subscription(
            TFMessage,
            "/tf",
            self.tf_callback,
            tf_qos,
        )

        tf_static_qos = QoSProfile(depth=100)
        tf_static_qos.reliability = ReliabilityPolicy.RELIABLE
        tf_static_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.tf_static_sub = self.create_subscription(
            TFMessage,
            "/tf_static",
            self.tf_static_callback,
            tf_static_qos,
        )

        self.transform_store = TransformStore()
        self.warned: Dict[str, bool] = {}

        self.car_bbox = RectangularExtent(
            front=self.car_front_extent,
            rear=self.car_rear_extent,
            left=self.car_side_extent,
            right=self.car_side_extent,
        )
        self.margin_bbox = self.car_bbox.expand(self.margin_x, self.margin_y)

        self.get_logger().info(
            "BarOpCostmapVisualizer ready\n"
            f"scan_topic={self.scan_topic}, map_frame={self.map_frame}, odom_frame={self.odom_frame}, "
            f"base_frame={self.base_frame}, bar_frame={self.bar_frame}, resolution={self.resolution}"
        )

    def tf_callback(self, msg: TFMessage):
        for transform in msg.transforms:
            self._store_transform(transform, is_static=False)

    def tf_static_callback(self, msg: TFMessage):
        for transform in msg.transforms:
            self._store_transform(transform, is_static=True)

    def _store_transform(self, transform, is_static: bool):
        stamp_ns = sec_nsec_to_ns(
            transform.header.stamp.sec,
            transform.header.stamp.nanosec,
        )
        data = TransformData(
            translation=(
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z,
            ),
            rotation=normalize_quaternion(
                (
                    transform.transform.rotation.x,
                    transform.transform.rotation.y,
                    transform.transform.rotation.z,
                    transform.transform.rotation.w,
                )
            ),
        )
        self.transform_store.add_transform(
            transform.header.frame_id,
            transform.child_frame_id,
            stamp_ns,
            data,
            is_static=is_static,
        )

    def scan_callback(self, msg: LaserScan):
        stamp_ns = sec_nsec_to_ns(msg.header.stamp.sec, msg.header.stamp.nanosec)
        try:
            map_to_odom = self.transform_store.lookup(self.map_frame, self.odom_frame, stamp_ns)
        except KeyError:
            warn_once(
                self.warned,
                self.get_logger(),
                "map_odom_missing",
                f"map→{self.odom_frame} 변환을 {stamp_ns}ns 시각에서 찾지 못했습니다.",
            )
            return

        try:
            odom_to_base = self.transform_store.lookup(self.odom_frame, self.base_frame, stamp_ns)
        except KeyError:
            warn_once(
                self.warned,
                self.get_logger(),
                "odom_base_missing",
                f"{self.odom_frame}→{self.base_frame} 변환을 {stamp_ns}ns 시각에서 찾지 못했습니다.",
            )
            return

        map_to_base = compose_transform(map_to_odom, odom_to_base)

        base_to_bar = resolve_bar_transform(
            self.transform_store,
            self.base_frame,
            self.bar_frame,
            self.bar_chain if self.bar_chain else None,
            stamp_ns,
            self.warned,
            self.get_logger(),
        )
        if base_to_bar is None:
            return

        self.warned.pop("bar_chain_missing", None)

        map_to_bar = compose_transform(map_to_base, base_to_bar)
        car_x, car_y, _ = map_to_base.translation
        bar_x, bar_y, _ = map_to_bar.translation
        self.publish_costmap(car_x, car_y, msg.header)
        if self.marker_pub is not None:
            self.publish_markers(
                map_to_base.translation,
                map_to_base.rotation,
                map_to_bar.translation,
                msg.header,
            )

    def publish_costmap(self, center_x: float, center_y: float, header):
        margin_box = self.margin_bbox
        car_box = self.car_bbox

        min_x = center_x - margin_box.rear
        max_x = center_x + margin_box.front
        min_y = center_y - margin_box.right
        max_y = center_y + margin_box.left

        width_cells = max(
            1, math.ceil((max_x - min_x) / self.resolution)
        )
        height_cells = max(
            1, math.ceil((max_y - min_y) / self.resolution)
        )

        size = width_cells * height_cells
        data = [self.free_cost] * size
        origin_x = min_x
        origin_y = min_y

        for iy in range(height_cells):
            y_center = origin_y + (iy + 0.5) * self.resolution
            for ix in range(width_cells):
                x_center = origin_x + (ix + 0.5) * self.resolution
                idx = iy * width_cells + ix

                dx = x_center - center_x
                dy = y_center - center_y

                if not margin_box.contains(dx, dy):
                    continue

                data[idx] = self.margin_cost

                if car_box.contains(dx, dy):
                    data[idx] = self.bar_cost

        grid = OccupancyGrid()
        grid.header.frame_id = self.map_frame
        grid.header.stamp = header.stamp
        grid.info.resolution = self.resolution
        grid.info.width = width_cells
        grid.info.height = height_cells
        grid.info.origin.position.x = origin_x
        grid.info.origin.position.y = origin_y
        grid.info.origin.position.z = 0.0
        grid.info.origin.orientation.w = 1.0
        grid.data = data

        self.grid_pub.publish(grid)

    def publish_markers(
        self,
        car_position: Tuple[float, float, float],
        car_orientation: Tuple[float, float, float, float],
        bar_position: Tuple[float, float, float],
        header,
    ):
        center_marker = Marker()
        center_marker.header.frame_id = self.map_frame
        center_marker.header.stamp = header.stamp
        center_marker.ns = "bar_op_center"
        center_marker.id = 0
        center_marker.type = Marker.SPHERE
        center_marker.action = Marker.ADD
        center_marker.scale.x = self.marker_center
        center_marker.scale.y = self.marker_center
        center_marker.scale.z = self.marker_center
        center_marker.color.r = 1.0
        center_marker.color.g = 1.0
        center_marker.color.b = 0.0
        center_marker.color.a = 1.0
        center_marker.pose.position.x = bar_position[0]
        center_marker.pose.position.y = bar_position[1]
        center_marker.pose.position.z = 0.05
        center_marker.pose.orientation.w = 1.0

        car_box = self.car_bbox
        margin_box = self.margin_bbox

        car_center = apply_xy_offset(
            car_position,
            car_orientation,
            car_box.offset_x,
            car_box.offset_y,
        )
        margin_center = apply_xy_offset(
            car_position,
            car_orientation,
            margin_box.offset_x,
            margin_box.offset_y,
        )

        bar_marker = Marker()
        bar_marker.header.frame_id = self.map_frame
        bar_marker.header.stamp = header.stamp
        bar_marker.ns = "bar_op_box"
        bar_marker.id = 1
        bar_marker.type = Marker.CUBE
        bar_marker.action = Marker.ADD
        bar_marker.scale.x = car_box.width
        bar_marker.scale.y = car_box.height
        bar_marker.scale.z = self.marker_height
        bar_marker.color.r = 1.0
        bar_marker.color.g = 0.0
        bar_marker.color.b = 0.0
        bar_marker.color.a = 0.65
        bar_marker.pose.position.x = car_center[0]
        bar_marker.pose.position.y = car_center[1]
        bar_marker.pose.position.z = car_center[2] + 0.05
        bar_marker.pose.orientation.x = car_orientation[0]
        bar_marker.pose.orientation.y = car_orientation[1]
        bar_marker.pose.orientation.z = car_orientation[2]
        bar_marker.pose.orientation.w = car_orientation[3]

        margin_marker = Marker()
        margin_marker.header.frame_id = self.map_frame
        margin_marker.header.stamp = header.stamp
        margin_marker.ns = "bar_op_margin"
        margin_marker.id = 2
        margin_marker.type = Marker.CUBE
        margin_marker.action = Marker.ADD
        margin_marker.scale.x = margin_box.width
        margin_marker.scale.y = margin_box.height
        margin_marker.scale.z = self.marker_height / 2.0
        margin_marker.color.r = 0.0
        margin_marker.color.g = 0.5
        margin_marker.color.b = 1.0
        margin_marker.color.a = 0.35
        margin_marker.pose.position.x = margin_center[0]
        margin_marker.pose.position.y = margin_center[1]
        margin_marker.pose.position.z = margin_center[2] + 0.05
        margin_marker.pose.orientation.x = car_orientation[0]
        margin_marker.pose.orientation.y = car_orientation[1]
        margin_marker.pose.orientation.z = car_orientation[2]
        margin_marker.pose.orientation.w = car_orientation[3]

        self.marker_pub.publish(center_marker)
        self.marker_pub.publish(bar_marker)
        self.marker_pub.publish(margin_marker)


def main():
    rclpy.init()
    node = BarOpCostmapVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Extract map-frame poses for base_link and base_link_op at /ferrari/scan timestamps.
"""

import argparse
import csv
import math
import os
import sqlite3
from bisect import bisect_right
from dataclasses import dataclass
from typing import Dict, Iterable, List, Optional, Sequence, Set, Tuple

import yaml
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage


DEFAULT_BAG_PATH = "/root/f1tenth_simulator/src/machine_learning/bagfiles/data_3"
DEFAULT_OUTPUT = "/root/f1tenth_simulator/src/machine_learning/outputs/ferrari_links_3.csv"
DEFAULT_SCAN_TOPIC = "/ferrari/scan"
DEFAULT_LINK_SPECS = ("base_link:odom", "base_link_op:odom_op")


def parse_args():
    parser = argparse.ArgumentParser(
        description="map 프레임 기준 base_link와 base_link_op 자세를 CSV로 저장합니다."
    )
    parser.add_argument(
        "bag_path",
        nargs="?",
        default=DEFAULT_BAG_PATH,
        help=f"rosbag2(.db3) 디렉터리 경로 (기본: {DEFAULT_BAG_PATH})",
    )
    parser.add_argument(
        "-o", "--output",
        default=DEFAULT_OUTPUT,
        help=f"생성할 CSV 경로 (기본: {DEFAULT_OUTPUT})",
    )
    parser.add_argument(
        "--scan-topic",
        default=DEFAULT_SCAN_TOPIC,
        help=f"frame index 기준이 되는 LaserScan 토픽 (기본: {DEFAULT_SCAN_TOPIC})",
    )
    parser.add_argument(
        "--map-frame",
        default="map",
        help="map 프레임 이름 (기본: map)",
    )
    parser.add_argument(
        "--links",
        nargs="+",
        default=list(DEFAULT_LINK_SPECS),
        help=(
            "기록할 base_link와 해당 odom 프레임을 base:odom 형태로 지정합니다. "
            "기본값: 'base_link:odom base_link_op:odom_op'"
        ),
    )
    return parser.parse_args()


def load_metadata(bag_path: str) -> dict:
    metadata_path = os.path.join(bag_path, "metadata.yaml")
    if not os.path.isfile(metadata_path):
        raise FileNotFoundError(f"metadata.yaml을 찾을 수 없습니다: {metadata_path}")
    with open(metadata_path, "r") as f:
        return yaml.safe_load(f)


def list_db_files(bag_path: str, metadata: dict) -> List[str]:
    info = metadata.get("rosbag2_bagfile_information", {})
    files = info.get("files", [])
    db_files: List[str] = []
    for file_info in files:
        relative = file_info.get("path")
        if not relative:
            continue
        db_files.append(os.path.join(bag_path, relative))

    if db_files:
        return db_files

    db_files = [
        os.path.join(bag_path, f)
        for f in os.listdir(bag_path)
        if f.endswith(".db3")
    ]
    if not db_files:
        raise FileNotFoundError("bag 디렉터리에 .db3 파일이 없습니다.")
    return sorted(db_files)


@dataclass
class TransformData:
    translation: Tuple[float, float, float]
    rotation: Tuple[float, float, float, float]


@dataclass(frozen=True)
class LinkSpec:
    base_frame: str
    odom_frame: str


def parse_link_specs(values: Iterable[str]) -> List[LinkSpec]:
    specs: List[LinkSpec] = []
    for value in values:
        if ":" not in value:
            raise ValueError(f"--links 항목 '{value}'에서 ':' 구분자를 찾을 수 없습니다.")
        base, odom = value.split(":", 1)
        base = base.strip()
        odom = odom.strip()
        if not base or not odom:
            raise ValueError(f"--links 항목 '{value}'의 프레임 이름이 비어 있습니다.")
        specs.append(LinkSpec(base, odom))
    if not specs:
        raise ValueError("최소 1개의 --links 항목이 필요합니다.")
    return specs


class TransformHistory:
    __slots__ = ("times", "transforms")

    def __init__(self):
        self.times: List[int] = []
        self.transforms: List[TransformData] = []

    def add(self, stamp: int, transform: TransformData):
        self.times.append(stamp)
        self.transforms.append(transform)

    def lookup(self, stamp: int) -> TransformData:
        idx = bisect_right(self.times, stamp) - 1
        if idx < 0:
            raise KeyError("transform not yet available for requested time")
        return self.transforms[idx]


class TransformStore:
    def __init__(self):
        self.static: Dict[Tuple[str, str], TransformData] = {}
        self.dynamic: Dict[Tuple[str, str], TransformHistory] = {}
        self.children: Dict[str, Set[str]] = {}

    def _record_edge(self, parent: str, child: str):
        self.children.setdefault(parent, set()).add(child)

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


IDENTITY_ROT = (0.0, 0.0, 0.0, 1.0)
IDENTITY_TRANSFORM = TransformData((0.0, 0.0, 0.0), IDENTITY_ROT)


def normalize_quaternion(q: Sequence[float]) -> Tuple[float, float, float, float]:
    norm = math.sqrt(sum(v * v for v in q))
    if norm < 1e-12:
        return IDENTITY_ROT
    return tuple(v / norm for v in q)  # type: ignore[return-value]


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


def sec_nsec_to_ns(sec: int, nsec: int) -> int:
    return sec * 1_000_000_000 + nsec


def quaternion_to_yaw(q: Tuple[float, float, float, float]) -> float:
    x, y, z, w = q
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def load_tf_static(conn: sqlite3.Connection, store: TransformStore, topics: Dict[int, Tuple[str, str]]):
    static_ids = [
        topic_id
        for topic_id, (name, msg_type) in topics.items()
        if msg_type == "tf2_msgs/msg/TFMessage" and "tf_static" in name
    ]
    if not static_ids:
        return

    placeholders = ",".join("?" for _ in static_ids)
    cursor = conn.execute(
        f"SELECT data FROM messages WHERE topic_id IN ({placeholders})",
        tuple(static_ids),
    )
    for row in cursor:
        tf_msg = deserialize_message(row["data"], TFMessage)
        for transform in tf_msg.transforms:
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
            store.add_transform(
                transform.header.frame_id,
                transform.child_frame_id,
                stamp_ns,
                data,
                is_static=True,
            )


def warn_once(warned: Dict[str, bool], key: str, message: str):
    if key not in warned:
        print(message)
        warned[key] = True


def handle_scan_frame(
    store: TransformStore,
    stamp_ns: int,
    map_frame: str,
    links: Sequence[LinkSpec],
    frame_index: int,
    stamp_sec: int,
    stamp_nsec: int,
    writer: csv.writer,
    warned: Dict[str, bool],
) -> bool:
    row: List[str] = [
        str(frame_index),
        str(stamp_sec),
        str(stamp_nsec),
    ]

    for spec in links:
        try:
            map_to_odom = store.lookup(map_frame, spec.odom_frame, stamp_ns)
        except KeyError:
            warn_once(
                warned,
                f"map_{spec.odom_frame}_missing",
                f"[warn] map→{spec.odom_frame} 변환을 {stamp_ns}ns 시각에서 찾을 수 없습니다.",
            )
            return False

        try:
            odom_to_base = store.lookup(spec.odom_frame, spec.base_frame, stamp_ns)
        except KeyError:
            warn_once(
                warned,
                f"{spec.odom_frame}_{spec.base_frame}_missing",
                f"[warn] {spec.odom_frame}→{spec.base_frame} 변환을 {stamp_ns}ns 시각에서 찾을 수 없습니다.",
            )
            return False

        map_to_base = compose_transform(map_to_odom, odom_to_base)
        x, y, _ = map_to_base.translation
        yaw = quaternion_to_yaw(map_to_base.rotation)
        row.extend([
            f"{x:.6f}",
            f"{y:.6f}",
            f"{yaw:.6f}",
        ])

    writer.writerow(row)
    return True


def process_dynamic_and_scan(
    conn: sqlite3.Connection,
    store: TransformStore,
    topics: Dict[int, Tuple[str, str]],
    scan_topic: str,
    map_frame: str,
    links: Sequence[LinkSpec],
    writer: csv.writer,
    start_frame_index: int,
    warned: Dict[str, bool],
) -> Tuple[int, int]:
    id_by_name = {name: topic_id for topic_id, (name, _) in topics.items()}
    scan_topic_id = id_by_name.get(scan_topic)
    dynamic_tf_ids = [
        topic_id
        for topic_id, (name, msg_type) in topics.items()
        if msg_type == "tf2_msgs/msg/TFMessage" and "tf_static" not in name
    ]

    relevant_ids: List[int] = []
    if scan_topic_id is not None:
        relevant_ids.append(scan_topic_id)
    relevant_ids.extend(dynamic_tf_ids)

    if not relevant_ids:
        return start_frame_index, 0

    placeholders = ",".join("?" for _ in relevant_ids)
    cursor = conn.execute(
        f"SELECT timestamp, topic_id, data FROM messages "
        f"WHERE topic_id IN ({placeholders}) ORDER BY timestamp",
        tuple(relevant_ids),
    )

    frame_index = start_frame_index
    written_rows = 0

    for row in cursor:
        topic_id = row["topic_id"]
        data = row["data"]

        if scan_topic_id is not None and topic_id == scan_topic_id:
            msg = deserialize_message(data, LaserScan)
            stamp_ns = sec_nsec_to_ns(msg.header.stamp.sec, msg.header.stamp.nanosec)
            processed = handle_scan_frame(
                store,
                stamp_ns,
                map_frame,
                links,
                frame_index,
                msg.header.stamp.sec,
                msg.header.stamp.nanosec,
                writer,
                warned,
            )
            if processed:
                written_rows += 1
            frame_index += 1
        else:
            tf_msg = deserialize_message(data, TFMessage)
            for transform in tf_msg.transforms:
                stamp_ns = sec_nsec_to_ns(
                    transform.header.stamp.sec,
                    transform.header.stamp.nanosec,
                )
                transform_data = TransformData(
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
                store.add_transform(
                    transform.header.frame_id,
                    transform.child_frame_id,
                    stamp_ns,
                    transform_data,
                    is_static=False,
                )

    return frame_index, written_rows


def convert_bag_to_csv(
    bag_path: str,
    output_csv: str,
    scan_topic: str,
    map_frame: str,
    links: Sequence[LinkSpec],
) -> None:
    if not os.path.isdir(bag_path):
        raise FileNotFoundError(f"rosbag 디렉터리를 찾을 수 없습니다: {bag_path}")

    metadata = load_metadata(bag_path)
    db_files = list_db_files(bag_path, metadata)
    os.makedirs(os.path.dirname(output_csv) or ".", exist_ok=True)

    transform_store = TransformStore()

    for db_file in db_files:
        conn = sqlite3.connect(db_file)
        conn.row_factory = sqlite3.Row
        try:
            topics = {
                row["id"]: (row["name"], row["type"])
                for row in conn.execute("SELECT id, name, type FROM topics")
            }
            load_tf_static(conn, transform_store, topics)
        finally:
            conn.close()

    frame_index = 0
    rows = 0
    warned: Dict[str, bool] = {}

    with open(output_csv, "w", newline="") as csvfile:
        writer = csv.writer(csvfile)
        header = ["frame_index", "stamp_sec", "stamp_nsec"]
        for spec in links:
            header.extend([
                f"{spec.base_frame}_x",
                f"{spec.base_frame}_y",
                f"{spec.base_frame}_yaw",
            ])
        writer.writerow(header)

        for db_file in db_files:
            conn = sqlite3.connect(db_file)
            conn.row_factory = sqlite3.Row
            try:
                topics = {
                    row["id"]: (row["name"], row["type"])
                    for row in conn.execute("SELECT id, name, type FROM topics")
                }
                frame_index, processed = process_dynamic_and_scan(
                    conn,
                    transform_store,
                    topics,
                    scan_topic,
                    map_frame,
                    links,
                    writer,
                    frame_index,
                    warned,
                )
                rows += processed
            finally:
                conn.close()

    if rows == 0:
        raise RuntimeError(f"{scan_topic} 프레임 기준으로 기록된 링크 좌표가 없습니다.")

    print(f"완료: frame {rows}개 좌표를 {output_csv}에 저장했습니다.")


def main():
    args = parse_args()
    links = parse_link_specs(args.links)
    convert_bag_to_csv(
        args.bag_path,
        args.output,
        args.scan_topic,
        args.map_frame,
        links,
    )


if __name__ == "__main__":
    main()

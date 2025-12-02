#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
ROS 2 rosbag(.db3) → CSV 변환 스크립트.

LaserScan(/scan) 메시지를 읽어 각 프레임마다
  1. scan index
  2. 해당 거리 값
  3. base_link 기준 좌표(local_x, local_y)
  4. map 기준 좌표(global_x, global_y)
를 CSV로 저장한다.
"""

import argparse
import csv
import math
import os
import sqlite3
from bisect import bisect_right
from dataclasses import dataclass
from typing import Dict, List, Tuple

import yaml
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage


DEFAULT_BAG_PATH = "/root/f1tenth_simulator/src/machine_learning/bagfiles/data_1"
DEFAULT_OUTPUT = "/root/f1tenth_simulator/src/machine_learning/config/data_1.csv"


def parse_args():
    parser = argparse.ArgumentParser(
        description="rosbag2 LaserScan 데이터를 CSV로 변환합니다."
    )
    parser.add_argument(
        "bag_path",
        nargs="?",
        default=DEFAULT_BAG_PATH,
        help=(
            "rosbag2 디렉터리 경로 (metadata.yaml이 있는 폴더). "
            f"미지정 시 기본값: {DEFAULT_BAG_PATH}"
        ),
    )
    parser.add_argument(
        "-o", "--output",
        default=DEFAULT_OUTPUT,
        help=f"저장할 CSV 파일 경로 (기본: {DEFAULT_OUTPUT})",
    )
    parser.add_argument(
        "-t", "--topic",
        default="/ferrari/scan",
        help="LaserScan 토픽 이름 (기본: /ferrari/scan)",
    )
    parser.add_argument(
        "--map-frame",
        default="map",
        help="Global 프레임 이름 (기본: map)",
    )
    parser.add_argument(
        "--odom-frame",
        default="odom",
        help="odom 프레임 이름 (기본: odom)",
    )
    parser.add_argument(
        "--base-frame",
        default="base_link",
        help="차량 기준 프레임 이름 (기본: base_link)",
    )
    return parser.parse_args()


def load_metadata(bag_path: str) -> dict:
    metadata_path = os.path.join(bag_path, "metadata.yaml")
    if not os.path.isfile(metadata_path):
        raise FileNotFoundError(
            f"metadata.yaml을 찾을 수 없습니다: {metadata_path}"
        )
    with open(metadata_path, "r") as f:
        return yaml.safe_load(f)


def list_db_files(bag_path: str, metadata: dict) -> List[str]:
    info = metadata.get("rosbag2_bagfile_information", {})
    files = info.get("files", [])
    db_files = []
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
        raise FileNotFoundError("bag 디렉터리에서 .db3 파일을 찾을 수 없습니다.")
    return sorted(db_files)


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
        idx = bisect_right(self.times, stamp) - 1
        if idx < 0:
            raise KeyError("transform not yet available for requested time")
        return self.transforms[idx]


class TransformStore:
    def __init__(self):
        self.static: Dict[Tuple[str, str], TransformData] = {}
        self.dynamic: Dict[Tuple[str, str], TransformHistory] = {}

    def add_transform(
        self,
        parent: str,
        child: str,
        stamp_ns: int,
        transform: TransformData,
        is_static: bool,
    ):
        key = (parent, child)
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
LASER_FALLBACK = TransformData(
    translation=(0.0, 0.0, 0.075),  # laser_frame이 base_link 위쪽에 위치
    rotation=IDENTITY_ROT,
)


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
    # t1 ∘ t2 : 먼저 t2 적용 후 t1 적용
    translated = transform_point(t1, t2.translation)
    rotated = quaternion_multiply(t1.rotation, t2.rotation)
    return TransformData(translated, normalize_quaternion(rotated))


def sec_nsec_to_ns(sec: int, nsec: int) -> int:
    return sec * 1_000_000_000 + nsec


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


def process_dynamic_and_scan(
    conn: sqlite3.Connection,
    store: TransformStore,
    topics: Dict[int, Tuple[str, str]],
    scan_topic: str,
    map_frame: str,
    odom_frame: str,
    base_frame: str,
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
            frame_written = handle_scan_message(
                store,
                msg,
                stamp_ns,
                map_frame,
                odom_frame,
                base_frame,
                frame_index,
                writer,
                warned,
            )
            frame_index += 1
            written_rows += frame_written
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


def warn_once(warned: Dict[str, bool], key: str, message: str):
    if key not in warned:
        print(message)
        warned[key] = True


def handle_scan_message(
    store: TransformStore,
    msg: LaserScan,
    stamp_ns: int,
    map_frame: str,
    odom_frame: str,
    base_frame: str,
    frame_index: int,
    writer: csv.writer,
    warned: Dict[str, bool],
) -> int:
    try:
        map_to_odom = store.lookup(map_frame, odom_frame, stamp_ns)
    except KeyError:
        warn_once(
            warned,
            "map_odom_missing",
            f"[warn] map→odom 변환을 {stamp_ns}ns 시각에서 찾을 수 없어 frame {frame_index}를 건너뜁니다.",
        )
        return 0

    try:
        odom_to_base = store.lookup(odom_frame, base_frame, stamp_ns)
    except KeyError:
        warn_once(
            warned,
            "odom_base_missing",
            f"[warn] odom→{base_frame} 변환을 {stamp_ns}ns 시각에서 찾을 수 없어 frame {frame_index}를 건너뜁니다.",
        )
        return 0

    map_to_base = compose_transform(map_to_odom, odom_to_base)

    laser_frame = msg.header.frame_id or base_frame
    if laser_frame == base_frame:
        base_to_laser = TransformData((0.0, 0.0, 0.0), IDENTITY_ROT)
    else:
        try:
            base_to_laser = store.lookup(base_frame, laser_frame, stamp_ns)
        except KeyError:
            warn_once(
                warned,
                f"base_laser_missing_{laser_frame}",
                f"[warn] {base_frame}→{laser_frame} 변환을 찾을 수 없어 기본값(0,0,0.075)을 사용합니다.",
            )
            store.add_transform(
                base_frame,
                laser_frame,
                0,
                LASER_FALLBACK,
                is_static=True,
            )
            base_to_laser = LASER_FALLBACK

    points = extract_points(msg)
    rows_written = 0
    stamp_sec = msg.header.stamp.sec
    stamp_nsec = msg.header.stamp.nanosec

    for idx, distance, laser_x, laser_y in points:
        point_laser = (laser_x, laser_y, 0.0)
        point_base = transform_point(base_to_laser, point_laser)
        point_map = transform_point(map_to_base, point_base)

        writer.writerow([
            frame_index,
            stamp_sec,
            stamp_nsec,
            idx,
            f"{distance:.6f}",
            f"{point_base[0]:.6f}",
            f"{point_base[1]:.6f}",
            f"{point_map[0]:.6f}",
            f"{point_map[1]:.6f}",
        ])
        rows_written += 1

    return rows_written


def extract_points(msg: LaserScan) -> List[Tuple[int, float, float, float]]:
    points = []
    angle = msg.angle_min
    for idx, distance in enumerate(msg.ranges):
        if not math.isfinite(distance):
            angle += msg.angle_increment
            continue

        if distance < msg.range_min or distance > msg.range_max:
            angle += msg.angle_increment
            continue

        x = distance * math.cos(angle)
        y = distance * math.sin(angle)
        points.append((idx, distance, x, y))
        angle += msg.angle_increment

    return points


def convert_bag_to_csv(
    bag_path: str,
    output_csv: str,
    topic: str,
    map_frame: str,
    odom_frame: str,
    base_frame: str,
) -> None:
    if not os.path.isdir(bag_path):
        raise FileNotFoundError(f"rosbag 디렉터리를 찾을 수 없습니다: {bag_path}")

    metadata = load_metadata(bag_path)
    db_files = list_db_files(bag_path, metadata)
    os.makedirs(os.path.dirname(output_csv) or ".", exist_ok=True)

    transform_store = TransformStore()

    # 1) tf_static 선처리
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

    # 2) LaserScan + 동적 TF 처리
    frame_index = 0
    written_rows = 0
    warned: Dict[str, bool] = {}

    with open(output_csv, "w", newline="") as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow([
            "frame_index",
            "stamp_sec",
            "stamp_nsec",
            "scan_index",
            "distance",
            "local_x",
            "local_y",
            "global_x",
            "global_y",
        ])

        for db_file in db_files:
            conn = sqlite3.connect(db_file)
            conn.row_factory = sqlite3.Row
            try:
                topics = {
                    row["id"]: (row["name"], row["type"])
                    for row in conn.execute("SELECT id, name, type FROM topics")
                }
                frame_index, rows = process_dynamic_and_scan(
                    conn,
                    transform_store,
                    topics,
                    topic,
                    map_frame,
                    odom_frame,
                    base_frame,
                    writer,
                    frame_index,
                    warned,
                )
                written_rows += rows
            finally:
                conn.close()

    if frame_index == 0:
        raise RuntimeError(f"{topic} 토픽에서 LaserScan 메시지를 찾을 수 없습니다.")

    print(
        f"완료: frame {frame_index}개, 포인트 {written_rows}개를 "
        f"{output_csv}에 저장했습니다."
    )


def main():
    args = parse_args()
    convert_bag_to_csv(
        args.bag_path,
        args.output,
        args.topic,
        args.map_frame,
        args.odom_frame,
        args.base_frame,
    )


if __name__ == "__main__":
    main()

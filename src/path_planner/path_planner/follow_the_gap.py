#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from math import acos, cos, pi, sqrt
import numpy as np


class Follow_the_gap:
    def __init__(self):
        self.last_min_distance = float("inf")
        self._threshold = None
        # 조향 오차(rad)에 따른 권장 속도 [m/s]
        self.speed_profile = {
            "tight": 1.0,
            "medium": 1.5,
            "open": 2.0,
        }

    def preprocess_lidar(self, scan_msg):
        ranges_raw = np.array(scan_msg.ranges)

        half_window = 3
        mvg_window = 2 * half_window + 1
        padding = np.array([np.nan] * half_window)
        ranges = np.append(np.append(padding, ranges_raw), padding)
        ranges = np.convolve(ranges, np.ones(mvg_window), "valid") / mvg_window

        ranges[np.isinf(ranges) | (ranges > scan_msg.range_max)] = 10

        angle_ranges = np.arange(len(ranges_raw)) * scan_msg.angle_increment - 3 * pi / 4
        mask = (angle_ranges >= -45 / 180 * pi) & (angle_ranges <= 45 / 180 * pi)
        proc_ranges = ranges[mask]
        angle_ranges = angle_ranges[mask]

        return angle_ranges, proc_ranges

    def compute_errors(self, angle_ranges, dist_ranges, threshold):
        """가장 큰 갭을 찾고 (조향 오차(rad), 권장 속도[m/s])를 반환."""
        self._threshold = threshold
        start_idx, end_idx = self.find_max_gap(dist_ranges, threshold)

        if start_idx is None or end_idx is None:
            return 0.0, self.speed_profile["tight"]

        theta = self.calculate_angle(angle_ranges, dist_ranges, start_idx, end_idx)
        theta = float(np.clip(theta, -0.5, 0.5))  # 안전한 조향 범위로 제한

        abs_theta = abs(theta)
        if abs_theta > 0.35:
            target_speed = self.speed_profile["tight"]
        elif abs_theta > 0.175:
            target_speed = self.speed_profile["medium"]
        else:
            target_speed = self.speed_profile["open"]

        return theta, target_speed

    def find_max_gap(self, free_space_ranges, threshold):
        start_idx = 0
        max_length = 0
        curr_length = 0
        curr_idx = 0

        for k, distance in enumerate(free_space_ranges):
            if distance > threshold:
                curr_length += 1
                if curr_length == 1:
                    curr_idx = k
            else:
                if curr_length > max_length:
                    max_length = curr_length
                    start_idx = curr_idx
                curr_length = 0

        if curr_length > max_length:
            max_length = curr_length
            start_idx = curr_idx

        if max_length == 0:
            return None, None

        return start_idx, start_idx + max_length - 1

    def calculate_angle(self, angle_ranges, proc_ranges, start_idx, end_idx):
        if start_idx is None or end_idx is None:
            return 0.0

        safety_idx = 10
        start_idx = max(0, start_idx - safety_idx)
        end_idx = min(len(angle_ranges) - 1, end_idx + safety_idx)

        d1 = proc_ranges[start_idx]
        d2 = proc_ranges[end_idx]
        phi1 = abs(angle_ranges[start_idx])
        phi2 = abs(angle_ranges[end_idx])

        numerator = d1 + d2 * cos(phi1 + phi2)
        denominator = sqrt(d1**2 + d2**2 + 2 * d1 * d2 * cos(phi1 + phi2))
        if denominator <= 1e-6:
            return 0.0

        value = numerator / denominator
        value = float(np.clip(value, -1.0, 1.0))
        theta = acos(value) - phi1
        return theta
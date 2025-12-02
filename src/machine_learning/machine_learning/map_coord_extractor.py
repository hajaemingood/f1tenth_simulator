#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import yaml
import cv2
import csv
import math
import os

# 사용자 입력 부분 ----------------------------------------------------------------
yaml_path = "/root/f1tenth_simulator/src/ferrari/maps/iccas_track.yaml"   # 여기에 yaml 파일 경로 입력
output_csv = "/root/f1tenth_simulator/src/machine_learning/config/map_coord.csv"  # 저장할 CSV 파일 경로 입력
threshold = 100    # 0~255 값 중 벽을 인식할 임계값 (어두울수록 벽)
include_border = True      # 맵 이미지 경계선 전체를 벽으로 가정할지 여부
border_pixel_value = 0     # 경계선 추가 시 기록할 픽셀 값
# -------------------------------------------------------------------------------

# YAML 읽기
with open(yaml_path, 'r') as f:
    y = yaml.safe_load(f)

pgm_path = y["image"]
if not os.path.isabs(pgm_path):
    pgm_path = os.path.join(os.path.dirname(yaml_path), pgm_path)
resolution = y["resolution"]
origin = y["origin"]  # [origin_x, origin_y, yaw]
origin_x, origin_y, origin_yaw = origin

# 이미지 읽기
img = cv2.imread(pgm_path, cv2.IMREAD_GRAYSCALE)
if img is None:
    raise FileNotFoundError("PGM 파일을 찾을 수 없습니다.")

H, W = img.shape

obstacles = []
visited = set()

cos_yaw = math.cos(origin_yaw)
sin_yaw = math.sin(origin_yaw)

def grid_to_world(u, v):
    dx = (u + 0.5) * resolution
    dy = (H - v - 0.5) * resolution
    x_val = origin_x + dx * cos_yaw - dy * sin_yaw
    y_val = origin_y + dx * sin_yaw + dy * cos_yaw
    return x_val, y_val

for v in range(H):
    for u in range(W):
        pixel = img[v, u]

        # 벽이라고 판단할 픽셀 조건 (어두운 색일수록 벽)
        if pixel < threshold:

            x, y = grid_to_world(u, v)
            obstacles.append((x, y, pixel))
            visited.add((u, v))

if include_border:
    border_cells = set()
    if H > 0 and W > 0:
        for u in range(W):
            border_cells.add((u, 0))
            border_cells.add((u, H - 1))
        for v in range(H):
            border_cells.add((0, v))
            border_cells.add((W - 1, v))

    for u, v in border_cells:
        if (u, v) in visited:
            continue
        x, y = grid_to_world(u, v)
        obstacles.append((x, y, border_pixel_value))

# CSV로 저장
with open(output_csv, "w", newline="") as f:
    writer = csv.writer(f)
    writer.writerow(["x", "y", "pixel_value"])
    for o in obstacles:
        writer.writerow(o)

print(f"완료: {len(obstacles)}개의 벽 픽셀을 추출했습니다.")
print(f"CSV 저장 경로: {output_csv}")

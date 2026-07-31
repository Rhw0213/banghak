#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
relocalize.py
================
"저장된 지도"와 "지금 라이다가 찍은 스캔"을 비교해서, 로봇이 지금 그 지도
안에서 어디에(x_mm, y_mm) 어느 방향(theta_deg)을 보고 있는지 추정하는 모듈입니다.

방법: correlative scan matching (상관관계 기반 스캔 매칭)
    1. 저장된 지도에서 "각 픽셀이 가장 가까운 벽까지 거리가 얼마인지"를
       미리 계산해둡니다 (distance transform).
    2. 후보 위치(x,y)와 후보 각도를 격자로 촘촘히 돌면서, 그 위치/각도라고
       가정했을 때 지금 스캔 점들이 실제로 벽 근처에 잘 떨어지는지 점수를
       매깁니다 (점수 = 스캔 점들이 벽에서 떨어진 평균 거리, 낮을수록 좋음).
    3. 점수가 가장 좋은 (x, y, theta)를 정답으로 채택합니다.

한계 (반드시 인지하고 쓰세요):
    - 트랙이 대칭 모양이면(예: 정사각형을 90도 돌려도 벽 모양이 똑같으면)
      엉뚱한 각도를 정답으로 착각할 수 있습니다. 트랙 안 비대칭 요소
      (소품, 출입구 등)가 이 문제를 줄여주지만 100% 보장은 아닙니다.
    - 탐색 범위(x_range_mm, y_range_mm)를 벗어난 곳에 있으면 못 찾습니다.
      로봇이 대략 어디 있는지 짐작이 가면 범위를 좁혀서 정확도/속도를
      높일 수 있습니다.
    - 계산 시간이 있습니다 (범위/각도 해상도에 비례). 처음엔 --coarse로
      거칠게 빨리 찾고, 필요하면 좁은 범위로 --fine 정밀 재탐색하는 것도
      고려해볼 만합니다 (이 파일에서는 1단계 탐색만 구현되어 있습니다).
"""

import math

import numpy as np
import cv2


def build_distance_transform(map_array, obstacle_max_value=100):
    """
    map_array(0~255)에서 장애물(obstacle_max_value 미만인 픽셀)까지의
    거리(px)를, 지도의 모든 픽셀에 대해 미리 계산해둡니다.
    """
    obstacle_mask = (map_array < obstacle_max_value).astype(np.uint8)
    non_obstacle = ((1 - obstacle_mask) * 255).astype(np.uint8)
    dist_transform = cv2.distanceTransform(non_obstacle, cv2.DIST_L2, 5)
    return dist_transform


def localize(dist_transform, origin_px, mm_per_px, scan_angles_deg, scan_distances_mm,
             x_range_mm=1500, y_range_mm=1500, xy_step_mm=50, angle_step_deg=3,
             center_x_mm=0.0, center_y_mm=0.0, angle_center_deg=0.0, angle_range_deg=None):
    """
    dist_transform: build_distance_transform()의 결과
    origin_px, mm_per_px: 지도 픽셀 <-> mm 변환 기준 (astar_planner.py의 값과 일치해야 함)
    scan_angles_deg, scan_distances_mm: 지금 라이다 스캔 (센서 기준 각도/거리, mm)
    x_range_mm, y_range_mm: (center_x_mm, center_y_mm)을 중심으로 +- 얼마나 탐색할지
    xy_step_mm: 위치 탐색 간격 (작을수록 정밀하지만 느려짐)
    angle_step_deg: 각도 탐색 간격
    center_x_mm, center_y_mm: 탐색 중심 위치 (기본 지도 원점=0,0. 이전 추정 위치를
        중심으로 좁게 탐색하면 "계속 추적"용으로 훨씬 빠르게 쓸 수 있음)
    angle_center_deg, angle_range_deg: angle_range_deg를 주면 angle_center_deg 기준
        +-angle_range_deg만 탐색 (None이면 기존처럼 0~360 전체 탐색)

    반환값: (best_x_mm, best_y_mm, best_theta_deg, best_score)
        best_score는 낮을수록 잘 맞은 것 (스캔 점들이 벽에서 떨어진 평균 거리, px 단위)
    """
    h, w = dist_transform.shape
    xs = np.arange(center_x_mm - x_range_mm, center_x_mm + x_range_mm + 1, xy_step_mm)
    ys = np.arange(center_y_mm - y_range_mm, center_y_mm + y_range_mm + 1, xy_step_mm)
    X, Y = np.meshgrid(xs, ys)   # (H, W)

    if angle_range_deg is None:
        angles = np.arange(0, 360, angle_step_deg)
    else:
        angles = np.arange(angle_center_deg - angle_range_deg,
                            angle_center_deg + angle_range_deg + 1e-9, angle_step_deg)

    best_score = None
    best = None

    for theta in angles:
        rad = np.radians(scan_angles_deg + theta)
        local_dx = scan_distances_mm * np.cos(rad)   # (N,)
        local_dy = scan_distances_mm * np.sin(rad)

        gx = X[:, :, None] + local_dx[None, None, :]   # (H, W, N) mm
        gy = Y[:, :, None] + local_dy[None, None, :]
        col = np.clip((origin_px + gx / mm_per_px).astype(np.int32), 0, w - 1)
        row = np.clip((origin_px + gy / mm_per_px).astype(np.int32), 0, h - 1)

        d = dist_transform[row, col]        # (H, W, N)
        score = d.mean(axis=2)              # (H, W), 낮을수록 좋음

        idx = np.unravel_index(np.argmin(score), score.shape)
        s = float(score[idx])
        if best_score is None or s < best_score:
            best_score = s
            best = (float(X[idx]), float(Y[idx]), float(theta) % 360.0, s)

    return best


def compose_transform(offset_x_mm, offset_y_mm, offset_theta_deg,
                       local_x_mm, local_y_mm, local_theta_deg):
    """
    '이번 세션 SLAM이 말하는 위치(local_x/y/theta, 세션 원점 기준)'를,
    relocalize()로 찾은 오프셋(offset_x/y/theta)을 적용해서 '저장된 지도
    기준 실제 좌표'로 변환합니다. (2D 강체변환 합성)
    """
    rad = math.radians(offset_theta_deg)
    cos_t, sin_t = math.cos(rad), math.sin(rad)
    global_x = offset_x_mm + (local_x_mm * cos_t - local_y_mm * sin_t)
    global_y = offset_y_mm + (local_x_mm * sin_t + local_y_mm * cos_t)
    global_theta = offset_theta_deg + local_theta_deg
    return global_x, global_y, global_theta
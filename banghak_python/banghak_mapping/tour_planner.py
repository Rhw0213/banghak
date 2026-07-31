#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
tour_planner.py
================
6단계 확장 (단순화 버전): 출발점(트랙 중앙)에서 시작해서, 갈 수 있는 자유공간을
전부 한 번씩 훑고(지그재그로 왕복하면서 전체 탐색) 다시 출발점으로 복귀하는
경로를 만듭니다.

지도가 이제 작은 트랙 크기(6m x 6m)로 줄어들었기 때문에, 예전처럼 ROI로
영역을 따로 잘라내거나 테두리를 뽑는 복잡한 과정 없이, "출발점에서 갈 수 있는
영역 전체"를 그대로 대상으로 삼습니다.

동작 방식:
    1. 지도를 불러와서 occupancy grid로 바꾸고, 안전마진(--inflate-m)을 적용합니다.
    2. 안전마진이 적용된 grid에서, 출발점과 실제로 이어져 있는 자유공간
       덩어리만 골라냅니다 (연결 안 된 다른 빈 공간은 무시).
    3. 그 영역을 위쪽 줄부터 아래쪽 줄까지, --lane-spacing-m 간격으로 잘라가며
       각 줄의 좌우 끝을 왕복하는 지그재그(보움트로페돈/잔디깎기) 패턴으로
       웨이포인트를 만듭니다.
    4. 출발점 -> 첫 줄 -> 다음 줄 -> ... -> 마지막 줄 -> 출발점(복귀) 순서로
       각 구간을 A*로 이어붙여서 전체 경로를 만듭니다.

주의: 아주 단순한 방식이라, 트랙 모양이 복잡해서 한 줄 안에 자유공간이
여러 조각(섬처럼 끊어짐)으로 나뉘는 경우는 그 중 가장 큰 조각만 훑습니다.
지금처럼 단순한 소형 트랙 테스트용으로는 충분하지만, 100% 완전 커버리지가
보장되는 알고리즘은 아닙니다.

아직 실제 주행과는 연결하지 않습니다 (7단계에서 합니다).

사용 예시
    python3 tour_planner.py --file logs/slam_map_20260727_110057.npy \\
        --start 300,300

    # 훑는 줄 간격을 더 촘촘하게 (기본 0.2m)
    python3 tour_planner.py --file logs/slam_map_20260727_110057.npy \\
        --start 300,300 --lane-spacing-m 0.15
"""

import argparse
import time
from datetime import datetime
import os

import numpy as np

try:
    import cv2
except ImportError:
    cv2 = None

from log_cleanup import cleanup_old_logs
from map_viewer import load_map
from astar_planner import (
    astar,
    inflate_obstacles,
    parse_point,
    PIXELS_PER_METER,
    DEFAULT_INFLATE_M,
)

TOUR_KEEP_LAST = 5

# 지그재그로 훑을 때 줄 사이 간격(미터). 로봇 폭보다 좁게 잡아야 훑고 지나간
# 자리 사이에 안 훑은 틈이 안 남습니다. 필요하면 조정하세요.
DEFAULT_LANE_SPACING_M = 0.2

# 이 값보다 밝은(큰) 픽셀만 "확실히 빈 공간(라이다가 직접 확인함)"으로 인정합니다.
# BreezySLAM 지도는 장애물(낮은값,~0) / 미탐색(중간값,~127) / 빈공간(높은값,~255)
# 이렇게 3단계인데, 기존 코드는 "장애물이 아니면 다 자유공간"으로 잘못 취급해서
# 미탐색 지역(트랙 바깥의 회색 지역)까지 커버리지 대상에 끼어들었습니다.
# 이 값을 기준으로 미탐색 지역은 장애물과 똑같이 "못 가는 곳"으로 처리합니다.
DEFAULT_FREE_MIN_VALUE = 200


def to_confirmed_free_grid(map_array, free_min_value):
    """
    map_array(0~255)에서, free_min_value보다 밝은 픽셀(=라이다가 실제로 훑어서
    "여긴 비어있다"고 확인한 곳)만 자유공간(0)으로 인정하고, 나머지(장애물 +
    아직 못 본 미탐색 지역)는 전부 장애물(1)과 똑같이 취급합니다.
    (map_viewer.py의 to_occupancy_grid는 "장애물이냐 아니냐"만 구분해서
     미탐색 지역이 자유공간으로 새는 문제가 있어, 커버리지 계획에는 이 함수를 씁니다.)
    """
    return (map_array <= free_min_value).astype(np.uint8)


# ============================================================
# 출발점이 속한 자유공간 덩어리만 골라내기
# ============================================================
def connected_free_region(planning_grid, start_rc):
    """
    planning_grid(0=자유공간,1=장애물, 안전마진 이미 적용됨)에서, 출발점이
    속한 "연결된 자유공간 덩어리"만 1로 표시한 마스크를 반환합니다.
    """
    if cv2 is None:
        raise RuntimeError("OpenCV(cv2)가 설치되어 있지 않습니다. "
                            "pip install --break-system-packages opencv-python-headless")

    free_mask = (planning_grid == 0).astype(np.uint8)
    sr, sc = start_rc
    if free_mask[sr, sc] == 0:
        raise ValueError("출발점이 자유공간이 아닙니다 (장애물 위이거나 안전마진에 걸림).")

    num_labels, labels = cv2.connectedComponents(free_mask, connectivity=8)
    start_label = labels[sr, sc]
    region_mask = (labels == start_label).astype(np.uint8)
    return region_mask


# ============================================================
# 지그재그(보움트로페돈) 커버리지 웨이포인트 생성
# ============================================================
def _sweep_rows(rows_with_cols, left_to_right_start=True):
    """
    [(row, col_start, col_end), ...] 목록을, 줄마다 좌우를 번갈아 오가는
    지그재그로 방문하는 웨이포인트 리스트로 변환하는 헬퍼."""
    waypoints = []
    left_to_right = left_to_right_start
    for row, c1, c2 in rows_with_cols:
        if left_to_right:
            waypoints.append((row, c1))
            waypoints.append((row, c2))
        else:
            waypoints.append((row, c2))
            waypoints.append((row, c1))
        left_to_right = not left_to_right
    return waypoints


def generate_coverage_waypoints(region_mask, row_spacing_px, min_segment_px=3):
    """
    region_mask(1=이 구역 자유공간) 안을, 위쪽 줄부터 아래쪽 줄까지
    row_spacing_px 간격으로 훑는 웨이포인트 리스트를 만듭니다.

    한 줄에 자유공간 조각이 1개뿐이면(뻥 뚫린 방) 그냥 지그재그로 왕복합니다.

    가운데에 큰 장애물이 있어서 한 줄에 조각이 2개(왼쪽 통로 + 오른쪽 통로)로
    자주 끊기는 "고리형" 트랙인 경우: 매 줄마다 좌우를 오가면 장애물 때문에
    한 줄 안에서 바로 못 건너가서 매번 크게 돌아가야 하는 비효율이 생깁니다.
    이런 경우는 왼쪽 통로를 위->아래로 통째로 다 훑은 다음, 오른쪽 통로를
    아래->위로 통째로 훑어서, 옆으로 건너가는 구간이 딱 한 번만 생기게 합니다.
    """
    h, w = region_mask.shape
    rows_with_free = np.where(region_mask.any(axis=1))[0]
    if rows_with_free.size == 0:
        raise ValueError("탐색 가능한 자유공간이 없습니다.")

    row_start, row_end = int(rows_with_free.min()), int(rows_with_free.max())

    # 먼저 각 줄의 자유공간 조각들을 전부 뽑아둡니다.
    row_segments = {}
    row = row_start
    while row <= row_end:
        cols = np.where(region_mask[row] == 1)[0]
        if cols.size > 0:
            splits = np.where(np.diff(cols) > 1)[0] + 1
            segs = [seg for seg in np.split(cols, splits) if len(seg) >= min_segment_px]
            if segs:
                row_segments[row] = [(int(s[0]), int(s[-1])) for s in segs]
        row += row_spacing_px

    if not row_segments:
        raise ValueError("탐색 가능한 자유공간이 없습니다 (조각이 너무 작음).")

    max_segments = max(len(segs) for segs in row_segments.values())

    if max_segments < 2:
        # 뻥 뚫린 방: 기존처럼 줄마다 지그재그 왕복
        rows_with_cols = [
            (row, segs[0][0], segs[0][1]) for row, segs in sorted(row_segments.items())
        ]
        return _sweep_rows(rows_with_cols)

    # 고리형: 조각이 2개인 줄은 (첫 조각=왼쪽 통로 / 마지막 조각=오른쪽 통로)로
    # 나누고, 조각이 1개뿐인 줄(트랙 위/아래 뚫린 구간)은 왼쪽 통로 쪽에 붙여서
    # 같이 훑습니다.
    left_rows, right_rows = [], []
    for row in sorted(row_segments):
        segs = row_segments[row]
        left_rows.append((row, segs[0][0], segs[0][1]))
        if len(segs) >= 2:
            right_rows.append((row, segs[-1][0], segs[-1][1]))

    left_waypoints = _sweep_rows(left_rows, left_to_right_start=True)
    # 오른쪽 통로는 왼쪽을 다 훑고 내려온 지점에서 이어받도록, 아래->위 순서로 훑습니다.
    right_waypoints = _sweep_rows(list(reversed(right_rows)), left_to_right_start=True)

    return left_waypoints + right_waypoints


# ============================================================
# 투어 경로 만들기 (출발 -> 웨이포인트들 -> 출발 복귀)
# ============================================================
def build_tour(planning_grid, start_rc, waypoints_rc, allow_diagonal):
    """
    출발점 -> 웨이포인트1 -> ... -> 웨이포인트N -> 출발점(복귀) 순서로,
    각 구간을 A*로 이어붙여서 하나의 전체 경로를 만듭니다.
    """
    stops = list(waypoints_rc) + [start_rc]   # 마지막엔 반드시 출발점으로 복귀
    full_path = [start_rc]
    total_cost = 0.0
    leg_info = []
    current = start_rc

    for i, target in enumerate(stops, start=1):
        leg_path, leg_cost = astar(planning_grid, current, target,
                                    allow_diagonal=allow_diagonal)
        if leg_path is None:
            raise RuntimeError(
                f"{i}번째 구간 경로를 찾지 못했습니다: {current} -> {target}. "
                f"--inflate-m을 줄이거나 --lane-spacing-m을 조정해보세요."
            )
        full_path.extend(leg_path[1:])   # 이전 구간의 도착점과 중복되는 첫 점은 제외
        total_cost += leg_cost
        leg_info.append((current, target, leg_cost))
        current = target

    return full_path, total_cost, leg_info


# ============================================================
# 결과 저장
# ============================================================
def save_tour_results(map_array, planning_grid, start_rc, waypoints_rc,
                       full_path, source_file, out_path):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("[에러] matplotlib이 설치되어 있지 않습니다.")
        return

    os.makedirs("logs", exist_ok=True)
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")

    if out_path is None:
        base = os.path.splitext(os.path.basename(source_file))[0]
        out_path = os.path.join("logs", f"tour_path_{base}_{ts}.png")

    path_rows = [p[0] for p in full_path]
    path_cols = [p[1] for p in full_path]
    wp_rows = [r for r, c in waypoints_rc]
    wp_cols = [c for r, c in waypoints_rc]

    fig, axes = plt.subplots(1, 2, figsize=(13, 6))

    for ax, grid, title in [
        (axes[0], map_array, "Raw SLAM map + coverage tour"),
        (axes[1], planning_grid, "Planning grid (safety margin) + coverage tour"),
    ]:
        cmap = "gray" if grid is map_array else "gray_r"
        vmax = 255 if grid is map_array else 1
        ax.imshow(grid, cmap=cmap, vmin=0, vmax=vmax)
        ax.plot(path_cols, path_rows, color="red", linewidth=1.0, label="coverage path")
        ax.plot(wp_cols, wp_rows, "o", color="orange", markersize=3)
        ax.plot(start_rc[1], start_rc[0], "g*", markersize=14, label="start/home")
        ax.set_title(title)
        ax.legend(loc="upper right", fontsize=8)

    plt.tight_layout()
    fig.savefig(out_path, dpi=120)
    plt.close(fig)
    print(f"[저장 완료] 시각화 이미지: {out_path}")

    npy_out = os.path.splitext(out_path)[0] + ".npy"
    np.save(npy_out, np.array(full_path, dtype=np.int32))
    print(f"[저장 완료] 전체 투어 경로 좌표(row,col): {npy_out}")

    cleanup_old_logs("logs", "tour_path_*.png", keep_last=TOUR_KEEP_LAST)
    cleanup_old_logs("logs", "tour_path_*.npy", keep_last=TOUR_KEEP_LAST)


# ============================================================
# 메인
# ============================================================
def main():
    p = argparse.ArgumentParser(
        description="출발점(트랙 중앙)에서 갈 수 있는 영역 전체를 지그재그로 훑고 복귀하는 경로 테스트")
    p.add_argument("--file", required=True, help="5단계에서 저장한 지도 파일 경로 (.npy)")
    p.add_argument("--start", required=True, help="출발점(=복귀할 곳) 'x,y'")
    p.add_argument("--unit", choices=["px", "m"], default="px", help="--start 좌표의 단위 (기본 px)")
    p.add_argument("--lane-spacing-m", type=float, default=DEFAULT_LANE_SPACING_M,
                   help=f"지그재그로 훑을 때 줄 간격(미터), 기본 {DEFAULT_LANE_SPACING_M}m")
    p.add_argument("--free-min-value", type=int, default=DEFAULT_FREE_MIN_VALUE,
                   help=f"이 값보다 밝은 픽셀만 '확실한 자유공간'으로 인정 (기본 {DEFAULT_FREE_MIN_VALUE}, "
                        f"미탐색 지역이 자유공간으로 새는 것 방지)")
    p.add_argument("--inflate-m", type=float, default=DEFAULT_INFLATE_M,
                   help=f"안전마진(미터), 기본 {DEFAULT_INFLATE_M}m")
    p.add_argument("--no-diagonal", action="store_true", help="대각선 이동 금지")
    p.add_argument("--out", default=None, help="결과 이미지 저장 경로")
    args = p.parse_args()

    if cv2 is None:
        print("[에러] OpenCV(cv2)가 필요합니다. "
              "pip install --break-system-packages opencv-python-headless")
        return

    map_array = load_map(args.file)
    occupancy = to_confirmed_free_grid(map_array, args.free_min_value)

    inflate_px = int(round(args.inflate_m * PIXELS_PER_METER)) if args.inflate_m > 0 else 0
    planning_grid = inflate_obstacles(occupancy, inflate_px) if inflate_px > 0 else occupancy

    start_rc = parse_point(args.start, args.unit)
    print(f"출발점(복귀지점): px=(x={start_rc[1]}, y={start_rc[0]})")

    try:
        region_mask = connected_free_region(planning_grid, start_rc)
    except ValueError as e:
        print(f"[에러] {e}")
        return

    row_spacing_px = max(1, int(round(args.lane_spacing_m * PIXELS_PER_METER)))
    try:
        waypoints = generate_coverage_waypoints(region_mask, row_spacing_px)
    except ValueError as e:
        print(f"[에러] {e}")
        return
    print(f"[커버리지] 줄 간격 {args.lane_spacing_m}m({row_spacing_px}px) 기준 "
          f"웨이포인트 {len(waypoints)}개 생성")

    t0 = time.time()
    try:
        full_path, total_cost_px, leg_info = build_tour(
            planning_grid, start_rc, waypoints, allow_diagonal=not args.no_diagonal)
    except RuntimeError as e:
        print(f"[실패] {e}")
        return
    elapsed = time.time() - t0

    dist_m = total_cost_px / PIXELS_PER_METER
    print(f"[성공] 전체 탐색 경로 완성! {len(leg_info)}개 구간, "
          f"전체 이동거리≈{dist_m:.2f}m, 계산시간={elapsed:.2f}초")

    save_tour_results(map_array, planning_grid, start_rc, waypoints,
                       full_path, args.file, args.out)


if __name__ == "__main__":
    main()
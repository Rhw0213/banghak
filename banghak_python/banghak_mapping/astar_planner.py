#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
astar_planner.py
================
6단계: 5단계(map_viewer.py)에서 저장한 occupancy grid(.npy)를 불러와서
      임의의 시작점/목표점 사이에 A* 경로가 잘 나오는지 확인하는 단계.

아직 실제 주행(모터 제어)과는 연결하지 않습니다. 그건 7단계(통합)에서 합니다.
이 단계의 목표는 딱 하나: "저장된 지도 위에서 A*가 제대로 된 경로를 뽑아내는가?"

좌표 입력 방법:
    5단계에서 저장한 slam_map_*_view.png 파일을 열어서(VS Code 파일탐색기 등)
    시작점/목표점으로 쓰고 싶은 위치의 픽셀 좌표를 눈으로 확인한 다음
    --start "x,y" --goal "x,y" 형태로 넘겨주면 됩니다.
    (이미지 좌표계 기준: 왼쪽 위가 (0,0), x=가로 방향, y=세로 방향)

    미터 단위로 넣고 싶으면 --unit m 옵션을 추가하세요. 단, 이 경우
    "지도 왼쪽 위 픽셀 (0,0) = (0m, 0m)"라고 단순하게 가정하고 변환합니다.
    실제 로봇의 SLAM 원점과 지도 픽셀 원점이 정확히 일치하는지는 아직
    검증되지 않았으니(7단계에서 실제 좌표 연결할 때 다시 확인 필요),
    지금 단계에서는 --unit px로 지도를 직접 보면서 좌표를 고르는 걸 추천합니다.

사용 예시
    # 기본 사용 (픽셀 좌표, 안전마진 0.15m 자동 적용)
    python3 astar_planner.py --file logs/slam_map_20260727_110057.npy \\
        --start 200,900 --goal 900,200

    # 안전마진 없이, 대각선 이동도 금지하고 테스트
    python3 astar_planner.py --file logs/slam_map_20260727_110057.npy \\
        --start 200,900 --goal 900,200 --inflate-m 0 --no-diagonal

    # 미터 단위로 입력
    python3 astar_planner.py --file logs/slam_map_20260727_110057.npy \\
        --start 4.0,18.0 --goal 20.0,5.0 --unit m
"""

import argparse
import heapq
import math
import os
import time
from datetime import datetime

import numpy as np

from log_cleanup import cleanup_old_logs
from map_viewer import load_map, to_occupancy_grid, DEFAULT_THRESHOLD

# ============================================================
# 설정값
# ============================================================
# teleop_slam.py에서 지도를 저장할 때 쓴 값과 반드시 동일해야 합니다.
# (지도 자체는 항상 1250x1250 픽셀로 저장되지만, 그게 실제로 몇 미터를
#  나타내는지는 이 두 값의 비율로 정해집니다. teleop_slam.py를 고치면
#  여기도 같이 고쳐야 합니다.)
# ---- 지금 실제로 쓰고 있는 설정 (소형 트랙, 6m) ----
MAP_SIZE_PIXELS = 1250
MAP_SIZE_METERS = 6.0

# ---- 나중에 큰 방/트랙 전체를 다시 매핑할 경우 ----
# MAP_SIZE_PIXELS = 1250
# MAP_SIZE_METERS = 50.0
PIXELS_PER_METER = MAP_SIZE_PIXELS / MAP_SIZE_METERS

# 로봇 반경만큼 장애물을 부풀려서(inflate) 경로가 벽에 바짝 붙지 않게 하는
# 기본 안전마진(미터). 0으로 주면 이 기능을 끕니다.
DEFAULT_INFLATE_M = 0.15

ASTAR_KEEP_LAST = 5   # 결과 이미지/경로파일도 오래된 건 정리 (5단계와 동일한 방식)

SQRT2 = math.sqrt(2.0)


# ============================================================
# 장애물 팽창 (안전마진)
# ============================================================
def inflate_obstacles(occupancy, radius_px):
    """
    occupancy(1=장애물, 0=갈 수 있음) 안의 모든 장애물 픽셀 주변
    radius_px 반경 안쪽도 전부 장애물로 표시해서, 로봇이 벽에 바짝 붙어
    지나가는 경로를 A*가 고르지 못하게 만듭니다.

    scipy 없이(라즈베리파이에 안 깔려있을 수 있어서) numpy만으로 구현:
    반경 안의 (dy, dx) 오프셋들을 미리 계산해두고, 장애물 좌표들을
    그 오프셋만큼씩 이동시켜서 전부 1로 찍는 방식입니다.
    """
    if radius_px <= 0:
        return occupancy.copy()

    h, w = occupancy.shape
    inflated = occupancy.copy()

    obstacle_rows, obstacle_cols = np.where(occupancy == 1)
    if obstacle_rows.size == 0:
        # 장애물이 하나도 없으면 팽창할 게 없음
        return inflated

    r = int(radius_px)
    offsets = [
        (dy, dx)
        for dy in range(-r, r + 1)
        for dx in range(-r, r + 1)
        if dy * dy + dx * dx <= r * r
    ]

    for dy, dx in offsets:
        shifted_rows = obstacle_rows + dy
        shifted_cols = obstacle_cols + dx
        valid = (
            (shifted_rows >= 0) & (shifted_rows < h) &
            (shifted_cols >= 0) & (shifted_cols < w)
        )
        inflated[shifted_rows[valid], shifted_cols[valid]] = 1

    return inflated


# ============================================================
# A* 알고리즘
# ============================================================
def _reconstruct_path(came_from, current):
    path = [current]
    while current in came_from:
        current = came_from[current]
        path.append(current)
    path.reverse()
    return path


def astar(occupancy, start, goal, allow_diagonal=True):
    """
    occupancy: 2차원 numpy 배열 (1=장애물, 0=갈 수 있음)
    start, goal: (row, col) 튜플 (이미지 좌표계의 y, x 순서 = 배열의 axis0, axis1)
    allow_diagonal: True면 8방향 이동 허용, False면 4방향(상하좌우)만 허용

    반환값: (path, cost)
        path  - [(row, col), ...] 시작점부터 목표점까지의 경로. 못 찾으면 None
        cost  - 경로의 총 이동 거리(픽셀 단위, 대각선은 sqrt(2)로 계산). 못 찾으면 None
    """
    h, w = occupancy.shape
    sr, sc = start
    gr, gc = goal

    if not (0 <= sr < h and 0 <= sc < w):
        raise ValueError(f"시작점이 지도 범위를 벗어났습니다: (row={sr}, col={sc}), 지도 크기=({h},{w})")
    if not (0 <= gr < h and 0 <= gc < w):
        raise ValueError(f"목표점이 지도 범위를 벗어났습니다: (row={gr}, col={gc}), 지도 크기=({h},{w})")
    if occupancy[sr, sc] == 1:
        raise ValueError("시작점이 장애물(또는 안전마진) 위에 있습니다. 다른 좌표를 골라보세요.")
    if occupancy[gr, gc] == 1:
        raise ValueError("목표점이 장애물(또는 안전마진) 위에 있습니다. 다른 좌표를 골라보세요.")

    if allow_diagonal:
        # (drow, dcol, 이동비용)
        neighbors = [
            (-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
            (-1, -1, SQRT2), (-1, 1, SQRT2), (1, -1, SQRT2), (1, 1, SQRT2),
        ]
    else:
        neighbors = [(-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0)]

    def heuristic(r, c):
        dr = abs(r - gr)
        dc = abs(c - gc)
        if allow_diagonal:
            # octile distance: 대각선 이동이 있을 때 더 정확한 추정치
            return (dr + dc) + (SQRT2 - 2) * min(dr, dc)
        return dr + dc

    open_heap = [(heuristic(sr, sc), 0.0, (sr, sc))]
    came_from = {}
    g_score = {(sr, sc): 0.0}
    closed = set()

    while open_heap:
        _, cur_g, current = heapq.heappop(open_heap)

        if current in closed:
            continue
        closed.add(current)

        if current == (gr, gc):
            return _reconstruct_path(came_from, current), cur_g

        cr, cc = current
        for dr, dc, step_cost in neighbors:
            nr, nc = cr + dr, cc + dc
            if not (0 <= nr < h and 0 <= nc < w):
                continue
            if occupancy[nr, nc] == 1:
                continue

            # 대각선 이동인데 양옆 두 칸이 모두 장애물이면, 벽 모서리를 뚫고
            # 지나가는 셈이라 실제로는 불가능한 이동입니다. 이런 경우는 막습니다.
            if dr != 0 and dc != 0:
                if occupancy[cr + dr, cc] == 1 and occupancy[cr, cc + dc] == 1:
                    continue

            tentative_g = cur_g + step_cost
            neighbor = (nr, nc)
            if tentative_g < g_score.get(neighbor, float("inf")):
                g_score[neighbor] = tentative_g
                came_from[neighbor] = current
                f_score = tentative_g + heuristic(nr, nc)
                heapq.heappush(open_heap, (f_score, tentative_g, neighbor))

    return None, None


# ============================================================
# 좌표 입력 파싱
# ============================================================
def parse_point(text, unit):
    """
    "x,y" 형태의 문자열을 받아서 (row, col) 픽셀 좌표 튜플로 변환합니다.
    unit="m"이면 미터 단위 입력으로 보고 PIXELS_PER_METER로 환산합니다.
    """
    try:
        x_str, y_str = text.split(",")
        x, y = float(x_str), float(y_str)
    except ValueError:
        raise ValueError(f"좌표 형식이 잘못됐습니다: '{text}' (예: '200,900')")

    if unit == "m":
        x *= PIXELS_PER_METER
        y *= PIXELS_PER_METER

    col = int(round(x))
    row = int(round(y))
    return row, col


# ============================================================
# 결과 저장 (시각화 png + 경로 npy)
# ============================================================
def save_results(map_array, occupancy, planning_grid, path, source_file, out_path):
    try:
        import matplotlib
        matplotlib.use("Agg")   # SSH/헤드리스 환경에서도 파일 저장은 가능하게
        import matplotlib.pyplot as plt
    except ImportError:
        print("[에러] matplotlib이 설치되어 있지 않습니다. "
              "pip install --break-system-packages matplotlib")
        return

    os.makedirs("logs", exist_ok=True)
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")

    if out_path is None:
        base = os.path.splitext(os.path.basename(source_file))[0]
        out_path = os.path.join("logs", f"astar_path_{base}_{ts}.png")

    path_rows = [p[0] for p in path]
    path_cols = [p[1] for p in path]

    fig, axes = plt.subplots(1, 2, figsize=(13, 6))

    # 왼쪽: 원본 지도 위에 경로
    axes[0].imshow(map_array, cmap="gray", vmin=0, vmax=255)
    axes[0].plot(path_cols, path_rows, color="red", linewidth=1.5, label="A* path")
    axes[0].plot(path_cols[0], path_rows[0], "go", markersize=8, label="start")
    axes[0].plot(path_cols[-1], path_rows[-1], "bo", markersize=8, label="goal")
    axes[0].set_title("Raw SLAM map + A* path")
    axes[0].legend(loc="upper right", fontsize=8)

    # 오른쪽: 실제로 A*가 계산에 쓴 grid(안전마진 반영된 것) 위에 경로
    axes[1].imshow(planning_grid, cmap="gray_r", vmin=0, vmax=1)
    axes[1].plot(path_cols, path_rows, color="red", linewidth=1.5)
    axes[1].plot(path_cols[0], path_rows[0], "go", markersize=8)
    axes[1].plot(path_cols[-1], path_rows[-1], "bo", markersize=8)
    axes[1].set_title("Planning grid (with safety margin) + A* path")

    plt.tight_layout()
    fig.savefig(out_path, dpi=120)
    plt.close(fig)
    print(f"[저장 완료] 시각화 이미지: {out_path}")

    # 경로 좌표 자체도 나중에(7단계) 그대로 불러써야 하니 npy로 같이 저장
    npy_out = os.path.splitext(out_path)[0] + ".npy"
    np.save(npy_out, np.array(path, dtype=np.int32))
    print(f"[저장 완료] 경로 좌표(row,col) 배열: {npy_out}")

    # 오래된 결과 파일 정리 (5단계와 동일한 방식)
    cleanup_old_logs("logs", "astar_path_*.png", keep_last=ASTAR_KEEP_LAST)
    cleanup_old_logs("logs", "astar_path_*.npy", keep_last=ASTAR_KEEP_LAST)


# ============================================================
# 메인
# ============================================================
def main():
    p = argparse.ArgumentParser(description="저장된 SLAM 지도(.npy) 위에서 A* 경로계획 테스트")
    p.add_argument("--file", required=True, help="5단계에서 저장한 지도 파일 경로 (.npy)")
    p.add_argument("--start", required=True, help="시작점 'x,y' (기본은 픽셀 단위)")
    p.add_argument("--goal", required=True, help="목표점 'x,y' (기본은 픽셀 단위)")
    p.add_argument("--unit", choices=["px", "m"], default="px",
                   help="좌표 단위 (기본 px). m으로 주면 미터 단위 입력으로 처리")
    p.add_argument("--threshold", type=int, default=DEFAULT_THRESHOLD,
                   help=f"장애물 판단 임계값 (기본 {DEFAULT_THRESHOLD}, map_viewer.py와 동일)")
    p.add_argument("--inflate-m", type=float, default=DEFAULT_INFLATE_M,
                   help=f"장애물 안전마진(미터), 로봇 반경 정도로 생각하면 됨 (기본 {DEFAULT_INFLATE_M}m, 0=끔)")
    p.add_argument("--no-diagonal", action="store_true", help="대각선 이동 금지 (4방향만 허용)")
    p.add_argument("--out", default=None, help="결과 이미지 저장 경로 (미지정시 자동 생성)")
    args = p.parse_args()

    map_array = load_map(args.file)
    occupancy = to_occupancy_grid(map_array, threshold=args.threshold)

    inflate_px = int(round(args.inflate_m * PIXELS_PER_METER)) if args.inflate_m > 0 else 0
    if inflate_px > 0:
        print(f"[안전마진] 장애물을 {args.inflate_m}m ({inflate_px}px) 만큼 부풀립니다...")
        planning_grid = inflate_obstacles(occupancy, inflate_px)
    else:
        print("[안전마진] 사용하지 않음 (--inflate-m 0)")
        planning_grid = occupancy

    start_rc = parse_point(args.start, args.unit)
    goal_rc = parse_point(args.goal, args.unit)
    print(f"시작점: px=(x={start_rc[1]}, y={start_rc[0]})   목표점: px=(x={goal_rc[1]}, y={goal_rc[0]})")

    t0 = time.time()
    try:
        path, cost_px = astar(planning_grid, start_rc, goal_rc,
                               allow_diagonal=not args.no_diagonal)
    except ValueError as e:
        print(f"[에러] {e}")
        return
    elapsed = time.time() - t0

    if path is None:
        print(f"[실패] 경로를 찾지 못했습니다. (계산시간={elapsed:.2f}초)")
        print("       --inflate-m 값을 줄이거나, --threshold 값을 조정해보거나, "
              "지도 자체에 시작점과 목표점 사이가 실제로 막혀있는지 확인해보세요.")
        return

    dist_m = cost_px / PIXELS_PER_METER
    print(f"[성공] 경로 발견! 웨이포인트 {len(path)}개, 총 거리≈{dist_m:.2f}m, "
          f"계산시간={elapsed:.2f}초")

    save_results(map_array, occupancy, planning_grid, path, args.file, args.out)


if __name__ == "__main__":
    main()
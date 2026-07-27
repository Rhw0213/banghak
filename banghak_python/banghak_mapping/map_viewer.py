#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
map_viewer.py
================
5단계: teleop_slam.py가 저장한 지도(slam_map_*.npy)를 불러와서
      - 기본 통계(크기, 픽셀값 분포)를 출력하고
      - matplotlib으로 원본 지도를 눈으로 확인하고
      - 6단계(A*)에서 쓸 "장애물/빈길" 격자(occupancy grid)로 변환한 모습도
        미리 보여줍니다.

배경 지식 (BreezySLAM 지도 픽셀값 의미):
    slam.getmap()이 채워주는 값은 0~255 사이의 밝기값입니다.
    - 값이 높을수록(밝을수록, 255에 가까울수록) "비어있는 공간(자유공간)"일 확률이 높고
    - 값이 낮을수록(어두울수록, 0에 가까울수록) "장애물(벽 등)"일 확률이 높습니다.
    - 아직 라이다가 못 본 영역은 중간값(회색, 보통 127 근처)으로 남아있습니다.

    지난 4단계 스크린샷에서 "흰 배경에 검은 점들"로 보였던 게 바로 이 원리입니다.
    (흰색=자유공간, 검은 점=장애물로 감지된 위치)

사용 예시
    python3 map_viewer.py --file logs/slam_map_20260727_110057.npy --stats
    python3 map_viewer.py --file logs/slam_map_20260727_110057.npy --show
    python3 map_viewer.py --file logs/slam_map_20260727_110057.npy --show --threshold 100
"""

import argparse
import os

import numpy as np

from log_cleanup import cleanup_old_logs 

# ============================================================
# 설정값
# ============================================================
# 이 값보다 픽셀이 어두우면(작으면) "장애물"로, 밝으면(크면) "빈 길"로 판단합니다.
# BreezySLAM 기본 관례상 127 근처가 "미탐색 영역"이라, 그보다 확실히 낮은 값을
# 기본 임계값으로 둡니다. 지도를 보고 장애물이 너무 많거나 적게 잡히면
# --threshold 옵션으로 직접 조정해서 테스트해보세요.
DEFAULT_THRESHOLD = 100
VIEW_KEEP_LAST = 5

def load_map(path):
    """저장된 .npy 지도 파일을 numpy 2차원 배열로 불러옵니다."""
    return np.load(path)


def print_stats(map_array):
    """지도의 크기, 픽셀값 분포를 출력해서 저장이 제대로 됐는지 확인합니다."""
    h, w = map_array.shape
    print("=== 지도 통계 ===")
    print(f"크기               : {w} x {h} 픽셀")
    print(f"픽셀값 범위        : {map_array.min()} ~ {map_array.max()}")
    print(f"평균 픽셀값        : {map_array.mean():.1f}")

    # 임계값 기준으로 대략 몇 %가 장애물/빈길/미탐색인지 참고용으로 계산
    obstacle_ratio = (map_array < DEFAULT_THRESHOLD).sum() / map_array.size * 100
    free_ratio = (map_array > 200).sum() / map_array.size * 100
    unknown_ratio = 100 - obstacle_ratio - free_ratio

    print(f"장애물 추정 비율   : {obstacle_ratio:.1f}%  (픽셀값 < {DEFAULT_THRESHOLD})")
    print(f"빈 공간 추정 비율  : {free_ratio:.1f}%  (픽셀값 > 200)")
    print(f"미탐색/애매 비율   : {unknown_ratio:.1f}%")

    if obstacle_ratio < 0.5:
        print("[참고] 장애물로 잡힌 픽셀이 거의 없습니다. "
              "실제로 벽/물체 근처를 지나가며 스캔했는지 확인해보세요.")


def to_occupancy_grid(map_array, threshold=DEFAULT_THRESHOLD):
    """
    BreezySLAM의 0~255 밝기값 지도를, A* 알고리즘이 바로 쓸 수 있는
    "1=장애물, 0=갈 수 있음"짜리 단순한 격자(occupancy grid)로 변환합니다.

    threshold보다 어두운 픽셀(장애물 확률이 높은 곳)만 1로 표시하고,
    나머지(빈 공간 + 아직 못 본 미탐색 영역)는 전부 0(갈 수 있음)으로 취급합니다.
    -> 미탐색 영역을 "일단 갈 수 있다"로 취급하는 건 다소 낙관적인 가정입니다.
       나중에 실제 주행 시 초음파 센서로 즉시 정지하는 안전장치와 반드시 같이 써야 합니다.
    """
    occupancy = (map_array < threshold).astype(np.uint8)
    return occupancy


def show_map(map_array, occupancy, output_path):
    """
    원격 SSH/VS Code 환경에는 화면(GUI 디스플레이)이 없는 경우가 많아서
    plt.show()로 창을 띄우는 대신, 결과를 이미지 파일로 저장합니다.
    저장된 파일은 VS Code 파일탐색기나 다른 이미지 뷰어로 열어서 확인하시면 됩니다.
    (한글 제목은 matplotlib 기본 폰트가 지원 안 해서 깨지는 경고가 나므로 영문으로 표기)
    """
    try:
        import matplotlib
        matplotlib.use("Agg")   # 화면 없이도 이미지 파일 저장만 가능한 백엔드로 강제 지정
        import matplotlib.pyplot as plt
    except ImportError:
        print("[에러] matplotlib이 설치되어 있지 않습니다. "
              "pip install --break-system-packages matplotlib")
        return

    fig, axes = plt.subplots(1, 2, figsize=(12, 6))

    axes[0].imshow(map_array, cmap="gray", vmin=0, vmax=255)
    axes[0].set_title("Raw SLAM map (brighter = free space)")

    # occupancy grid는 1(장애물)을 검게, 0(갈 수 있음)을 희게 보여줌
    axes[1].imshow(occupancy, cmap="gray_r", vmin=0, vmax=1)
    axes[1].set_title("Occupancy grid (black = obstacle)")

    plt.tight_layout()
    fig.savefig(output_path, dpi=120)
    plt.close(fig)
    print(f"[저장 완료] 시각화 이미지: {output_path}")
    print("VS Code 파일탐색기에서 이 파일을 열어 확인하세요.")

    # 새로 저장된 view.png까지 포함해서, 오래된 것은 정리하고 최근 것만 남김
    out_dir = os.path.dirname(output_path) or "."
    cleanup_old_logs(out_dir, "slam_map_2*_view.png", keep_last=VIEW_KEEP_LAST)

def main():
    p = argparse.ArgumentParser(description="SLAM 지도(.npy) 검증/시각화 도구")
    p.add_argument("--file", required=True, help="지도 파일 경로 (.npy)")
    p.add_argument("--stats", action="store_true", help="기본 통계 출력")
    p.add_argument("--show", action="store_true", help="matplotlib으로 시각화 (원본 + occupancy grid)")
    p.add_argument("--threshold", type=int, default=DEFAULT_THRESHOLD,
                   help=f"장애물 판단 임계값 (기본 {DEFAULT_THRESHOLD}, 낮을수록 장애물이 더 엄격하게 잡힘)")
    p.add_argument("--out", default=None,
                   help="시각화 결과를 저장할 png 경로 (미지정시 원본 파일명 기반으로 자동 생성)")
    args = p.parse_args()

    map_array = load_map(args.file)

    if args.stats or not args.show:
        print_stats(map_array)

    if args.show:
        occupancy = to_occupancy_grid(map_array, threshold=args.threshold)

        out_path = args.out
        if out_path is None:
            # 예: logs/slam_map_20260727_123225.npy -> logs/slam_map_20260727_123225_view.png
            base, _ = os.path.splitext(args.file)
            out_path = base + "_view.png"

        show_map(map_array, occupancy, out_path)


if __name__ == "__main__":
    main()
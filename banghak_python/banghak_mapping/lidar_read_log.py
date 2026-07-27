#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
read_lidar_log.py
==================
record_lidar.py로 저장한 로그(csv / jsonl / npy)를 다시 불러와서
- 스캔 개수, 포인트 개수, 기록 시간 등 기본 통계를 출력하고
- 원하면 특정 스캔 하나를 극좌표 형태로 간단히 시각화(matplotlib)합니다.

이 스크립트는 "저장이 제대로 됐는지" 검증하는 용도입니다.
나중에 SLAM 코드에서는 여기서 쓰는 load_* 함수들을 그대로 가져다 써도 됩니다.

사용 예시
  python read_lidar_log.py --file logs/run1.csv --stats
  python read_lidar_log.py --file logs/run1.csv --plot-scan 10
  python read_lidar_log.py --file logs/run1.npy --stats
"""

import argparse
import csv
import json
import os

import numpy as np


def load_csv(path):
    """csv 로그를 (scan_id, timestamp, quality, angle, distance) 구조화 배열로 로드"""
    rows = []
    with open(path, "r", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for r in reader:
            rows.append((
                int(r["scan_id"]),
                float(r["timestamp_unix"]),
                int(r["quality"]),
                float(r["angle_deg"]),
                float(r["distance_mm"]),
            ))
    dtype = np.dtype([
        ("scan_id", "i4"), ("timestamp", "f8"),
        ("quality", "i4"), ("angle", "f4"), ("distance", "f4"),
    ])
    return np.array(rows, dtype=dtype)


def load_jsonl(path):
    rows = []
    with open(path, "r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            rec = json.loads(line)
            for pt in rec["points"]:
                rows.append((
                    rec["scan_id"], rec["timestamp_unix"],
                    pt["quality"], pt["angle"], pt["distance"],
                ))
    dtype = np.dtype([
        ("scan_id", "i4"), ("timestamp", "f8"),
        ("quality", "i4"), ("angle", "f4"), ("distance", "f4"),
    ])
    return np.array(rows, dtype=dtype)


def load_npy(path):
    return np.load(path)


def load_log(path):
    ext = os.path.splitext(path)[1].lower()
    if ext == ".csv":
        return load_csv(path)
    elif ext == ".jsonl":
        return load_jsonl(path)
    elif ext == ".npy":
        return load_npy(path)
    else:
        raise ValueError(f"지원하지 않는 확장자입니다: {ext}")


def print_stats(arr):
    n_points = len(arr)
    n_scans = len(np.unique(arr["scan_id"])) if n_points else 0
    if n_points == 0:
        print("[통계] 데이터가 비어있습니다.")
        return
    duration = arr["timestamp"].max() - arr["timestamp"].min()
    print("=== 로그 통계 ===")
    print(f"총 포인트 수      : {n_points}")
    print(f"총 스캔(회전) 수  : {n_scans}")
    print(f"기록 시간         : {duration:.2f} 초")
    if duration > 0:
        print(f"평균 스캔 주파수  : {n_scans / duration:.2f} Hz")
    print(f"거리(mm) 범위     : {arr['distance'].min():.1f} ~ {arr['distance'].max():.1f}")
    print(f"quality 범위      : {arr['quality'].min()} ~ {arr['quality'].max()}")


def plot_scan(arr, scan_id):
    try:
        import matplotlib.pyplot as plt
    except ImportError:
        print("[에러] matplotlib이 설치되어 있지 않습니다. pip install matplotlib")
        return

    mask = arr["scan_id"] == scan_id
    if not mask.any():
        print(f"[에러] scan_id={scan_id} 데이터가 없습니다. "
              f"(유효 범위: {arr['scan_id'].min()} ~ {arr['scan_id'].max()})")
        return

    sub = arr[mask]
    angles_rad = np.deg2rad(sub["angle"])
    distances = sub["distance"]

    fig = plt.figure(figsize=(6, 6))
    ax = fig.add_subplot(111, projection="polar")
    ax.scatter(angles_rad, distances, s=4)
    ax.set_title(f"scan_id = {scan_id} ({len(sub)} 포인트)")
    plt.show()


def main():
    p = argparse.ArgumentParser(description="라이다 로그 검증/시각화")
    p.add_argument("--file", required=True, help="로그 파일 경로 (.csv / .jsonl / .npy)")
    p.add_argument("--stats", action="store_true", help="기본 통계 출력")
    p.add_argument("--plot-scan", type=int, default=None, help="지정한 scan_id 하나를 극좌표로 시각화")
    args = p.parse_args()

    arr = load_log(args.file)

    if args.stats or args.plot_scan is None:
        print_stats(arr)

    if args.plot_scan is not None:
        plot_scan(arr, args.plot_scan)
#실행할 때는 python3 lidar_read_log.py --file logs/방금저장된파일명(lidar_record.py에서 한 거).csv --stats를 실행하면 됩니다.

if __name__ == "__main__":
    main()
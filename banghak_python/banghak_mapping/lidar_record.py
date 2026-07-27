#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
record_lidar.py
================
2단계: 라이다 데이터 저장 (로그 수집)

- SLAM은 붙이지 않고, lidar.iter_scans()에서 나오는
  (quality, angle, distance) 원본 스캔 데이터를 타임스탬프와 함께
  파일로만 저장하는 것이 목표.
- 한 번 저장해두면 이후 SLAM 파라미터를 바꿔가며 "같은 데이터"로
  여러 번 재실행(replay)해서 디버깅할 수 있음.

지원 저장 포맷
  - csv   : 사람이 읽기 편하고, 한 줄씩 즉시 flush되어 중간에 프로그램이
            죽어도 데이터 유실이 가장 적음. (기본값, 추천)
  - jsonl : 한 줄에 스캔 하나(JSON). csv보다 구조가 유연함.
  - npy   : 넘파이 구조화 배열. 가장 빠르게 읽고 SLAM 코드에 바로 넣기 좋지만,
            프로그램이 끝나야(또는 주기적으로) 파일에 기록됨.

사용 예시
  python record_lidar.py --port /dev/ttyUSB0 --format csv --out logs/run1.csv
  python record_lidar.py --port COM3 --format npy --out logs/run1.npy
  python record_lidar.py --port /dev/ttyUSB0 --duration 60   # 60초만 기록하고 자동 종료

의존성
  pip install rplidar-roboticia
  (import 이름은 그대로 `rplidar` 입니다)
"""

import argparse
import csv
import json
import os
import signal
import sys
import time
from datetime import datetime
from log_cleanup import cleanup_old_logs

try:
    from rplidar import RPLidar, RPLidarException
except ImportError:
    print("[에러] rplidar 패키지가 설치되어 있지 않습니다.")
    print("       pip install rplidar-roboticia  를 먼저 실행하세요.")
    sys.exit(1)

try:
    import numpy as np
except ImportError:
    np = None  # npy 포맷을 쓸 때만 필요


class LidarLogger:
    def __init__(self, port, output_path, fmt="csv", baudrate=115200,
                 timeout=3, max_scans=None, max_seconds=None,
                 flush_every=5):
        self.port = port
        self.output_path = output_path
        self.fmt = fmt
        self.baudrate = baudrate
        self.timeout = timeout
        self.max_scans = max_scans
        self.max_seconds = max_seconds
        self.flush_every = flush_every

        self.lidar = None
        self._stop_requested = False

        # npy/jsonl 버퍼용 (csv는 즉시 파일에 씀)
        self._buffer_rows = []  # (scan_id, timestamp, quality, angle, distance)

        os.makedirs(os.path.dirname(os.path.abspath(output_path)) or ".", exist_ok=True)

        log_dir = os.path.dirname(os.path.abspath(output_path)) or "."
        cleanup_old_logs(log_dir, f"lidar_*.{fmt}", keep_last=4)

    # ------------------------------------------------------------------
    # 종료 처리 (Ctrl+C 등)
    # ------------------------------------------------------------------
    def _handle_sigint(self, signum, frame):
        print("\n[알림] 종료 요청 감지. 안전하게 정지 및 저장 중...")
        self._stop_requested = True

    # ------------------------------------------------------------------
    # 메인 기록 루프
    # ------------------------------------------------------------------
    def run(self):
        signal.signal(signal.SIGINT, self._handle_sigint)

        print(f"[정보] 라이다 연결 시도: port={self.port}, baudrate={self.baudrate}")
        self.lidar = RPLidar(self.port, baudrate=self.baudrate, timeout=self.timeout)

        try:
            info = self.lidar.get_info()
            health = self.lidar.get_health()
            print(f"[정보] 라이다 정보: {info}")
            print(f"[정보] 라이다 상태: {health}")
        except RPLidarException as e:
            print(f"[경고] 정보 조회 실패(계속 진행): {e}")

        scan_id = 0
        start_time = time.time()

        # csv는 스트리밍 write (한 줄씩 즉시 flush) -> 중간에 죽어도 안전
        csv_file = None
        csv_writer = None
        if self.fmt == "csv":
            csv_file = open(self.output_path, "w", newline="", encoding="utf-8")
            csv_writer = csv.writer(csv_file)
            csv_writer.writerow(["scan_id", "timestamp_unix", "timestamp_iso",
                                  "quality", "angle_deg", "distance_mm"])

        jsonl_file = None
        if self.fmt == "jsonl":
            jsonl_file = open(self.output_path, "w", encoding="utf-8")

        rows_since_flush = 0

        try:
            print("[정보] 스캔 시작. Ctrl+C로 안전하게 종료할 수 있습니다.")
            for scan in self.lidar.iter_scans():
                # scan: [(quality, angle, distance), ...]  -> 한 바퀴(360도) 분량
                now = time.time()
                ts_iso = datetime.fromtimestamp(now).isoformat(timespec="milliseconds")

                if self.fmt == "csv":
                    for quality, angle, distance in scan:
                        csv_writer.writerow([scan_id, f"{now:.6f}", ts_iso,
                                              quality, f"{angle:.3f}", f"{distance:.2f}"])
                    rows_since_flush += 1
                    if rows_since_flush >= self.flush_every:
                        csv_file.flush()
                        os.fsync(csv_file.fileno())
                        rows_since_flush = 0

                elif self.fmt == "jsonl":
                    record = {
                        "scan_id": scan_id,
                        "timestamp_unix": now,
                        "timestamp_iso": ts_iso,
                        "points": [
                            {"quality": q, "angle": a, "distance": d}
                            for q, a, d in scan
                        ],
                    }
                    jsonl_file.write(json.dumps(record, ensure_ascii=False) + "\n")
                    rows_since_flush += 1
                    if rows_since_flush >= self.flush_every:
                        jsonl_file.flush()
                        os.fsync(jsonl_file.fileno())
                        rows_since_flush = 0

                elif self.fmt == "npy":
                    for quality, angle, distance in scan:
                        self._buffer_rows.append((scan_id, now, quality, angle, distance))

                if scan_id % 20 == 0:
                    elapsed = now - start_time
                    print(f"[진행] scan #{scan_id} | 포인트 {len(scan)}개 | "
                          f"경과 {elapsed:.1f}s", end="\r")

                scan_id += 1

                # 종료 조건 체크
                if self._stop_requested:
                    break
                if self.max_scans is not None and scan_id >= self.max_scans:
                    print(f"\n[정보] 지정한 max_scans({self.max_scans})에 도달하여 종료합니다.")
                    break
                if self.max_seconds is not None and (now - start_time) >= self.max_seconds:
                    print(f"\n[정보] 지정한 시간({self.max_seconds}초)에 도달하여 종료합니다.")
                    break

        except RPLidarException as e:
            print(f"\n[에러] 라이다 통신 오류: {e}")
        finally:
            self._shutdown(csv_file, jsonl_file, scan_id, start_time)

    # ------------------------------------------------------------------
    def _shutdown(self, csv_file, jsonl_file, scan_id, start_time):
        print("\n[정보] 라이다 정지 및 연결 해제 중...")
        try:
            self.lidar.stop()
            self.lidar.stop_motor()
            self.lidar.disconnect()
        except Exception as e:
            print(f"[경고] 라이다 정지 중 오류(무시): {e}")

        if csv_file:
            csv_file.close()
        if jsonl_file:
            jsonl_file.close()

        if self.fmt == "npy":
            if np is None:
                print("[에러] numpy가 설치되어 있지 않아 npy로 저장할 수 없습니다. "
                      "pip install numpy 를 먼저 실행하세요.")
            else:
                dtype = np.dtype([
                    ("scan_id", "i4"),
                    ("timestamp", "f8"),
                    ("quality", "i4"),
                    ("angle", "f4"),
                    ("distance", "f4"),
                ])
                arr = np.array(self._buffer_rows, dtype=dtype)
                np.save(self.output_path, arr)
                print(f"[정보] npy 저장 완료: {self.output_path} (총 {len(arr)} 포인트)")

        elapsed = time.time() - start_time
        print(f"[완료] 총 {scan_id}개 스캔, {elapsed:.1f}초 동안 기록됨.")
        print(f"[완료] 저장 위치: {os.path.abspath(self.output_path)}")


def parse_args():
    p = argparse.ArgumentParser(description="라이다 원본 스캔 데이터 로거 (SLAM 미적용, 저장 전용)")
    p.add_argument("--port", required=True, help="라이다 시리얼 포트 (예: /dev/ttyUSB0 또는 COM3)")
    p.add_argument("--out", default=None, help="출력 파일 경로 (미지정시 logs/lidar_YYYYmmdd_HHMMSS.<ext> 자동생성)")
    p.add_argument("--format", choices=["csv", "jsonl", "npy"], default="csv", help="저장 포맷")
    p.add_argument("--baudrate", type=int, default=115200)
    p.add_argument("--timeout", type=int, default=3)
    p.add_argument("--max-scans", type=int, default=None, help="이 스캔 수만큼 기록 후 자동 종료")
    p.add_argument("--duration", type=float, default=None, help="이 초(second)만큼 기록 후 자동 종료")
    p.add_argument("--flush-every", type=int, default=5, help="csv/jsonl에서 몇 스캔마다 디스크에 flush할지")
    return p.parse_args()


def main():
    args = parse_args()

    out = args.out
    if out is None:
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        ext = {"csv": "csv", "jsonl": "jsonl", "npy": "npy"}[args.format]
        out = os.path.join("logs", f"lidar_{ts}.{ext}")

    logger = LidarLogger(
        port=args.port,
        output_path=out,
        fmt=args.format,
        baudrate=args.baudrate,
        timeout=args.timeout,
        max_scans=args.max_scans,
        max_seconds=args.duration,
        flush_every=args.flush_every,
    )
    logger.run()
# 실행 명령어는 python3 lidar_record.py --port /dev/ttyUSB0입니다

if __name__ == "__main__":
    main()
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
lidar_live_mapping.py
=======================
역할: 차량을 움직이지 않고, 그 자리에서 라이다가 스캔하는 것을 실시간으로
      BreezySLAM에 넣어 지도를 계속 갱신하고, 그 지도를 노트북 브라우저에서
      실시간으로 볼 수 있게 합니다.

live_mapping.py와의 차이점:
    - 조종(w/a/s/d 키보드, robot_hat 모터/서보) 관련 코드를 전부 제거했습니다.
    - curses도 필요 없어져서, 그냥 터미널에 상태를 print로 출력합니다.
    - 차량은 가만히 있고, 라이다만 그 자리에서 계속 회전하며 스캔합니다.

동작 구조 (스레드 2개):
    메인 스레드        : Ctrl+C를 기다리며, 주기적으로 상태(스캔 수, 추정 위치)를 출력
    라이다+SLAM 스레드  : lidar.iter_scans()로 스캔을 받아 즉시 slam.update() 반영,
                        몇 스캔마다 지도 이미지를 web/live_map.png로 저장
    (웹서버는 별도 스레드에서 표준 라이브러리 http.server로 돌아갑니다)

노트북에서 보는 방법:
    1) 라즈베리파이에서 hostname -I 로 IP 확인 (예: 192.168.0.23)
    2) 노트북 브라우저에서 http://192.168.0.23:8000 접속
    3) 지도가 1초마다 자동 갱신됨

주의:
    - 지금은 정확한 오도메트리가 필요 없습니다. 차량이 움직이지 않으니(제자리 스캔),
      SLAM은 사실상 "위치 추정" 없이 "그 자리에서 보이는 벽만 지도에 쌓는" 역할을 합니다.
    - 이 스크립트를 돌리는 동안 lidar_record.py 등 라이다를 쓰는 다른 프로그램과
      동시에 실행하면 안 됩니다 (시리얼 포트 충돌).

사용법:
    python3 lidar_live_mapping.py --port /dev/ttyUSB0

의존성:
    BreezySLAM (GitHub에서 빌드 설치, slam_offline_test.py 안내 참고)
    pip install Pillow numpy rplidar-roboticia --break-system-packages
"""

import argparse
import http.server
import os
import signal
import socketserver
import sys
import threading
import time

import numpy as np
from PIL import Image

from rplidar import RPLidar, RPLidarException
from breezyslam.algorithms import RMHC_SLAM
from breezyslam.sensors import RPLidarA1


# ============================================================
# 설정값 - 라이다 (lidar_record.py / lidar_ultra_avoidance.py와 동일)
# ============================================================
SCAN_MIN_LEN = 60
MAX_DISTANCE_MM = 12000

# ============================================================
# 설정값 - SLAM / 지도
# ============================================================
MAP_SIZE_PIXELS = 500
MAP_SIZE_METERS = 10
MAP_UPDATE_EVERY_N_SCANS = 3   # 몇 스캔마다 웹에 보여줄 지도 이미지를 새로 저장할지

# ============================================================
# 설정값 - 웹서버
# ============================================================
WEB_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "web")
HTTP_PORT = 8000


stop_event = threading.Event()

status = {
    "lidar_connected": False,
    "scan_count": 0,
    "last_error": "",
    "pose_x_mm": 0.0,
    "pose_y_mm": 0.0,
    "pose_theta_deg": 0.0,
}


# ============================================================
# 스캔 -> BreezySLAM 포맷 변환 (slam_offline_test.py와 동일 로직)
# ============================================================
def scan_to_breezyslam_format(scan, scan_size=360, max_distance_mm=MAX_DISTANCE_MM):
    distances = [0] * scan_size
    for quality, angle, distance in scan:
        if distance <= 0 or distance > max_distance_mm:
            continue
        idx = int(round(angle)) % scan_size
        d = int(distance)
        if distances[idx] == 0 or d < distances[idx]:
            distances[idx] = d
    return distances


# ============================================================
# 지도 이미지를 파일로 저장 (원자적 교체: 읽는 도중 깨진 파일 방지)
# ============================================================
def save_map_image(slam, path):
    mapbytes = bytearray(MAP_SIZE_PIXELS * MAP_SIZE_PIXELS)
    slam.getmap(mapbytes)
    arr = np.frombuffer(bytes(mapbytes), dtype=np.uint8).reshape(MAP_SIZE_PIXELS, MAP_SIZE_PIXELS)
    img = Image.fromarray(arr, mode="L")
    tmp_path = path + ".tmp"
    img.save(tmp_path, format="PNG")  # 확장자가 .tmp라 PIL이 포맷을 못 알아채므로 명시
    os.replace(tmp_path, path)  # 원자적 교체


# ============================================================
# 라이다 + 실시간 SLAM 스레드
# ============================================================
def lidar_slam_worker(port):
    try:
        lidar = RPLidar(port)
    except Exception as e:
        status["last_error"] = f"라이다 연결 실패: {e}"
        return

    status["lidar_connected"] = True

    laser = RPLidarA1()
    laser.distance_no_detection_mm = MAX_DISTANCE_MM
    slam = RMHC_SLAM(laser, MAP_SIZE_PIXELS, MAP_SIZE_METERS)

    map_path = os.path.join(WEB_DIR, "live_map.png")
    scan_id = 0

    try:
        for scan in lidar.iter_scans(min_len=SCAN_MIN_LEN):
            if len(scan) < 30:
                continue

            distances_mm = scan_to_breezyslam_format(scan, scan_size=laser.scan_size)

            # 차량이 움직이지 않으므로 오도메트리 없이 그대로 반영
            slam.update(distances_mm)

            scan_id += 1
            status["scan_count"] = scan_id

            if scan_id % MAP_UPDATE_EVERY_N_SCANS == 0:
                x_mm, y_mm, theta_deg = slam.getpos()
                status["pose_x_mm"] = x_mm
                status["pose_y_mm"] = y_mm
                status["pose_theta_deg"] = theta_deg
                save_map_image(slam, map_path)

            if stop_event.is_set():
                break

    except RPLidarException as e:
        status["last_error"] = f"라이다 통신 오류: {e}"
    finally:
        try:
            lidar.stop()
            lidar.stop_motor()
            lidar.disconnect()
        except Exception:
            pass
        status["lidar_connected"] = False


# ============================================================
# 웹서버 (표준 라이브러리만 사용, 노트북에 설치할 것 없음)
# ============================================================
def setup_web_dir():
    os.makedirs(WEB_DIR, exist_ok=True)
    index_path = os.path.join(WEB_DIR, "index.html")
    with open(index_path, "w", encoding="utf-8") as f:
        f.write("""<!DOCTYPE html>
<html><head><meta charset="utf-8"><title>실시간 지도</title></head>
<body style="margin:0;background:#1e1e1e;display:flex;justify-content:center;align-items:center;height:100vh;">
  <div style="text-align:center;">
    <img id="map" src="live_map.png" style="max-width:90vw;max-height:85vh;image-rendering:pixelated;border:1px solid #444;">
    <p style="color:#aaa;font-family:sans-serif;">1초마다 자동 갱신됩니다</p>
  </div>
  <script>
    setInterval(function () {
      document.getElementById('map').src = 'live_map.png?' + Date.now();
    }, 1000);
  </script>
</body></html>""")

    # 최초 접속 시 이미지가 없어서 깨지지 않도록, 빈 회색 지도를 미리 하나 만들어둠
    map_path = os.path.join(WEB_DIR, "live_map.png")
    if not os.path.exists(map_path):
        blank = np.full((MAP_SIZE_PIXELS, MAP_SIZE_PIXELS), 127, dtype=np.uint8)
        Image.fromarray(blank, mode="L").save(map_path)


class QuietHandler(http.server.SimpleHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=WEB_DIR, **kwargs)

    def log_message(self, fmt, *args):
        pass  # 요청마다 로그가 찍히면 상태 출력과 뒤섞이니 조용히 함


def start_web_server():
    httpd = socketserver.ThreadingTCPServer(("0.0.0.0", HTTP_PORT), QuietHandler)
    thread = threading.Thread(target=httpd.serve_forever, daemon=True)
    thread.start()
    return httpd


# ============================================================
# 메인
# ============================================================
def handle_sigint(signum, frame):
    print("\n[알림] 종료 요청 감지. 안전하게 정리 중...")
    stop_event.set()


def main():
    parser = argparse.ArgumentParser(description="제자리 라이다 실시간 SLAM + 웹 뷰어")
    parser.add_argument("--port", required=True, help="라이다 시리얼 포트 (예: /dev/ttyUSB0)")
    args = parser.parse_args()

    signal.signal(signal.SIGINT, handle_sigint)

    setup_web_dir()
    httpd = start_web_server()

    print("=== 제자리 라이다 실시간 매핑 ===")
    print(f"라즈베리파이 IP를 확인하려면 다른 터미널에서 hostname -I 를 입력하세요.")
    print(f"노트북 브라우저에서: http://<라즈베리파이IP>:{HTTP_PORT}")
    print("Ctrl+C로 안전하게 종료할 수 있습니다.")
    print("-" * 55)

    slam_thread = threading.Thread(target=lidar_slam_worker, args=(args.port,), daemon=True)
    slam_thread.start()

    try:
        while not stop_event.is_set():
            time.sleep(1)
            conn = "연결됨" if status["lidar_connected"] else "연결 대기/종료"
            print(f"[상태] 라이다: {conn} | 처리된 스캔 수: {status['scan_count']} | "
                  f"추정 위치: x={status['pose_x_mm']:.0f}mm "
                  f"y={status['pose_y_mm']:.0f}mm theta={status['pose_theta_deg']:.1f}deg",
                  end="\r")
            if status["last_error"]:
                print(f"\n[경고] {status['last_error']}")
                status["last_error"] = ""
    except KeyboardInterrupt:
        stop_event.set()
    finally:
        stop_event.set()
        slam_thread.join(timeout=5)
        httpd.shutdown()
        print("\n종료 완료.")


if __name__ == "__main__":
    main()

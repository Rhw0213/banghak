#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
teleop_slam.py
================
4단계: 조종(teleop) + 라이다 실시간 SLAM(지도 그리기)을 동시에 돌려보는 단계.
 
목적:
    - 사람이 키보드로 차를 몰면서 (2단계에서 만든 teleop 로직 그대로 재사용)
    - 동시에 라이다 스캔을 BreezySLAM에 실시간으로 넣어서 지도를 갱신하고
    - 이 과정에서 라즈베리 파이 CPU가 버티는지(부하가 어느 정도인지) 확인합니다.
 
    아직 "저장된 지도로 길찾기(A*)"는 하지 않습니다. 그건 5~6단계입니다.
    이 단계의 목표는 딱 하나: "조종 + 실시간 매핑을 같이 돌려도 시스템이 안 죽는가?"
 
구조 (2단계 teleop_record_lidar.py와 거의 동일한 스레드 구조입니다):
    메인 스레드      : curses로 키 입력을 읽어 모터/서보를 제어 (조종)
    백그라운드 스레드 : 라이다 스캔을 계속 받아서 BreezySLAM에 넣고 지도를 갱신
                       + 일정 주기로 지도를 이미지 파일로 저장 (눈으로 확인용)
                       + 일정 주기로 시스템 부하(load average)를 측정해서 공유 변수에 기록
 
조작 방법:
    w : 전진   s : 후진   a : 좌회전   d : 우회전
    x : 조향 정면복귀   space : 정지   q : 종료 (조종+매핑 모두 안전 종료)
 
저장물:
    logs/slam_map_YYYYmmdd_HHMMSS.png  (주기적으로 최신 지도로 덮어써서 저장됨)
 
의존성:
    - breezyslam (3단계에서 이미 설치하셨다고 하셨으니 별도 설치 불필요)
    - numpy (이미 설치되어 있음, 2단계에서 사용함)
    - Pillow(PIL) : 지도를 png로 저장하기 위해 필요. 없으면
        pip install --break-system-packages Pillow
"""
 
import curses
import os
import threading
import time
from datetime import datetime
 
import numpy as np
 
from robot_hat import Motor, Servo, Pin, PWM, reset_mcu
from rplidar import RPLidar, RPLidarException
 
from breezyslam.algorithms import RMHC_SLAM
from breezyslam.sensors import RPLidarA1 as LaserModel   # RPLidar A1 전용 캘리브레이션 값 내장
from log_cleanup import cleanup_old_logs
 
try:
    from PIL import Image
except ImportError:
    Image = None  # png 저장을 못 할 뿐, 나머지는 정상 동작하게 해둠
 
 
# ============================================================
# 설정값 - 조종 (기존 teleop_keyboard.py / teleop_record_lidar.py와 동일)
# ============================================================
MAX_SPEED = 50
SPEED_STEP = 3
 
STEER_LIMIT = 35
STEER_STEP = 5
 
KEY_REPEAT_TIMEOUT_MS = 100
 
# ============================================================
# 설정값 - 라이다 (lidar_ultra_avoidance.py / lidar_record.py와 동일)
# ============================================================
LIDAR_PORT = '/dev/ttyUSB0'
LIDAR_OFFSET = 90          # 기존 회피 코드와 동일한 장착 각도 보정값
                           # (SLAM 지도 방향을 실제 주행 방향과 맞추기 위해 그대로 재사용)
 
# ============================================================
# 설정값 - SLAM (BreezySLAM)
# ============================================================
MAP_SIZE_PIXELS = 500      # 지도를 500x500 픽셀 격자로 표현
MAP_SIZE_METERS = 10.0     # 그 격자가 실제로 가로세로 10m 공간을 나타냄
                            # (경기장/방 크기에 맞춰 나중에 조절 가능. 너무 작으면 지도 밖으로 나갈 때 잘림)
 
MAP_SAVE_EVERY_SEC = 5.0   # 이 주기(초)마다 지도를 png로 저장 (눈으로 진행상황 확인용)
LOAD_CHECK_EVERY_SEC = 2.0 # 이 주기(초)마다 시스템 부하(load average)를 갱신
 
 
# ============================================================
# 조종용 전역 상태 (기존과 동일)
# ============================================================
SPEED_FAST = 0
TARGET_SPEED = 0
TARGET_STEER = 0
 
stop_event = threading.Event()
 
# 화면에 보여줄 SLAM/시스템 상태 (스레드 간 공유, 표시 전용이라 락 없이 사용)
slam_status = {
    "connected": False,
    "scan_count": 0,
    "robot_x_mm": 0.0,
    "robot_y_mm": 0.0,
    "robot_theta_deg": 0.0,
    "last_error": "",
    "map_saved_path": "",
    "load_1min": 0.0,       # 최근 1분 평균 시스템 부하 (os.getloadavg() 첫 번째 값)
}
 
 
# ============================================================
# 조종 함수 (기존과 완전히 동일 - 그대로 재사용)
# ============================================================
def set_speed(left_motor, right_motor, target):
    global SPEED_FAST
    if SPEED_FAST < target:
        SPEED_FAST = min(SPEED_FAST + SPEED_STEP, target)
    elif SPEED_FAST > target:
        SPEED_FAST = max(SPEED_FAST - SPEED_STEP, target)
 
    left_motor.speed(-SPEED_FAST)
    right_motor.speed(SPEED_FAST)
 
    if SPEED_FAST == 0:
        left_motor.speed(0)
        right_motor.speed(0)
    return SPEED_FAST
 
 
def set_steer(steer_servo, angle):
    angle = max(-STEER_LIMIT, min(STEER_LIMIT, angle))
    steer_servo.angle(angle)
    return angle
 
 
def handle_key(key):
    global TARGET_SPEED, TARGET_STEER
    if key == ord('w'):
        TARGET_SPEED = MAX_SPEED
    elif key == ord('s'):
        TARGET_SPEED = -MAX_SPEED
    elif key == ord(' '):
        TARGET_SPEED = 0
    elif key == ord('a'):
        TARGET_STEER -= STEER_STEP
    elif key == ord('d'):
        TARGET_STEER += STEER_STEP
    elif key == ord('x'):
        TARGET_STEER = 0
    TARGET_STEER = max(-STEER_LIMIT, min(STEER_LIMIT, TARGET_STEER))
 
 
# ============================================================
# 라이다 스캔 -> BreezySLAM 입력 형식으로 변환
# ============================================================
def scan_to_distance_array(scan, scan_size):
    """
    라이다가 주는 원본 스캔 [(quality, angle, distance_mm), ...] (한 바퀴 분량, 개수 불규칙)을
    BreezySLAM이 요구하는 "0~359도, 정수 인덱스 360개짜리 거리 배열(mm)"로 바꿔줍니다.
 
    - 못 잡은 각도(값이 없는 각도)는 0으로 채워둡니다. (BreezySLAM은 0을 "측정 안 됨"으로 처리)
    - LIDAR_OFFSET을 적용해서, 기존 회피 코드와 동일한 방향 기준을 씁니다.
    """
    distances = [0] * scan_size   # 기본값 0 = "이 각도는 못 쟀음"
 
    for quality, angle, distance in scan:
        if distance <= 0:
            continue
        # 기존 lidar_ultra_avoidance.py의 normalize_angle과 같은 방식으로 오프셋 보정
        corrected = (angle + LIDAR_OFFSET) % 360
        index = int(corrected) % scan_size
        distances[index] = distance
 
    return distances
 
 
# ============================================================
# SLAM + 라이다 기록 스레드
# ============================================================
def slam_worker():
    """
    백그라운드 스레드.
    라이다에 연결해서 스캔이 들어올 때마다 SLAM을 갱신하고,
    주기적으로 (1) 지도를 png로 저장 (2) 시스템 부하를 측정해서 slam_status에 기록합니다.
    """
    # ---------- 라이다 모델 및 SLAM 객체 준비 ----------
    laser = LaserModel()   # RPLidar A1에 맞춰 캘리브레이션된 파라미터 (scan_size=360 등 내장)
    slam = RMHC_SLAM(laser, MAP_SIZE_PIXELS, MAP_SIZE_METERS)
 
    # SLAM이 그린 지도를 담을 바이트 배열 (한 칸 = 1픽셀, 0~255 밝기값)
    mapbytes = bytearray(MAP_SIZE_PIXELS * MAP_SIZE_PIXELS)
 
    os.makedirs("logs", exist_ok=True)

    cleanup_old_logs("logs", "slam_map_2*.png", keep_last=3)
 
    try:
        lidar = RPLidar(LIDAR_PORT)
    except Exception as e:
        slam_status["last_error"] = f"라이다 연결 실패: {e}"
        return
 
    slam_status["connected"] = True
    scan_id = 0
 
    last_map_save = 0.0
    last_load_check = 0.0
 
    try:
        for scan in lidar.iter_scans(min_len=60):   # min_len=60: 기존 주행 코드와 동일한 기준
            if stop_event.is_set():
                break
 
            # ---------- 1. 스캔을 SLAM 입력 형식으로 변환 ----------
            distances_mm = scan_to_distance_array(scan, laser.scan_size)
 
            # ---------- 2. SLAM 갱신 ----------
            # 바퀴 엔코더가 없으므로 pose_change(주행거리 정보)는 안 넘기고
            # 순수하게 라이다 스캔 매칭만으로 위치를 추정합니다. (이전에 설명드린 "라이다 오도메트리")
            slam.update(distances_mm)
 
            # 현재 추정 위치(로봇 기준 좌표계, mm 단위 x/y + 각도)
            x_mm, y_mm, theta_deg = slam.getpos()
            slam_status["robot_x_mm"] = x_mm
            slam_status["robot_y_mm"] = y_mm
            slam_status["robot_theta_deg"] = theta_deg
 
            scan_id += 1
            slam_status["scan_count"] = scan_id
 
            now = time.time()
 
            # ---------- 3. 주기적으로 지도를 이미지로 저장 (진행상황 확인용) ----------
            if Image is not None and (now - last_map_save) >= MAP_SAVE_EVERY_SEC:
                slam.getmap(mapbytes)
                # mapbytes는 1차원 바이트 배열 -> 정사각형 2차원 이미지로 재배열
                map_array = np.frombuffer(mapbytes, dtype=np.uint8).reshape(
                    (MAP_SIZE_PIXELS, MAP_SIZE_PIXELS)
                )
                img = Image.fromarray(map_array)
                save_path = os.path.join("logs", "slam_map_latest.png")
                img.save(save_path)
                slam_status["map_saved_path"] = save_path
                last_map_save = now
 
            # ---------- 4. 주기적으로 시스템 부하 체크 ----------
            if (now - last_load_check) >= LOAD_CHECK_EVERY_SEC:
                # os.getloadavg()는 라즈베리파이/리눅스에서 별도 설치 없이 바로 쓸 수 있는
                # "최근 1분/5분/15분 평균 부하"입니다. 코어 개수를 넘으면 과부하 상태로 보시면 됩니다.
                # (Pi4는 코어 4개이므로, 이 값이 4를 넘으면 CPU가 버거워하고 있다는 뜻입니다)
                load1, load5, load15 = os.getloadavg()
                slam_status["load_1min"] = load1
                last_load_check = now
 
    except RPLidarException as e:
        slam_status["last_error"] = f"라이다 통신 오류: {e}"
    finally:
        try:
            lidar.stop()
            lidar.stop_motor()
            lidar.disconnect()
        except Exception:
            pass
        slam_status["connected"] = False
 
        # 종료 직전, 마지막 지도를 한 번 더 저장 (타임스탬프 붙여서 따로 보관)
        if Image is not None:
            try:
                slam.getmap(mapbytes)
                map_array = np.frombuffer(mapbytes, dtype=np.uint8).reshape(
                    (MAP_SIZE_PIXELS, MAP_SIZE_PIXELS)
                )
                ts = datetime.now().strftime("%Y%m%d_%H%M%S")
                final_path = os.path.join("logs", f"slam_map_{ts}.png")
                Image.fromarray(map_array).save(final_path)
                slam_status["map_saved_path"] = final_path
            except Exception:
                pass
 
 
# ============================================================
# 메인 (curses)
# ============================================================
def main(stdscr):
    global SPEED_FAST, TARGET_SPEED, TARGET_STEER
 
    curses.curs_set(0)
    stdscr.nodelay(True)
    stdscr.timeout(KEY_REPEAT_TIMEOUT_MS)
 
    reset_mcu()
    time.sleep(0.5)
 
    left_motor = Motor(PWM("P13"), Pin("D4"))
    right_motor = Motor(PWM("P12"), Pin("D5"))
    steer_servo = Servo("P2")
 
    steer_servo.angle(0)
    time.sleep(0.5)
 
    slam_thread = threading.Thread(target=slam_worker, daemon=True)
    slam_thread.start()
 
    stdscr.addstr(0, 0, "=== 조종 + 실시간 SLAM 매핑 (4단계) ===")
    stdscr.addstr(1, 0, "w: 전진  s: 후진  a: 좌회전  d: 우회전")
    stdscr.addstr(2, 0, "x: 조향 정면복귀  space: 정지  q: 종료")
    stdscr.addstr(3, 0, "-" * 55)
 
    try:
        while True:
            key = stdscr.getch()
            if key == ord('q'):
                break
            if key != -1:
                handle_key(key)
 
            set_speed(left_motor, right_motor, TARGET_SPEED)
            set_steer(steer_servo, TARGET_STEER)
 
            # ---------- 조종 상태 표시 ----------
            stdscr.addstr(5, 0, f"현재속도(SPEED_FAST): {SPEED_FAST:4d}   ")
            stdscr.addstr(6, 0, f"목표속도(TARGET_SPEED): {TARGET_SPEED:4d}   ")
            stdscr.addstr(7, 0, f"조향각(TARGET_STEER): {TARGET_STEER:4d}   ")
            stdscr.addstr(9, 0, "-" * 55)
 
            # ---------- SLAM 상태 표시 ----------
            conn = "연결됨" if slam_status["connected"] else "연결 대기/종료"
            stdscr.addstr(10, 0, f"라이다 상태       : {conn}                ")
            stdscr.addstr(11, 0, f"처리된 스캔 수     : {slam_status['scan_count']:6d}          ")
            stdscr.addstr(12, 0, (
                f"추정 위치(mm)     : x={slam_status['robot_x_mm']:.0f}, "
                f"y={slam_status['robot_y_mm']:.0f}, "
                f"각도={slam_status['robot_theta_deg']:.1f}도      "
            ))
            stdscr.addstr(13, 0, f"최근 저장된 지도   : {slam_status['map_saved_path']}                    ")
 
            # ---------- CPU 부하 표시 (이번 단계의 핵심 확인 포인트) ----------
            load = slam_status["load_1min"]
            warn = "  <-- 코어 4개 기준 과부하 의심!" if load > 4.0 else ""
            stdscr.addstr(15, 0, f"시스템 부하(1분평균): {load:.2f}{warn}                    ")
 
            err = slam_status["last_error"]
            if err:
                stdscr.addstr(17, 0, f"[경고] {err}                                          ")
 
            stdscr.refresh()
 
    finally:
        TARGET_SPEED = 0
        while SPEED_FAST != 0:
            set_speed(left_motor, right_motor, 0)
            time.sleep(0.05)
 
        set_steer(steer_servo, 0)
        left_motor.speed(0)
        right_motor.speed(0)
 
        stop_event.set()
        slam_thread.join(timeout=5)
 
 
if __name__ == "__main__":
    curses.wrapper(main)
    print("조종 및 SLAM 매핑 종료 완료.")
    print(f"최종 지도 저장 위치: {slam_status['map_saved_path']}")
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
teleop_slam.py
================
4단계: 조종(teleop) + 라이다 실시간 SLAM(지도 그리기)을 동시에 돌려보는 단계.

목적:
    - 사람이 키보드로 차를 몰면서 (2단계에서 만든 teleop 로직 그대로 재사용)
    - 동시에 라이다 스캔을 BreezySLAM에 실시간으로 넣어서 지도를 갱신하고
    - 이 과정에서 라즈베리파이 CPU가 버티는지(부하가 어느 정도인지) 확인합니다.

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
    logs/slam_map_latest.png            (몇 초마다 최신 지도로 덮어써서 저장 - 진행상황 확인용)
    logs/slam_map_YYYYmmdd_HHMMSS.png    (종료할 때 한 번 더, 타임스탬프 붙여서 최종본 보관)
    logs/slam_map_YYYYmmdd_HHMMSS.npy    (위 png와 같은 지도를, 6단계 A*에서 바로 쓸 숫자 배열로 저장)

    * png와 npy 둘 다, 오래된 것은 자동으로 정리되어 최근 것만 남습니다
      (MAP_KEEP_LAST 값 참고 - 기본 5개).

의존성:
    - breezyslam (3단계에서 이미 설치하셨다고 하셨으니 별도 설치 불필요)
    - numpy (이미 설치되어 있음, 2단계에서 사용함)
    - Pillow(PIL) : 지도를 png로 저장하기 위해 필요. 없으면
        pip install --break-system-packages Pillow

------------------------------------------------------------------------
[중요: 이번 수정에서 실측(직접 측정)이 필요한 값 2개]

  아래 "설정값 - 오도메트리 추정" 섹션에 있는
      SPEED_MM_PER_SEC_AT_MAX
      WHEEL_BASE_MM
  두 값은 지금 임시 추정치가 들어있습니다. 실측하지 않아도 프로그램은 정상
  동작하지만(에러 안 남), 값이 실제와 다르면 SLAM 위치추정 보정 효과가
  떨어집니다. 정확도를 높이려면 아래 방법으로 꼭 실측해서 넣어주세요.

  [1] SPEED_MM_PER_SEC_AT_MAX 재는 법 (차가 "최고속도"로 초당 몇 mm 가는지)
      1. 바닥에 테이프나 표시로 출발선을 표시합니다.
      2. w키를 눌러 차를 완전히 최고속도(SPEED_FAST가 MAX_SPEED=50에 도달)
         상태로 만든 뒤, 그 상태를 정확히 3초간 유지하고 정지시킵니다.
         (가속하는 구간은 최고속도가 아니므로 재는 구간에서 제외하는 셈치고,
          "출발 직후 ~ 3초 후"로 넉넉히 재도 초반 오차는 크지 않습니다.
          더 정확히 하려면 이미 최고속도로 달리고 있는 상태에서 스톱워치로
          3초를 재고 그동안 이동한 거리만 줄자로 재는 방법을 추천합니다.)
      3. 이동한 거리(mm)를 줄자로 잽니다. 예: 1500mm 이동했다면
      4. 1500mm ÷ 3초 = 500mm/s  ->  이 값(500.0)을 SPEED_MM_PER_SEC_AT_MAX에 입력.
      (단위는 "1초에 몇 mm 가는가" 입니다. "1mm당 값"이 아니라
       "1초당 mm" 값이니 헷갈리지 않게 주의하세요.)

  [2] WHEEL_BASE_MM 재는 법 (앞바퀴 축 중심 ~ 뒷바퀴 축 중심까지 거리)
      1. 차량을 뒤집거나 옆에서 봐서, 앞바퀴 차축 중심선과 뒷바퀴 차축
         중심선 사이의 직선 거리를 줄자로 잽니다. (좌우 폭이 아니라
         앞뒤 길이 방향 거리입니다.)
      2. 잰 값을 mm 단위로 그대로 WHEEL_BASE_MM에 입력합니다.
         예: 13cm면 130.0을 입력.

  실측 전까지는 코드 내 기본값(500.0mm/s, 130.0mm)이 임시로 쓰이며,
  이 값들은 "안 넣는 것보다는 낫다" 수준의 대략치입니다.
------------------------------------------------------------------------

[팀원들을 위한 안내]
이 파일은 여러 사람이 같이 보고 수정할 걸 감안해서, 웬만한 코드 블록마다
"이게 왜 필요한지 / 뭘 하는지"를 주석으로 남겨뒀습니다. 코드를 고치실 때
주석도 같이 최신 상태로 유지해주시면 다음 사람이 훨씬 편해집니다.
------------------------------------------------------------------------
"""

import curses
import math
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
    # Pillow가 없어도 프로그램 전체가 죽지 않도록, Image를 None으로 두고
    # 아래 코드에서 "Image is not None"으로 체크해서 png 저장 부분만 건너뛰게 만들어둠.
    Image = None


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
LIDAR_OFFSET = 90

# ============================================================
# 설정값 - SLAM (BreezySLAM)
# ============================================================
MAP_SIZE_PIXELS = 1250
MAP_SIZE_METERS = 50.0

MAP_SAVE_EVERY_SEC = 5.0
LOAD_CHECK_EVERY_SEC = 2.0

# 지도 파일(png/npy) 각각 최근 몇 개까지 남길지. 이 개수를 넘는 오래된 파일은
# slam_worker 시작 시점과 종료 시점에 자동으로 삭제됩니다. png/npy 둘 다 같은 개수로 관리합니다.
MAP_KEEP_LAST = 5

# ============================================================
# 설정값 - 오도메트리 추정 (SLAM 위치추정 보정용)
# ============================================================
# [실측 필요] SPEED_FAST가 최대값(MAX_SPEED)일 때, 차가 실제로 초당 몇 mm
# 움직이는지를 나타냅니다. 단위는 "mm/초" 입니다.
#
# 왜 필요한가:
#   원래 BreezySLAM은 slam.update(scan)만 호출하면, 로봇이 얼마나 움직였는지를
#   순전히 "새 라이다 스캔을 기존 지도와 비교(스캔 매칭)"해서 추측합니다.
#   그런데 차가 빠르게 움직이거나 주변에 벽/모서리 같은 특징이 부족한 구간을
#   지나가면 이 추측이 크게 틀어질 수 있고, 그 결과로 이미 정확히 그려둔
#   벽이 엉뚱하게 지워지거나 새 벽이 엉뚱한 위치에 찍히는 문제가 생깁니다.
#   여기서는 "조종 중인 속도값(SPEED_FAST)"을 이 계수로 환산해서 대략적인
#   이동거리 힌트(pose_change)를 SLAM에 함께 넘겨줘서, 위치추정이 스캔
#   매칭에만 의존하지 않게 만듭니다. 정확하지 않은 대략값이어도 아예
#   안 주는 것보다 SLAM 안정성이 훨씬 좋아집니다.
#
# [실측 방법] 파일 맨 위 docstring의 "[1] SPEED_MM_PER_SEC_AT_MAX 재는 법"
# 항목을 참고해서 실측 후 아래 값을 교체하세요.
# 예시 계산: 최고속도로 3초간 이동했더니 실제로 1500mm 이동했다면
#            1500 / 3 = 500.0 을 입력하면 됩니다.
SPEED_MM_PER_SEC_AT_MAX = 298.0   # <- [실측 전 임시값] 반드시 실측해서 교체하세요 (단위: mm/초)

# [실측 필요] 앞바퀴 차축 중심 ~ 뒷바퀴 차축 중심까지의 거리 (단위: mm).
# 조향각(TARGET_STEER)으로 "지금 얼마나 빠르게 회전하고 있는지(회전각속도)"를
# 추정하는 자전거 모델(bicycle model) 계산에 사용됩니다. 축간거리가 실제와
# 다르면 회전량 추정이 부정확해지므로, 실측값을 넣어주는 게 좋습니다.
#
# [실측 방법] 파일 맨 위 docstring의 "[2] WHEEL_BASE_MM 재는 법" 항목 참고.
WHEEL_BASE_MM = 97.0   # <- [실측 전 임시값] 반드시 실측해서 교체하세요 (단위: mm)


# ============================================================
# 조종용 전역 상태 (기존과 동일)
# ============================================================
SPEED_FAST = 0
TARGET_SPEED = 0
TARGET_STEER = 0

stop_event = threading.Event()

slam_status = {
    "connected": False,
    "scan_count": 0,
    "robot_x_mm": 0.0,
    "robot_y_mm": 0.0,
    "robot_theta_deg": 0.0,
    "last_error": "",
    "map_saved_path": "",
    "load_1min": 0.0,
}


# ============================================================
# 화면 출력 안전 헬퍼 (터미널 창 크기 문제로 프로그램이 죽는 것 방지)
# ============================================================
def safe_addstr(stdscr, y, x, text):
    """
    curses의 stdscr.addstr()를 그대로 쓰면, 출력하려는 문자열이 터미널 창의
    오른쪽 끝이나 맨 아래줄을 넘어갈 때 에러를 내며 프로그램이 죽습니다.
    이 함수는 화면 범위를 넘는 부분을 안전하게 잘라내거나 무시해서 그 문제를 막습니다.
    """
    max_y, max_x = stdscr.getmaxyx()

    if y < 0 or y >= max_y or x < 0 or x >= max_x:
        return

    available = max_x - x - 1
    if available <= 0:
        return

    try:
        stdscr.addstr(y, x, text[:available])
    except curses.error:
        pass


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
    라이다 원본 스캔 [(quality, angle, distance_mm), ...]을
    BreezySLAM이 요구하는 "0~359도, 360개짜리 거리 배열(mm)"로 변환.
    LIDAR_OFFSET으로 기존 회피 코드와 동일한 방향 기준을 맞춥니다.
    """
    distances = [0] * scan_size

    for quality, angle, distance in scan:
        if distance <= 0:
            continue
        corrected = (angle + LIDAR_OFFSET) % 360
        index = int(corrected) % scan_size
        distances[index] = distance

    return distances


# ============================================================
# 오도메트리(이동량) 추정
# ============================================================
def estimate_pose_change(dt):
    """
    직전 스캔 처리 이후 흐른 시간(dt, 초) 동안 로봇이 대략 얼마나 움직였는지를
    "현재 조종 중인 속도(SPEED_FAST)와 조향각(TARGET_STEER)"으로부터 추정합니다.

    반환값: (dxy_mm, dtheta_deg, dt)
        - dxy_mm     : 추정 이동 거리 (mm)
        - dtheta_deg : 추정 회전각 (도, +는 한쪽 방향 회전)
        - dt         : 그대로 전달 (BreezySLAM이 요구하는 형식)

    이 값은 정확한 엔코더 기반 오도메트리가 아니라 "대략적인 힌트"입니다.
    SPEED_MM_PER_SEC_AT_MAX와 WHEEL_BASE_MM을 실측할수록 정확해집니다.
    (자세한 실측 방법은 파일 맨 위 docstring 참고)
    """
    # 1) 현재 속도(SPEED_FAST, 0~MAX_SPEED 범위)를 실제 mm/s로 환산
    if MAX_SPEED:
        speed_mm_s = (SPEED_FAST / MAX_SPEED) * SPEED_MM_PER_SEC_AT_MAX
    else:
        speed_mm_s = 0.0

    dxy_mm = speed_mm_s * dt

    # 2) 자전거 모델(bicycle model)로 조향각 -> 회전각속도 근사 계산
    #    조향각이 0(직진)이면 회전량도 0.
    steer_rad = math.radians(TARGET_STEER)
    if WHEEL_BASE_MM > 0 and abs(steer_rad) > 1e-6:
        turn_rate_deg_s = math.degrees(
            (speed_mm_s / WHEEL_BASE_MM) * math.tan(steer_rad)
        )
    else:
        turn_rate_deg_s = 0.0

    dtheta_deg = turn_rate_deg_s * dt

    return dxy_mm, dtheta_deg, dt


# ============================================================
# SLAM + 라이다 기록 스레드
# ============================================================
def slam_worker():
    """
    백그라운드 스레드. 라이다에 연결해서 스캔이 들어올 때마다 SLAM을 갱신하고,
    주기적으로 지도 저장 + 시스템 부하 측정을 합니다.

    [수정 사항] slam.update() 호출 시 pose_change(오도메트리 힌트)를 함께
    넘겨줘서, 위치추정이 라이다 스캔 매칭에만 의존하지 않도록 개선했습니다.
    이렇게 하면 차가 움직일 때 이미 그려둔 벽이 잘못 지워지거나 엉뚱한
    위치에 새 벽이 찍히는 문제가 줄어듭니다. (원리는 estimate_pose_change
    함수 주석과 파일 맨 위 docstring 참고)
    """
    laser = LaserModel()
    slam = RMHC_SLAM(laser, MAP_SIZE_PIXELS, MAP_SIZE_METERS)

    mapbytes = bytearray(MAP_SIZE_PIXELS * MAP_SIZE_PIXELS)

    os.makedirs("logs", exist_ok=True)

    # 이전 실행에서 남은 오래된 지도 파일 정리 (png/npy 둘 다, 최근 MAP_KEEP_LAST개만 남김)
    cleanup_old_logs("logs", "slam_map_2*.png", keep_last=MAP_KEEP_LAST)
    cleanup_old_logs("logs", "slam_map_2*.npy", keep_last=MAP_KEEP_LAST)

    try:
        lidar = RPLidar(LIDAR_PORT)
    except Exception as e:
        slam_status["last_error"] = f"라이다 연결 실패: {e}"
        return

    slam_status["connected"] = True
    scan_id = 0

    last_map_save = 0.0
    last_load_check = 0.0
    last_scan_time = None   # 직전 스캔을 처리한 시각 (오도메트리 dt 계산용)

    try:
        for scan in lidar.iter_scans(min_len=60):
            if stop_event.is_set():
                break

            distances_mm = scan_to_distance_array(scan, laser.scan_size)

            now_scan = time.time()

            if last_scan_time is None:
                # 첫 스캔은 "직전 시각"이 없어서 이동량(dt)을 계산할 수 없으므로
                # 오도메트리 힌트 없이 첫 위치만 잡습니다.
                slam.update(distances_mm)
            else:
                dt = now_scan - last_scan_time
                dxy_mm, dtheta_deg, dt = estimate_pose_change(dt)
                slam.update(distances_mm, pose_change=(dxy_mm, dtheta_deg, dt))

            last_scan_time = now_scan

            x_mm, y_mm, theta_deg = slam.getpos()
            slam_status["robot_x_mm"] = x_mm
            slam_status["robot_y_mm"] = y_mm
            slam_status["robot_theta_deg"] = theta_deg

            scan_id += 1
            slam_status["scan_count"] = scan_id

            now = time.time()

            if Image is not None and (now - last_map_save) >= MAP_SAVE_EVERY_SEC:
                slam.getmap(mapbytes)
                map_array = np.frombuffer(mapbytes, dtype=np.uint8).reshape(
                    (MAP_SIZE_PIXELS, MAP_SIZE_PIXELS)
                )
                img = Image.fromarray(map_array)
                save_path = os.path.join("logs", "slam_map_latest.png")
                img.save(save_path)
                slam_status["map_saved_path"] = save_path
                last_map_save = now

            if (now - last_load_check) >= LOAD_CHECK_EVERY_SEC:
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

        # 종료 직전, 마지막 지도를 한 번 더 저장 (타임스탬프 붙여서 최종본으로 따로 보관)
        # png(눈으로 확인용)와 npy(6단계 A*에서 바로 쓸 숫자 배열) 둘 다 저장합니다.
        if Image is not None:
            try:
                slam.getmap(mapbytes)
                map_array = np.frombuffer(mapbytes, dtype=np.uint8).reshape(
                    (MAP_SIZE_PIXELS, MAP_SIZE_PIXELS)
                )
                ts = datetime.now().strftime("%Y%m%d_%H%M%S")

                final_path = os.path.join("logs", f"slam_map_{ts}.png")
                Image.fromarray(map_array).save(final_path)

                npy_path = os.path.join("logs", f"slam_map_{ts}.npy")
                np.save(npy_path, map_array)

                # 이번 실행으로 새로 하나씩 생긴 png/npy까지 포함해서, 다시 한번
                # 최근 MAP_KEEP_LAST개만 남기고 정리 (안 하면 실행할 때마다 계속 쌓이기만 함)
                cleanup_old_logs("logs", "slam_map_2*.png", keep_last=MAP_KEEP_LAST)
                cleanup_old_logs("logs", "slam_map_2*.npy", keep_last=MAP_KEEP_LAST)

                # (이전 버전에서는 이 줄 바로 다음에 map_saved_path를 final_path로
                #  다시 덮어써버려서 npy 경로 정보가 사라지는 버그가 있었음 - 수정함)
                slam_status["map_saved_path"] = f"{final_path} / {npy_path}"
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

    safe_addstr(stdscr, 0, 0, "=== 조종 + 실시간 SLAM 매핑 (4단계) ===")
    safe_addstr(stdscr, 1, 0, "w: 전진  s: 후진  a: 좌회전  d: 우회전")
    safe_addstr(stdscr, 2, 0, "x: 조향 정면복귀  space: 정지  q: 종료")
    safe_addstr(stdscr, 3, 0, "-" * 55)

    try:
        while True:
            key = stdscr.getch()
            if key == ord('q'):
                break
            if key != -1:
                handle_key(key)

            set_speed(left_motor, right_motor, TARGET_SPEED)
            set_steer(steer_servo, TARGET_STEER)

            safe_addstr(stdscr, 5, 0, f"현재속도(SPEED_FAST): {SPEED_FAST:4d}   ")
            safe_addstr(stdscr, 6, 0, f"목표속도(TARGET_SPEED): {TARGET_SPEED:4d}   ")
            safe_addstr(stdscr, 7, 0, f"조향각(TARGET_STEER): {TARGET_STEER:4d}   ")
            safe_addstr(stdscr, 9, 0, "-" * 55)

            conn = "연결됨" if slam_status["connected"] else "연결 대기/종료"
            safe_addstr(stdscr, 10, 0, f"라이다 상태       : {conn}                ")
            safe_addstr(stdscr, 11, 0, f"처리된 스캔 수     : {slam_status['scan_count']:6d}          ")
            safe_addstr(stdscr, 12, 0, (
                f"추정 위치(mm)     : x={slam_status['robot_x_mm']:.0f}, "
                f"y={slam_status['robot_y_mm']:.0f}, "
                f"각도={slam_status['robot_theta_deg']:.1f}도      "
            ))
            safe_addstr(stdscr, 13, 0, f"최근 저장된 지도   : {slam_status['map_saved_path']}                    ")

            load = slam_status["load_1min"]
            warn = "  <-- 코어 4개 기준 과부하 의심!" if load > 4.0 else ""
            safe_addstr(stdscr, 15, 0, f"시스템 부하(1분평균): {load:.2f}{warn}                    ")

            err = slam_status["last_error"]
            if err:
                safe_addstr(stdscr, 17, 0, f"[경고] {err}                                          ")

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
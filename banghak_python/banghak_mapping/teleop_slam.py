#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
teleop_slam.py
26.7.27
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
    logs/odom_debug.log                  (임시 디버그: 스캔마다 오도메트리 계산값과 추정 위치를 기록)

    * png와 npy 둘 다, 오래된 것은 자동으로 정리되어 최근 것만 남습니다
      (MAP_KEEP_LAST 값 참고 - 기본 5개).

의존성:
    - breezyslam (3단계에서 이미 설치하셨다고 하셨으니 별도 설치 불필요)
    - numpy (이미 설치되어 있음, 2단계에서 사용함)
    - Pillow(PIL) : 지도를 png로 저장하기 위해 필요. 없으면
        pip install --break-system-packages Pillow

------------------------------------------------------------------------
[팀원들을 위한 안내]
이 파일은 여러 사람이 같이 보고 수정할 걸 감안해서, 웬만한 코드 블록마다
"이게 왜 필요한지 / 뭘 하는지"를 주석으로 남겨뒀습니다. 코드를 고치실 때
주석도 같이 최신 상태로 유지해주시면 다음 사람이 훨씬 편해집니다.
------------------------------------------------------------------------
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
    # Pillow가 없어도 프로그램 전체가 죽지 않도록, Image를 None으로 두고
    # 아래 코드에서 "Image is not None"으로 체크해서 png 저장 부분만 건너뛰게 만들어둠.
    Image = None


# ============================================================
# 설정값 - 조종 (기존 teleop_keyboard.py / teleop_record_lidar.py와 동일)
# ============================================================
MAX_SPEED = 50
SPEED_STEP = 3

STEER_LIMIT = 40   # lidar_ver2.py와 동일하게 맞춤 (기존 35 -> 40)
STEER_STEP = 5

KEY_REPEAT_TIMEOUT_MS = 100

# lidar_ver2.py와 동일: 조향 방향이 반대로 나오면 True로 바꿔서 반전
GAIN_REVERSE = False
# lidar_ver2.py의 set_steer()에서 실제 서보에 보내기 직전 "+5"를 더하던 것과 동일한
# 하드웨어 보정값. 서보의 물리적 중심(0도)이 실제 정중앙과 5도 어긋나 있다는 뜻이라,
# 이 스크립트에서도 서보에 각도를 보낼 때 항상 이만큼 더해줍니다.
STEER_HW_OFFSET_DEG = 5

# odom_debug.log로 역산한 실제 직진 편향(트림). 조향각 0(직진 명령)을 줘도
# 차량이 실제로는 약간 휘어져 가는 현상을, 서보에 이 각도만큼 미리 보정해서
# 상쇄시킵니다.
#
# [주의] 이전에 딱 2개의 노이즈 낀 실측 지점만으로 멀리 외삽(extrapolate)해서
# STEER_BIAS_DEG_BACKWARD=-6.1을 시도했더니, 실제로는 후진 시 반원을 그릴 정도로
# 심하게 꺾여버렸습니다 (계산상 총 오프셋은 -1.1도로 작은 값이었는데도 실제
# 물리적 반응은 그와 전혀 다르게 나타남 - 즉 이 정도 외삽은 신뢰할 수 없다는 뜻).
# 그래서 후진 보정은 일단 보수적인 값으로 되돌리고, 여기서부터는 큰 폭으로
# 점프하지 말고 ±1도 정도씩 소폭으로만 조정하며 재테스트하는 걸 권장합니다.
STEER_BIAS_DEG_FORWARD = -1.5     # 전진 시 적용 (이전 테스트에서 개선 확인됨, 유지)
STEER_BIAS_DEG_BACKWARD = 7.0   # 후진 시 적용 - 반대 방향으로 소폭 테스트 (0 -> -2.0)

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
# 설정값 - 디버그 로그 (오도메트리 검증용, 확인 끝나면 DEBUG_ODOM=False로 끄면 됨)
# ============================================================
DEBUG_ODOM = True
DEBUG_ODOM_PATH = os.path.join("logs", "odom_debug.log")

# ============================================================
# 설정값 - 좌우 모터 트림 보정 (물리적으로 확인됨: 조향 0에서도 차가 휘어짐)
# ============================================================
# 실측 결과, TARGET_STEER=0(직진)으로 몰아도 차가 한쪽으로 휘는 게 확인됐습니다
# (양쪽 바퀴 실속도가 미세하게 다름). 여기서 한쪽 모터 속도를 살짝 줄이거나
# 늘려서, 실제 물리적으로 양쪽 바퀴 속도가 같아지도록 보정합니다.
#
# 1.0 = 보정 없음. 로그(odom_debug.log)에서 theta가 오른쪽(음수 방향)으로
# 계속 틀어졌으니, 왼쪽 바퀴가 상대적으로 더 세게 밀고 있을 가능성이 높습니다.
# -> 우선 LEFT_MOTOR_TRIM을 살짝 낮추는 쪽(예: 0.95)부터 시도해보세요.
# 방향이 반대로 나오면(더 심하게 휘면) RIGHT_MOTOR_TRIM 쪽을 조정하는 걸로 바꾸면 됩니다.
#
# 튜닝 방법: 아래 값을 바꿔가며 직진 테스트 -> odom_debug.log에서
# dtheta 누적(=theta 변화량)이 0에 가까워지는 값을 찾을 때까지 반복.
LEFT_MOTOR_TRIM = 1.0    # 필요시 0.90~1.00 사이에서 조정
RIGHT_MOTOR_TRIM = 1.0   # 필요시 1.00~1.10 사이에서 조정


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

    # 좌우 트림 보정: SPEED_FAST(목표 반영값) 자체는 그대로 두고,
    # 실제 모터에 넘기는 값에만 트림 비율을 곱합니다. 이렇게 하면
    # 오도메트리 계산(estimate_pose_change)에 쓰이는 SPEED_FAST는
    # 트림과 무관하게 "명령한 속도" 그대로 유지되어 계산이 헷갈리지 않습니다.
    left_motor.speed(-SPEED_FAST * LEFT_MOTOR_TRIM)
    right_motor.speed(SPEED_FAST * RIGHT_MOTOR_TRIM)

    if SPEED_FAST == 0:
        left_motor.speed(0)
        right_motor.speed(0)
    return SPEED_FAST


def set_steer(steer_servo, angle):
    """
    조향 서보를 원하는 각도로 즉시 돌립니다.

    [중요 - 2026-07-28 발견] 후진 중에는 조향각이 정확히 0에서 조금만 벗어나도
    각도가 계속 커지며 발산(제자리에서 계속 도는 현상)하는 게 로그로 확인됐습니다.
    앞바퀴 조향 차량은 원래 후진 시 "자기교정"이 아니라 "발산" 구조라서,
    아주 작은 조향 오차도 시간이 갈수록 점점 커집니다 (233도까지 돌아버린 사례 있음).

    그래서 STEER_HW_OFFSET_DEG(+5도, lidar_ver2.py 기준값)는 전진에만 적용합니다.
    원본 lidar_ver2.py도 후진 시엔 이 set_steer()를 아예 안 쓰고 별도의
    WallBackup 로직을 쓰기 때문에, +5도가 후진에서 검증된 적이 없는 값이었습니다.
    후진 중에는 STEER_BIAS_DEG_BACKWARD만 적용하고, 이 값은 최대한 0에 가깝게
    유지해야 합니다 (0이 아니면 발산 위험이 있으므로 아주 신중하게, 아주 작은
    단위로만 조정하세요).
    """
    angle = max(-STEER_LIMIT, min(STEER_LIMIT, angle))
    if GAIN_REVERSE:
        angle = -angle

    if SPEED_FAST >= 0:
        steer_servo.angle(angle + STEER_HW_OFFSET_DEG + STEER_BIAS_DEG_FORWARD)
    else:
        # 후진: HW 오프셋 적용 안 함 (원본 코드도 후진엔 이 함수 자체를 안 씀)
        steer_servo.angle(angle + STEER_BIAS_DEG_BACKWARD)
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
def scan_to_distance_array(scan, scan_size, min_quality=10):
    """
    quality가 낮은 포인트(유리 반사, 노이즈 등으로 신뢰도 낮은 측정값)는
    아예 배제해서, 지도에 엉뚱한 값이 찍히는 걸 줄입니다.
    min_quality 값은 실제 데이터 보면서 조정이 필요할 수 있습니다.
    """
    distances = [0] * scan_size

    for quality, angle, distance in scan:
        if distance <= 0:
            continue
        if quality < min_quality:   # 신뢰도 낮은 포인트는 스킵
            continue
        corrected = (angle + LIDAR_OFFSET) % 360
        index = int(corrected) % scan_size
        distances[index] = distance

    return distances

import math   # 파일 상단 import 구역에 추가 필요

# 오도메트리 디버그 로그(odom_debug.log)로 역산한 실측 보정값입니다.
# (기존 SPEED_MM_PER_SEC_AT_MAX=333.0 하나로 전진/후진을 같이 계산했더니
#  전진은 83.6%, 후진은 62.8%만 맞아서, 전진/후진을 따로 분리했습니다.
#  기어 백래시나 모터 특성 차이로 후진이 전진보다 실제 속도가 느린 걸로 보입니다.)
SPEED_MM_PER_SEC_AT_MAX_FORWARD = 226    # 전진 최고속도(mm/s), 로그 기반 보정값
SPEED_MM_PER_SEC_AT_MAX_BACKWARD = 209.1   # 후진 최고속도(mm/s), 로그 기반 보정값
WHEEL_BASE_MM = 97.0              # 실측 완료

def estimate_pose_change(dt):
    # SPEED_FAST 부호로 전진/후진을 구분해서 서로 다른 속도 상수를 적용
    if SPEED_FAST >= 0:
        max_speed_mm_s = SPEED_MM_PER_SEC_AT_MAX_FORWARD
    else:
        max_speed_mm_s = SPEED_MM_PER_SEC_AT_MAX_BACKWARD

    if MAX_SPEED:
        speed_mm_s = (SPEED_FAST / MAX_SPEED) * max_speed_mm_s
    else:
        speed_mm_s = 0.0

    dxy_mm = speed_mm_s * dt

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
    """
    laser = LaserModel()
    slam = RMHC_SLAM(
        laser, MAP_SIZE_PIXELS, MAP_SIZE_METERS,
        hole_width_mm=1000,
        sigma_xy_mm=100,          # 기본값 100mm -> 200mm로 조금 넓힘
        sigma_theta_degrees=20,   # 기본값 20도 -> 30도로 조금 넓힘
    )

    mapbytes = bytearray(MAP_SIZE_PIXELS * MAP_SIZE_PIXELS)

    os.makedirs("logs", exist_ok=True)

    # 이전 실행에서 남은 오래된 지도 파일 정리 (png/npy 둘 다, 최근 MAP_KEEP_LAST개만 남김)
    cleanup_old_logs("logs", "slam_map_2*.png", keep_last=MAP_KEEP_LAST)
    cleanup_old_logs("logs", "slam_map_2*.npy", keep_last=MAP_KEEP_LAST)

    # ---- 디버그 로그 파일 준비 (오도메트리 검증용) ----
    # 매번 실행할 때마다 이전 로그는 지우고 새로 시작합니다 (누적 안 함).
    # 예전엔 "a"(이어붙이기)라서 실행할수록 파일이 계속 길어져서 스크롤이
    # 불편했는데, 여기서 "w"(덮어쓰기)로 바꿔서 매 실행 시작 시 파일을 비웁니다.
    if DEBUG_ODOM:
        with open(DEBUG_ODOM_PATH, "w", encoding="utf-8") as f:
            f.write(f"===== 새 실행 시작: {datetime.now().isoformat(timespec='seconds')} "
                    f"(FWD={SPEED_MM_PER_SEC_AT_MAX_FORWARD}, BACK={SPEED_MM_PER_SEC_AT_MAX_BACKWARD}, "
                    f"sigma_xy_mm={slam.sigma_xy_mm}, sigma_theta_degrees={slam.sigma_theta_degrees}) =====\n")

    try:
        lidar = RPLidar(LIDAR_PORT)
    except Exception as e:
        slam_status["last_error"] = f"라이다 연결 실패: {e}"
        return

    slam_status["connected"] = True
    scan_id = 0

    last_map_save = 0.0
    last_load_check = 0.0
    last_scan_time = None

    # 디버그용: 오도메트리가 계산한 dxy_mm을 누적해서 "이론상 이동거리 합계"를 같이 봄
    cumulative_dxy_mm = 0.0

    try:
        for scan in lidar.iter_scans(min_len=60):
            if stop_event.is_set():
                break

            distances_mm = scan_to_distance_array(scan, laser.scan_size)

            now_scan = time.time()
            if last_scan_time is None:
                # 첫 스캔은 직전 시각이 없어서 이동량 계산 불가 -> 힌트 없이 위치만 잡음
                slam.update(distances_mm)
                dxy_mm, dtheta_deg, dt = 0.0, 0.0, 0.0
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

            # ---- 디버그 로그 기록 ----
            # 스캔마다: 그 순간 속도값/조향값, 계산된 dt/dxy/dtheta, 그리고 SLAM이
            # 최종적으로 내놓은 추정 위치(pos)를 한 줄씩 남깁니다.
            # cumulative_dxy_mm은 "오도메트리 계산만 따라가면 총 몇 mm 이동한 것으로
            # 나오는지"를 보여줘서, 실제 SLAM 위치 변화량과 비교하기 위한 값입니다.
            if DEBUG_ODOM:
                cumulative_dxy_mm += dxy_mm
                with open(DEBUG_ODOM_PATH, "a", encoding="utf-8") as f:
                    f.write(
                        f"scan={scan_id:5d} "
                        f"SPEED_FAST={SPEED_FAST:4d} TARGET_STEER={TARGET_STEER:4d} "
                        f"dt={dt:.3f} dxy_mm={dxy_mm:7.2f} dtheta_deg={dtheta_deg:7.2f} "
                        f"누적dxy_mm={cumulative_dxy_mm:9.1f} "
                        f"pos=(x={x_mm:.1f}, y={y_mm:.1f}, theta={theta_deg:.1f})\n"
                    )

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
    if DEBUG_ODOM:
        safe_addstr(stdscr, 4, 0, f"[디버그 로그 켜짐] -> {DEBUG_ODOM_PATH}")

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
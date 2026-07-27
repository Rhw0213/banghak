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

의존성:
    - breezyslam (3단계에서 이미 설치하셨다고 하셨으니 별도 설치 불필요)
    - numpy (이미 설치되어 있음, 2단계에서 사용함)
    - Pillow(PIL) : 지도를 png로 저장하기 위해 필요. 없으면
        pip install --break-system-packages Pillow

------------------------------------------------------------------------
[팀원들을 위한 안내]
이 파일은 여러 사람이 같이 보고 수정할 걸 감안해서, 웬만한 코드 블록마다
"이게 왜 필요한지 / 뭘 하는지"를 주석으로 최대한 자세히 남겨뒀습니다.
특히 curses(터미널 화면 제어), threading(스레드), BreezySLAM 관련 부분은
처음 보면 낯설 수 있어서 더 길게 설명을 붙였습니다. 코드를 고치실 때
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
# 아래 상수들은 이 로봇의 하드웨어 특성(모터 최대 속도, 서보 최대 각도 등)에
# 맞춰 이미 팀에서 값을 맞춰둔 것들입니다. 다른 로봇에 이 코드를 옮겨 쓸 경우
# 이 값들부터 다시 확인해야 합니다.
MAX_SPEED = 50          # 모터에 줄 수 있는 최대 속도값 (단위는 robot_hat의 speed()가 받는 임의 단위)
SPEED_STEP = 3          # 한 번 루프 돌 때마다 현재 속도를 이 값만큼 목표 속도 쪽으로 이동시킴
                        # (급가속/급정지 대신 부드럽게 속도를 바꾸는 "슬로우스타트/슬로우스탑" 구현)

STEER_LIMIT = 35        # 조향 서보가 꺾을 수 있는 최대 각도(도). 이 이상 꺾으면 기구적으로 무리가 갈 수 있음
STEER_STEP = 5          # 좌/우 키를 한 번 누를 때 조향각을 이만큼(도) 바꿈

KEY_REPEAT_TIMEOUT_MS = 100   # curses가 키 입력을 몇 ms 동안 기다렸다가 "입력 없음"으로 넘어갈지
                              # (이 값이 곧 메인 루프가 도는 주기이기도 함 - 100ms마다 한 번씩 돔)

# ============================================================
# 설정값 - 라이다 (lidar_ultra_avoidance.py / lidar_record.py와 동일)
# ============================================================
LIDAR_PORT = '/dev/ttyUSB0'   # 라이다가 연결된 시리얼 포트. 케이블을 뽑았다 꽂으면 번호가
                              # ttyUSB1처럼 바뀔 수 있으니, 안 될 때는 `ls /dev/ttyUSB*`로 확인할 것
LIDAR_OFFSET = 90             # 라이다가 로봇 몸체에 물리적으로 90도 돌아간 채로 장착되어 있어서
                              # 생기는 오차를 보정하는 값. 기존 회피 코드(lidar_ultra_avoidance.py)와
                              # 반드시 같은 값을 써야, SLAM 지도의 방향과 실제 주행 방향이 일치함.

# ============================================================
# 설정값 - SLAM (BreezySLAM)
# ============================================================
MAP_SIZE_PIXELS = 500      # 지도를 500x500 픽셀 격자로 표현
MAP_SIZE_METERS = 10.0     # 그 격자가 실제로 가로세로 10m 공간을 나타냄
                            # (경기장/방 크기에 맞춰 나중에 조절 가능. 너무 작으면 지도 밖으로 나갈 때 잘림)
                            # 예: MAP_SIZE_PIXELS=500, MAP_SIZE_METERS=10 이면
                            #     1픽셀 = 10m / 500px = 20mm 를 의미함 (해상도가 그만큼 거칠다는 뜻)

MAP_SAVE_EVERY_SEC = 5.0   # 이 주기(초)마다 지도를 png로 저장 (눈으로 진행상황 확인용)
LOAD_CHECK_EVERY_SEC = 2.0 # 이 주기(초)마다 시스템 부하(load average)를 갱신


# ============================================================
# 조종용 전역 상태 (기존과 동일)
# ============================================================
# 스레드 2개(메인=조종, 백그라운드=SLAM)가 서로 값을 주고받아야 하는데,
# 파이썬에서 이런 간단한 상태 공유는 보통 전역 변수 + dict로 충분합니다.
# (여기서는 "매 루프 화면에 보여주기만 하는 값"이라 락(lock) 없이 씁니다.
#  만약 이 값들로 실제 제어 로직 분기를 한다면 그때는 락을 고려해야 합니다.)
SPEED_FAST = 0        # 지금 실제로 모터에 걸려있는 "현재 속도" (목표 속도를 향해 서서히 변함)
TARGET_SPEED = 0      # 사용자가 키보드로 원하는 "목표 속도" (w/s/space로 바뀜)
TARGET_STEER = 0      # 목표 조향각 (a/d/x로 바뀜, 조향은 슬로우스타트 없이 즉시 반영)

stop_event = threading.Event()
# ↑ 메인 스레드가 q를 눌러서 종료할 때, "SLAM 스레드야 너도 이제 멈춰"라고
#   신호를 보내는 용도. threading.Event는 스레드 간에 안전하게 공유 가능한
#   on/off 스위치라고 생각하면 됩니다. slam_worker() 안의 while 루프가
#   이 값을 계속 확인하다가, set()이 호출되면 루프를 빠져나갑니다.

# 화면에 보여줄 SLAM/시스템 상태 (스레드 간 공유, 표시 전용이라 락 없이 사용)
slam_status = {
    "connected": False,        # 라이다 연결이 지금 살아있는지
    "scan_count": 0,           # 지금까지 SLAM에 반영된 스캔(=라이다 한 바퀴) 개수
    "robot_x_mm": 0.0,         # SLAM이 추정한 로봇의 x좌표 (지도 좌표계 기준, mm)
    "robot_y_mm": 0.0,         # SLAM이 추정한 로봇의 y좌표 (mm)
    "robot_theta_deg": 0.0,    # SLAM이 추정한 로봇의 방향(각도, 도 단위)
    "last_error": "",          # 최근에 발생한 에러 메시지 (없으면 빈 문자열)
    "map_saved_path": "",      # 가장 최근에 저장된 지도 이미지 파일 경로
    "load_1min": 0.0,          # 최근 1분 평균 시스템 부하 (os.getloadavg() 첫 번째 값)
}


# ============================================================
# 화면 출력 안전 헬퍼 (터미널 창 크기 문제로 프로그램이 죽는 것 방지)
# ============================================================
def safe_addstr(stdscr, y, x, text):
    """
    curses의 stdscr.addstr()를 그대로 쓰면, 출력하려는 문자열이 터미널 창의
    오른쪽 끝이나 맨 아래줄을 넘어갈 때 `_curses.error: addwstr() returned ERR`
    라는 에러를 내면서 프로그램 전체가 죽어버립니다.

    (실제로 이전 단계인 lidar_record.py에서 이 문제로 한 번 프로그램이 죽었던
     적이 있습니다 - SSH 터미널 창이 좁을 때 특히 잘 생깁니다.)

    이 함수는 그 문제를 막기 위한 안전판입니다:
      1) y, x 좌표 자체가 화면 범위를 벗어나면 -> 아예 출력하지 않고 조용히 무시
      2) 문자열이 그 줄에 남은 공간보다 길면 -> 남는 공간만큼만 잘라서 출력
      3) 그래도 무슨 이유로 에러가 나면 -> try/except로 잡아서 프로그램은 계속 돌아가게 함
         (화면에 그 줄 하나가 안 보이는 것뿐이지, 조종/SLAM 자체가 멈추면 안 되니까)

    이후 이 파일 안의 모든 stdscr.addstr(...) 호출은 반드시 이 함수를 통해서만
    하도록 통일했습니다. 새로운 화면 출력 줄을 추가할 때도 이 함수를 쓰세요.
    """
    max_y, max_x = stdscr.getmaxyx()  # 지금 터미널 창의 (세로줄 수, 가로칸 수)

    if y < 0 or y >= max_y or x < 0 or x >= max_x:
        return  # 애초에 화면 밖 좌표면 출력 시도 자체를 안 함

    # 이 줄, 이 시작 위치(x)에서 실제로 쓸 수 있는 최대 글자 수.
    # max_x - 1을 하는 이유: curses는 화면의 "맨 마지막 칸"까지 딱 맞게 글자를
    # 쓰면 커서를 어디로 옮겨야 할지 애매해져서 에러를 내는 경우가 있어서,
    # 안전하게 마지막 한 칸은 항상 비워둡니다.
    available = max_x - x - 1
    if available <= 0:
        return

    try:
        stdscr.addstr(y, x, text[:available])
    except curses.error:
        # 위에서 다 체크했는데도 혹시 모를 예외 상황(터미널 종류에 따른 차이 등)에
        # 대비한 최후의 안전장치. 화면 출력만 포기하고 프로그램은 계속 진행.
        pass


# ============================================================
# 조종 함수 (기존과 완전히 동일 - 그대로 재사용)
# ============================================================
def set_speed(left_motor, right_motor, target):
    """
    현재 속도(SPEED_FAST)를 target(목표 속도) 쪽으로 SPEED_STEP만큼만 이동시킵니다.
    이 함수를 매 루프(100ms마다)마다 계속 호출해줘야 속도가 서서히 변합니다.
    한 번만 부르면 딱 SPEED_STEP만큼만 바뀌고 멈춥니다 - 그래서 메인 루프
    안에서 매번 호출하는 구조로 되어 있습니다.

    예시: 지금 속도 0, 목표 50 -> 3, 6, 9, ... 이렇게 서서히 올라감 (급발진 방지)
          지금 속도 30, 목표 0  -> 27, 24, 21, ... 이렇게 서서히 내려감 (급정거 방지)
    """
    global SPEED_FAST
    if SPEED_FAST < target:
        SPEED_FAST = min(SPEED_FAST + SPEED_STEP, target)  # target을 넘어서 올라가지 않게 min으로 제한
    elif SPEED_FAST > target:
        SPEED_FAST = max(SPEED_FAST - SPEED_STEP, target)  # target 밑으로 내려가지 않게 max로 제한
    # SPEED_FAST == target이면 이 if/elif 둘 다 안 타서 그대로 유지됨

    # 실제 모터에 값 반영. 왼쪽 모터는 배선/기어 방향 때문에 부호를 반대로 줘야
    # 양쪽 바퀴가 같은 방향(둘 다 전진 또는 둘 다 후진)으로 돕니다.
    left_motor.speed(-SPEED_FAST)
    right_motor.speed(SPEED_FAST)

    if SPEED_FAST == 0:
        # 완전 정지 시엔 0을 한 번 더 확실히 박아줘서, 모터가 미세하게 떨리는(지터) 걸 방지
        left_motor.speed(0)
        right_motor.speed(0)
    return SPEED_FAST


def set_steer(steer_servo, angle):
    """
    조향 서보를 즉시 원하는 각도로 돌립니다. (속도와 달리 조향은 슬로우스타트 없이 바로 반영)
    STEER_LIMIT을 넘는 각도가 들어와도 안전하게 그 범위 안으로 잘라줍니다.
    """
    angle = max(-STEER_LIMIT, min(STEER_LIMIT, angle))
    steer_servo.angle(angle)
    return angle


def handle_key(key):
    """
    눌린 키 코드(key) 하나를 보고 TARGET_SPEED/TARGET_STEER "목표값"만 바꿉니다.
    실제로 모터를 움직이는 일은 여기서 하지 않습니다 - 메인 루프에서 매번
    set_speed()/set_steer()를 호출할 때 이 목표값을 향해 서서히 반영됩니다.
    이렇게 "목표값 갱신"과 "실제 제어"를 분리해두면, 키를 안 누르고 있는
    순간에도 슬로우스타트가 끊기지 않고 계속 진행될 수 있습니다.
    """
    global TARGET_SPEED, TARGET_STEER
    if key == ord('w'):
        TARGET_SPEED = MAX_SPEED       # 전진 목표 속도를 최대치로 (실제 도달은 서서히)
    elif key == ord('s'):
        TARGET_SPEED = -MAX_SPEED      # 후진은 음수 속도로 표현
    elif key == ord(' '):
        TARGET_SPEED = 0               # 스페이스바 = 정지 목표 (급정거 아니라 슬로우스탑)
    elif key == ord('a'):
        TARGET_STEER -= STEER_STEP     # 좌회전: 각도를 음수 방향으로
    elif key == ord('d'):
        TARGET_STEER += STEER_STEP     # 우회전: 각도를 양수 방향으로
    elif key == ord('x'):
        TARGET_STEER = 0               # 조향 정면 복귀

    # 위에서 -=/+=로 누적되다 보니 STEER_LIMIT을 넘어갈 수 있어서, 여기서 한 번 더 안전하게 제한
    TARGET_STEER = max(-STEER_LIMIT, min(STEER_LIMIT, TARGET_STEER))


# ============================================================
# 라이다 스캔 -> BreezySLAM 입력 형식으로 변환
# ============================================================
def scan_to_distance_array(scan, scan_size):
    """
    라이다가 주는 원본 스캔 [(quality, angle, distance_mm), ...]
    (한 바퀴 분량이지만 각도가 불규칙하고 개수도 300~700개로 들쭉날쭉함)을,
    BreezySLAM이 요구하는 "0~359도, 정수 인덱스 360개짜리 거리 배열(mm)"로 바꿔줍니다.

    변환 규칙:
      - 못 잡은 각도(그 인덱스에 해당하는 값이 없는 각도)는 0으로 채워둡니다.
        BreezySLAM은 0을 "이 방향은 측정 안 됨/무응답"으로 해석합니다.
      - LIDAR_OFFSET을 적용해서, 기존 회피 코드(lidar_ultra_avoidance.py)의
        normalize_angle()과 같은 방식으로 방향 기준을 통일합니다. 이렇게 해야
        SLAM이 그리는 지도의 "정면 방향"이 실제 로봇의 정면과 일치합니다.

    인자:
        scan       : lidar.iter_scans()에서 나온 한 바퀴 분량의 원본 스캔
        scan_size  : BreezySLAM 라이다 모델이 요구하는 배열 길이 (RPLidarA1은 보통 360)

    반환:
        길이 scan_size인 정수 리스트. 인덱스 i = 각도 i도, 값 = 그 방향 거리(mm) (0=무응답)
    """
    distances = [0] * scan_size   # 기본값 0 = "이 각도는 못 쟀음"

    for quality, angle, distance in scan:
        if distance <= 0:
            continue  # 거리 0 이하는 유효하지 않은 값이라 건너뜀

        # 기존 lidar_ultra_avoidance.py의 normalize_angle()과 동일한 방식으로 오프셋 보정.
        # (원래 그 함수는 -180~180 범위로 정규화했지만, 여기서는 배열 인덱스로 쓸 거라
        #  0~359 범위로 맞춥니다. 보정 원리 자체는 동일합니다.)
        corrected = (angle + LIDAR_OFFSET) % 360
        index = int(corrected) % scan_size
        distances[index] = distance
        # 참고: 여러 원본 점이 같은 정수 각도(같은 index)로 반올림될 수 있는데,
        # 여기서는 "마지막으로 들어온 값"이 그냥 덮어씁니다. 필요하면 나중에
        # "더 가까운 값 우선" 같은 규칙으로 바꿀 수 있습니다 (slam_offline_test.py 참고).

    return distances


# ============================================================
# SLAM + 라이다 기록 스레드
# ============================================================
def slam_worker():
    """
    백그라운드 스레드에서 계속 실행되는 함수.
    라이다에 연결해서 스캔이 들어올 때마다 SLAM을 갱신하고,
    주기적으로 (1) 지도를 png로 저장 (2) 시스템 부하를 측정해서 slam_status에 기록합니다.

    이 함수가 "스레드"로 도는 이유:
        메인 스레드는 100ms마다 한 번씩 키 입력을 확인하며 조종을 담당해야 하는데,
        라이다 스캔을 기다리고 SLAM 연산을 하는 건 그보다 오래 걸릴 수 있습니다.
        만약 이걸 메인 루프 안에서 순서대로 처리하면, SLAM 연산이 끝날 때까지
        조종 반응이 멈춰버립니다(끊기는 느낌). 그래서 별도 스레드로 분리해서
        "조종은 조종대로, SLAM은 SLAM대로" 동시에 돌아가게 만든 것입니다.
    """
    # ---------- 라이다 모델 및 SLAM 객체 준비 ----------
    laser = LaserModel()   # RPLidar A1에 맞춰 캘리브레이션된 파라미터 (scan_size=360 등 내장)
    slam = RMHC_SLAM(laser, MAP_SIZE_PIXELS, MAP_SIZE_METERS)

    # SLAM이 그린 지도를 담을 바이트 배열 (한 칸 = 1픽셀, 0~255 밝기값)
    # BreezySLAM의 getmap()은 이 배열을 "미리 만들어서 넘겨주면 그 안에 채워주는" 방식으로 동작합니다.
    mapbytes = bytearray(MAP_SIZE_PIXELS * MAP_SIZE_PIXELS)

    os.makedirs("logs", exist_ok=True)

    # 이전 실행에서 남은 오래된 slam_map_*.png들을 정리 (최근 3개만 남김)
    # - keep_last=3이라 4번째로 오래된 것부터 삭제됨. 값을 바꾸고 싶으면 여기서 조정.
    cleanup_old_logs("logs", "slam_map_2*.png", keep_last=3)

    try:
        lidar = RPLidar(LIDAR_PORT)
    except Exception as e:
        # 라이다 연결 자체가 실패하면, 에러 메시지만 남기고 이 스레드는 조용히 종료.
        # (메인 스레드의 조종 자체는 라이다 없이도 계속 동작해야 하므로 여기서 프로그램
        #  전체를 죽이지 않습니다 - 화면에 [경고] 메시지로만 표시됩니다.)
        slam_status["last_error"] = f"라이다 연결 실패: {e}"
        return

    slam_status["connected"] = True
    scan_id = 0

    last_map_save = 0.0     # 마지막으로 지도를 저장했던 시각 (time.time() 값)
    last_load_check = 0.0   # 마지막으로 시스템 부하를 측정했던 시각

    try:
        # lidar.iter_scans()는 "라이다 한 바퀴(360도)"가 끝날 때마다 그 스캔 데이터를
        # 하나씩 넘겨주는 제너레이터입니다. min_len=60은 "한 바퀴에 점이 60개 미만이면
        # 노이즈성 스캔으로 보고 건너뛴다"는 뜻으로, 기존 주행 코드와 기준을 맞췄습니다.
        for scan in lidar.iter_scans(min_len=60):
            if stop_event.is_set():
                # 메인 스레드에서 q를 눌러 종료 신호를 보낸 경우, 다음 스캔을
                # 기다리지 않고 바로 루프를 빠져나감 (반응성을 위해 매 스캔마다 체크)
                break

            # ---------- 1. 스캔을 SLAM 입력 형식으로 변환 ----------
            distances_mm = scan_to_distance_array(scan, laser.scan_size)

            # ---------- 2. SLAM 갱신 ----------
            # 바퀴 엔코더가 없으므로 pose_change(주행거리 정보)는 안 넘기고
            # 순수하게 라이다 스캔 매칭만으로 위치를 추정합니다 ("라이다 오도메트리").
            # 이 방식은 로봇이 빠르게, 크게 움직일수록 오차가 커질 수 있다는 점을
            # 감안해야 합니다 (나중에 엔코더나 IMU로 보완 가능).
            slam.update(distances_mm)

            # 현재 추정 위치(로봇 기준 좌표계, mm 단위 x/y + 각도)를 읽어와서
            # 화면에 보여줄 수 있게 공유 딕셔너리에 저장
            x_mm, y_mm, theta_deg = slam.getpos()
            slam_status["robot_x_mm"] = x_mm
            slam_status["robot_y_mm"] = y_mm
            slam_status["robot_theta_deg"] = theta_deg

            scan_id += 1
            slam_status["scan_count"] = scan_id

            now = time.time()

            # ---------- 3. 주기적으로 지도를 이미지로 저장 (진행상황 확인용) ----------
            # 매 스캔마다 저장하면 파일 쓰기 때문에 CPU/디스크 부하가 커지니,
            # MAP_SAVE_EVERY_SEC(기본 5초)에 한 번씩만 저장하도록 시간 간격을 둠.
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

            # ---------- 4. 주기적으로 시스템 부하 체크 (이번 단계의 핵심 확인 포인트) ----------
            if (now - last_load_check) >= LOAD_CHECK_EVERY_SEC:
                # os.getloadavg()는 라즈베리파이/리눅스에서 별도 설치 없이 바로 쓸 수 있는
                # "최근 1분/5분/15분 평균 부하"입니다. 코어 개수를 넘으면 과부하 상태로 보시면 됩니다.
                # (Pi4는 코어 4개이므로, 이 값이 4를 넘으면 CPU가 버거워하고 있다는 뜻입니다.
                #  값이 계속 올라가기만 하고 안 떨어지면, 조종+SLAM을 동시에 돌리기엔
                #  이 라즈베리파이 모델이 부족하다는 신호이니 다음 단계 설계를
                #  다시 고민해봐야 합니다.)
                load1, load5, load15 = os.getloadavg()
                slam_status["load_1min"] = load1
                last_load_check = now

    except RPLidarException as e:
        # 라이다와 통신하다가 중간에 에러가 나는 경우 (케이블 접촉 불량, 노이즈 등)
        # 프로그램을 죽이지 않고 에러 메시지만 기록. 메인 조종 루프는 계속 동작함.
        slam_status["last_error"] = f"라이다 통신 오류: {e}"
    finally:
        # ---------- 종료 처리: 무슨 이유로 루프를 빠져나오든(정상 종료/에러/q) 항상 실행됨 ----------
        try:
            lidar.stop()
            lidar.stop_motor()
            lidar.disconnect()
        except Exception:
            # 이미 연결이 끊겨있는 등의 이유로 정지/해제 자체가 실패해도 무시하고 진행
            # (종료 처리 중에 또 에러가 나서 프로그램이 멈추는 것보다는 나음)
            pass
        slam_status["connected"] = False

        # 종료 직전, 마지막 지도를 한 번 더 저장 (타임스탬프 붙여서 최종본으로 따로 보관)
        # -> logs/slam_map_latest.png는 계속 덮어써지지만, 이 파일은 "이번 실행의 최종 결과물"로
        #    남겨두는 용도라서 실행할 때마다 새 파일이 하나씩 쌓입니다.
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
                # 종료 처리 중 저장 실패는 치명적이지 않으니 조용히 넘어감
                pass


# ============================================================
# 메인 (curses)
# ============================================================
def main(stdscr):
    """
    curses.wrapper()가 자동으로 호출해주는 진입점 함수.
    stdscr는 curses가 만들어주는 "터미널 화면 전체를 나타내는 객체"로,
    여기에 글자를 쓰고(addstr) refresh()를 호출해야 실제 화면에 반영됩니다.
    """
    global SPEED_FAST, TARGET_SPEED, TARGET_STEER

    # ---------- curses 초기 설정 ----------
    curses.curs_set(0)          # 터미널 커서(깜빡이는 막대) 숨기기 - 화면이 지저분해 보이는 걸 방지
    stdscr.nodelay(True)        # getch()가 키 입력이 없어도 기다리지 않고 즉시 -1을 반환하게 함
                                 # (이게 없으면 키를 누를 때까지 프로그램 전체가 멈춰버림)
    stdscr.timeout(KEY_REPEAT_TIMEOUT_MS)  # getch()가 최대 이 시간(ms)만큼만 기다리도록 설정
                                            # (nodelay와 timeout을 같이 쓰면, "키가 없으면 이 시간
                                            #  안에 -1을 반환"하는 식으로 동작 - 결과적으로 메인 루프가
                                            #  대략 이 주기로 계속 돌게 됨)

    # ---------- 하드웨어 초기화 (기존 코드와 동일한 핀 배치) ----------
    reset_mcu()       # robot_hat 보드의 마이크로컨트롤러를 초기 상태로 리셋
    time.sleep(0.5)   # 리셋 직후 바로 명령을 보내면 씹힐 수 있어서 약간 대기

    left_motor = Motor(PWM("P13"), Pin("D4"))   # 왼쪽 바퀴 모터 (PWM 핀 + 방향 핀)
    right_motor = Motor(PWM("P12"), Pin("D5"))  # 오른쪽 바퀴 모터
    steer_servo = Servo("P2")                    # 조향 서보모터

    steer_servo.angle(0)   # 시작할 때 바퀴를 정면(0도)으로 정렬
    time.sleep(0.5)

    # ---------- SLAM 스레드 시작 ----------
    # daemon=True로 만들면, 메인 스레드가 완전히 끝날 때 이 스레드도 강제로 같이 종료됩니다.
    # (하지만 정상적인 경우엔 아래 finally 블록에서 stop_event.set() + join()으로
    #  먼저 "정상적으로" 종료 요청을 하고 마무리 시간을 주기 때문에, daemon은 최후의 안전장치입니다.)
    slam_thread = threading.Thread(target=slam_worker, daemon=True)
    slam_thread.start()

    # ---------- 화면 안내 문구 (한 번만 출력하면 되는 고정 텍스트) ----------
    safe_addstr(stdscr, 0, 0, "=== 조종 + 실시간 SLAM 매핑 (4단계) ===")
    safe_addstr(stdscr, 1, 0, "w: 전진  s: 후진  a: 좌회전  d: 우회전")
    safe_addstr(stdscr, 2, 0, "x: 조향 정면복귀  space: 정지  q: 종료")
    safe_addstr(stdscr, 3, 0, "-" * 55)

    try:
        while True:
            # ---------- 1. 키 입력 확인 ----------
            key = stdscr.getch()   # 키가 눌렸으면 그 키 코드를, 안 눌렸으면 -1을 반환 (nodelay+timeout 덕분)

            if key == ord('q'):
                break   # q를 누르면 while 루프를 빠져나가서 아래 finally(종료 처리)로 넘어감

            if key != -1:
                handle_key(key)   # 뭔가 눌렸으면 목표 속도/조향값 갱신

            # ---------- 2. 실제 모터 제어 (매 루프마다 반드시 호출) ----------
            # 키를 안 누른 순간에도 이 두 줄은 계속 실행되어야, 슬로우스타트/슬로우스탑이
            # 끊기지 않고 목표값을 향해 계속 진행됩니다.
            set_speed(left_motor, right_motor, TARGET_SPEED)
            set_steer(steer_servo, TARGET_STEER)

            # ---------- 3. 조종 상태 표시 (매 루프 갱신되는 부분) ----------
            safe_addstr(stdscr, 5, 0, f"현재속도(SPEED_FAST): {SPEED_FAST:4d}   ")
            safe_addstr(stdscr, 6, 0, f"목표속도(TARGET_SPEED): {TARGET_SPEED:4d}   ")
            safe_addstr(stdscr, 7, 0, f"조향각(TARGET_STEER): {TARGET_STEER:4d}   ")
            safe_addstr(stdscr, 9, 0, "-" * 55)

            # ---------- 4. SLAM 상태 표시 ----------
            # slam_status는 백그라운드 스레드(slam_worker)가 계속 갱신하고 있는 값이라,
            # 여기서는 그냥 "읽어서 화면에 보여주기만" 합니다.
            conn = "연결됨" if slam_status["connected"] else "연결 대기/종료"
            safe_addstr(stdscr, 10, 0, f"라이다 상태       : {conn}                ")
            safe_addstr(stdscr, 11, 0, f"처리된 스캔 수     : {slam_status['scan_count']:6d}          ")
            safe_addstr(stdscr, 12, 0, (
                f"추정 위치(mm)     : x={slam_status['robot_x_mm']:.0f}, "
                f"y={slam_status['robot_y_mm']:.0f}, "
                f"각도={slam_status['robot_theta_deg']:.1f}도      "
            ))
            safe_addstr(stdscr, 13, 0, f"최근 저장된 지도   : {slam_status['map_saved_path']}                    ")

            # ---------- 5. CPU 부하 표시 (이번 4단계의 핵심 확인 포인트) ----------
            # "조종 + 실시간 SLAM을 같이 돌려도 라즈베리파이가 버티는가?"를 확인하기 위해
            # 이 값을 계속 지켜봐야 합니다. 코어 4개(Pi4 기준) 넘으면 경고 문구를 붙여줌.
            load = slam_status["load_1min"]
            warn = "  <-- 코어 4개 기준 과부하 의심!" if load > 4.0 else ""
            safe_addstr(stdscr, 15, 0, f"시스템 부하(1분평균): {load:.2f}{warn}                    ")

            # ---------- 6. 에러 메시지 표시 (있을 때만) ----------
            err = slam_status["last_error"]
            if err:
                safe_addstr(stdscr, 17, 0, f"[경고] {err}                                          ")

            stdscr.refresh()   # 지금까지 addstr로 쓴 내용을 실제 화면에 반영 (이거 없으면 화면이 안 바뀜)

    finally:
        # ---------- 종료 처리: q를 누르든, 에러로 빠져나오든 항상 실행됨 (try/finally 구조) ----------

        # 1) 조종 쪽 안전 정지: 목표 속도를 0으로 두고, 실제 속도(SPEED_FAST)가 0이 될 때까지
        #    계속 set_speed()를 불러서 "서서히" 멈춤 (급정거 방지, 슬로우스탑)
        TARGET_SPEED = 0
        while SPEED_FAST != 0:
            set_speed(left_motor, right_motor, 0)
            time.sleep(0.05)

        set_steer(steer_servo, 0)     # 조향도 정면으로 원위치
        left_motor.speed(0)           # 혹시 모를 잔여 속도까지 확실하게 0으로 고정
        right_motor.speed(0)

        # 2) SLAM 스레드에게 "너도 이제 멈춰"라고 신호를 보내고, 최대 5초까지 정상 종료를 기다림.
        #    join(timeout=5)은 "5초 안에 스레드가 안 끝나도 일단 다음 코드로 넘어간다"는 뜻 -
        #    프로그램이 무한정 멈춰있지 않도록 하는 안전장치입니다.
        stop_event.set()
        slam_thread.join(timeout=5)


if __name__ == "__main__":
    curses.wrapper(main)
    # curses.wrapper()를 쓰면, main() 안에서 어떤 이유로 프로그램이 죽거나 끝나더라도
    # 터미널 화면 설정을 자동으로 원래 상태로 복구해줍니다 (안 쓰면 터미널이 이상하게 깨질 수 있음).
    print("조종 및 SLAM 매핑 종료 완료.")
    print(f"최종 지도 저장 위치: {slam_status['map_saved_path']}")
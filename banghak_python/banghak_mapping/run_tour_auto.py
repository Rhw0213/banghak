#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
run_tour_auto.py
================
run_tour.py와 다른 점: **로봇을 특정 위치/방향에 정확히 맞춰놓지 않아도** 됩니다.

시작하면 먼저 라이다로 몇 바퀴 스캔을 떠서, 저장된 지도와 비교해 "지금 로봇이
그 지도 안에서 어디에, 어느 방향을 보고 있는지"를 스스로 추정합니다
(relocalize.py). 그 다음 그 추정 위치를 기준으로, 저장된 투어 경로를 따라
자동 주행합니다.

*** 한계는 꼭 알아두세요 (relocalize.py 상단 설명 참고) ***
    - 계산에 시간이 좀 걸립니다 (라즈베리파이에서 수 초~수십 초 예상, 최초 1회만).
    - 트랙이 대칭적이면 엉뚱한 방향으로 오판할 수 있습니다. 위치추정 결과
      (점수, 후보 위치)를 시작 전에 화면에 보여주니, 이상해 보이면 Ctrl+C로
      중단하고 로봇을 확실히 트랙 안쪽으로 옮겨서 다시 시도하세요.
    - 로봇이 대략 트랙 범위(±--search-range-m) 안에 있어야 찾을 수 있습니다.

사용법
    python3 run_tour_auto.py --map logs/slam_map_XXXX.npy \\
        --tour-path logs/tour_path_XXXX.npy

    # 시작점까지만 먼저 테스트
    python3 run_tour_auto.py --map logs/slam_map_XXXX.npy \\
        --tour-path logs/tour_path_XXXX.npy --stop-at-start-only

    # 탐색 범위/정밀도 조정 (기본: +-1.5m, 위치 50mm 간격, 각도 3도 간격)
    python3 run_tour_auto.py --map logs/slam_map_XXXX.npy \\
        --tour-path logs/tour_path_XXXX.npy --search-range-m 2.0 --angle-step-deg 2
"""

import argparse
import curses
import math
import os
import re
import threading
import time

import numpy as np

import teleop_slam as ts
from astar_planner import PIXELS_PER_METER, MAP_SIZE_PIXELS
from relocalize import build_distance_transform, localize


# ============================================================
# 설정값 (run_tour.py와 동일)
# ============================================================
ORIGIN_PX = MAP_SIZE_PIXELS // 2
MM_PER_PX = 1000.0 / PIXELS_PER_METER

ARRIVE_TOLERANCE_MM = 80.0
SLOWDOWN_DIST_MM = 500.0
APPROACH_SPEED_DEFAULT = ts.MAX_SPEED   # teleop_slam.py 수동조종(w키)과 동일하게 항상 50으로 통일
MIN_SPEED = ts.MAX_SPEED   # 위와 같은 이유로 감속 로직도 사실상 항상 50이 되게 함 (오도메트리 캘리브레이션이
                            # 정확히 MAX_SPEED 기준으로만 실측되어 있어서, 다른 속도는 선형 비례를 가정한
                            # 미검증 구간이었음 - 아예 그 불확실성을 없애기 위해 속도를 하나로 고정)
STEER_KP_DEFAULT = 1.2

# ts.STEER_LIMIT(40도)까지 조향을 쓰면 지면 저항 때문에 모터 힘이 부족해서
# 차가 못 움직이는 게 실측으로 확인됐습니다 (무부하에선 도는데 지면에선 안 움직임).
# 주행 중에는 이 값으로 조향을 제한합니다. 필요하면 --max-drive-steer로 조정하세요.
MAX_DRIVE_STEER_DEG = 25
EMERGENCY_STOP_CM = 30.0   # 항상 최고속도(50)로 달리므로 반응거리 여유를 더 둠 (기존 15 -> 30)
CONTROL_TICK_MS = 100

LOAD_CHECK_EVERY_SEC = 2.0   # teleop_slam.py와 동일한 방식의 시스템 부하 체크
MIN_WAYPOINT_GAP_MM = 100.0


def px_to_mm(row, col):
    return (col - ORIGIN_PX) * MM_PER_PX, (row - ORIGIN_PX) * MM_PER_PX


def simplify_waypoints(points_mm, min_gap_mm):
    if len(points_mm) == 0:
        return points_mm
    simplified = [points_mm[0]]
    for p in points_mm[1:]:
        last = simplified[-1]
        if math.hypot(p[0] - last[0], p[1] - last[1]) >= min_gap_mm:
            simplified.append(p)
    if simplified[-1] != points_mm[-1]:
        simplified.append(points_mm[-1])
    return simplified


# ============================================================
# 위치추적 전용 스레드: SLAM으로 새 지도를 만드는 대신, 이미 저장된 지도
# (map_array)와 매 스캔을 직접 비교해서 계속 위치를 다시 찾습니다.
# ============================================================
def connect_lidar_with_retry(port, max_retries=5):
    """
    lidar_ultra_vision.py의 connect_lidar()와 동일한 패턴: 연결 -> stop/reset ->
    안정화 대기 -> health 확인까지 통째로 재시도합니다. 단순히 스캔
    제너레이터만 새로 만드는 것보다 훨씬 확실하게 연결 자체를 리셋합니다.
    """
    for attempt in range(1, max_retries + 1):
        lidar = None
        try:
            lidar = ts.RPLidar(port)
            try:
                lidar.stop()
                lidar.stop_motor()
            except Exception:
                pass
            time.sleep(0.3)
            try:
                lidar.reset()
            except Exception:
                pass
            time.sleep(1.5)
            try:
                lidar._serial.reset_input_buffer()
            except Exception:
                pass
            lidar.get_health()   # 여기서 실패하면 아래 except로 넘어가 재시도
            return lidar
        except Exception as e:
            if lidar is not None:
                try:
                    lidar.disconnect()
                except Exception:
                    pass
            if attempt < max_retries:
                time.sleep(1.5)
    return None


def tracking_worker(map_array, num_init_scans, search_range_m, xy_step_mm,
                     angle_step_deg, score_warn_threshold):
    """
    한 번 연결한 라이다로 계속 스캔을 받으면서:
    1) 처음 num_init_scans바퀴는 스캔을 모아서, 넓은 범위(search_range_m)로
       전체 위치를 한 번 찾습니다 (초기 고정, 몇 초~몇십 초 걸림).
    2) 그 다음부터는 매 스캔마다 "직전 위치 주변 좁은 범위"만 다시 확인해서
       위치를 계속 갱신합니다 (좁은 범위라 훨씬 빠름).

    teleop_slam.py의 RMHC_SLAM(새 지도를 처음부터 그리는 방식)은 아예 안 씁니다.
    이유: 새로 지도를 그리면서 추적하면, 이번 세션에서 만들어지는 지도가
    아직 부실해서(백지에 가까움) 위치가 방향을 잃고 표류하는 문제가 실측으로
    확인됐습니다 (지도 범위 6m인데 -5000mm 같은 말도 안 되는 값이 나옴).
    여기서는 항상 "이미 완성된, 확실한 지도"랑만 비교하기 때문에 표류할
    이유가 없습니다.
    """
    dist_transform = build_distance_transform(map_array)
    ts.slam_status["initial_fix_done"] = False

    lidar = connect_lidar_with_retry(ts.LIDAR_PORT)
    if lidar is None:
        ts.slam_status["last_error"] = "라이다 연결 실패 (재시도 5회 모두 실패)"
        return

    ts.slam_status["connected"] = True
    scan_id = 0
    current_x, current_y, current_theta = 0.0, 0.0, 0.0
    init_angles_list, init_distances_list = [], []
    skip_count = 0
    consecutive_fail = 0

    scan_iter = lidar.iter_scans(min_len=60)

    try:
        while not ts.stop_event.is_set():
            try:
                scan = next(scan_iter)

                angles, distances = [], []
                for quality, angle, distance in scan:
                    if distance <= 0 or quality < 10:
                        continue
                    corrected = (angle + ts.LIDAR_OFFSET) % 360
                    angles.append(corrected)
                    distances.append(distance)
                angles = np.array(angles, dtype=np.float64)
                distances = np.array(distances, dtype=np.float64)

                scan_id += 1
                ts.slam_status["scan_count"] = scan_id

                if not ts.slam_status["initial_fix_done"]:
                    # ---- 초기 고정: 스캔 몇 바퀴 모아서 넓게 한 번 탐색 ----
                    init_angles_list.append(angles)
                    init_distances_list.append(distances)
                    if len(init_angles_list) < num_init_scans:
                        continue

                    all_angles = np.concatenate(init_angles_list)
                    all_distances = np.concatenate(init_distances_list)
                    x, y, theta, score = localize(
                        dist_transform, ORIGIN_PX, MM_PER_PX, all_angles, all_distances,
                        x_range_mm=search_range_m * 1000, y_range_mm=search_range_m * 1000,
                        xy_step_mm=xy_step_mm, angle_step_deg=angle_step_deg)
                    current_x, current_y, current_theta = x, y, theta

                    if score > score_warn_threshold:
                        ts.slam_status["last_error"] = (
                            f"초기 위치추정 점수 낮음({score:.1f}px) - 위치가 틀렸을 수 있음")
                    ts.slam_status["initial_fix_done"] = True
                else:
                    # ---- 계속 추적: 직전 위치 주변 좁은 범위만 빠르게 재탐색 ----
                    x, y, theta, score = localize(
                        dist_transform, ORIGIN_PX, MM_PER_PX, angles, distances,
                        x_range_mm=300, y_range_mm=300, xy_step_mm=25,
                        center_x_mm=current_x, center_y_mm=current_y,
                        angle_center_deg=current_theta, angle_range_deg=30, angle_step_deg=3)
                    current_x, current_y, current_theta = x, y, theta

                ts.slam_status["robot_x_mm"] = current_x
                ts.slam_status["robot_y_mm"] = current_y
                ts.slam_status["robot_theta_deg"] = current_theta
                consecutive_fail = 0

            except StopIteration:
                break
            except ts.RPLidarException as e:
                skip_count += 1
                consecutive_fail += 1
                cable_hint = "  <-- 계속 실패 중, USB 케이블/커넥터 확인해보세요!" if consecutive_fail >= 100 else ""
                ts.slam_status["last_error"] = (
                    f"라이다 통신 오류(스캔 건너뜀, 누적 {skip_count}회, 연속 {consecutive_fail}회): {e}{cable_hint}")
                # 제너레이터는 한 번 예외를 던지면 그 객체 자체가 끝나버려서(파이썬
                # 기본 동작), 그대로 두면 다음 next() 호출에서 계속 StopIteration만
                # 나며 조용히 루프가 끝나버립니다. 가벼운 경우는 제너레이터만
                # 새로 만들고, 30번 연속 실패처럼 심한 경우는 lidar_ultra_vision.py의
                # connect_lidar()처럼 연결 자체를 완전히 끊었다 재연결합니다
                # (제너레이터만 새로 만드는 것보다 훨씬 확실하게 리셋됨).
                if consecutive_fail % 30 == 0:
                    ts.slam_status["last_error"] += "  (라이다 완전 재연결 시도 중...)"
                    try:
                        lidar.disconnect()
                    except Exception:
                        pass
                    new_lidar = connect_lidar_with_retry(ts.LIDAR_PORT)
                    if new_lidar is not None:
                        lidar = new_lidar
                        scan_iter = lidar.iter_scans(min_len=60)
                    else:
                        ts.slam_status["last_error"] = "라이다 재연결 실패 - 케이블/전원 확인 필요"
                else:
                    try:
                        scan_iter = lidar.iter_scans(min_len=60)
                    except Exception:
                        pass
                time.sleep(0.02)   # 통신이 계속 불안정할 때 CPU를 100% 갈아먹지 않도록 살짝 쉼
                continue
            except Exception as e:
                # "many values to unpack" 등 라이다 라이브러리 내부의 일시적 파싱
                # 오류 - 스캔을 가져오는 단계뿐 아니라, 가져온 스캔 안의 개별
                # 포인트를 처리하는 단계나 localize() 계산 중에 나는 오류까지
                # 전부 여기서 잡아서, 그 스캔 한 번만 건너뛰고 스레드는 계속
                # 살려둡니다. (재연결 전략은 위 RPLidarException 처리와 동일)
                skip_count += 1
                consecutive_fail += 1
                cable_hint = "  <-- 계속 실패 중, USB 케이블/커넥터 확인해보세요!" if consecutive_fail >= 100 else ""
                ts.slam_status["last_error"] = (
                    f"스캔 처리 오류(건너뜀, 누적 {skip_count}회, 연속 {consecutive_fail}회): "
                    f"{type(e).__name__}: {e}{cable_hint}")
                if consecutive_fail % 30 == 0:
                    ts.slam_status["last_error"] += "  (라이다 완전 재연결 시도 중...)"
                    try:
                        lidar.disconnect()
                    except Exception:
                        pass
                    new_lidar = connect_lidar_with_retry(ts.LIDAR_PORT)
                    if new_lidar is not None:
                        lidar = new_lidar
                        scan_iter = lidar.iter_scans(min_len=60)
                    else:
                        ts.slam_status["last_error"] = "라이다 재연결 실패 - 케이블/전원 확인 필요"
                else:
                    try:
                        scan_iter = lidar.iter_scans(min_len=60)
                    except Exception:
                        pass
                time.sleep(0.02)
                continue

    except ts.RPLidarException as e:
        ts.slam_status["last_error"] = f"라이다 통신 오류: {e}"
    except Exception as e:
        ts.slam_status["last_error"] = f"예상치 못한 오류로 위치추적 스레드 종료: {type(e).__name__}: {e}"
    finally:
        try:
            lidar.stop()
            lidar.stop_motor()
            lidar.disconnect()
        except Exception:
            pass
        ts.slam_status["connected"] = False


def compute_control(x_mm, y_mm, theta_deg, target_x, target_y, steer_kp, approach_speed):
    dx = target_x - x_mm
    dy = target_y - y_mm
    distance = math.hypot(dx, dy)
    target_bearing_deg = math.degrees(math.atan2(dy, dx))
    heading_error_deg = target_bearing_deg - theta_deg
    heading_error_deg = (heading_error_deg + 180) % 360 - 180
    # ts.STEER_LIMIT(40도)가 아니라 MAX_DRIVE_STEER_DEG(25도)로 제한합니다.
    # 실측 결과, 조향 40도 근처에서는 지면 저항 때문에 모터 힘이 부족해서
    # 차가 아예 못 움직이는 게 확인됐습니다 (무부하에서는 잘 도는데 지면에
    # 닿으면 안 움직임 = 토크 부족). 25도로 낮춰서 저항을 줄입니다.
    steer_cmd = max(-MAX_DRIVE_STEER_DEG, min(MAX_DRIVE_STEER_DEG, steer_kp * heading_error_deg))
    if distance <= ARRIVE_TOLERANCE_MM:
        # 목표 근처에서는 atan2 각도 계산이 불안정해져서 조향도 같이 0으로 고정
        speed_cmd = 0
        steer_cmd = 0.0
    else:
        if distance < SLOWDOWN_DIST_MM:
            ratio = distance / SLOWDOWN_DIST_MM
            speed_cmd = int(MIN_SPEED + (approach_speed - MIN_SPEED) * ratio)
        else:
            speed_cmd = approach_speed

        # 조향이 많이 꺾일수록 속도를 같이 줄입니다. 축간거리가 짧은 차는
        # 조향을 많이 꺾은 채 빠르게 전진시키면 바퀴가 헛돌거나 걸려서
        # 실제로는 거의 못 움직이면서, 소프트웨어(오도메트리 추정)만
        # "명령한 대로 다 움직였다"고 믿어버려 위치가 확 틀어지는 문제가
        # 있었습니다. 최대조향 근처에서는 최소속도까지 낮춰서 이 문제를 완화합니다.
        steer_ratio = abs(steer_cmd) / MAX_DRIVE_STEER_DEG   # 0(직진)~1(최대조향)
        steer_speed_scale = 1.0 - 0.6 * steer_ratio      # 최대조향 시 40%까지 감속
        speed_cmd = max(MIN_SPEED, int(speed_cmd * steer_speed_scale))

    return steer_cmd, speed_cmd, distance, heading_error_deg


def guess_map_path(tour_path):
    """
    tour_planner.py가 저장한 파일명은 'tour_path_<지도이름>_<타임스탬프>.npy'
    형식입니다 (예: tour_path_slam_map_20260729_142133_20260729_145423.npy
    -> 원본 지도는 slam_map_20260729_142133.npy). 이 규칙을 이용해서
    --map을 안 줬을 때 자동으로 원본 지도 경로를 유추합니다.
    """
    base = os.path.basename(tour_path)
    name, _ = os.path.splitext(base)

    m = re.match(r"^tour_path_(.+)_\d{8}_\d{6}$", name)
    if not m:
        return None
    map_name = m.group(1) + ".npy"
    return os.path.join(os.path.dirname(tour_path) or ".", map_name)


# ============================================================
# 메인 (curses)
# ============================================================
def main(stdscr, args, waypoints_mm, map_array):
    curses.curs_set(0)
    stdscr.nodelay(True)
    stdscr.timeout(CONTROL_TICK_MS)

    ts.reset_mcu()
    time.sleep(0.5)

    left_motor = ts.Motor(ts.PWM("P13"), ts.Pin("D4"))
    right_motor = ts.Motor(ts.PWM("P12"), ts.Pin("D5"))
    steer_servo = ts.Servo("P2")
    sonar = ts.Ultrasonic(ts.Pin("D2"), ts.Pin("D3")) if hasattr(ts, "Ultrasonic") else None

    steer_servo.angle(0)
    time.sleep(0.5)

    tracking_thread = threading.Thread(
        target=tracking_worker,
        args=(map_array, args.num_init_scans, args.search_range_m,
              args.xy_step_mm, args.angle_step_deg, args.score_warn_threshold),
        daemon=True)
    tracking_thread.start()

    wp_index = 0
    finished = False
    arrived_hold_since = None
    last_load_check = 0.0
    load_1min = 0.0

    ts.safe_addstr(stdscr, 0, 0, "=== 재위치추정(연속) + 투어 자동 실행 ===")
    ts.safe_addstr(stdscr, 1, 0, f"웨이포인트 {len(waypoints_mm)}개  "
                    f"{'(시작점까지만)' if args.stop_at_start_only else '(전체 투어)'}  |  q: 중단")
    ts.safe_addstr(stdscr, 3, 0, "-" * 60)

    try:
        while True:
            key = stdscr.getch()
            if key == ord('q'):
                break

            if not ts.slam_status.get("initial_fix_done", False):
                # 아직 초기 위치를 못 찾았으면 절대 움직이지 않고 대기만 합니다.
                ts.safe_addstr(stdscr, 5, 0, f"[초기 위치추정 중...] 스캔 {ts.slam_status['scan_count']}개 처리됨. "
                                f"로봇을 움직이지 마세요.        ")
                err = ts.slam_status["last_error"]
                if err:
                    ts.safe_addstr(stdscr, 14, 0, f"[경고] {err}                                          ")
                stdscr.refresh()
                continue

            x_mm = ts.slam_status["robot_x_mm"]
            y_mm = ts.slam_status["robot_y_mm"]
            theta_deg = ts.slam_status["robot_theta_deg"]

            if finished:
                steer_cmd, speed_cmd, distance, heading_error = 0.0, 0, 0.0, 0.0
            else:
                target_x, target_y = waypoints_mm[wp_index]
                steer_cmd, speed_cmd, distance, heading_error = compute_control(
                    x_mm, y_mm, theta_deg, target_x, target_y,
                    args.steer_kp, args.approach_speed)
                ts.TARGET_STEER = int(round(steer_cmd))

                if distance <= ARRIVE_TOLERANCE_MM:
                    if args.stop_at_start_only and wp_index == 0:
                        finished = True
                        speed_cmd = 0
                        arrived_hold_since = arrived_hold_since or time.time()
                    elif wp_index < len(waypoints_mm) - 1:
                        wp_index += 1
                    else:
                        finished = True
                        speed_cmd = 0
                        arrived_hold_since = arrived_hold_since or time.time()

            emergency = False
            ultra_cm = -1.0
            if sonar is not None:
                try:
                    d = sonar.read()
                    if d is not None and d > 0:
                        ultra_cm = d
                        if d < EMERGENCY_STOP_CM:
                            emergency = True
                except Exception:
                    pass

            slam_dead = not ts.slam_status["connected"]
            if slam_dead:
                emergency = True

            if emergency:
                speed_cmd = 0
                steer_cmd = 0
                # ts.set_speed()는 감속도 SPEED_STEP(3)씩 서서히 줄이는 방식이라,
                # 지금처럼 항상 최고속도(50)로 달리는 상황에서는 비상정지가 감지돼도
                # 실제로 멈추기까지 1초 넘게 걸려서(50->0에 약 17틱=1.7초) 그 사이에
                # 벽을 박을 수 있습니다. 비상상황에서는 램프 다 무시하고 그냥
                # 모터에 즉시 0을 박아버립니다.
                ts.SPEED_FAST = 0
                left_motor.speed(0)
                right_motor.speed(0)
                ts.set_steer(steer_servo, 0)
            else:
                ts.set_speed(left_motor, right_motor, speed_cmd)
                ts.set_steer(steer_servo, steer_cmd)

            now = time.time()
            if (now - last_load_check) >= LOAD_CHECK_EVERY_SEC:
                load_1min, _, _ = os.getloadavg()
                last_load_check = now
            load_warn = "  <-- 코어 4개 기준 과부하 의심!" if load_1min > 4.0 else ""

            ts.safe_addstr(stdscr, 5, 0, f"웨이포인트     : {wp_index + 1}/{len(waypoints_mm)}                    ")
            ts.safe_addstr(stdscr, 6, 0, f"위치(지도기준,mm): x={x_mm:7.1f} y={y_mm:7.1f} theta={theta_deg:6.1f}도    ")
            ts.safe_addstr(stdscr, 7, 0, f"목표까지 거리   : {distance:7.1f}mm   각도오차: {heading_error:6.1f}도      ")
            ts.safe_addstr(stdscr, 8, 0, f"명령           : steer={steer_cmd:5.1f}  speed={speed_cmd:4d}  "
                            f"(실제모터 SPEED_FAST={ts.SPEED_FAST:4d})          ")
            ts.safe_addstr(stdscr, 9, 0, f"초음파 전방거리 : {ultra_cm:5.1f}cm" +
                            ("  <-- 비상정지!" if emergency else "                "))
            ts.safe_addstr(stdscr, 10, 0, "-" * 60)
            ts.safe_addstr(stdscr, 11, 0, f"시스템 부하(1분평균): {load_1min:.2f}{load_warn}                    ")

            if slam_dead:
                ts.safe_addstr(stdscr, 12, 0, "[치명적] 위치추정 스레드가 죽었습니다! 정지함. q로 종료하세요      ")
            elif finished:
                held = time.time() - arrived_hold_since
                ts.safe_addstr(stdscr, 12, 0, f"[완료] 목표 지점 도착 ({held:.1f}초째) - q로 종료하세요        ")
            else:
                ts.safe_addstr(stdscr, 12, 0, " " * 60)

            err = ts.slam_status["last_error"]
            if err:
                ts.safe_addstr(stdscr, 14, 0, f"[경고] {err}                                          ")

            stdscr.refresh()

    finally:
        ts.TARGET_SPEED = 0
        ts.TARGET_STEER = 0
        while ts.SPEED_FAST != 0:
            ts.set_speed(left_motor, right_motor, 0)
            time.sleep(0.05)
        ts.set_steer(steer_servo, 0)
        left_motor.speed(0)
        right_motor.speed(0)

        ts.stop_event.set()
        tracking_thread.join(timeout=5)


# ============================================================
# 진단용: 위치추정/SLAM/투어경로 다 빼고, 모터 명령만 순수하게 테스트
# ============================================================
def motor_test(steer_deg, speed_target, duration_sec):
    """
    run_tour_auto.py의 나머지 로직(위치추정, SLAM, 제어계산)을 전부 빼고,
    딱 '모터 객체 만들고 -> set_speed/set_steer 호출'만 반복합니다.
    이걸로도 안 움직이면, 문제는 위치추정/제어 로직이 아니라 이 스크립트가
    모터 객체를 만들거나 명령을 넣는 방식 자체에 있다는 뜻입니다.
    (반대로 이건 잘 움직이는데 평소엔 안 움직인다면, 문제가 그 사이의
     다른 로직 - 예: 스레드 경합, curses 루프 타이밍 등 - 에 있다는 뜻)
    """
    print(f"[모터 테스트] steer={steer_deg}도, speed={speed_target}, {duration_sec}초간 실행합니다.")
    print("차량 뒷바퀴가 지면에서 뜬 안전한 상태인지 확인하고 Enter를 누르세요...")
    input()

    ts.reset_mcu()
    time.sleep(0.5)

    left_motor = ts.Motor(ts.PWM("P13"), ts.Pin("D4"))
    right_motor = ts.Motor(ts.PWM("P12"), ts.Pin("D5"))
    steer_servo = ts.Servo("P2")
    steer_servo.angle(0)
    time.sleep(0.5)

    ts.TARGET_STEER = int(steer_deg)   # estimate_pose_change 등에서 참조하는 전역값도 맞춰줌

    t0 = time.time()
    tick = 0
    try:
        while time.time() - t0 < duration_sec:
            ts.set_speed(left_motor, right_motor, speed_target)
            ts.set_steer(steer_servo, steer_deg)
            tick += 1
            if tick % 5 == 0:
                print(f"  tick={tick}  SPEED_FAST={ts.SPEED_FAST}  (이 값이 {speed_target}까지 올라가는데 "
                      f"실제로 안 움직이면 모터/배선 쪽 문제입니다)")
            time.sleep(0.1)
    finally:
        ts.set_speed(left_motor, right_motor, 0)
        ts.set_steer(steer_servo, 0)
        left_motor.speed(0)
        right_motor.speed(0)
        print("[모터 테스트] 종료.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="위치추정 후 저장된 투어 경로를 자동 실행")
    parser.add_argument("--motor-test", action="store_true",
                        help="위치추정/SLAM/투어경로 다 빼고 모터 명령만 순수하게 테스트")
    parser.add_argument("--motor-test-steer", type=float, default=-40.0)
    parser.add_argument("--motor-test-speed", type=int, default=30)
    parser.add_argument("--motor-test-sec", type=float, default=3.0)
    parser.add_argument("--map", default=None,
                        help="위치추정에 쓸 지도 파일 (.npy). 안 주면 --tour-path 파일명에서 자동 유추 시도.")
    parser.add_argument("--tour-path", default=None, help="tour_planner.py가 저장한 경로 파일 (tour_path_*.npy)")
    parser.add_argument("--stop-at-start-only", action="store_true")
    parser.add_argument("--steer-kp", type=float, default=STEER_KP_DEFAULT)
    parser.add_argument("--max-drive-steer", type=float, default=MAX_DRIVE_STEER_DEG,
                        help=f"주행 중 조향 상한(도), 기본 {MAX_DRIVE_STEER_DEG} (지면 저항으로 힘이 부족하면 더 낮춰보세요)")
    parser.add_argument("--approach-speed", type=int, default=APPROACH_SPEED_DEFAULT)
    parser.add_argument("--num-init-scans", type=int, default=3,
                        help="위치추정에 쓸 초기 스캔 바퀴 수 (기본 3, 많을수록 정확하지만 느림)")
    parser.add_argument("--search-range-m", type=float, default=1.5,
                        help="위치추정 탐색 범위(+-미터), 기본 1.5m (트랙 크기에 맞게 조정)")
    parser.add_argument("--xy-step-mm", type=float, default=50.0, help="위치 탐색 간격(mm), 기본 50")
    parser.add_argument("--angle-step-deg", type=float, default=3.0, help="각도 탐색 간격(도), 기본 3")
    parser.add_argument("--score-warn-threshold", type=float, default=25.0,
                        help="이 값보다 매칭 점수가 나쁘면 경고 표시 (px 단위, 기본 25)")
    args = parser.parse_args()

    MAX_DRIVE_STEER_DEG = args.max_drive_steer   # CLI로 조정 가능하게 덮어씀

    if args.motor_test:
        motor_test(args.motor_test_steer, args.motor_test_speed, args.motor_test_sec)
        raise SystemExit(0)

    if args.tour_path is None:
        print("[에러] --tour-path가 필요합니다 (모터 테스트만 하려면 --motor-test를 쓰세요).")
        raise SystemExit(1)

    map_path = args.map
    if map_path is None:
        guessed = guess_map_path(args.tour_path)
        if guessed is None or not os.path.exists(guessed):
            print("[에러] --map을 안 주셨는데 --tour-path 파일명에서 원본 지도를 유추하지 못했습니다.")
            print("       --map logs/원본지도.npy 형태로 직접 지정해주세요.")
            raise SystemExit(1)
        map_path = guessed
        print(f"[지도 자동 유추] --map을 안 주셔서 --tour-path 파일명으로부터 유추: {map_path}")

    map_array = np.load(map_path)
    raw_path = np.load(args.tour_path)
    points_mm = [px_to_mm(r, c) for r, c in raw_path]
    waypoints_mm = simplify_waypoints(points_mm, MIN_WAYPOINT_GAP_MM)
    print(f"[경로 로드] 원본 {len(raw_path)}점 -> 단순화 후 {len(waypoints_mm)}개 웨이포인트")
    print("[안내] 위치추정은 프로그램 실행 화면 안에서 진행됩니다 (로봇을 움직이지 마세요).")

    curses.wrapper(main, args, waypoints_mm, map_array)
    print("종료 완료.")
#!/usr/bin/env python3
"""
lidar_obstacle_avoidance.py

라즈베리파이 + Robot HAT(PiCar-X 계열) + RPLidar(2D 라이다) + 카메라를 이용한
"직진 주행 + 장애물 회피 + 원래 직선 복귀" 스크립트.

동작 방식
---------
1. 터미널에 Q 를 입력하고 Enter -> 주행 시작 (직진)
2. 터미널에 E 를 입력하고 Enter -> 즉시 정지 (다시 Q 입력하면 재시작 가능)
3. Ctrl+C -> 프로그램 완전 종료

주행 로직 (상태 머신)
---------------------
STRAIGHT      : 정면 장애물이 없으면 그대로 직진
AVOID_TURN    : 정면에 위험 거리 이내 장애물 발견 -> 더 여유 있는 방향으로 회전
AVOID_STRAIGHT: 회전한 방향을 유지한 채 잠시 직진하며 장애물 옆을 통과
RETURN_TURN   : 반대 방향으로 같은 만큼 회전해 원래 진행 방향(직선)으로 복귀
STRAIGHT      : 다시 직진 상태로 복귀, 반복

장애물 인식 센서 역할 분담
--------------------------
- 라이다(RPLidar): "얼마나 가까운 물체가 있는가"를 정확한 거리(mm)로 판단.
  회피 여부/타이밍을 결정하는 주 센서. 얇은 물체(의자 다리 등)도 스캔 높이에
  걸리기만 하면 카메라보다 안정적으로 잡아낸다.
- 카메라: 정면에 물체가 실제로 "보이는지" 시각적으로 한 번 더 확인(교차검증)
  하는 보조 센서. 간단한 프레임 차분/명암 대비 기반 감지를 사용한다.
  (필요하다면 이 부분을 YOLO 등 딥러닝 검출기로 교체해 정확도를 높일 수 있음)

설치
----
    pip3 install rplidar-roboticia opencv-python
    pip3 install robot-hat   # PiCar-X 예제 설치 시 보통 포함

하드웨어에 맞게 반드시 확인/수정할 부분
--------------------------------------
    - LIDAR_PORT : RPLidar가 연결된 시리얼 포트 (예: /dev/ttyUSB0)
    - CAMERA_INDEX : 카메라 장치 번호 (보통 0)
    - MotorController 클래스 내부 : 실제 사용 중인 라이브러리(Picarx 등)에 맞게 교체
"""

import sys
import time
import threading

import cv2
import numpy as np
from rplidar import RPLidar, RPLidarException


# ----------------------------------------------------------------------
# 설정값 (하드웨어/환경에 맞게 조정)
# ----------------------------------------------------------------------
LIDAR_PORT = "/dev/ttyUSB0"
CAMERA_INDEX = 0

DANGER_DISTANCE_MM = 300      # 이 거리 이내면 즉시 회피 시작
SAFE_DISTANCE_MM = 600        # 참고용(로그 표시). 회피 트리거는 DANGER 기준.

FORWARD_SPEED = 30            # 직진 속도 (0~100)
TURN_SPEED = 25               # 회피 중 속도
AVOID_STEER_DEG = 28          # 회피 시 조향각(도)

AVOID_TURN_TIME = 0.5         # 회피 방향으로 꺾는 시간(초)
AVOID_STRAIGHT_TIME = 0.8     # 꺾은 채로 옆을 통과하는 시간(초)
RETURN_TURN_TIME = 0.5        # 원래 방향으로 되돌리는 시간(초) - 보통 AVOID_TURN_TIME과 동일

FRONT_SECTOR_DEG = 25         # 라이다 정면 판정 폭(±도)
SIDE_SECTOR_DEG = 60          # 라이다 좌/우 판정 폭(도)

LOOP_HZ = 10                  # 제어 루프 주파수

CAM_DIFF_THRESHOLD = 25       # 카메라 프레임 차분 민감도
CAM_MIN_CONTOUR_AREA = 4000   # 이 크기 이상 윤곽선이 잡히면 "물체 있음"으로 판단


# ----------------------------------------------------------------------
# 모터 제어 (Robot HAT / PiCar-X)
# ----------------------------------------------------------------------
class MotorController:
    """
    실제 사용 중인 Robot HAT 라이브러리에 맞춰 이 클래스만 교체하면
    나머지 상태 머신 로직은 그대로 재사용할 수 있습니다.
    아래는 SunFounder PiCar-X(Picarx 클래스) 기준 예시입니다.
    """

    def __init__(self):
        try:
            from picarx import Picarx
            self.car = Picarx()
            self.available = True
        except ImportError:
            print("[경고] picarx 라이브러리를 찾을 수 없어 시뮬레이션 모드로 동작합니다.")
            self.car = None
            self.available = False

    def drive(self, speed, angle):
        """angle: 0=직진, 음수=좌회전, 양수=우회전"""
        if self.available:
            self.car.set_dir_servo_angle(angle)
            self.car.forward(speed)
        else:
            print(f"[SIM] drive(speed={speed}, angle={angle})")

    def stop(self):
        if self.available:
            self.car.stop()
        else:
            print("[SIM] stop()")


# ----------------------------------------------------------------------
# 라이다 스캐너 (별도 스레드에서 계속 최신 스캔 유지)
# ----------------------------------------------------------------------
class LidarScanner:
    def __init__(self, port):
        self.lidar = RPLidar(port)
        self.latest_scan = {}
        self.lock = threading.Lock()
        self.running = False
        self.thread = None

    def start(self):
        self.running = True
        self.thread = threading.Thread(target=self._scan_loop, daemon=True)
        self.thread.start()

    def _scan_loop(self):
        try:
            for scan in self.lidar.iter_scans():
                if not self.running:
                    break
                with self.lock:
                    self.latest_scan = {
                        int(angle) % 360: distance
                        for (_quality, angle, distance) in scan
                        if distance > 0
                    }
        except RPLidarException as e:
            print(f"[라이다 오류] {e}")

    def get_scan(self):
        with self.lock:
            return dict(self.latest_scan)

    def stop(self):
        self.running = False
        try:
            self.lidar.stop()
            self.lidar.stop_motor()
            self.lidar.disconnect()
        except Exception:
            pass


def min_distance_in_range(scan, center_deg, half_width_deg):
    distances = []
    for offset in range(-half_width_deg, half_width_deg + 1):
        deg = (center_deg + offset) % 360
        if deg in scan:
            distances.append(scan[deg])
    return min(distances) if distances else float("inf")


# ----------------------------------------------------------------------
# 카메라 기반 보조 인식 (프레임 차분으로 "정면에 물체가 보이는가"만 판단)
# ----------------------------------------------------------------------
class CameraObstacleDetector:
    def __init__(self, index=0):
        self.cap = cv2.VideoCapture(index)
        self.prev_gray = None

    def check(self):
        """
        정면 중앙 영역에서 이전 프레임 대비 큰 변화(=물체 등장)가 있으면 True.
        라이다와의 교차검증용 보조 신호이며, 메인 회피 판단은 라이다가 담당한다.
        """
        ok, frame = self.cap.read()
        if not ok:
            return False, None

        h, w = frame.shape[:2]
        roi = frame[int(h * 0.4):h, int(w * 0.3):int(w * 0.7)]  # 정면 중앙 하단부
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (5, 5), 0)

        detected = False
        if self.prev_gray is not None:
            diff = cv2.absdiff(self.prev_gray, gray)
            _, thresh = cv2.threshold(diff, CAM_DIFF_THRESHOLD, 255, cv2.THRESH_BINARY)
            contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for c in contours:
                if cv2.contourArea(c) > CAM_MIN_CONTOUR_AREA:
                    detected = True
                    break

        self.prev_gray = gray
        return detected, frame

    def release(self):
        self.cap.release()


# ----------------------------------------------------------------------
# 키보드 입력 스레드 (Q: 시작, E: 정지)
# ----------------------------------------------------------------------
class KeyboardController:
    def __init__(self):
        self.should_run = False   # True = 주행 중, False = 정지
        self.exit_flag = False
        self.thread = threading.Thread(target=self._listen, daemon=True)

    def start(self):
        self.thread.start()

    def _listen(self):
        print("Q + Enter = 시작 / E + Enter = 정지 / Ctrl+C = 종료")
        while not self.exit_flag:
            try:
                key = input().strip().lower()
            except EOFError:
                break
            if key == "q":
                self.should_run = True
                print(">> 주행 시작")
            elif key == "e":
                self.should_run = False
                print(">> 정지")


# ----------------------------------------------------------------------
# 회피 상태 머신
# ----------------------------------------------------------------------
STATE_STRAIGHT = "STRAIGHT"
STATE_AVOID_TURN = "AVOID_TURN"
STATE_AVOID_STRAIGHT = "AVOID_STRAIGHT"
STATE_RETURN_TURN = "RETURN_TURN"


class AvoidanceStateMachine:
    def __init__(self):
        self.state = STATE_STRAIGHT
        self.state_entered_at = time.time()
        self.avoid_direction = 0  # -1: 좌회피, +1: 우회피

    def _enter(self, new_state):
        self.state = new_state
        self.state_entered_at = time.time()

    def elapsed(self):
        return time.time() - self.state_entered_at

    def update(self, scan, cam_detected):
        front = min_distance_in_range(scan, 0, FRONT_SECTOR_DEG)
        left = min_distance_in_range(scan, 90, SIDE_SECTOR_DEG // 2)
        right = min_distance_in_range(scan, 270, SIDE_SECTOR_DEG // 2)

        obstacle_ahead = front < DANGER_DISTANCE_MM or (cam_detected and front < SAFE_DISTANCE_MM)

        if self.state == STATE_STRAIGHT:
            if obstacle_ahead:
                self.avoid_direction = -1 if left > right else 1
                self._enter(STATE_AVOID_TURN)
            return self._drive_command(front)

        if self.state == STATE_AVOID_TURN:
            if self.elapsed() >= AVOID_TURN_TIME:
                self._enter(STATE_AVOID_STRAIGHT)
            return self._drive_command(front)

        if self.state == STATE_AVOID_STRAIGHT:
            # 옆을 지나가는 도중에도 새 장애물이 잡히면 다시 회피 판단
            if obstacle_ahead:
                self.avoid_direction = -1 if left > right else 1
                self._enter(STATE_AVOID_TURN)
            elif self.elapsed() >= AVOID_STRAIGHT_TIME:
                self._enter(STATE_RETURN_TURN)
            return self._drive_command(front)

        if self.state == STATE_RETURN_TURN:
            if self.elapsed() >= RETURN_TURN_TIME:
                self._enter(STATE_STRAIGHT)
            return self._drive_command(front)

        # 안전장치: 알 수 없는 상태면 직진 상태로 리셋
        self._enter(STATE_STRAIGHT)
        return self._drive_command(front)

    def _drive_command(self, front):
        if self.state == STATE_STRAIGHT:
            return FORWARD_SPEED, 0, front
        if self.state == STATE_AVOID_TURN:
            return TURN_SPEED, AVOID_STEER_DEG * self.avoid_direction, front
        if self.state == STATE_AVOID_STRAIGHT:
            return TURN_SPEED, AVOID_STEER_DEG * self.avoid_direction, front
        if self.state == STATE_RETURN_TURN:
            return TURN_SPEED, -AVOID_STEER_DEG * self.avoid_direction, front
        return 0, 0, front


# ----------------------------------------------------------------------
# 메인 루프
# ----------------------------------------------------------------------
def main():
    print("라이다/카메라 초기화 중...")
    scanner = LidarScanner(LIDAR_PORT)
    scanner.start()

    camera = CameraObstacleDetector(CAMERA_INDEX)
    motor = MotorController()
    keyboard = KeyboardController()
    keyboard.start()

    fsm = AvoidanceStateMachine()

    time.sleep(2)  # 라이다/카메라 워밍업 대기

    try:
        while True:
            if not keyboard.should_run:
                motor.stop()
                fsm.state = STATE_STRAIGHT  # 정지 중엔 상태를 리셋해 재시작 시 직진부터
                time.sleep(1.0 / LOOP_HZ)
                continue

            scan = scanner.get_scan()
            cam_detected, _frame = camera.check()

            if not scan:
                motor.stop()
                time.sleep(1.0 / LOOP_HZ)
                continue

            speed, angle, front_dist = fsm.update(scan, cam_detected)
            motor.drive(speed, angle)

            print(f"[상태={fsm.state}] 정면거리={front_dist:.0f}mm "
                  f"카메라감지={cam_detected} 속도={speed} 조향={angle}")

            time.sleep(1.0 / LOOP_HZ)

    except KeyboardInterrupt:
        print("\n종료 신호 수신, 정지합니다.")

    finally:
        keyboard.exit_flag = True
        motor.stop()
        scanner.stop()
        camera.release()
        print("안전하게 종료되었습니다.")


if __name__ == "__main__":
    sys.exit(main())

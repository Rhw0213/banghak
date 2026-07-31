"""
robot_pick.py
역할: 카메라 인식(실시간 스트리밍 포함) + 차량 이동/정지 + 로봇팔 제어 통합

흐름:
1. 차량이 전진하며 카메라로 파란색(가벼운) 물체를 찾음
2. 물체가 감지되고, 추정 거리가 STOP_DISTANCE_CM 이하로 가까워지면 차량 정지
3. 팔이 내려가 그리퍼로 물체를 집음
4. 집은 채로 팔을 원위치(빠꾸)로 이동

실시간 스트리밍:
- 브라우저에서 http://<라즈베리파이_IP>:8000 접속 시 실시간 영상 확인 가능
- 필요 패키지: pip install flask picamera2 (picamera2는 라즈베리파이 OS에 보통 기본 설치됨)

카메라 관련 중요 참고:
- cv2.VideoCapture(0)는 CSI 카메라 + libcamera 스택 환경에서 프레임을 못 읽는 경우가 많음
  (V4L2 레거시 방식과 libcamera 방식이 안 맞아서 발생하는 문제)
- 그래서 Picamera2로 프레임을 받아온 뒤 OpenCV용으로 BGR 변환해서 사용
"""

import os
import threading
import time

import cv2
import numpy as np
from flask import Flask, Response
from picamera2 import Picamera2

from picarx import Picarx
from robot_hat import Servo, device  # reset_mcu() deprecated -> device.reset_mcu() 사용


# ============================================================
# 1. CAMERA - 물체의 색 인식 (파란색)
# ============================================================

"""
빨간색은 H값 기준으로 0 근처와 179 근처 양쪽 끝에 걸쳐 있기에 코드가 깁니다.

RED_LOWER1 = np.array([0, 100, 100])
RED_UPPER1 = np.array([10, 255, 255])

RED_LOWER2 = np.array([170, 100, 100])
RED_UPPER2 = np.array([179, 255, 255])

def detect_red_object(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask1 = cv2.inRange(hsv, RED_LOWER1, RED_UPPER1)
    mask2 = cv2.inRange(hsv, RED_LOWER2, RED_UPPER2)
    mask = cv2.bitwise_or(mask1, mask2)  # 두 마스크를 합침
    # 이후 로직은 detect_yellow_object와 동일 (contour 찾기 등)
"""

BLUE_LOWER = np.array([90, 100, 100])
BLUE_UPPER = np.array([130, 255, 255])

"""
다른 HSV 범위 (실제 조명/물체에 맞춰 반드시 튜닝 필요) 아래는 예시
YELLOW_LOWER = np.array([20, 100, 100])
YELLOW_UPPER = np.array([35, 255, 255])

주황    10~20
노랑    20~35
초록    35~85
파랑    90~130
보라    130~160
이는 HSV 중 H에 해당되는 사항이다
"""

MIN_AREA = 500  # 이보다 작은 영역은 노이즈로 간주하고 무시


def detect_blue_object(frame):
    """
    입력: BGR 프레임
    출력: (detected, cx, cy, area)
    """
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, BLUE_LOWER, BLUE_UPPER)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if not contours:
        return False, None, None, 0

    largest = max(contours, key=cv2.contourArea)
    area = cv2.contourArea(largest)

    if area < MIN_AREA:
        return False, None, None, 0

    m = cv2.moments(largest)
    cx = int(m["m10"] / m["m00"])
    cy = int(m["m01"] / m["m00"])

    return True, cx, cy, area


def estimate_distance(area):
    """
    화면상 물체 면적(area)을 거리(cm)로 근사 변환.
    TODO: 실측 캘리브레이션 필요 (10cm/20cm/30cm에서의 area 값을 찍어보고 공식 재조정)
    """
    if area <= 0:
        return None
    return 5000 / (area ** 0.5)  # 임시 공식 - 튜닝 필요


# ============================================================
# 2. MOVEMENT - 차량 전진/정지
# ============================================================

px = Picarx()

MOVE_SPEED = 8       # 0~100, 목표 속도 (15도 빠르다고 하셔서 더 낮춤. 그래도 빠르면 5~6까지도 가능)
RAMP_STEP = 2         # 슬로우 스타트 시 한 번에 올릴 속도 단위
RAMP_INTERVAL = 0.15  # 슬로우 스타트 단계 사이 대기 시간(초)

# 연속 속도(MOVE_SPEED)를 최대한 낮춰도 여전히 빠르다면, 모터 자체의 최소 구동
# 임계값(데드존) 때문일 수 있습니다. 이 경우 아래 펄스 구동 방식으로 바꿔보세요:
# 짧게 움직였다가 짧게 멈추기를 반복해서 평균 속도를 더 낮추는 방식입니다.
USE_PULSE_DRIVE = False   # True로 바꾸면 펄스 구동 방식 사용
PULSE_ON_TIME = 0.15      # 한 번에 움직이는 시간(초)
PULSE_OFF_TIME = 0.25     # 그 사이 멈추는 시간(초)
PULSE_SPEED = 15          # 펄스로 움직일 때의 순간 속도 (짧게만 움직이므로 조금 더 높아도 됨)

_current_speed = 0
_is_moving = False


def move_forward(target_speed=MOVE_SPEED):
    """
    정지 상태에서 출발할 때는 0부터 target_speed까지 서서히 가속(슬로우 스타트).
    이미 움직이는 중이면 바로 목표 속도로 유지.
    USE_PULSE_DRIVE가 True면 대신 짧게 움직였다 멈췄다를 반복(펄스 구동).
    """
    global _current_speed, _is_moving

    px.set_dir_servo_angle(0)

    if USE_PULSE_DRIVE:
        px.forward(PULSE_SPEED)
        time.sleep(PULSE_ON_TIME)
        px.stop()
        time.sleep(PULSE_OFF_TIME)
        return

    if not _is_moving:
        _current_speed = 0
        while _current_speed < target_speed:
            _current_speed = min(_current_speed + RAMP_STEP, target_speed)
            px.forward(_current_speed)
            time.sleep(RAMP_INTERVAL)
        _is_moving = True
    else:
        px.forward(target_speed)


def stop_car():
    global _current_speed, _is_moving
    px.stop()
    _current_speed = 0
    _is_moving = False  # 다음에 다시 출발할 때 슬로우 스타트 재적용


# ============================================================
# 3. ARM - 로봇팔 4개 서보 제어
# ============================================================

device.reset_mcu()  # reset_mcu() 대체
time.sleep(0.2)

base = Servo("P3")
shoulder = Servo("P5")
elbow = Servo("P6")
gripper = Servo("P7")

ANGLE_READY = {"base": 0, "shoulder": 30, "elbow": 30}
ANGLE_RETREAT = {"base": 0, "shoulder": -20, "elbow": -20}

GRIPPER_OPEN = 0
GRIPPER_CLOSE = 60


def set_arm_angles(base_angle=0, shoulder_angle=0, elbow_angle=0, gripper_angle=0):
    base.angle(base_angle)
    shoulder.angle(shoulder_angle)
    elbow.angle(elbow_angle)
    gripper.angle(gripper_angle)


def align_arm_to_distance(distance):
    print(f"[1단계] 거리 {distance:.1f}cm에 맞춰 팔 위치 조정 중...")
    set_arm_angles(
        base_angle=ANGLE_READY["base"],
        shoulder_angle=ANGLE_READY["shoulder"],
        elbow_angle=ANGLE_READY["elbow"],
        gripper_angle=GRIPPER_OPEN,
    )
    time.sleep(1)


def grab_by_gripper():
    print("[2단계] 그리퍼로 물체 집는 중...")
    gripper.angle(GRIPPER_CLOSE)
    time.sleep(1)


def retreat_after_grab():
    print("[3단계] 집은 채로 팔 빠꾸 중...")
    set_arm_angles(
        base_angle=ANGLE_RETREAT["base"],
        shoulder_angle=ANGLE_RETREAT["shoulder"],
        elbow_angle=ANGLE_RETREAT["elbow"],
        gripper_angle=GRIPPER_CLOSE,
    )
    time.sleep(1)


def pick_sequence(distance):
    align_arm_to_distance(distance)
    grab_by_gripper()
    retreat_after_grab()


# ============================================================
# 4. 실시간 스트리밍 + 로봇 제어 루프
# ============================================================

STOP_DISTANCE_CM = 10  # 이 거리 이하로 가까워지면 정지

latest_frame = None       # 스트리밍용 최신 프레임 (감지 표시 포함)
frame_lock = threading.Lock()
picked = False


def camera_and_control_loop():
    """
    카메라 프레임을 계속 받아오면서:
    - 화면에 감지 표시를 그려서 latest_frame에 저장 (스트리밍용)
    - 동시에 로봇 제어(전진/정지/집기) 로직 실행
    """
    global latest_frame, picked

    picam2 = Picamera2()
    config = picam2.create_video_configuration(main={"format": "RGB888", "size": (640, 480)})
    picam2.configure(config)
    picam2.start()
    time.sleep(1)  # 카메라 워밍업 대기

    last_log_time = 0

    while True:
        # Picamera2의 "RGB888" 포맷은 이름과 달리 실제로는 이미 [B, G, R] 순서로 나옴
        # (OpenCV가 원하는 BGR과 동일) -> 별도 변환 불필요. cv2.cvtColor로 다시 변환하면
        # R/B 채널이 뒤바뀌어 색상 인식이 완전히 틀어지므로 절대 변환하지 말 것.
        frame = picam2.capture_array()

        detected, cx, cy, area = detect_blue_object(frame)
        distance = None

        # 0.5초마다 감지 상태 로그 출력 (디버깅용 - 실제로 파란색이 잡히는지 확인)
        now = time.time()
        if now - last_log_time > 0.5:
            if detected:
                print(f"[감지됨] area={area:.0f}, 추정거리={estimate_distance(area):.1f}cm")
            else:
                print("[감지 안 됨]")
            last_log_time = now

        if detected:
            distance = estimate_distance(area)
            cv2.circle(frame, (cx, cy), 8, (255, 0, 0), -1)
            cv2.putText(frame, f"dist~{distance:.1f}cm", (cx + 10, cy),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)

        with frame_lock:
            latest_frame = frame.copy()

        if not picked:
            if detected and distance is not None and distance <= STOP_DISTANCE_CM:
                print(f"충분히 가까움(추정 거리 {distance:.1f}cm) -> 정지 후 팔 동작 시작")
                stop_car()
                pick_sequence(distance)
                picked = True
                print("집기 완료!")
            else:
                move_forward()
        else:
            stop_car()

        time.sleep(0.03)


# ============================================================
# 5. FLASK - 실시간 스트리밍 서버
# ============================================================

app = Flask(__name__)


def generate_mjpeg():
    while True:
        with frame_lock:
            if latest_frame is None:
                continue
            ok, buffer = cv2.imencode(".jpg", latest_frame)
        if not ok:
            continue
        frame_bytes = buffer.tobytes()
        yield (b"--frame\r\n"
               b"Content-Type: image/jpeg\r\n\r\n" + frame_bytes + b"\r\n")


@app.route("/video_feed")
def video_feed():
    return Response(generate_mjpeg(), mimetype="multipart/x-mixed-replace; boundary=frame")


@app.route("/")
def index():
    return "<html><body><h3>Robot Camera</h3><img src='/video_feed'></body></html>"


@app.route("/status")
def status():
    # 일부 카메라 뷰어 앱이 접속 전 상태 확인용으로 이 경로를 두드리는 경우가 있어 추가
    return {"status": "ok", "picked": picked}


# ============================================================
# 6. 종료 처리 - 터미널에서 'q' + Enter 입력 시 안전 종료
# ============================================================

def keyboard_listener():
    """
    터미널에서 'q'를 입력하고 Enter를 누르면 차량을 정지시키고 프로세스를 종료.
    (Ctrl+C가 스레드/Flask 서버 조합에서 바로 안 먹히는 문제를 대체하기 위함)
    """
    print("종료하려면 'q'를 입력하고 Enter를 누르세요.")
    while True:
        try:
            cmd = input()
        except EOFError:
            break
        if cmd.strip().lower() == "q":
            print("종료 신호 감지 -> 차량 정지 후 프로그램을 종료합니다.")
            stop_car()
            os._exit(0)


if __name__ == "__main__":
    cam_thread = threading.Thread(target=camera_and_control_loop, daemon=True)
    cam_thread.start()

    key_thread = threading.Thread(target=keyboard_listener, daemon=True)
    key_thread.start()

    print("브라우저에서 http://<라즈베리파이_IP>:8000 접속하면 실시간 영상 확인 가능")
    app.run(host="0.0.0.0", port=8000, threaded=True)
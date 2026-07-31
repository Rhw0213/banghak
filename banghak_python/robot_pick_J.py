"""
robot_pick_J.py
역할: 카메라 인식(바운딩 박스 + 실시간 스트리밍) + 차량 이동/정지(부드러운 연속 가속) + 로봇팔 제어

흐름:
1. 차량이 부드럽게 서서히 가속하며 전진, 카메라로 파란색 물체를 찾음
2. 파란색이 감지되는 순간 -> 거리를 4cm로 간주하고 즉시 정지
3. 팔이 내려가 그리퍼로 물체를 집음
4. 집은 채로 팔을 원위치(빠꾸)로 이동 + 차량 자체도 실제로 후진

실행 전 반드시 확인:
    ps aux | grep python3        # 이전에 안 죽은 프로세스 있는지 확인
    pkill -9 -f robot_pick       # 있으면 강제 종료
    sudo lsof /dev/video0        # 카메라 점유 프로세스 없는지 확인

실시간 스트리밍: http://192.168.0.82:8000
종료 방법: 터미널에서 'q' + Enter, 또는 Ctrl+C (둘 다 모터를 반드시 정지시킴)
"""

import os
import signal
import threading
import time
import traceback

import cv2
import numpy as np
from flask import Flask, Response
from picamera2 import Picamera2

from picarx import Picarx
from robot_hat import Servo, device  # reset_mcu() deprecated -> device.reset_mcu() 사용


# ============================================================
# 0. 공유 상태 (카메라 스레드 <-> 제어 스레드 <-> Flask가 함께 사용)
# ============================================================

state_lock = threading.Lock()
shared = {
    "frame": None,       # 스트리밍용 최신 프레임 (바운딩 박스 그려짐)
    "detected": False,   # 파란 물체 감지 여부
    "area": 0,           # 감지된 영역 픽셀 면적
    "picked": False,     # 집기 완료 여부
}


# ============================================================
# 1. CAMERA - 파란색 물체 인식 (바운딩 박스로 물체 전체를 감쌈)
# ============================================================

"""
빨간색은 H값 기준으로 0 근처와 179 근처 양쪽 끝에 걸쳐 있기에 코드가 깁니다.

RED_LOWER1 = np.array([0, 100, 100])
RED_UPPER1 = np.array([10, 255, 255])
RED_LOWER2 = np.array([170, 100, 100])
RED_UPPER2 = np.array([179, 255, 255])
"""

BLUE_LOWER = np.array([90, 100, 100])
BLUE_UPPER = np.array([130, 255, 255])

MIN_AREA = 300  # 이보다 작은 영역은 노이즈로 간주 (기존 500 -> 낮춤, 필요시 조정)


def detect_blue_object(frame):
    """
    입력: BGR 프레임 (Picamera2의 'RGB888' 포맷은 실제로 이미 BGR 순서라 별도 변환 불필요)
    출력: (detected, x, y, w, h, area)
        x, y, w, h: 물체 전체를 감싸는 바운딩 박스 좌표 (YOLO 박스처럼 표시하기 위함)
    """
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, BLUE_LOWER, BLUE_UPPER)

    # 작은 노이즈 점들 제거 (침식 후 팽창)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if not contours:
        return False, 0, 0, 0, 0, 0

    largest = max(contours, key=cv2.contourArea)
    area = cv2.contourArea(largest)

    if area < MIN_AREA:
        return False, 0, 0, 0, 0, 0

    x, y, w, h = cv2.boundingRect(largest)  # 물체 전체를 감싸는 사각형
    return True, x, y, w, h, area


# ============================================================
# 2. MOVEMENT - 부드러운 연속 가속/감속 방식
#
# 이전에 "속도값을 낮춰도 안 바뀐다"고 느끼신 건, 알고보니 이전 실행에서
# 안 죽은 좀비 프로세스가 원인이었습니다(해결됨). 그래서 다시 연속 방식으로
# 돌아오되, 매 루프마다 속도를 아주 조금씩만 올려서 부드럽게 가속하도록
# 만들었습니다. (펄스처럼 끊기지 않음)
# ============================================================

px = Picarx()

MOVE_SPEED = 10        # 목표(최고) 속도, 0~100. 그래도 빠르면 더 낮추세요.
RAMP_STEP = 1           # 매 루프마다 올릴 속도 단위 (작을수록 더 부드러움)
RAMP_INTERVAL = 0.05    # 루프 주기(초) - 아래 control_loop에서 이 간격으로 호출됨

BACKWARD_SPEED = 15     # 집은 후 후진할 때 속도
BACKWARD_DURATION = 1.5  # 후진 지속 시간(초)

_current_speed = 0


def move_forward_smooth(target_speed=MOVE_SPEED):
    """호출될 때마다 RAMP_STEP만큼만 속도를 올려서 부드럽게 가속. 매 루프 반복 호출 전제."""
    global _current_speed
    px.set_dir_servo_angle(0)
    if _current_speed < target_speed:
        _current_speed = min(_current_speed + RAMP_STEP, target_speed)
    px.forward(_current_speed)


def stop_car():
    global _current_speed
    px.stop()
    _current_speed = 0  # 다음에 다시 출발할 때 처음부터 서서히 가속


def move_backward_after_pick():
    """물건을 집은 후 차량을 실제로 후진시킴 (기존엔 팔만 원위치로 갔음)"""
    print(f"[후진] 집은 물건을 유지한 채 {BACKWARD_DURATION}초간 후진합니다...")
    px.set_dir_servo_angle(0)
    px.backward(BACKWARD_SPEED)
    time.sleep(BACKWARD_DURATION)
    px.stop()
    print("[후진] 완료.")


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

GRIPPER_OPEN = 30   # 캘리브레이션 결과: 닫히는 이동폭이 좁으면(예: 10->0) 악력이 약해서 넓게 열어둠
GRIPPER_CLOSE = 0   # 캘리브레이션 결과: 0도가 실제로 꽉 잡는 각도


def set_arm_angles(base_angle=0, shoulder_angle=0, elbow_angle=0, gripper_angle=0):
    base.angle(base_angle)
    shoulder.angle(shoulder_angle)
    elbow.angle(elbow_angle)
    gripper.angle(gripper_angle)


def align_arm_to_distance(distance):
    print(f"[1단계] 거리 {distance}cm(고정값)에 맞춰 팔 위치 조정 중...")
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
    time.sleep(1.5)  # 서보가 완전히 닫힐 때까지 충분히 대기 (기존 1초 -> 1.5초)


def retreat_after_grab():
    print("[3단계] 집은 채로 팔 빠꾸 중...")
    set_arm_angles(
        base_angle=ANGLE_RETREAT["base"],
        shoulder_angle=ANGLE_RETREAT["shoulder"],
        elbow_angle=ANGLE_RETREAT["elbow"],
        gripper_angle=GRIPPER_CLOSE,
    )
    time.sleep(1)


def pick_sequence(distance=4):
    align_arm_to_distance(distance)
    grab_by_gripper()
    retreat_after_grab()


# ============================================================
# 4. 카메라 캡처 + 인식 스레드 (움직임과 분리 -> 감지 반응 속도 향상)
# ============================================================

def capture_and_detect_loop():
    print("[카메라] 초기화 시작...")
    try:
        picam2 = Picamera2()
        config = picam2.create_video_configuration(main={"format": "RGB888", "size": (640, 480)})
        picam2.configure(config)
        picam2.start()
        time.sleep(1)  # 워밍업 대기
        print("[카메라] 초기화 완료. 인식 시작.")
    except Exception:
        print("[카메라] 초기화 실패! 아래 에러 내용을 확인하세요:")
        traceback.print_exc()
        return

    last_log = 0

    while True:
        try:
            frame = picam2.capture_array()  # 이미 BGR 순서 (추가 변환 금지)

            detected, x, y, w, h, area = detect_blue_object(frame)

            display = frame.copy()
            if detected:
                cv2.rectangle(display, (x, y), (x + w, y + h), (255, 0, 0), 2)
                cv2.putText(display, f"BLUE area={int(area)}", (x, max(y - 10, 0)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)

            with state_lock:
                shared["frame"] = display
                shared["detected"] = detected
                shared["area"] = area

            now = time.time()
            if now - last_log > 0.5:
                print(f"[감지됨] area={area:.0f}" if detected else "[감지 안 됨]")
                last_log = now

        except Exception:
            print("[카메라] 프레임 처리 중 오류 발생:")
            traceback.print_exc()

        time.sleep(0.03)


# ============================================================
# 5. 로봇 제어 스레드 (전진/정지/집기 판단)
# ============================================================

def control_loop():
    while True:
        with state_lock:
            detected = shared["detected"]
            picked = shared["picked"]

        if picked:
            stop_car()
            time.sleep(0.1)
            continue

        if detected:
            print("파란 물체 감지! -> 거리 4cm로 간주하고 정지 + 집기 시작")
            stop_car()
            pick_sequence(distance=4)
            move_backward_after_pick()  # 집은 후 실제로 차량 후진
            with state_lock:
                shared["picked"] = True
            print("집기 + 후진 완료! 이제 대기 상태로 전환합니다.")
        else:
            move_forward_smooth()
            time.sleep(RAMP_INTERVAL)  # 부드러운 가속을 위한 주기 유지


# ============================================================
# 6. FLASK - 실시간 스트리밍 서버
# ============================================================

app = Flask(__name__)


def generate_mjpeg():
    while True:
        with state_lock:
            frame = shared["frame"]
        if frame is None:
            time.sleep(0.05)
            continue
        ok, buffer = cv2.imencode(".jpg", frame)
        if not ok:
            continue
        yield (b"--frame\r\n"
               b"Content-Type: image/jpeg\r\n\r\n" + buffer.tobytes() + b"\r\n")


@app.route("/video_feed")
def video_feed():
    return Response(generate_mjpeg(), mimetype="multipart/x-mixed-replace; boundary=frame")


@app.route("/")
def index():
    return "<html><body><h3>Robot Camera - 192.168.0.82:8000</h3><img src='/video_feed'></body></html>"


@app.route("/status")
def status():
    with state_lock:
        return {"status": "ok", "detected": shared["detected"], "picked": shared["picked"]}


# ============================================================
# 7. 종료 처리 - Ctrl+C / kill / 'q' 입력 어떤 경우든 반드시 모터 정지
# ============================================================

def shutdown(*_args):
    print("\n종료 신호 수신 -> 모터 정지 후 프로그램을 종료합니다.")
    try:
        stop_car()
    except Exception:
        pass
    os._exit(0)  # 스레드가 여러 개라 sys.exit()로는 안 죽을 수 있어 강제 종료 사용


signal.signal(signal.SIGINT, shutdown)   # Ctrl+C
signal.signal(signal.SIGTERM, shutdown)  # kill 명령


def keyboard_listener():
    print("종료하려면 'q' + Enter 를 입력하세요. (Ctrl+C도 동일하게 동작)")
    while True:
        try:
            cmd = input()
        except EOFError:
            break
        if cmd.strip().lower() == "q":
            shutdown()


if __name__ == "__main__":
    cam_thread = threading.Thread(target=capture_and_detect_loop, daemon=True)
    cam_thread.start()

    ctrl_thread = threading.Thread(target=control_loop, daemon=True)
    ctrl_thread.start()

    key_thread = threading.Thread(target=keyboard_listener, daemon=True)
    key_thread.start()

    print("브라우저에서 http://192.168.0.82:8000 접속하면 실시간 영상 확인 가능")
    app.run(host="0.0.0.0", port=8000, threaded=True)
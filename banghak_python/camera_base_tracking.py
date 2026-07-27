"""
로봇팔 base + 카메라 head(pan/tilt)가 동시에 노란 물체를 따라감.

- 카메라 프레임 캡처 & 노란 물체 인식은 한 번만 수행
- 좌우(cx 오차)   -> 로봇팔 base 회전 (SmoothJoint.move_to, 키보드 코드와 동일 방식)
                  -> 카메라 pan도 같이 좌우로 (Picarx.set_cam_pan_angle)
- 상하(cy 오차)   -> 카메라 tilt로 상하 (Picarx.set_cam_tilt_angle)
  (로봇팔 shoulder/elbow는 이번엔 안 건드림 - 필요하면 나중에 추가 가능)

화면은 Flask MJPEG 스트리밍으로 브라우저에서 확인.

[필요 라이브러리]
    pip install flask opencv-python-headless numpy --break-system-packages
    (picarx, picamera2, robot_hat 은 PiCar-X/로봇팔 키트에 기본 포함)

[실행 후 접속]
    http://<라즈베리파이 IP>:8080/
"""

import time
import threading

import cv2
import numpy as np
from flask import Flask, Response
from picamera2 import Picamera2
from picarx import Picarx

from robot_hat import Servo, reset_mcu
from arm_setup import build_arm

# ------------------ 사용자 설정값 ------------------
FRAME_SIZE = (640, 480)

STREAM_HOST = "0.0.0.0"
STREAM_PORT = 8080

# 노란색 HSV 범위 (조명에 따라 조정 필요)
YELLOW_LOWER = np.array([20, 100, 100])
YELLOW_UPPER = np.array([35, 255, 255])
MIN_CONTOUR_AREA = 500

DEADBAND_PX = 15

# 로봇팔 base
BASE_STEP = 2            # move_to() 한 번에 움직일 각도
BASE_SPEED = 30           # move_to() 속도(도/초)

# 카메라 head (Picarx)
PAN_STEP = 1.5
TILT_STEP = 1.5
PAN_MIN, PAN_MAX = -35, 35
TILT_MIN, TILT_MAX = -20, 30

LOOP_DELAY = 0.03
JPEG_QUALITY = 80
# ---------------------------------------------------


app = Flask(__name__)
latest_jpeg = None
frame_lock = threading.Lock()


def tracking_loop():
    global latest_jpeg

    # ---- 로봇팔 초기화 ----
    print("MCU 리셋 중...")
    reset_mcu()
    time.sleep(0.5)
    base, shoulder, elbow, gripper = build_arm(servo_factory=Servo)
    base.move_to(0, speed=BASE_SPEED)
    shoulder.move_to(0, speed=BASE_SPEED)
    elbow.move_to(0, speed=BASE_SPEED)
    gripper.move_to(0, speed=BASE_SPEED)

    # ---- 카메라 head(PiCar-X) 초기화 ----
    px = Picarx()
    current_pan = 0.0
    current_tilt = 0.0
    px.set_cam_pan_angle(current_pan)
    px.set_cam_tilt_angle(current_tilt)

    # ---- 카메라 캡처 초기화 ----
    picam2 = Picamera2()
    picam2.configure(picam2.create_preview_configuration(main={"size": FRAME_SIZE}))
    picam2.start()

    frame_center_x = FRAME_SIZE[0] // 2
    frame_center_y = FRAME_SIZE[1] // 2

    print("로봇팔 + 카메라 head 동시 추적을 시작합니다.")

    while True:
        frame = picam2.capture_array()
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, YELLOW_LOWER, YELLOW_UPPER)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        if contours:
            biggest = max(contours, key=cv2.contourArea)
            if cv2.contourArea(biggest) > MIN_CONTOUR_AREA:
                x, y, w, h = cv2.boundingRect(biggest)
                cx, cy = x + w // 2, y + h // 2

                error_x = cx - frame_center_x
                error_y = cy - frame_center_y

                # ---- 좌우: 로봇팔 base + 카메라 pan 동시에 ----
                if abs(error_x) > DEADBAND_PX:
                    direction_x = 1 if error_x > 0 else -1

                    # 로봇팔 base (실제 반대로 돌면 이 부호만 뒤집기)
                    base.move_to(base.angle - BASE_STEP * direction_x, speed=BASE_SPEED)

                    # 카메라 pan (실제 반대로 돌면 이 부호만 뒤집기)
                    current_pan += PAN_STEP * direction_x
                    current_pan = max(PAN_MIN, min(PAN_MAX, current_pan))
                    px.set_cam_pan_angle(current_pan)

                # ---- 상하: 카메라 tilt만 ----
                if abs(error_y) > DEADBAND_PX:
                    direction_y = 1 if error_y > 0 else -1
                    current_tilt -= TILT_STEP * direction_y
                    current_tilt = max(TILT_MIN, min(TILT_MAX, current_tilt))
                    px.set_cam_tilt_angle(current_tilt)

                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.circle(frame, (cx, cy), 5, (255, 0, 0), -1)
                cv2.putText(
                    frame,
                    f"base={base.angle:.1f} pan={current_pan:.1f} tilt={current_tilt:.1f}",
                    (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (0, 255, 0),
                    2,
                )

        ok, buffer = cv2.imencode(
            ".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY]
        )
        if ok:
            with frame_lock:
                latest_jpeg = buffer.tobytes()

        time.sleep(LOOP_DELAY)


def mjpeg_generator():
    while True:
        with frame_lock:
            jpeg = latest_jpeg
        if jpeg is not None:
            yield (
                b"--frame\r\n"
                b"Content-Type: image/jpeg\r\n\r\n" + jpeg + b"\r\n"
            )
        time.sleep(0.03)


@app.route("/")
def index():
    return (
        "<html><body style='margin:0;background:#111;'>"
        "<img src='/video' style='width:100%;'>"
        "</body></html>"
    )


@app.route("/video")
def video():
    return Response(
        mjpeg_generator(),
        mimetype="multipart/x-mixed-replace; boundary=frame",
    )


if __name__ == "__main__":
    t = threading.Thread(target=tracking_loop, daemon=True)
    t.start()

    print(f"스트리밍 서버 시작: http://<라즈베리파이 IP>:{STREAM_PORT}/")
    app.run(host=STREAM_HOST, port=STREAM_PORT, threaded=True)

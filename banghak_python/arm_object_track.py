"""
arm_track.py
목적: 카메라로 노란색 물체를 인식하고, 팔 베이스(P4)가 물체 방향을 따라가게 함
      + 노트북 브라우저로 인식 결과 실시간 확인

동작 방식:
    1. 카메라 프레임에서 노란색 영역만 골라냄 (HSV 색 범위)
    2. 가장 큰 노란 덩어리의 화면상 가로 중심(cx)을 찾음
    3. cx가 화면 중앙에서 벗어난 정도(error)에 비례해 베이스 각도를 조금씩 이동
       -> 물체가 오른쪽이면 팔도 오른쪽, 물체가 움직이면 팔도 따라감 (실시간 추적)
    4. 인식 결과를 그려 넣은 영상을 웹으로 송출

실행:
    sudo python3 arm_track.py
    -> 노트북 브라우저에서 http://<라즈베리파이_IP>:8000 접속
종료:
    Ctrl+C
"""
from picamera2 import Picamera2
from robot_hat import Servo, reset_mcu
from flask import Flask, Response
import cv2
import numpy as np
import threading
import time
import logging

# ==================== 튜닝 파라미터 ====================
WIDTH, HEIGHT = 320, 240

# --- 노란색 HSV 범위 (조명에 따라 조정) ---
# H(색상): 노랑은 대략 20~35. S(채도)/V(명도)는 낮으면 흐린 색까지 잡힘
HSV_LOW  = np.array([20, 80, 80])
HSV_HIGH = np.array([35, 255, 255])
MIN_AREA = 500             # 이보다 작은 노란 덩어리는 노이즈로 무시

# --- 팔 베이스 제어 ---
BASE_PORT = "P4"           # 베이스(좌우 회전) 관절
BASE_MIN = -90             # 베이스 각도 하한 (물리적으로 갈 수 있는 범위로 제한)
BASE_MAX = 90              # 베이스 각도 상한
BASE_INIT = 0             # 시작 각도 (정면)

KP = 0.04                  # 추적 게인: error(px) 1당 몇 도 보정할지. 크면 민감/떨림
DEADZONE = 15              # 화면 중앙 ±이 픽셀 안에 있으면 "가운데"로 보고 안 움직임(떨림 방지)
MAX_STEP = 2.0             # 한 프레임에 베이스가 움직일 수 있는 최대 각도(급격한 점프 방지)

# --- 카메라가 팔 회전 방향과 반대로 움직이면 여기를 -1로 ---
DIRECTION = -1

LOST_HOLD = True           # 물체를 놓치면 마지막 위치 유지(True) / 정면 복귀(False)
# ======================================================

logging.getLogger("werkzeug").setLevel(logging.ERROR)
app = Flask(__name__)

view_frame = None
mask_frame = None
frame_lock = threading.Lock()
running = True


def find_object(frame):
    """
    프레임에서 노란 물체의 가로 중심(cx)을 찾음.
    반환: (cx 또는 None, 마스크 이미지, 바운딩박스 또는 None)
    """
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, HSV_LOW, HSV_HIGH)

    # 노이즈 제거: 작은 점 지우고(열림), 구멍 메움(닫힘)
    k = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None, mask, None

    largest = max(contours, key=cv2.contourArea)
    if cv2.contourArea(largest) < MIN_AREA:
        return None, mask, None

    x, y, w, h = cv2.boundingRect(largest)
    cx = x + w // 2
    return cx, mask, (x, y, w, h)


def annotate(frame, cx, box, base_angle, found):
    """인식 결과를 프레임에 그려 넣음"""
    out = frame.copy()
    center = WIDTH // 2

    # 화면 중앙선 + 데드존
    cv2.line(out, (center, 0), (center, HEIGHT), (200, 200, 200), 1)
    cv2.rectangle(out, (center - DEADZONE, 0),
                  (center + DEADZONE, HEIGHT), (120, 120, 120), 1)

    if found:
        x, y, w, h = box
        cv2.rectangle(out, (x, y), (x + w, y + h), (0, 255, 0), 2)
        cv2.circle(out, (cx, y + h // 2), 5, (0, 0, 255), -1)
        cv2.line(out, (center, HEIGHT // 2), (cx, HEIGHT // 2), (0, 0, 255), 2)
        status = f"base {base_angle:+.1f}  err {cx - center:+d}"
        color = (0, 255, 0)
    else:
        status = "NO OBJECT"
        color = (0, 0, 255)

    cv2.putText(out, status, (5, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
    return out


# ==================== 웹 스트리밍 ====================
def mjpeg(get_frame):
    while running:
        with frame_lock:
            f = get_frame()
        if f is None:
            time.sleep(0.05)
            continue
        ok, jpg = cv2.imencode(".jpg", f, [cv2.IMWRITE_JPEG_QUALITY, 70])
        if ok:
            yield (b"--frame\r\nContent-Type: image/jpeg\r\n\r\n"
                   + jpg.tobytes() + b"\r\n")
        time.sleep(0.04)


@app.route("/")
def index():
    return """
    <html><body style="margin:0;background:#1a1a1a;color:#ddd;font-family:sans-serif">
      <div style="display:flex;gap:10px;padding:10px;flex-wrap:wrap">
        <div><p>인식 결과 (초록 박스=추적 중인 물체)</p>
             <img src="/video" style="width:480px"></div>
        <div><p>노란색 마스크</p>
             <img src="/mask" style="width:480px"></div>
      </div>
      <p style="padding:0 10px">마스크에서 물체만 하얗게 나오도록 HSV 범위를 조절하세요.</p>
    </body></html>
    """


@app.route("/video")
def video():
    return Response(mjpeg(lambda: view_frame),
                    mimetype="multipart/x-mixed-replace; boundary=frame")


@app.route("/mask")
def mask():
    return Response(mjpeg(lambda: mask_frame),
                    mimetype="multipart/x-mixed-replace; boundary=frame")


# ==================== 메인 ====================
def main():
    global view_frame, mask_frame, running

    reset_mcu()
    time.sleep(0.5)

    base = Servo(BASE_PORT)
    base_angle = float(BASE_INIT)
    base.angle(base_angle)

    picam2 = Picamera2()
    picam2.configure(picam2.create_preview_configuration(
        main={"size": (WIDTH, HEIGHT), "format": "RGB888"}))
    picam2.start()
    time.sleep(1.0)

    t = threading.Thread(
        target=lambda: app.run(host="0.0.0.0", port=8000,
                               debug=False, threaded=True),
        daemon=True)
    t.start()

    print("스트리밍 시작: http://<라즈베리파이_IP>:8000")
    print("노란 물체를 카메라 앞에서 좌우로 움직여 보세요.")
    print("종료하려면 Ctrl+C\n")

    try:
        while True:
            frame = picam2.capture_array()
            cx, mask_img, box = find_object(frame)
            found = cx is not None

            if found:
                error = cx - (WIDTH // 2)          # 양수 = 물체가 오른쪽

                if abs(error) > DEADZONE:          # 데드존 밖일 때만 움직임
                    # error에 비례한 목표 이동량 (P 제어)
                    delta = KP * error * DIRECTION
                    # 한 프레임 최대 이동량 제한 (부드럽게)
                    delta = max(-MAX_STEP, min(MAX_STEP, delta))
                    base_angle += delta
                    base_angle = max(BASE_MIN, min(BASE_MAX, base_angle))
                    base.angle(base_angle)
            else:
                if not LOST_HOLD:
                    # 물체를 놓치면 정면으로 천천히 복귀
                    if abs(base_angle - BASE_INIT) > MAX_STEP:
                        base_angle += MAX_STEP * (1 if BASE_INIT > base_angle else -1)
                    else:
                        base_angle = BASE_INIT
                    base.angle(base_angle)
                # LOST_HOLD=True면 마지막 위치 그대로 유지 (아무것도 안 함)

            with frame_lock:
                view_frame = annotate(frame, cx if found else 0, box,
                                      base_angle, found)
                mask_frame = cv2.cvtColor(mask_img, cv2.COLOR_GRAY2BGR)

            time.sleep(0.03)

    except KeyboardInterrupt:
        print("\n중단")
    finally:
        running = False
        base.angle(BASE_INIT)     # 정면으로 복귀 후 종료
        picam2.stop()
        print("정지 완료")


if __name__ == "__main__":
    main()

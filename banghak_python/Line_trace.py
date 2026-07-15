"""
line_trace.py
목적: 카메라로 흰 선을 인식해서 따라가기 + 노트북 브라우저로 실시간 확인
대상: 검은 매트 위 흰 선 트랙 (오른쪽 위 소형 루프)

동작 방식:
    1. 카메라 프레임을 흑백으로 바꾸고 밝은 부분(흰 선)만 골라냄
    2. 화면 아래쪽 띠(ROI)에서 선 후보들을 찾고, 직전 위치에 가장 가까운 것을 선택
       -> 바깥 트랙 선이나 루프 건너편 선으로 튀는 것을 방지
    3. 화면 중앙에서 벗어난 정도로 앞바퀴를 꺾음 (PD 제어)
    4. 인식 결과를 그려 넣은 영상을 웹으로 송출

실행:
    sudo python3 line_trace.py
    -> 노트북 브라우저에서 http://192.168.0.82:8000 접속

종료:
    Ctrl+C
"""
from picamera2 import Picamera2
from picarx import Picarx
from flask import Flask, Response
import cv2
import numpy as np
import threading
import time
import logging

# ==================== 튜닝 파라미터 ====================
WIDTH, HEIGHT = 320, 240   # 해상도 (낮을수록 빠름)

THRESHOLD = 180            # 흰색 판정 밝기 (0~255). 매트 반사광이 하얗게 잡히면 올릴 것
ROI_TOP = 150              # 관심영역 위쪽 y (차에서 먼 쪽)
ROI_BOTTOM = 225           # 관심영역 아래쪽 y (차에서 가까운 쪽)
MIN_AREA = 300             # 이보다 작은 흰 덩어리는 노이즈로 무시 (반사광 제거용)
MAX_JUMP = 60              # 직전 위치에서 이만큼(px) 넘게 떨어진 후보는 다른 선으로 보고 무시
MIN_THICK = 8              # 선의 최소 굵기(px). 이보다 얇으면 매트 이음새/반사광으로 간주
MAX_THICK = 120            # 선의 최대 굵기(px). 이보다 두꺼우면 번진 반사광으로 간주
OPEN_KERNEL = 3            # 모폴로지 열림 커널 크기. 작은 반사광 점을 지움
SMOOTH = 0.6               # cx 스무딩 계수 (0~1). 낮을수록 부드럽지만 반응 느림

KP = 0.25                  # 조향 P 게인. 급커브라 기본값보다 올림
KD = 0.05                  # 조향 D 게인. 노이즈를 증폭하므로 낮게 시작
MAX_STEER = 30             # 최대 조향각 (도)
STEER_OFFSET = 0           # 앞바퀴가 한쪽으로 틀어져 있으면 여기서 보정

DRIVE_DUTY = 20            # 주행 듀티 사이클(%). 급커브라 느리게
CAM_PAN = 0                # 카메라 좌우 (정면 = 0)
CAM_TILT = -40             # 카메라 상하. 루프 건너편 선이 화면에 들어오면 더 내릴 것(-50)

LOST_GRACE = 0.4           # 선을 놓친 뒤 직전 조향으로 버티는 시간(초). 넘으면 정지
# ======================================================

logging.getLogger("werkzeug").setLevel(logging.ERROR)

app = Flask(__name__)

view_frame = None
mask_frame = None
frame_lock = threading.Lock()
running = True


def move_raw(px, duty, direction="forward"):
    """
    duty(0~100%)를 모터 PWM에 그대로 적용.
    picarx의 forward()는 최소 50% 듀티부터 시작해서 라인트레이싱엔 너무 빠름 -> 직접 제어.
    """
    duty = max(0, min(100, duty))
    if direction == "forward":
        px.motor_direction_pins[0].low()
        px.motor_direction_pins[1].high()
    else:
        px.motor_direction_pins[0].high()
        px.motor_direction_pins[1].low()
    px.motor_speed_pins[0].pulse_width_percent(duty)
    px.motor_speed_pins[1].pulse_width_percent(duty)


def find_line(frame, last_cx):
    """
    프레임에서 따라갈 선의 가로 중심(cx)을 찾음.

    핵심: "가장 큰 덩어리"가 아니라 "직전 위치에 가장 가까운 덩어리"를 고른다.
    이 트랙은 바깥 트랙 선과 루프 건너편 선이 같이 보일 수 있어서,
    크기로 고르면 엉뚱한 선으로 튄다.

    반환: (cx 또는 None, 이진화 마스크, 후보 cx 목록)
    """
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    _, binary = cv2.threshold(blur, THRESHOLD, 255, cv2.THRESH_BINARY)

    # 모폴로지 열림: 작은 흰 점(매트 반사광)을 지움. 굵은 선은 살아남음.
    k = np.ones((OPEN_KERNEL, OPEN_KERNEL), np.uint8)
    binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, k)

    roi = binary[ROI_TOP:ROI_BOTTOM, :]

    contours, _ = cv2.findContours(roi, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # 면적 + 굵기 기준을 통과한 후보들의 중심 x 좌표를 모음
    candidates = []
    for c in contours:
        if cv2.contourArea(c) < MIN_AREA:
            continue
        # 선의 굵기로 한 번 더 거름.
        # boundingRect의 폭은 선이 기울면 실제 굵기보다 훨씬 커지므로 쓰면 안 된다.
        # minAreaRect는 기울어진 사각형을 재므로, 짧은 변 = 진짜 굵기.
        (_, _), (w, h), _ = cv2.minAreaRect(c)
        thick = min(w, h)
        if thick < MIN_THICK or thick > MAX_THICK:
            continue
        M = cv2.moments(c)
        if M["m00"] == 0:
            continue
        candidates.append(int(M["m10"] / M["m00"]))

    if not candidates:
        return None, binary, []

    if last_cx is None:
        # 첫 프레임(또는 선을 잃은 직후): 화면 중앙에 가장 가까운 것을 선택
        #  -> 차를 선 위에 올려놓고 시작한다는 전제
        ref = WIDTH // 2
        cx = min(candidates, key=lambda x: abs(x - ref))
    else:
        # 평소: 직전 위치에 가장 가까운 것을 선택 (선을 계속 추적)
        cx = min(candidates, key=lambda x: abs(x - last_cx))
        if abs(cx - last_cx) > MAX_JUMP:
            # 너무 멀리 떨어짐 = 다른 선을 잡은 것. 무시.
            return None, binary, candidates

    return cx, binary, candidates


def annotate(frame, cx, candidates, steer, found):
    """인식 결과를 프레임에 그려 넣음 (튜닝용)"""
    out = frame.copy()
    center = WIDTH // 2
    cy = (ROI_TOP + ROI_BOTTOM) // 2

    cv2.rectangle(out, (0, ROI_TOP), (WIDTH - 1, ROI_BOTTOM), (0, 255, 255), 1)
    cv2.line(out, (center, ROI_TOP), (center, ROI_BOTTOM), (200, 200, 200), 1)

    # 선택되지 않은 후보들 (파란 점) - 다른 선이 몇 개나 보이는지 확인용
    for c in candidates:
        if not found or c != cx:
            cv2.circle(out, (c, cy), 4, (255, 150, 0), -1)

    if found:
        cv2.circle(out, (cx, cy), 6, (0, 0, 255), -1)
        cv2.line(out, (center, cy), (cx, cy), (0, 0, 255), 2)
        status = f"steer {steer:+.1f}  err {cx - center:+d}  cand {len(candidates)}"
        color = (0, 255, 0)
    else:
        status = f"LINE LOST  cand {len(candidates)}"
        color = (0, 0, 255)

    cv2.putText(out, status, (5, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1)
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
        <div><p>인식 결과 (빨강=추적 중인 선, 파랑=무시한 다른 선)</p>
             <img src="/video" style="width:480px"></div>
        <div><p>이진화 마스크</p>
             <img src="/mask" style="width:480px"></div>
      </div>
      <p style="padding:0 10px">마스크에서 흰 선만 남고 검은 매트가 완전히 까맣게 나오도록
         THRESHOLD를 조절하세요. 파란 점이 자주 보이면 CAM_TILT를 더 내리세요.</p>
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

    px = Picarx()
    px.set_dir_servo_angle(STEER_OFFSET)
    px.set_cam_pan_angle(CAM_PAN)
    px.set_cam_tilt_angle(CAM_TILT)

    picam2 = Picamera2()
    picam2.configure(picam2.create_preview_configuration(
        main={"size": (WIDTH, HEIGHT), "format": "RGB888"}))
    picam2.start()
    time.sleep(1.0)   # 카메라 노출 안정화

    t = threading.Thread(
        target=lambda: app.run(host="0.0.0.0", port=8000,
                               debug=False, threaded=True),
        daemon=True)
    t.start()

    print("스트리밍 시작: http://<라즈베리파이_IP>:8000")
    print("차를 흰 선 위에, 선 방향과 나란히 올려놓고 시작하세요.")
    print("종료하려면 Ctrl+C\n")

    last_seen = time.time()
    last_cx = None
    smooth_cx = None
    last_error = 0
    steer = STEER_OFFSET

    try:
        while True:
            frame = picam2.capture_array()

            cx, binary, candidates = find_line(frame, last_cx)
            found = cx is not None

            if found:
                # 스무딩: 한 프레임 튄 값이 조향에 바로 반영되지 않게 완만하게 섞음
                if smooth_cx is None:
                    smooth_cx = float(cx)
                else:
                    smooth_cx = SMOOTH * cx + (1 - SMOOTH) * smooth_cx

                error = smooth_cx - (WIDTH // 2)       # 양수 = 선이 오른쪽에 있음
                derivative = error - last_error        # 오차가 벌어지는 속도

                steer = KP * error + KD * derivative + STEER_OFFSET
                steer = max(-MAX_STEER, min(MAX_STEER, steer))

                px.set_dir_servo_angle(steer)
                move_raw(px, DRIVE_DUTY, "forward")

                last_error = error
                last_cx = cx
                last_seen = time.time()
            else:
                # 선을 놓침: 잠깐은 직전 조향 유지, 오래 못 찾으면 정지 후 추적 초기화
                if time.time() - last_seen > LOST_GRACE:
                    px.stop()
                    last_cx = None      # 다음엔 화면 중앙 기준으로 다시 찾음
                    smooth_cx = None
                    last_error = 0

            with frame_lock:
                view_frame = annotate(frame, cx if found else 0,
                                      candidates, steer, found)
                mask_frame = cv2.cvtColor(binary, cv2.COLOR_GRAY2BGR)

            time.sleep(0.02)

    except KeyboardInterrupt:
        print("\n중단")
    finally:
        running = False
        px.stop()
        px.set_dir_servo_angle(STEER_OFFSET)
        picam2.stop()
        print("정지 완료")


if __name__ == "__main__":
    main()
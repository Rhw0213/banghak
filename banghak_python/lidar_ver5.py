# lidar_ver5.py
# 라이다 + 초음파 + 카메라(노란 오브젝트) 융합 주행 + 웹 스트리밍
#
# 카메라 획득: picamera2 (libcamera 스택). cv2.VideoCapture 사용 안 함.
#
# 실행:
#   python3 lidar_ver5.py            # 본 주행 (스트리밍 동시 동작)
#   python3 lidar_ver5.py calib      # 차량 정지 상태로 카메라 튜닝만
#
# 브라우저에서 http://<라즈베리파이IP>:8000 접속

import os
import sys
import select
import tty
import termios
import time
import threading
import json
import socket
from urllib.parse import urlparse, parse_qs
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np

from picamera2 import Picamera2

from rplidar import RPLidar, RPLidarException
from robot_hat import Motor, Servo, Pin, PWM, reset_mcu, Ultrasonic
from arm_setup import build_arm   # [로봇팔 추가] 관절을 SmoothJoint로 감싸서 반환
from arm_visual_servo import (    # [픽업 추가] 빨간 마커 + 노란 타겟 시각 서보 픽업
    visual_servo_pick,
    VS_CAM_TILT_ANGLE,
    start_stream as vs_start_stream,
    stop_stream as vs_stop_stream,
)

from avoidance_go_back import Get_Stop_Distance, WallBackup, Get_Drive_Duty
from picarx import Picarx

from battery_driving_check import BatteryMonitor

# ===== 설정 =====
LIDAR_PORT = '/dev/ttyUSB0'

# 조향 (라이다 회피용)
STEER_GAIN = 0.3
STEER_DEADZONE = 0
STEER_ACTIVATE_DIST = 800
STEER_LIMIT = 35
GAIN_REVERSE = False

# 거리 임계값 (라이다=mm, 초음파=cm)
DANGER_DIST_MM = 500
STOP_DIST_MM = 300

# 후방 거리 기반 동적 후진거리
REAR_SECTOR_DEG = 25
REAR_SAFETY_MARGIN_CM = 11
DEFAULT_BACK_TARGET_CM = 60

# 속도
VELOCITY = 45
SPEED_FAST = 0
SPEED_BACK = 0
SPEED_SLOW = 30

SPEED_STEP = 3
SCAN_MIN_LEN = 60

# 초근접 탈출 (좁은 공간에서 조향 꺾인 채 후진하며 갇히는 것 방지)
# VERY_CLOSE_CM = 4
VERY_CLOSE_CM = 3
STEER_RESET_TOL = 3.0

# ===== [과부하 감시 추가] 시스템 부하(CPU) 설정 =====
LOAD_CHECK_INTERVAL_SEC = 2.0
LOAD_WARN_RATIO = 1.0


# =========================================================================
# ===== [비전 추가] 카메라 / 노란 오브젝트 인식 설정 =====
# =========================================================================

CAM_WIDTH = 320
CAM_HEIGHT = 240
CAM_FORMAT = "RGB888"
CAM_TILT_ANGLE = 10
CAM_SWAP_RB = False

COLOR_H_MIN, COLOR_H_MAX = 20, 35
COLOR_S_MIN, COLOR_S_MAX = 100, 255
COLOR_V_MIN, COLOR_V_MAX = 100, 255

MIN_TARGET_AREA = 400

TARGET_REAL_WIDTH_CM = 8.0
CAM_FOCAL_PX = 960.0

ARRIVE_DISTANCE_CM = 10 #선반 도착 확인 거리
# ARRIVE_DISTANCE_CM = 5
ARRIVE_CENTER_TOL = 0.15


# =========================================================================
# ===== [선반 정렬 추가] 초록색 선반(직사각형) 인식 설정 =====
# =========================================================================

GREEN_H_MIN, GREEN_H_MAX = 45, 85
GREEN_S_MIN, GREEN_S_MAX = 80, 255
GREEN_V_MIN, GREEN_V_MAX = 60, 255

MIN_SHELF_AREA = 500

SHELF_REAL_WIDTH_CM = 10.0

# =========================================================================
# ===== [파란 장애물 추가] 라이다 사각지대(라이다 높이보다 낮은) 오브젝트
#   장애물 인식 설정. 라이다는 이 물체를 아예 볼 수 없어서 카메라로 판단.
#   초음파는 감지 가능한 높이라서 "있다/탈출했다" 판정은 초음파 기반 유지.
# =========================================================================
BLUE_H_MIN, BLUE_H_MAX = 100, 125      # ★ calib 모드에서 웹 UI로 실측 후 교체
BLUE_S_MIN, BLUE_S_MAX = 60, 255
BLUE_V_MIN, BLUE_V_MAX = 40, 255

MIN_OBSTACLE_AREA = 400
OBSTACLE_REAL_WIDTH_CM = 8.0    # ★ 실제 장애물 가로폭으로 교체
OBSTACLE_STEER_GAIN = 25.0      # 카메라 기반 회피 조향 강도(도)

DOCK_ENTER_DISTANCE_CM = 40.0
# DOCK_SPEED = 22
DOCK_SPEED = 23
DOCK_OFFSET_TOL = 0.05
DOCK_SKEW_TOL = 0.15
SKEW_STEER_GAIN = 20.0

# [안전장치 추가] DOCK 중 카메라가 선반을 놓쳤을 때, 초음파가 여전히 이
# 거리 이내를 가리키면 "코앞에 뭔가 있다"고 보고 SEARCH로 풀어주지 않고
# 그냥 정지시킨다. (선반을 놓친 채로 SEARCH -> CRUISE로 넘어가면 정상속도로
# 다시 튀어나가서 목표물을 그냥 지나쳐버리는 문제가 있었음)
DOCK_LOST_SAFE_ULTRA_CM = 17

# [감속 추가] DOCK 중 이 거리 이내로 들어오면 속도를 점점 줄인다.
# 최소 속도는 DOCK_SPEED보다 너무 낮추지 않음 (예전에 바닥 마찰로 아예
# 안 움직이던 문제가 있었기 때문 - DOCK_SPEED=21까지 낮췄던 이력 참고)
DOCK_SLOWDOWN_START_CM = 15
DOCK_SLOWDOWN_MIN_SPEED = 21

# [로봇팔 추가] 종료 시 팔을 0도로 복귀시킬 때 쓰는 속도(도/초)
ARM_HOME_SPEED = 30.0

# [로봇팔 슬로우스타터 시험 추가] 'p' 입력 시 실행되는 arm_slow_demo.py와
# 동일한 패턴의 테스트 - 4관절을 +10도까지 이동했다가 잠깐 대기 후 0도로 복귀
ARM_TEST_ANGLE = 20
ARM_TEST_HOLD_SEC = 3.0


def run_arm_slow_test(joints):
    """
    arm_slow_demo.py와 동일한 패턴: 0(혹은 현재각) -> ARM_TEST_ANGLE -> 0
    새 Servo 객체를 만들지 않고, main()에서 이미 만든 joints를 그대로 재사용한다.
    """
    print(f"[로봇팔 테스트] 4관절 모두 {ARM_TEST_ANGLE}도까지 슬로우스타터로 이동 중...")
    for j in joints:
        j.move_to(ARM_TEST_ANGLE, speed=ARM_HOME_SPEED)
    print(f"[로봇팔 테스트] 이동 완료 - {ARM_TEST_HOLD_SEC:.0f}초 대기 후 0도로 복귀합니다...")
    time.sleep(ARM_TEST_HOLD_SEC)
    print("[로봇팔 테스트] 0도로 슬로우스타터 복귀 중...")
    for j in joints:
        j.move_to(0, speed=ARM_HOME_SPEED)
    print("[로봇팔 테스트] 복귀 완료 - 주행 재개")


def _check_keypress():
    """
    [로봇팔 슬로우스타터 시험 추가] 터미널이 cbreak 모드일 때, 지금 입력된
    키가 있으면 논블로킹으로 읽어서 반환. 없으면 None. (Enter 없이 한 글자만
    눌러도 즉시 감지됨 - main()에서 tty.setcbreak()로 미리 설정해둔 상태여야 함)
    """
    if select.select([sys.stdin], [], [], 0)[0]:
        return sys.stdin.read(1)
    return None


APPROACH_SPEED = 24
TARGET_STEER_GAIN = 30.0
LOST_TIMEOUT = 2.0
DETOUR_TIME = 1.2
DETOUR_STEER = 25.0
CONFIRM_HITS = 3
TARGET_TOLERANCE_RATIO = 0.35

SEARCH = "SEARCH"
APPROACH = "APPROACH"
DETOUR = "DETOUR"
DOCK = "DOCK"
ARRIVED = "ARRIVED"


class TargetInfo:
    def __init__(self, found=False, offset=0.0, distance_cm=-1.0,
                 area=0.0, width_px=0.0, box=None, ts=0.0, skew=0.0):
        self.found = found
        self.offset = offset
        self.distance_cm = distance_cm
        self.area = area
        self.width_px = width_px
        self.box = box
        self.ts = ts
        self.skew = skew

    def is_fresh(self, max_age=0.5):
        return (time.time() - self.ts) < max_age

    def is_centered(self, tol=0.15):
        return self.found and abs(self.offset) < tol

    def is_docked(self, offset_tol=DOCK_OFFSET_TOL, skew_tol=DOCK_SKEW_TOL):
        return self.found and abs(self.offset) < offset_tol and abs(self.skew) < skew_tol


_object_lock = threading.Lock()
_object_result = TargetInfo()
_shelf_lock = threading.Lock()
_shelf_result = TargetInfo()
_obstacle_lock = threading.Lock()      # [파란 장애물 추가]
_obstacle_result = TargetInfo()

_vision_running = False
_vision_thread = None
_picam2 = None
VISION_ENABLED = False
VISION_ERROR = ""

EDIT_COLOR = "yellow"


def get_yellow_hsv_range():
    lower = np.array([COLOR_H_MIN, COLOR_S_MIN, COLOR_V_MIN])
    upper = np.array([COLOR_H_MAX, COLOR_S_MAX, COLOR_V_MAX])
    return lower, upper


def get_green_hsv_range():
    lower = np.array([GREEN_H_MIN, GREEN_S_MIN, GREEN_V_MIN])
    upper = np.array([GREEN_H_MAX, GREEN_S_MAX, GREEN_V_MAX])
    return lower, upper


def get_blue_hsv_range():   # [파란 장애물 추가]
    lower = np.array([BLUE_H_MIN, BLUE_S_MIN, BLUE_V_MIN])
    upper = np.array([BLUE_H_MAX, BLUE_S_MAX, BLUE_V_MAX])
    return lower, upper


def grab_frame():
    try:
        arr = _picam2.capture_array()
    except Exception:
        return None
    if arr is None:
        return None
    if arr.ndim == 3 and arr.shape[2] == 4:
        frame = cv2.cvtColor(arr, cv2.COLOR_BGRA2BGR)
    elif arr.ndim == 3 and arr.shape[2] == 3:
        frame = arr
    else:
        return None
    if CAM_SWAP_RB:
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
    return frame


def _compute_skew(mask, bx, by, bw, bh):
    if bw < 8:
        return 0.0
    roi = mask[by:by + bh, bx:bx + bw]
    q = max(1, bw // 4)
    left_cols = roi[:, :q]
    right_cols = roi[:, -q:]
    left_h = float(np.count_nonzero(left_cols)) / q
    right_h = float(np.count_nonzero(right_cols)) / q
    denom = max(left_h, right_h, 1.0)
    return (right_h - left_h) / denom


def detect_color(frame, lower, upper, min_area, real_width_cm, compute_skew=False):
    h, w = frame.shape[:2]
    blurred = cv2.GaussianBlur(frame, (5, 5), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower, upper)
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return TargetInfo(found=False, ts=time.time()), mask
    largest = max(contours, key=cv2.contourArea)
    area = cv2.contourArea(largest)
    if area < min_area:
        return TargetInfo(found=False, ts=time.time()), mask
    bx, by, bw, bh = cv2.boundingRect(largest)
    cx = bx + bw / 2.0
    offset = (cx - w / 2.0) / (w / 2.0)
    distance = (real_width_cm * CAM_FOCAL_PX) / bw if bw > 0 else -1.0
    skew = _compute_skew(mask, bx, by, bw, bh) if compute_skew else 0.0
    info = TargetInfo(found=True, offset=offset, distance_cm=distance,
                      area=area, width_px=float(bw), box=(bx, by, bw, bh),
                      ts=time.time(), skew=skew)
    return info, mask


def detect_yellow(frame):
    lower, upper = get_yellow_hsv_range()
    return detect_color(frame, lower, upper, MIN_TARGET_AREA,
                        TARGET_REAL_WIDTH_CM, compute_skew=False)


def detect_shelf(frame):
    lower, upper = get_green_hsv_range()
    return detect_color(frame, lower, upper, MIN_SHELF_AREA,
                        SHELF_REAL_WIDTH_CM, compute_skew=True)


def detect_obstacle(frame):   # [파란 장애물 추가]
    """라이다 사각지대 낮은 오브젝트 장애물 인식. skew는 방향 판단에 불필요."""
    lower, upper = get_blue_hsv_range()
    return detect_color(frame, lower, upper, MIN_OBSTACLE_AREA,
                        OBSTACLE_REAL_WIDTH_CM, compute_skew=False)


def _vision_loop():
    global _object_result, _shelf_result, _obstacle_result
    last_stream = 0.0
    fail_count = 0
    while _vision_running:
        frame = grab_frame()
        if frame is None:
            fail_count += 1
            if fail_count % 50 == 1:
                print("[비전] 프레임 획득 실패")
            time.sleep(0.05)
            continue
        fail_count = 0
        obj_info, obj_mask = detect_yellow(frame)
        shelf_info, shelf_mask = detect_shelf(frame)
        obstacle_info, _ = detect_obstacle(frame)   # [파란 장애물 추가]
        with _object_lock:
            _object_result = obj_info
        with _shelf_lock:
            _shelf_result = shelf_info
        with _obstacle_lock:                        # [파란 장애물 추가]
            _obstacle_result = obstacle_info
        now = time.time()
        if _stream_clients > 0 and (now - last_stream) >= (1.0 / STREAM_FPS):
            last_stream = now
            _publish_frame(frame, obj_mask, shelf_mask, obj_info, shelf_info)
        time.sleep(0.01)


def start_vision():
    global _picam2, _vision_running, _vision_thread, VISION_ENABLED, VISION_ERROR
    try:
        _picam2 = Picamera2()
        config = _picam2.create_preview_configuration(
            main={"size": (CAM_WIDTH, CAM_HEIGHT), "format": CAM_FORMAT})
        _picam2.configure(config)
        _picam2.start()
        time.sleep(1.0)
        test = _picam2.capture_array()
        if test is None:
            raise RuntimeError("capture_array()가 None 반환")
        print(f"[비전] 첫 프레임 shape={test.shape}")
        _vision_running = True
        _vision_thread = threading.Thread(target=_vision_loop, daemon=True)
        _vision_thread.start()
        time.sleep(0.5)
        VISION_ENABLED = True
        VISION_ERROR = ""
        print("[비전] 카메라 시작 완료 (picamera2)")
    except Exception as e:
        VISION_ENABLED = False
        VISION_ERROR = str(e)
        print(f"[비전] 카메라 시작 실패({e}) - 비전 없이 기존 주행만 수행합니다")


def stop_vision():
    global _vision_running
    _vision_running = False
    if _vision_thread:
        _vision_thread.join(timeout=1.5)
    if _picam2:
        try:
            _picam2.stop()
            _picam2.close()
        except Exception:
            pass


def get_object():
    with _object_lock:
        return _object_result


def get_shelf():
    with _shelf_lock:
        return _shelf_result


def get_obstacle():   # [파란 장애물 추가]
    """라이다 사각지대 낮은 오브젝트 장애물 최신 인식 결과"""
    with _obstacle_lock:
        return _obstacle_result


def Get_Load_Warn_Threshold():
    cores = os.cpu_count() or 1
    return cores * LOAD_WARN_RATIO


class SystemLoadMonitor:
    def __init__(self, interval=LOAD_CHECK_INTERVAL_SEC):
        self.interval = interval
        self.last_time = 0
        self.last_load1 = None
        self.max_load1 = 0.0
        self.cores = os.cpu_count() or 1

    def read(self):
        try:
            load1, load5, load15 = os.getloadavg()
            if load1 > self.max_load1:
                self.max_load1 = load1
            self.last_load1 = load1
            return load1
        except Exception:
            return None

    def show(self):
        now = time.time()
        if now - self.last_time < self.interval:
            return None
        self.last_time = now
        load1 = self.read()
        if load1 is None:
            print("[CPU부하] 읽기 실패")
            return None
        threshold = Get_Load_Warn_Threshold()
        warn = f"  <-- 과부하 의심! (코어 {self.cores}개)" if load1 > threshold else ""
        print(f"##[CPU부하] {load1:.2f}  (최고 {self.max_load1:.2f}, "
              f"코어 {self.cores}개, 경고기준 {threshold:.1f}){warn}##")
        return load1

    def is_overloaded(self):
        if self.last_load1 is None:
            return False
        return self.last_load1 > Get_Load_Warn_Threshold()


# =========================================================================
# ===== [스트리밍 추가] MJPEG 웹 서버 + 실시간 HSV 튜닝 UI =====
# =========================================================================

STREAM_PORT = 8000
STREAM_FPS = 10
STREAM_QUALITY = 60
STREAM_SHOW_MASK = False

_stream_lock = threading.Lock()
_stream_jpeg = None
_stream_clients = 0
_stream_server = None
_app_running = True

_telemetry = {
    "state": SEARCH,
    "reason": "-",
    "ultra_cm": -1.0,
    "lidar_mm": -1.0,
    "speed": 0,
    "steer": 0.0,
    "load_1min": 0.0,
}
_telemetry_lock = threading.Lock()


def update_telemetry(**kwargs):
    with _telemetry_lock:
        _telemetry.update(kwargs)


def _make_placeholder(text):
    img = np.zeros((CAM_HEIGHT, CAM_WIDTH, 3), np.uint8)
    img[:] = (30, 30, 30)
    cv2.putText(img, "NO CAMERA", (int(CAM_WIDTH * 0.12), int(CAM_HEIGHT * 0.45)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
    cv2.putText(img, text[:34], (6, int(CAM_HEIGHT * 0.65)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.35, (200, 200, 200), 1)
    cv2.putText(img, "check terminal log", (6, int(CAM_HEIGHT * 0.80)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.35, (150, 150, 150), 1)
    ok, buf = cv2.imencode('.jpg', img)
    return buf.tobytes() if ok else None


def _draw_overlay(frame, obj_info, shelf_info):
    h, w = frame.shape[:2]
    out = frame.copy()
    cv2.line(out, (w // 2, 0), (w // 2, h), (200, 200, 200), 1)
    if obj_info.found and obj_info.box:
        bx, by, bw, bh = obj_info.box
        cv2.rectangle(out, (bx, by), (bx + bw, by + bh), (0, 220, 255), 2)
        cv2.putText(out, f"OBJ {obj_info.distance_cm:.0f}cm", (bx, max(15, by - 6)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 220, 255), 1)
    if shelf_info.found and shelf_info.box:
        bx, by, bw, bh = shelf_info.box
        cv2.rectangle(out, (bx, by), (bx + bw, by + bh), (0, 255, 0), 2)
        cx = int(bx + bw / 2)
        cy = int(by + bh / 2)
        cv2.circle(out, (cx, cy), 4, (0, 0, 255), -1)
        cv2.line(out, (w // 2, cy), (cx, cy), (0, 0, 255), 1)
        cv2.putText(out, f"SHELF {shelf_info.distance_cm:.0f}cm skew={shelf_info.skew:+.2f}",
                    (bx, min(h - 5, by + bh + 15)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 1)
    with _telemetry_lock:
        state = _telemetry["state"]
        ultra = _telemetry["ultra_cm"]
    if shelf_info.found:
        txt = f"{state} shelf off={shelf_info.offset:+.2f} skew={shelf_info.skew:+.2f} ultra={ultra:.0f}cm"
    else:
        txt = f"{state} no shelf ultra={ultra:.0f}cm"
    cv2.putText(out, txt, (5, h - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                (0, 255, 255), 1)
    return out


def _publish_frame(frame, obj_mask, shelf_mask, obj_info, shelf_info):
    global _stream_jpeg
    if STREAM_SHOW_MASK:
        mask = shelf_mask if EDIT_COLOR == "green" else obj_mask
        img = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
    else:
        img = _draw_overlay(frame, obj_info, shelf_info)
    ok, buf = cv2.imencode('.jpg', img,
                           [int(cv2.IMWRITE_JPEG_QUALITY), STREAM_QUALITY])
    if ok:
        with _stream_lock:
            _stream_jpeg = buf.tobytes()


PAGE_HTML = """<!DOCTYPE html>
<html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>PiCar Vision</title>
<style>
 body{font-family:sans-serif;background:#1a1a1a;color:#eee;margin:0;padding:12px}
 img{width:100%;max-width:640px;image-rendering:pixelated;border:1px solid #444}
 .row{display:flex;align-items:center;gap:8px;margin:4px 0}
 .row label{width:60px;font-size:13px}
 .row input[type=range]{flex:1}
 .row span{width:44px;text-align:right;font-size:13px;color:#8f8}
 #status{background:#222;padding:8px;border-radius:4px;font-size:13px;
         line-height:1.6;margin:10px 0;white-space:pre-wrap}
 button{padding:8px 14px;margin:0 6px 6px 0;background:#356;color:#fff;
        border:none;border-radius:4px;font-size:14px}
 h3{margin:14px 0 6px;font-size:15px;color:#9cf}
</style></head><body>
<img src="/stream.mjpg">
<div id="status">연결 중...</div>
<button onclick="fetch('/set?mask=toggle')">마스크 보기 전환</button>
<button onclick="fetch('/set?swaprb=toggle')">R/B 색상 반전</button>
<button onclick="dump()">현재 HSV 값 출력</button>
<h3>HSV 범위 튜닝 - 편집 대상:
  <button id="btn_yellow" onclick="setColor('yellow')">노랑(오브젝트)</button>
  <button id="btn_green" onclick="setColor('green')">초록(선반)</button>
  <button id="btn_blue" onclick="setColor('blue')">파랑(장애물)</button>
</h3>
<div id="sliders"></div>
<script>
const P=[["h_min",179],["h_max",179],["s_min",255],["s_max",255],["v_min",255],["v_max",255]];
const box=document.getElementById('sliders');
let curColor='yellow';
const COLORS=['yellow','green','blue'];
P.forEach(([k,max])=>{
  const d=document.createElement('div');d.className='row';
  d.innerHTML=`<label>${k}</label><input type=range min=0 max=${max} id="${k}">
               <span id="${k}v"></span>`;
  box.appendChild(d);
});
function send(){
  const q=P.map(([k])=>k+'='+document.getElementById(k).value).join('&')+'&color='+curColor;
  fetch('/set?'+q);
  P.forEach(([k])=>document.getElementById(k+'v').textContent=
      document.getElementById(k).value);
}
P.forEach(([k])=>document.getElementById(k).addEventListener('input',send));
function updateBtns(c){COLORS.forEach(x=>document.getElementById('btn_'+x).style.opacity=(x===c)?1:0.5);}
function setColor(c){
  curColor=c; window._init=false;
  fetch('/set?editcolor='+c); updateBtns(c);
}
function dump(){fetch('/set?dump=1').then(()=>alert('라즈베리파이 터미널에 출력했습니다'));}
async function poll(){
  try{
    const r=await fetch('/status');const s=await r.json();
    document.getElementById('status').textContent=
      `카메라: ${s.cam_ok?('정상  (R/B반전 '+(s.swap_rb?'ON':'OFF')+')'):('오류 - '+s.cam_err)}\\n`+
      `상태  : ${s.state}\\n사유  : ${s.reason}\\n`+
      `초음파: ${s.ultra_cm.toFixed(0)} cm   라이다: ${s.lidar_mm.toFixed(0)} mm\\n`+
      `속도  : ${s.speed}   조향: ${s.steer.toFixed(0)}도\\n`+
      `CPU부하: ${s.load_1min.toFixed(2)}\\n`+
      `초록 선반: ${s.shelf.found?('발견  오프셋 '+s.shelf.offset.toFixed(2)+
        '  skew '+s.shelf.skew.toFixed(2)+
        '  거리 '+s.shelf.distance_cm.toFixed(0)+'cm  폭 '+s.shelf.width_px.toFixed(0)+'px')
        :'없음'}\\n`+
      `노랑 오브젝트: ${s.object.found?('발견  오프셋 '+s.object.offset.toFixed(2)+
        '  거리 '+s.object.distance_cm.toFixed(0)+'cm'):'없음'}\\n`+
      `파랑 장애물: ${s.obstacle?s.obstacle.found?('발견  오프셋 '+s.obstacle.offset.toFixed(2)+
        '  거리 '+s.obstacle.distance_cm.toFixed(0)+'cm'):'없음':'없음'}`;
    if(!window._init){
      window._init=true; curColor=s.edit_color;
      const hsvMap={yellow:s.hsv_yellow,green:s.hsv_green,blue:s.hsv_blue};
      const hsv=hsvMap[s.edit_color]||s.hsv_yellow;
      P.forEach(([k])=>{document.getElementById(k).value=hsv[k];
                        document.getElementById(k+'v').textContent=hsv[k];});
      updateBtns(s.edit_color);
    }
  }catch(e){}
}
setInterval(poll,400);poll();
</script></body></html>"""


class _StreamHandler(BaseHTTPRequestHandler):
    def log_message(self, *args):
        pass

    def do_GET(self):
        global _stream_clients, STREAM_SHOW_MASK, CAM_SWAP_RB, EDIT_COLOR
        global COLOR_H_MIN, COLOR_H_MAX, COLOR_S_MIN
        global COLOR_S_MAX, COLOR_V_MIN, COLOR_V_MAX
        global GREEN_H_MIN, GREEN_H_MAX, GREEN_S_MIN
        global GREEN_S_MAX, GREEN_V_MIN, GREEN_V_MAX
        global BLUE_H_MIN, BLUE_H_MAX, BLUE_S_MIN
        global BLUE_S_MAX, BLUE_V_MIN, BLUE_V_MAX

        parsed = urlparse(self.path)
        path = parsed.path

        if path == '/':
            body = PAGE_HTML.encode('utf-8')
            self.send_response(200)
            self.send_header('Content-Type', 'text/html; charset=utf-8')
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        elif path == '/status':
            obj = get_object()
            shelf = get_shelf()
            obstacle = get_obstacle()
            with _telemetry_lock:
                data = dict(_telemetry)
            data.update({
                "cam_ok": VISION_ENABLED,
                "cam_err": VISION_ERROR,
                "swap_rb": CAM_SWAP_RB,
                "edit_color": EDIT_COLOR,
                "object": {
                    "found": bool(obj.found and obj.is_fresh()),
                    "offset": obj.offset,
                    "distance_cm": obj.distance_cm,
                    "width_px": obj.width_px,
                },
                "shelf": {
                    "found": bool(shelf.found and shelf.is_fresh()),
                    "offset": shelf.offset,
                    "skew": shelf.skew,
                    "distance_cm": shelf.distance_cm,
                    "width_px": shelf.width_px,
                },
                "obstacle": {
                    "found": bool(obstacle.found and obstacle.is_fresh()),
                    "offset": obstacle.offset,
                    "distance_cm": obstacle.distance_cm,
                    "width_px": obstacle.width_px,
                },
                "hsv_yellow": {
                    "h_min": COLOR_H_MIN, "h_max": COLOR_H_MAX,
                    "s_min": COLOR_S_MIN, "s_max": COLOR_S_MAX,
                    "v_min": COLOR_V_MIN, "v_max": COLOR_V_MAX,
                },
                "hsv_green": {
                    "h_min": GREEN_H_MIN, "h_max": GREEN_H_MAX,
                    "s_min": GREEN_S_MIN, "s_max": GREEN_S_MAX,
                    "v_min": GREEN_V_MIN, "v_max": GREEN_V_MAX,
                },
                "hsv_blue": {
                    "h_min": BLUE_H_MIN, "h_max": BLUE_H_MAX,
                    "s_min": BLUE_S_MIN, "s_max": BLUE_S_MAX,
                    "v_min": BLUE_V_MIN, "v_max": BLUE_V_MAX,
                },
            })
            body = json.dumps(data).encode('utf-8')
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        elif path == '/set':
            q = parse_qs(parsed.query)

            def gv(key, cur):
                try:
                    return int(q[key][0])
                except Exception:
                    return cur

            target_color = q.get('color', [EDIT_COLOR])[0]

            if target_color == 'green':
                GREEN_H_MIN = gv('h_min', GREEN_H_MIN)
                GREEN_H_MAX = gv('h_max', GREEN_H_MAX)
                GREEN_S_MIN = gv('s_min', GREEN_S_MIN)
                GREEN_S_MAX = gv('s_max', GREEN_S_MAX)
                GREEN_V_MIN = gv('v_min', GREEN_V_MIN)
                GREEN_V_MAX = gv('v_max', GREEN_V_MAX)
            elif target_color == 'blue':
                BLUE_H_MIN = gv('h_min', BLUE_H_MIN)
                BLUE_H_MAX = gv('h_max', BLUE_H_MAX)
                BLUE_S_MIN = gv('s_min', BLUE_S_MIN)
                BLUE_S_MAX = gv('s_max', BLUE_S_MAX)
                BLUE_V_MIN = gv('v_min', BLUE_V_MIN)
                BLUE_V_MAX = gv('v_max', BLUE_V_MAX)
            elif target_color == 'yellow':
                COLOR_H_MIN = gv('h_min', COLOR_H_MIN)
                COLOR_H_MAX = gv('h_max', COLOR_H_MAX)
                COLOR_S_MIN = gv('s_min', COLOR_S_MIN)
                COLOR_S_MAX = gv('s_max', COLOR_S_MAX)
                COLOR_V_MIN = gv('v_min', COLOR_V_MIN)
                COLOR_V_MAX = gv('v_max', COLOR_V_MAX)

            if 'editcolor' in q:
                EDIT_COLOR = q['editcolor'][0]
                print(f"[비전] 편집 대상 색상 = {EDIT_COLOR}")
            if 'mask' in q:
                STREAM_SHOW_MASK = not STREAM_SHOW_MASK
            if 'swaprb' in q:
                CAM_SWAP_RB = not CAM_SWAP_RB
                print(f"[비전] CAM_SWAP_RB = {CAM_SWAP_RB}")
            if 'dump' in q:
                print("\n===== 현재 설정 (코드에 반영하세요) =====")
                print(f"CAM_SWAP_RB = {CAM_SWAP_RB}")
                print(f"[노랑/오브젝트]")
                print(f"COLOR_H_MIN, COLOR_H_MAX = {COLOR_H_MIN}, {COLOR_H_MAX}")
                print(f"COLOR_S_MIN, COLOR_S_MAX = {COLOR_S_MIN}, {COLOR_S_MAX}")
                print(f"COLOR_V_MIN, COLOR_V_MAX = {COLOR_V_MIN}, {COLOR_V_MAX}")
                print(f"[초록/선반]")
                print(f"GREEN_H_MIN, GREEN_H_MAX = {GREEN_H_MIN}, {GREEN_H_MAX}")
                print(f"GREEN_S_MIN, GREEN_S_MAX = {GREEN_S_MIN}, {GREEN_S_MAX}")
                print(f"GREEN_V_MIN, GREEN_V_MAX = {GREEN_V_MIN}, {GREEN_V_MAX}")
                obj = get_object()
                shelf = get_shelf()
                if obj.found:
                    print(f"[참고-오브젝트] 폭={obj.width_px:.0f}px "
                          f"-> 30cm 기준 CAM_FOCAL_PX 추천값="
                          f"{(obj.width_px * 30.0) / TARGET_REAL_WIDTH_CM:.0f}")
                if shelf.found:
                    print(f"[참고-선반] 폭={shelf.width_px:.0f}px skew={shelf.skew:+.2f} "
                          f"-> 30cm 기준 CAM_FOCAL_PX 추천값="
                          f"{(shelf.width_px * 30.0) / SHELF_REAL_WIDTH_CM:.0f}")
                print("=======================================\n")

            self.send_response(204)
            self.end_headers()

        elif path == '/stream.mjpg':
            self.send_response(200)
            self.send_header('Age', '0')
            self.send_header('Cache-Control', 'no-cache, private')
            self.send_header('Pragma', 'no-cache')
            self.send_header('Content-Type',
                             'multipart/x-mixed-replace; boundary=FRAME')
            self.end_headers()
            _stream_clients += 1
            try:
                while _app_running:
                    with _stream_lock:
                        buf = _stream_jpeg
                    if buf is None:
                        buf = _make_placeholder(VISION_ERROR or "no frame yet")
                    if buf is None:
                        time.sleep(0.1)
                        continue
                    self.wfile.write(b'--FRAME\r\n')
                    self.wfile.write(b'Content-Type: image/jpeg\r\n')
                    self.wfile.write(
                        ('Content-Length: %d\r\n\r\n' % len(buf)).encode())
                    self.wfile.write(buf)
                    self.wfile.write(b'\r\n')
                    time.sleep(1.0 / STREAM_FPS)
            except (BrokenPipeError, ConnectionResetError):
                pass
            finally:
                _stream_clients -= 1
        else:
            self.send_error(404)


def _get_local_ip():
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(('8.8.8.8', 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except Exception:
        return '127.0.0.1'


def start_stream():
    global _stream_server
    try:
        _stream_server = ThreadingHTTPServer(('0.0.0.0', STREAM_PORT), _StreamHandler)
        _stream_server.daemon_threads = True
        threading.Thread(target=_stream_server.serve_forever, daemon=True).start()
        print(f"[스트리밍] http://{_get_local_ip()}:{STREAM_PORT} 접속하세요")
    except Exception as e:
        print(f"[스트리밍] 시작 실패: {e}")


def stop_stream():
    global _app_running
    _app_running = False
    if _stream_server:
        try:
            _stream_server.shutdown()
        except Exception:
            pass


# =========================================================================
# ===== [비전 추가] 미션 FSM =====
# =========================================================================

MISSION_STATE = SEARCH
_last_seen_time = 0.0
_last_offset = 0.0
_last_skew = 0.0
_detour_start = 0.0
_detour_dir = 1
_hit_count = 0
_detour_repeat_count = 0
MAX_DETOUR_REPEAT = 3


def is_the_shelf(shelf, ultra_cm):
    if not shelf.found or ultra_cm <= 0 or shelf.distance_cm <= 0:
        return False
    if abs(shelf.offset) > 0.8:
        return False
    tolerance = max(10.0, shelf.distance_cm * TARGET_TOLERANCE_RATIO)
    return abs(shelf.distance_cm - ultra_cm) < tolerance


class Command:
    def __init__(self, handled=False, speed=0, steer=0.0,
                 allow_backup=True, state=SEARCH, reason=""):
        self.handled = handled
        self.speed = speed
        self.steer = steer
        self.allow_backup = allow_backup
        self.state = state
        self.reason = reason


def _dock_steer(shelf):
    return max(-STEER_LIMIT, min(STEER_LIMIT,
               shelf.offset * TARGET_STEER_GAIN + shelf.skew * SKEW_STEER_GAIN))


def mission_step(ultra_cm, lidar_min):
    global MISSION_STATE, _last_seen_time, _last_offset, _last_skew
    global _detour_start, _detour_dir, _hit_count, _detour_repeat_count

    if not VISION_ENABLED:
        return Command(handled=False, state=SEARCH, reason="비전 비활성")

    shelf = get_shelf()
    now = time.time()

    if shelf.found and shelf.is_fresh():
        _last_seen_time = now
        _last_offset = shelf.offset
        _last_skew = shelf.skew
        _hit_count = min(_hit_count + 1, CONFIRM_HITS)
    else:
        _hit_count = 0

    if MISSION_STATE == ARRIVED:
        return Command(handled=True, speed=0, steer=0.0, allow_backup=False,
                       state=ARRIVED, reason="선반 도킹 완료 - 정지 유지")

    if MISSION_STATE == SEARCH:
        if _hit_count >= CONFIRM_HITS:
            print(f"[미션] 선반 발견(거리≈{shelf.distance_cm:.0f}cm) -> APPROACH")
            MISSION_STATE = APPROACH
        else:
            return Command(handled=False, state=SEARCH, reason="탐색 중")

    if MISSION_STATE == DETOUR:
        if now - _detour_start > DETOUR_TIME:
            print("[미션] 우회 종료 -> APPROACH 복귀")
            MISSION_STATE = APPROACH
        else:
            if 0 < ultra_cm < 8 or lidar_min < 200:
                return Command(handled=False, state=DETOUR,
                               reason="우회 중 초근접 - 기존 회피로 위임")
            return Command(handled=True, speed=APPROACH_SPEED,
                           steer=_detour_dir * DETOUR_STEER, allow_backup=True,
                           state=DETOUR, reason="장애물 우회 중")

    if MISSION_STATE == DOCK:
        if 0 < ultra_cm <= VERY_CLOSE_CM:
            return Command(handled=False, state=DOCK,
                           reason="도킹 중 초근접 - 기존 탈출로 위임")

        if not (shelf.found and shelf.is_fresh()):
            if now - _last_seen_time > LOST_TIMEOUT:
                # [안전장치 추가] 카메라는 선반을 놓쳤지만, 초음파가 여전히
                # 가까운 거리를 가리키고 있으면 "코앞에 뭔가 있다"는 뜻이므로
                # SEARCH로 풀어주지 않고 일단 정지 유지. (여기서 SEARCH로
                # 풀어주면 라이다도 이 물체를 못 보는 경우 CRUISE로 넘어가서
                # 정상속도로 그냥 지나쳐버리는 문제가 있었음)
                if 0 < ultra_cm <= DOCK_LOST_SAFE_ULTRA_CM:
                    print(f"[미션] 도킹 중 선반 상실했지만 초음파 {ultra_cm:.0f}cm "
                          f"근접 - SEARCH로 안 풀고 정지 유지")
                    return Command(handled=True, speed=0, steer=0.0,
                                   allow_backup=False, state=DOCK,
                                   reason=f"선반 상실 but 근접({ultra_cm:.0f}cm) - 정지 유지")
                print("[미션] 도킹 중 선반 상실 -> SEARCH 복귀")
                MISSION_STATE = SEARCH
                return Command(handled=False, state=SEARCH, reason="선반 상실")
            steer = max(-STEER_LIMIT, min(STEER_LIMIT,
                        _last_offset * TARGET_STEER_GAIN + _last_skew * SKEW_STEER_GAIN))
            return Command(handled=True, speed=DOCK_SPEED, steer=steer,
                           allow_backup=False, state=DOCK,
                           reason="도킹 중 일시 상실 - 관성 크리핑")

        if 0 < ultra_cm <= ARRIVE_DISTANCE_CM and shelf.is_docked():
            print(f"[미션] 선반 {ultra_cm:.0f}cm 도킹 완료 "
                  f"(offset={shelf.offset:+.2f} skew={shelf.skew:+.2f}) -> 정지")
            MISSION_STATE = ARRIVED
            return Command(handled=True, speed=0, steer=0.0, allow_backup=False,
                           state=ARRIVED, reason=f"선반 {ultra_cm:.0f}cm 도킹 완료")

        # [감속 추가] 지금까지는 거리와 무관하게 항상 DOCK_SPEED 고정이라,
        # 정렬이 이미 맞았어도 감속 없이 좁은 도착 판정 구간(ARRIVE_DISTANCE_CM
        # ~VERY_CLOSE_CM, 겨우 2cm 폭)을 그대로 통과해버려서 충돌이 발생했다.
        # DOCK_SLOWDOWN_START_CM 이내로 들어오면 거리에 비례해 속도를 낮추되,
        # 바닥 마찰로 안 움직이는 문제가 재발하지 않도록 최소값은 유지한다.
        dock_speed = DOCK_SPEED
        if 0 < ultra_cm <= DOCK_SLOWDOWN_START_CM:
            ratio = max(0.0, (ultra_cm - ARRIVE_DISTANCE_CM)
                        / (DOCK_SLOWDOWN_START_CM - ARRIVE_DISTANCE_CM))
            dock_speed = DOCK_SLOWDOWN_MIN_SPEED + \
                int((DOCK_SPEED - DOCK_SLOWDOWN_MIN_SPEED) * ratio)

        return Command(handled=True, speed=dock_speed, steer=_dock_steer(shelf),
                       allow_backup=False, state=DOCK,
                       reason=f"도킹 offset={shelf.offset:+.2f} skew={shelf.skew:+.2f} "
                              f"거리={ultra_cm:.0f}cm 속도={dock_speed}")

    if 0 < ultra_cm <= VERY_CLOSE_CM:
        return Command(handled=False, state=APPROACH, reason="접근 중 초근접 - 기존 탈출로 위임")

    if not (shelf.found and shelf.is_fresh()):
        if now - _last_seen_time > LOST_TIMEOUT:
            print("[미션] 선반 상실 -> SEARCH 복귀")
            MISSION_STATE = SEARCH
            return Command(handled=False, state=SEARCH, reason="선반 상실")
        return Command(handled=True, speed=APPROACH_SPEED,
                       steer=_last_offset * TARGET_STEER_GAIN, allow_backup=False,
                       state=APPROACH, reason="일시 상실 - 관성 주행")

    if 0 < ultra_cm <= DOCK_ENTER_DISTANCE_CM and shelf.is_centered(0.3):
        print(f"[미션] 선반 {ultra_cm:.0f}cm 근접 -> DOCK 진입")
        MISSION_STATE = DOCK
        return Command(handled=True, speed=DOCK_SPEED, steer=_dock_steer(shelf),
                       allow_backup=False, state=DOCK, reason="도킹 진입")

    blocked = (0 < ultra_cm < 25) or (0 < lidar_min < STOP_DIST_MM)
    if blocked and not is_the_shelf(shelf, ultra_cm):
        _detour_repeat_count += 1
        if _detour_repeat_count >= MAX_DETOUR_REPEAT:
            print(f"[미션] 우회 {_detour_repeat_count}회 반복 -> 강제 접근 전환")
            _detour_repeat_count = 0
        else:
            _detour_dir = 1 if shelf.offset < 0 else -1
            _detour_start = now
            MISSION_STATE = DETOUR
            print(f"[미션] 경로상 장애물(초음파 {ultra_cm:.0f}cm / 라이다 {lidar_min:.0f}mm) "
                  f"-> {_detour_dir} 방향 우회")
            return Command(handled=True, speed=APPROACH_SPEED,
                           steer=_detour_dir * DETOUR_STEER, allow_backup=True,
                           state=DETOUR, reason="우회 시작")
    else:
        _detour_repeat_count = 0

    steer = max(-STEER_LIMIT, min(STEER_LIMIT, shelf.offset * TARGET_STEER_GAIN))
    speed = max(1, int(VELOCITY * 0.7))
    if 0 < ultra_cm < 30:
        speed = max(18, int(APPROACH_SPEED * 0.5))

    return Command(handled=True, speed=speed, steer=steer, allow_backup=False,
                   state=APPROACH,
                   reason=f"접근 오프셋={shelf.offset:+.2f} "
                          f"카메라≈{shelf.distance_cm:.0f}cm 초음파={ultra_cm:.0f}cm "
                          f"라이다={lidar_min:.0f}mm")


def calibrate_vision():
    start_vision()
    start_stream()
    print("캘리브레이션 모드. 브라우저에서 튜닝하세요. Ctrl+C 종료")
    try:
        while True:
            obj = get_object()
            shelf = get_shelf()
            update_telemetry(state="CALIB", reason="카메라 튜닝 중",
                             ultra_cm=-1.0, lidar_mm=-1.0, speed=0, steer=0.0)
            if not VISION_ENABLED:
                print(f"카메라 비활성: {VISION_ERROR}")
            else:
                if obj.found:
                    suggested = (obj.width_px * 30.0) / TARGET_REAL_WIDTH_CM
                    print(f"[노랑] 폭={obj.width_px:.0f}px 오프셋={obj.offset:+.2f} "
                          f"| 거리≈{obj.distance_cm:.0f}cm "
                          f"| 30cm 기준 CAM_FOCAL_PX 추천값={suggested:.0f}")
                else:
                    print("[노랑] 미검출")
                if shelf.found:
                    suggested = (shelf.width_px * 30.0) / SHELF_REAL_WIDTH_CM
                    print(f"[초록] 폭={shelf.width_px:.0f}px 오프셋={shelf.offset:+.2f} "
                          f"skew={shelf.skew:+.2f} | 거리≈{shelf.distance_cm:.0f}cm "
                          f"| 30cm 기준 CAM_FOCAL_PX 추천값={suggested:.0f}")
                else:
                    print("[초록] 미검출 - 웹 UI에서 '초록(선반)' 선택 후 S/V 하한 조정")
            time.sleep(0.5)
    except KeyboardInterrupt:
        stop_stream()
        stop_vision()
        print("종료")


# =========================================================================
# ===== 기존 라이다 유틸 (로직 변경 없음) =====
# =========================================================================

def normalize_angle(angle):
    angle = angle % 360
    if angle > 180:
        angle -= 360
    return angle


def analyze_scan(scan):
    best_angle = 0
    best_distance = 0
    front_min = 99999
    for quality, angle, distance in scan:
        if distance <= 0:
            continue
        norm = normalize_angle(angle)
        if -90 <= norm <= 90:
            if distance > best_distance:
                best_distance = distance
                best_angle = norm
            if -25 <= norm <= 25 and distance < front_min:
                front_min = distance
    return best_angle, front_min


def get_rear_min_cm(scan, sector_deg=REAR_SECTOR_DEG):
    min_mm = None
    for quality, angle, distance in scan:
        if distance <= 0:
            continue
        norm = normalize_angle(angle)
        if abs(norm) >= (180 - sector_deg):
            if min_mm is None or distance < min_mm:
                min_mm = distance
    if min_mm is None:
        return None
    return min_mm / 10.0


def compute_dynamic_backspeed(rear_cm, base_backspeed):
    if rear_cm is None:
        return base_backspeed
    available_cm = rear_cm - REAR_SAFETY_MARGIN_CM
    if available_cm <= 0:
        return 0
    ratio = min(1.0, available_cm / DEFAULT_BACK_TARGET_CM)
    return max(1, int(base_backspeed * ratio))


def connect_lidar(port, max_retries=5):
    for attempt in range(1, max_retries + 1):
        lidar = None
        try:
            lidar = RPLidar(port)
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
            time.sleep(1.0)
            health = lidar.get_health()
            print(f"[라이다] 연결 성공 (시도 {attempt}/{max_retries}), 상태: {health}")
            return lidar
        except Exception as e:
            print(f"[라이다] 연결 시도 {attempt}/{max_retries} 실패: {e}")
            if lidar is not None:
                try:
                    lidar.disconnect()
                except Exception:
                    pass
            time.sleep(1.5)
    raise RuntimeError("라이다 연결 실패 - 케이블/전원을 확인하세요")


def main():
    reset_mcu()
    time.sleep(0.5)

    global SPEED_FAST
    global SPEED_BACK
    global VELOCITY

    x = Picarx()
    wallBackup = WallBackup(x, Get_Drive_Duty())
    batteryMonitor = BatteryMonitor()
    loadMonitor = SystemLoadMonitor()

    left_motor = Motor(PWM("P13"), Pin("D4"))
    right_motor = Motor(PWM("P12"), Pin("D5"))
    steer = Servo("P2")

    # [로봇팔 추가] arm_setup.build_arm()이 SmoothJoint로 감싸서 4관절 반환
    # (raw Servo가 아니라 SmoothJoint라서 .move_to()로 부드럽게 움직일 수 있음)
    arm_base, arm_shoulder, arm_elbow, arm_grab = build_arm(Servo)
    arm_joints = [arm_base, arm_shoulder, arm_elbow, arm_grab]

    # [로봇팔 슬로우스타터 시험 추가] 터미널을 cbreak 모드로 전환 - Enter 없이
    # 키 하나만 눌러도 즉시 감지됨. 종료 시(finally) 반드시 원래대로 복구해야 함.
    _stdin_fd = sys.stdin.fileno()
    _old_term_settings = termios.tcgetattr(_stdin_fd)
    tty.setcbreak(_stdin_fd)

    sonar = Ultrasonic(Pin("D2"), Pin("D3"))
    lidar = connect_lidar(LIDAR_PORT)

    start_vision()
    try:
        x.set_cam_tilt_angle(CAM_TILT_ANGLE)
        x.set_cam_pan_angle(0)
    except Exception as e:
        print(f"[비전] 카메라 짐벌 제어 실패: {e}")

    # [픽업 추가] visual_servo_pick()에 넘길 프레임 획득 함수
    # lidar_ver5의 grab_frame()을 그대로 재사용 (이미 picam2로 BGR 반환)
    def grab_frame_fn():
        return grab_frame()

    start_stream()
    vs_start_stream()   # [픽업 추가] arm_visual_servo 전용 스트리밍 (8001 포트)

    def set_incre_Move(target):
        global SPEED_FAST
        SPEED_FAST = min(SPEED_FAST + SPEED_STEP, target)
        left_motor.speed(-SPEED_FAST)
        right_motor.speed(SPEED_FAST)
        return SPEED_FAST

    def set_decre_Move(target=0):
        global SPEED_FAST
        SPEED_FAST = max(SPEED_FAST - SPEED_STEP, target)
        left_motor.speed(-SPEED_FAST)
        right_motor.speed(SPEED_FAST)
        if SPEED_FAST == 0:
            left_motor.speed(0)
            right_motor.speed(0)
        return SPEED_FAST

    def set_speed(v):
        global SPEED_FAST
        if SPEED_FAST < v:
            set_incre_Move(v)
        elif SPEED_FAST > v:
            set_decre_Move(v)
        else:
            left_motor.speed(-SPEED_FAST)
            right_motor.speed(SPEED_FAST)
            if SPEED_FAST == 0:
                left_motor.speed(0)
                right_motor.speed(0)
        return SPEED_FAST

    def back_incre_Move(target):
        global SPEED_BACK
        SPEED_BACK = min(SPEED_BACK + SPEED_STEP, target)
        return SPEED_BACK

    def set_steer(angle):
        angle = max(-STEER_LIMIT, min(STEER_LIMIT, angle))
        if GAIN_REVERSE:
            angle = -angle
        steer.angle(angle + 5)

    def read_ultra_cm():
        try:
            d = sonar.read()
            if d is None or d < 0:
                return -1
            return d
        except Exception:
            return -1

    print("라이다+초음파+카메라 주행 시작 (Ctrl+C 종료)")
    print(f"[CPU부하] 코어 {loadMonitor.cores}개 감지, "
          f"경고 기준 {Get_Load_Warn_Threshold():.1f}")
    steer.angle(0)
    time.sleep(1)

    backCnt = 0
    isBack = False
    isBackFlag = False
    # BACK_TARGET = VELOCITY * 0.65
    BACK_TARGET = 40

    backSpeed = 20 * (50 / BACK_TARGET) if BACK_TARGET != 0 else 0
    current_backSpeed = backSpeed
    steel_gain_result = 0

    _arrived_notified = False

    # [반대조향 탈출 추가] 2순위(AVOID)에서 쓴 조향값을 저장해뒀다가,
    # 장애물이 사라진 걸 확인하면 그 반대 방향으로 잠깐 꺾어서
    # 한쪽으로 계속 도는 상황을 방지한다.
    ESCAPE_ULTRA_CM = 10     # 이 거리 이내에 뭔가 있으면 "아직 탈출 안 함"
    ESCAPE_HOLD_SEC = 1.0    # 반대 조향을 유지하는 시간(초)
    _avoid_last_steer = 0.0
    _was_avoiding = False
    _recovery_until = 0.0
    _recovery_steer = 0.0

    try:
      while True:
        try:
            for scan in lidar.iter_scans(min_len=SCAN_MIN_LEN):
                # [로봇팔 슬로우스타터 시험 추가] 'p' 입력 감지 -> y/n 확인 -> 테스트 실행
                key = _check_keypress()
                if key == 'p':
                    print("\n[로봇팔] 슬로우 스타터 시험 코드를 동작하겠습니까? y/n")
                    confirm = sys.stdin.read(1)
                    if confirm.lower() == 'y':
                        run_arm_slow_test(arm_joints)
                    else:
                        print("[로봇팔] 테스트 취소")

                batteryMonitor.show()
                load1 = loadMonitor.show()
                if load1 is not None:
                    update_telemetry(load_1min=float(load1))

                clear_angle, lidar_min = analyze_scan(scan)
                ultra_cm = read_ultra_cm()

                if (isBack and (backCnt <= current_backSpeed)):
                    duty = back_incre_Move(BACK_TARGET)
                    wallBackup.update(duty)
                    update_telemetry(state="BACKUP",
                                     reason=f"후진 {backCnt}/{current_backSpeed}",
                                     ultra_cm=float(ultra_cm), lidar_mm=float(lidar_min),
                                     speed=-duty, steer=0.0)
                    print(f"[후진] duty={duty} ({backCnt}/{current_backSpeed})")
                    backCnt += 1
                    continue
                else:
                    if (isBack):
                        print("후진 종료, 조향 :0")
                        wallBackup.stop(steel_gain_result)
                        SPEED_BACK = 0
                        SPEED_FAST = 0
                        isBackFlag = True
                    isBack = False
                    backCnt = 0

                cmd = mission_step(ultra_cm, lidar_min)

                if cmd.state == ARRIVED:
                    set_decre_Move(0)
                    left_motor.speed(0)
                    right_motor.speed(0)
                    set_steer(0)
                    update_telemetry(state=ARRIVED, reason=cmd.reason,
                                     ultra_cm=float(ultra_cm), lidar_mm=float(lidar_min),
                                     speed=0, steer=0.0)
                    if not _arrived_notified:
                        _arrived_notified = True
                        print(f"★ 미션 완료: {cmd.reason} - 로봇팔 픽업 시작")

                        # [픽업 추가] 카메라를 위로 들어서 팔+타겟이 보이도록
                        try:
                            x.set_cam_tilt_angle(VS_CAM_TILT_ANGLE)
                            print(f"[픽업] 카메라 틸트 {VS_CAM_TILT_ANGLE}도로 변경")
                        except Exception as e:
                            print(f"[픽업] 카메라 틸트 실패({e}) - 현재 각도로 진행")

                        # [픽업 추가] 시각 서보 픽업 실행
                        # visual_servo_pick은 내부에서:
                        #   대기자세 → 하강탐색(빨간마커) → 인터리브 정렬 → 그랩+검증 → 복귀
                        try:
                            pick_success, pick_reason = visual_servo_pick(
                                arm_base, arm_shoulder, arm_elbow, arm_grab, grab_frame_fn
                            )
                            print(f"[픽업] {'성공' if pick_success else '실패'}: {pick_reason}")
                        except Exception as e:
                            print(f"[픽업] 예외 발생: {e}")

                        # 픽업 완료 후 카메라를 원래 주행 각도로 복귀
                        try:
                            x.set_cam_tilt_angle(CAM_TILT_ANGLE)
                        except Exception:
                            pass

                    continue

                if cmd.handled:
                    if not cmd.allow_backup:
                        isBack = False
                        backCnt = 0
                        isBackFlag = False
                    steel_gain_result = cmd.steer
                    set_speed(cmd.speed)
                    set_steer(cmd.steer)
                    update_telemetry(state=cmd.state, reason=cmd.reason,
                                     ultra_cm=float(ultra_cm), lidar_mm=float(lidar_min),
                                     speed=cmd.speed, steer=float(cmd.steer))
                    print(f"[{cmd.state}] {cmd.reason} (목표속도={cmd.speed} 실제속도={SPEED_FAST})")
                    continue

                if lidar_min < STOP_DIST_MM or (0 < ultra_cm < Get_Stop_Distance()):
                    set_decre_Move(0)
                    prev_steer = steel_gain_result

                    if 0 < ultra_cm <= VERY_CLOSE_CM and abs(prev_steer) > STEER_RESET_TOL:
                        steel_gain_result = 0.0
                        set_steer(0)
                        print(f"[탈출] 초근접({ultra_cm:.0f}cm) + 조향 {prev_steer:.0f}도 "
                              f"-> 0도로 리셋 후 직진 후진")
                        escape_reason = "초근접 조향리셋 탈출"
                    else:
                        steel_gain_result = clear_angle * STEER_GAIN
                        set_steer(steel_gain_result)
                        escape_reason = "일반 근접 정지"

                    rear_cm = get_rear_min_cm(scan)
                    current_backSpeed = compute_dynamic_backspeed(rear_cm, backSpeed)

                    if current_backSpeed == 0:
                        print(f"[후진 불가] 후방 {rear_cm}cm 이내 근접 - 후진 취소")
                        isBack = False
                    else:
                        isBack = True
                        backCnt = 0

                    update_telemetry(state="STOP", reason=escape_reason,
                                     ultra_cm=float(ultra_cm), lidar_mm=float(lidar_min),
                                     speed=0, steer=float(steel_gain_result))
                    continue

                if lidar_min < DANGER_DIST_MM or (0 < ultra_cm < Get_Stop_Distance()):
                    set_speed(SPEED_SLOW)

                    # [파란 장애물 추가] 카메라가 파란 장애물을 보고 있으면
                    # 그 위치로 회피 방향을 결정. 라이다가 못 보는 낮은 장애물의
                    # 경우 clear_angle이 신뢰할 수 없어서 카메라를 우선 사용.
                    # 카메라에도 안 보이면 기존 라이다 clear_angle 그대로 사용.
                    obstacle = get_obstacle()
                    if obstacle.found and obstacle.is_fresh():
                        avoid_dir = -1 if obstacle.offset < 0 else 1
                        steel_gain_result = avoid_dir * OBSTACLE_STEER_GAIN
                        avoid_source = f"카메라(파란 장애물 offset={obstacle.offset:+.2f})"
                    else:
                        steel_gain_result = clear_angle * STEER_GAIN
                        avoid_source = "라이다"

                    set_steer(steel_gain_result)
                    # [반대조향 탈출 추가] 지금 쓴 조향값을 저장해두고 "회피 중" 표시
                    _avoid_last_steer = steel_gain_result
                    _was_avoiding = True
                    update_telemetry(state="AVOID", reason="위험거리 회피",
                                     ultra_cm=float(ultra_cm), lidar_mm=float(lidar_min),
                                     speed=SPEED_SLOW, steer=float(steel_gain_result))
                    print(f"[회피] 라이다 {lidar_min:.0f}mm 초음파 {ultra_cm:.0f}cm "
                          f"판단근거={avoid_source} 조향={steel_gain_result:.0f}도")
                    continue

                # [반대조향 탈출 추가] 회피가 끝났으면(초음파 기준 10cm 이내에
                # 아무것도 없으면) 저장해둔 조향값의 반대 방향으로 잠깐 꺾는다.
                now = time.time()
                if _was_avoiding and not (0 < ultra_cm <= ESCAPE_ULTRA_CM):
                    _recovery_steer = -_avoid_last_steer
                    _recovery_until = now + ESCAPE_HOLD_SEC
                    _was_avoiding = False
                    print(f"[탈출] 장애물 벗어남 확인 - 반대 조향 {_recovery_steer:.0f}도로 "
                          f"{ESCAPE_HOLD_SEC:.0f}초간 보정")

                if now < _recovery_until:
                    steer_cmd = _recovery_steer
                    steel_gain_result = steer_cmd
                elif _recovery_until != 0.0:
                    # [반대조향 탈출 추가] 유지시간이 막 끝난 시점 - 다음 판단으로
                    # 넘기지 않고 일단 명시적으로 정방향(0도)으로 리셋한다.
                    steer_cmd = 0
                    steel_gain_result = 0.0
                    _recovery_until = 0.0
                    print("[탈출] 반대 조향 종료 - 바퀴 정방향(0도)으로 복귀")
                elif lidar_min < STEER_ACTIVATE_DIST and abs(clear_angle) >= STEER_DEADZONE:
                    steer_cmd = clear_angle * STEER_GAIN
                    steel_gain_result = steer_cmd
                else:
                    steer_cmd = 0

                if (not isBackFlag):
                    set_steer(steer_cmd)

                if (isBackFlag and steer_cmd == 0):
                    isBackFlag = False

                set_speed(VELOCITY)

                update_telemetry(state="CRUISE", reason="정상 주행",
                                 ultra_cm=float(ultra_cm), lidar_mm=float(lidar_min),
                                 speed=VELOCITY, steer=float(steer_cmd))
                print(f"[주행] 라이다 {lidar_min:.0f}mm 초음파 {ultra_cm:.0f}cm 조향 {steer_cmd:.0f}도")

        except RPLidarException as e:
            overload_note = ""
            if loadMonitor.is_overloaded():
                overload_note = (f" (당시 CPU부하 {loadMonitor.last_load1:.2f} - "
                                  f"과부하 상태였음!)")
            print(f"[라이다] 스캔 중 오류 발생: {e}{overload_note} -> 재연결 시도")
            set_decre_Move(0)
            set_steer(0)
            update_telemetry(state="LIDAR_RECONNECT",
                             reason=f"라이다 재연결 중: {e}",
                             ultra_cm=-1.0, lidar_mm=-1.0, speed=0, steer=0.0)
            try:
                lidar.stop()
                lidar.stop_motor()
                lidar.disconnect()
            except Exception:
                pass
            time.sleep(1.0)
            try:
                lidar = connect_lidar(LIDAR_PORT)
                isBack = False
                backCnt = 0
                isBackFlag = False
                SPEED_FAST = 0
                SPEED_BACK = 0
            except Exception as reconnect_err:
                print(f"[라이다] 재연결 실패: {reconnect_err} - 5초 후 재시도")
                time.sleep(5.0)

    except KeyboardInterrupt:
        print("\n종료 중...")
    finally:
        set_speed(0)
        set_steer(0)
        wallBackup.stop(0)

        # [로봇팔 슬로우스타터 시험 추가] 터미널을 원래 설정으로 복구
        try:
            termios.tcsetattr(_stdin_fd, termios.TCSADRAIN, _old_term_settings)
        except Exception as e:
            print(f"[터미널] 설정 복구 실패: {e}")

        # [로봇팔 추가] 종료 시(정상 종료든 Ctrl+C든) 4관절 전부 천천히 0도로 복귀
        # control_arm.py 종료부와 동일한 패턴: SmoothJoint.move_to()가 부드러운 이동을 담당
        try:
            print("[로봇팔] 종료 - 0도로 천천히 복귀 중...")
            for j in arm_joints:
                j.move_to(0, speed=ARM_HOME_SPEED)
            print("[로봇팔] 복귀 완료")
        except Exception as e:
            print(f"[로봇팔] 종료 시 복귀 실패: {e}")

        stop_stream()
        vs_stop_stream()   # [픽업 추가] arm_visual_servo 스트리밍 종료
        stop_vision()
        lidar.stop()
        lidar.stop_motor()
        lidar.disconnect()
        print(f"[CPU부하] 이번 실행 중 최고 부하: {loadMonitor.max_load1:.2f} "
              f"(경고 기준 {Get_Load_Warn_Threshold():.1f})")
        print("정지 완료")


if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] == "calib":
        calibrate_vision()
    else:
        main()
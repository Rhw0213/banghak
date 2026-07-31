# lidar_ultra_vision.py
# 라이다 + 초음파 + 카메라(노란 오브젝트) 융합 주행 + 웹 스트리밍
#
# 카메라 획득: picamera2 (libcamera 스택). cv2.VideoCapture 사용 안 함.
#
# 실행:
#   python3 lidar_ultra_vision.py            # 본 주행 (스트리밍 동시 동작)
#   python3 lidar_ultra_vision.py calib      # 차량 정지 상태로 카메라 튜닝만
#
# 브라우저에서 http://<라즈베리파이IP>:8000 접속
#
# 변경점 표시:
#   [비전 추가]      - 카메라 인식 / 미션 FSM (단순화 버전)
#   [스트리밍 추가]  - MJPEG 웹 스트리밍 + 실시간 HSV 튜닝 UI
#   [단순화]         - 팬 서보 스윕/추적/FINAL/COOLDOWN 전부 제거.
#                      카메라는 짐벌 정면(0도)에 고정한 채 절대 움직이지 않고,
#                      조향은 오직 화면 오프셋(offset)만으로 결정한다.
#   [재연결]         - RPLidarException 발생 시 자동 재연결 + 반복 재발 시 안전 종료
#   [과부하 감시]    - os.getloadavg() 기반 시스템 부하 모니터 추가 (신규)

import os
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
# VELOCITY = 40
VELOCITY = 0
SPEED_FAST = 0
SPEED_BACK = 0
SPEED_SLOW = 30

SPEED_STEP = 3
SCAN_MIN_LEN = 60

# 초근접 탈출 (좁은 공간에서 조향 꺾인 채 후진하며 갇히는 것 방지)
VERY_CLOSE_CM = 5          # 이 거리 이내는 "낀 상황"으로 간주
STEER_RESET_TOL = 3.0      # 이 각도 이상 꺾여 있으면 0으로 리셋 대상

# ===== [과부하 감시 추가] 시스템 부하(CPU) 설정 =====
LOAD_CHECK_INTERVAL_SEC = 2.0     # 이 간격마다 부하를 읽고 표시
LOAD_WARN_RATIO = 1.0             # 1분 평균 부하가 (코어 수 * 이 값)을 넘으면 경고


# =========================================================================
# ===== [비전 추가] 카메라 / 노란 오브젝트 인식 설정 =====
# =========================================================================

CAM_WIDTH = 320             # 라즈베리파이4에서 라이다와 병행하려면 320x240 권장
CAM_HEIGHT = 240
CAM_FORMAT = "RGB888"       # picamera2에서 이 이름은 실제로 BGR 순서로 나옴(주의)
CAM_TILT_ANGLE = 10        # 카메라 고개 들기
CAM_SWAP_RB = False         # 스트림에서 노란 물체가 파랗게 보이면 웹 UI로 토글

# 노란색 HSV 범위 (OpenCV H는 0~179). 웹 UI 슬라이더로 실시간 변경 가능.
COLOR_H_MIN, COLOR_H_MAX = 20, 35
COLOR_S_MIN, COLOR_S_MAX = 100, 255
COLOR_V_MIN, COLOR_V_MAX = 100, 255

# 파란색 코드
# COLOR_H_MIN, COLOR_H_MAX = 100, 125
# COLOR_S_MIN, COLOR_S_MAX = 60, 255
# COLOR_V_MIN, COLOR_V_MAX = 40, 255

MIN_TARGET_AREA = 400

# 거리 추정 (핀홀 모델): 거리cm = (실제폭cm * focal_px) / 화면폭px
TARGET_REAL_WIDTH_CM = 6.5  # ★ 본인 오브젝트 실제 가로폭으로 수정
CAM_FOCAL_PX = 528.0        # ★ 반드시 캘리브레이션 필요 (해상도 바꾸면 재측정)

ARRIVE_DISTANCE_CM = 10     # 이 거리 + 화면 중앙 정렬되면 도착 처리
ARRIVE_CENTER_TOL = 0.15    # 도착 판정용 중앙 정렬 허용 오프셋


# ===== [로봇팔 추가] 픽업 서보 설정 =====
ARM_HOME_SHOULDER = 0
ARM_HOME_ELBOW = 0
ARM_GRAB_OPEN = 0

ARM_PICK_SHOULDER = -40   # ★ arm_calib.py로 실측 후 교체
ARM_PICK_ELBOW = 20       # ★ arm_calib.py로 실측 후 교체
ARM_GRAB_CLOSE = 0       # ★ arm_calib.py로 실측 후 교체

ARM_MOVE_DELAY = 0.5

ARM_STEP_DEG = 2         # 한 번에 움직이는 각도 (작을수록 부드러움)
ARM_STEP_DELAY = 0.02    # 각 스텝 사이 대기시간(초) - 작을수록 빠름

APPROACH_SPEED = 20
TARGET_STEER_GAIN = 30.0    # 화면 오프셋(-1~+1) -> 조향각 변환 계수 (핵심)
LOST_TIMEOUT = 2.0
DETOUR_TIME = 1.2
DETOUR_STEER = 25.0
CONFIRM_HITS = 3
TARGET_TOLERANCE_RATIO = 0.35   # 목표물/장애물 판정 오차 허용 계수

# 미션 상태 (단순화: SEARCH -> APPROACH -> DETOUR -> ARRIVED 4개뿐)
SEARCH = "SEARCH"
APPROACH = "APPROACH"
DETOUR = "DETOUR"
ARRIVED = "ARRIVED"


class TargetInfo:
    def __init__(self, found=False, offset=0.0, distance_cm=-1.0,
                 area=0.0, width_px=0.0, box=None, ts=0.0):
        self.found = found
        self.offset = offset            # -1.0(맨왼쪽) ~ +1.0(맨오른쪽)
        self.distance_cm = distance_cm
        self.area = area
        self.width_px = width_px
        self.box = box                  # (x, y, w, h) - 스트리밍 오버레이용
        self.ts = ts

    def is_fresh(self, max_age=0.5):
        return (time.time() - self.ts) < max_age

    def is_centered(self, tol=0.15):
        return self.found and abs(self.offset) < tol


# ---- 카메라 스레드 공유 상태 ----
_vision_lock = threading.Lock()
_vision_result = TargetInfo()
_vision_running = False
_vision_thread = None
_picam2 = None
VISION_ENABLED = False
VISION_ERROR = ""           # 실패 사유 (웹 화면에 표시)


def get_hsv_range():
    lower = np.array([COLOR_H_MIN, COLOR_S_MIN, COLOR_V_MIN])
    upper = np.array([COLOR_H_MAX, COLOR_S_MAX, COLOR_V_MAX])
    return lower, upper


def grab_frame():
    """
    picamera2에서 프레임을 받아 OpenCV용 BGR 3채널로 정규화해서 반환.
    실패하면 None.
    """
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


def detect_yellow(frame):
    """프레임에서 가장 큰 노란 덩어리를 찾아 (TargetInfo, mask) 반환"""
    h, w = frame.shape[:2]
    lower, upper = get_hsv_range()

    blurred = cv2.GaussianBlur(frame, (5, 5), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower, upper)

    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)    # 점 노이즈 제거
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)   # 구멍 메우기

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return TargetInfo(found=False, ts=time.time()), mask

    largest = max(contours, key=cv2.contourArea)
    area = cv2.contourArea(largest)
    if area < MIN_TARGET_AREA:
        return TargetInfo(found=False, ts=time.time()), mask

    bx, by, bw, bh = cv2.boundingRect(largest)
    cx = bx + bw / 2.0
    offset = (cx - w / 2.0) / (w / 2.0)
    distance = (TARGET_REAL_WIDTH_CM * CAM_FOCAL_PX) / bw if bw > 0 else -1.0

    info = TargetInfo(found=True, offset=offset, distance_cm=distance,
                      area=area, width_px=float(bw), box=(bx, by, bw, bh),
                      ts=time.time())
    return info, mask


def _vision_loop():
    global _vision_result
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

        info, mask = detect_yellow(frame)
        with _vision_lock:
            _vision_result = info

        # ===== [스트리밍 추가] 보는 사람이 있을 때만 인코딩 (CPU 절약) =====
        now = time.time()
        if _stream_clients > 0 and (now - last_stream) >= (1.0 / STREAM_FPS):
            last_stream = now
            _publish_frame(frame, mask, info)

        time.sleep(0.01)


def start_vision():
    global _picam2, _vision_running, _vision_thread, VISION_ENABLED, VISION_ERROR
    try:
        _picam2 = Picamera2()
        config = _picam2.create_preview_configuration(
            main={"size": (CAM_WIDTH, CAM_HEIGHT), "format": CAM_FORMAT})
        _picam2.configure(config)
        _picam2.start()
        time.sleep(1.0)     # 센서 안정화. 이거 없으면 첫 프레임이 검게 나온다

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


def get_target():
    with _vision_lock:
        return _vision_result


# =========================================================================
# ===== [과부하 감시 추가] 시스템 부하(CPU) 모니터 =====
#   BatteryMonitor와 완전히 같은 패턴: interval마다만 실제로 읽고 출력해서
#   메인 루프에 부담을 주지 않는다.
#
#   os.getloadavg()는 "CPU 사용률(%)"이 아니라 "실행 대기 중인 프로세스
#   평균 개수(1분 이동평균)"다. 코어 수보다 이 값이 크면 CPU가 감당 못 하는
#   작업이 쌓이고 있다는 뜻으로 해석한다 (예: 4코어 기준 4.0 초과 시 과부하 의심).
# =========================================================================

def Get_Load_Warn_Threshold():
    """이 부하(1분 평균) 이상이면 '과부하' 경고를 표시할 기준값"""
    cores = os.cpu_count() or 1
    return cores * LOAD_WARN_RATIO


class SystemLoadMonitor:
    """
    주행 중 CPU 부하(load average)를 주기적으로 읽어서 표시하는 부품.
    BatteryMonitor와 동일한 사용 패턴: 메인 루프에서 show()를 반복 호출하면,
    정해진 간격마다만 실제로 읽고 출력한다.
    """

    def __init__(self, interval=LOAD_CHECK_INTERVAL_SEC):
        self.interval = interval
        self.last_time = 0
        self.last_load1 = None
        self.max_load1 = 0.0
        self.cores = os.cpu_count() or 1

    def read(self):
        """지금 1분 평균 부하를 읽어서 반환. 실패 시 None."""
        try:
            load1, load5, load15 = os.getloadavg()
            if load1 > self.max_load1:
                self.max_load1 = load1
            self.last_load1 = load1
            return load1
        except Exception:
            return None

    def show(self):
        """
        interval이 지났을 때만 부하를 읽고 한 줄 출력.
        반환값: 방금 표시했으면 load1(float), 아니면 None
        """
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
        """지금까지 마지막으로 읽은 값 기준, 과부하 상태인지 여부."""
        if self.last_load1 is None:
            return False
        return self.last_load1 > Get_Load_Warn_Threshold()


# =========================================================================
# ===== [스트리밍 추가] MJPEG 웹 서버 + 실시간 HSV 튜닝 UI =====
# =========================================================================

STREAM_PORT = 8000
STREAM_FPS = 10             # 높이면 주행 루프가 느려짐
STREAM_QUALITY = 60         # JPEG 품질 (낮출수록 가벼움)
STREAM_SHOW_MASK = False    # True면 원본 대신 마스크(흑백) 전송 - HSV 튜닝용

_stream_lock = threading.Lock()
_stream_jpeg = None
_stream_clients = 0         # 접속자 수. 0이면 인코딩 자체를 건너뜀
_stream_server = None
_app_running = True         # 종료 시 스트림 루프를 빠져나오기 위한 플래그

# 메인 루프가 갱신하는 텔레메트리 (웹 상태창에 표시)
_telemetry = {
    "state": SEARCH,
    "reason": "-",
    "ultra_cm": -1.0,
    "lidar_mm": -1.0,
    "speed": 0,
    "steer": 0.0,
    "load_1min": 0.0,          # ===== [과부하 감시 추가] =====
}
_telemetry_lock = threading.Lock()


def update_telemetry(**kwargs):
    with _telemetry_lock:
        _telemetry.update(kwargs)


def _make_placeholder(text):
    """카메라가 없을 때 보여줄 안내 프레임 JPEG 생성"""
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


def _draw_overlay(frame, info):
    """검출 결과를 프레임에 그려서 반환"""
    h, w = frame.shape[:2]
    out = frame.copy()

    # 화면 중앙선 (조향 기준선)
    cv2.line(out, (w // 2, 0), (w // 2, h), (200, 200, 200), 1)

    if info.found and info.box:
        bx, by, bw, bh = info.box
        cv2.rectangle(out, (bx, by), (bx + bw, by + bh), (0, 255, 0), 2)
        cx = int(bx + bw / 2)
        cy = int(by + bh / 2)
        cv2.circle(out, (cx, cy), 4, (0, 0, 255), -1)
        cv2.line(out, (w // 2, cy), (cx, cy), (0, 0, 255), 1)
        cv2.putText(out, f"{info.distance_cm:.0f}cm", (bx, max(15, by - 6)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

    with _telemetry_lock:
        state = _telemetry["state"]
        ultra = _telemetry["ultra_cm"]

    if info.found:
        txt = f"{state} off={info.offset:+.2f} ultra={ultra:.0f}cm"
    else:
        txt = f"{state} no target ultra={ultra:.0f}cm"
    cv2.putText(out, txt, (5, h - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                (0, 255, 255), 1)
    return out


def _publish_frame(frame, mask, info):
    """카메라 스레드가 호출. 최신 JPEG을 스트리밍 버퍼에 넣는다."""
    global _stream_jpeg
    if STREAM_SHOW_MASK:
        img = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
    else:
        img = _draw_overlay(frame, info)

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
<h3>HSV 범위 (목표물 인식 튜닝)</h3>
<div id="sliders"></div>
<script>
const P=[["h_min",179],["h_max",179],["s_min",255],["s_max",255],["v_min",255],["v_max",255]];
const box=document.getElementById('sliders');
P.forEach(([k,max])=>{
  const d=document.createElement('div');d.className='row';
  d.innerHTML=`<label>${k}</label><input type=range min=0 max=${max} id="${k}">
               <span id="${k}v"></span>`;
  box.appendChild(d);
});
function send(){
  const q=P.map(([k])=>k+'='+document.getElementById(k).value).join('&');
  fetch('/set?'+q);
  P.forEach(([k])=>document.getElementById(k+'v').textContent=
      document.getElementById(k).value);
}
P.forEach(([k])=>document.getElementById(k).addEventListener('input',send));
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
      `목표물: ${s.found?('발견  오프셋 '+s.offset.toFixed(2)+
        '  카메라거리 '+s.distance_cm.toFixed(0)+'cm  폭 '+s.width_px.toFixed(0)+'px')
        :'없음'}`;
    if(!window._init){
      window._init=true;
      P.forEach(([k])=>{document.getElementById(k).value=s.hsv[k];
                        document.getElementById(k+'v').textContent=s.hsv[k];});
    }
  }catch(e){}
}
setInterval(poll,400);poll();
</script></body></html>"""


class _StreamHandler(BaseHTTPRequestHandler):
    def log_message(self, *args):
        pass    # 요청 로그로 터미널이 도배되는 것 방지

    def do_GET(self):
        global _stream_clients, STREAM_SHOW_MASK, CAM_SWAP_RB
        global COLOR_H_MIN, COLOR_H_MAX, COLOR_S_MIN
        global COLOR_S_MAX, COLOR_V_MIN, COLOR_V_MAX

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
            t = get_target()
            with _telemetry_lock:
                data = dict(_telemetry)
            data.update({
                "cam_ok": VISION_ENABLED,
                "cam_err": VISION_ERROR,
                "swap_rb": CAM_SWAP_RB,
                "found": bool(t.found and t.is_fresh()),
                "offset": t.offset,
                "distance_cm": t.distance_cm,
                "width_px": t.width_px,
                "hsv": {
                    "h_min": COLOR_H_MIN, "h_max": COLOR_H_MAX,
                    "s_min": COLOR_S_MIN, "s_max": COLOR_S_MAX,
                    "v_min": COLOR_V_MIN, "v_max": COLOR_V_MAX,
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

            COLOR_H_MIN = gv('h_min', COLOR_H_MIN)
            COLOR_H_MAX = gv('h_max', COLOR_H_MAX)
            COLOR_S_MIN = gv('s_min', COLOR_S_MIN)
            COLOR_S_MAX = gv('s_max', COLOR_S_MAX)
            COLOR_V_MIN = gv('v_min', COLOR_V_MIN)
            COLOR_V_MAX = gv('v_max', COLOR_V_MAX)

            if 'mask' in q:
                STREAM_SHOW_MASK = not STREAM_SHOW_MASK
            if 'swaprb' in q:
                CAM_SWAP_RB = not CAM_SWAP_RB
                print(f"[비전] CAM_SWAP_RB = {CAM_SWAP_RB}")
            if 'dump' in q:
                print("\n===== 현재 설정 (코드에 반영하세요) =====")
                print(f"CAM_SWAP_RB = {CAM_SWAP_RB}")
                print(f"COLOR_H_MIN, COLOR_H_MAX = {COLOR_H_MIN}, {COLOR_H_MAX}")
                print(f"COLOR_S_MIN, COLOR_S_MAX = {COLOR_S_MIN}, {COLOR_S_MAX}")
                print(f"COLOR_V_MIN, COLOR_V_MAX = {COLOR_V_MIN}, {COLOR_V_MAX}")
                t = get_target()
                if t.found:
                    print(f"[참고] 지금 폭={t.width_px:.0f}px "
                          f"-> 30cm 기준 CAM_FOCAL_PX 추천값="
                          f"{(t.width_px * 30.0) / TARGET_REAL_WIDTH_CM:.0f}")
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
                        # 카메라가 죽었어도 안내 화면은 계속 보내준다
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
                pass    # 브라우저 탭 닫힘 - 정상
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
# ===== [비전 추가] 미션 FSM (단순화 버전) =====
#   SEARCH -> APPROACH -> (필요시 DETOUR로 잠깐 우회) -> ARRIVED
#   카메라 팬은 항상 정면 고정 (스윕/추적 없음). 조향은 오직 화면
#   오프셋(offset)으로만 결정된다. "정면 정렬"은 별도 단계가 아니라,
#   접근하는 동안 offset이 0에 가까워지는 되먹임 결과로 자연히 달성됨.
# =========================================================================

MISSION_STATE = SEARCH
_last_seen_time = 0.0
_last_offset = 0.0
_detour_start = 0.0
_detour_dir = 1
_hit_count = 0
# 0730 - 12:30 / 추가 
_detour_repeat_count = 0   # ★ 추가

def is_the_target(target, ultra_cm):
    """
    전방 물체가 목표물인지 장애물인지 판정.
    카메라 추정거리와 초음파 거리가 비슷하면 목표물, 크게 다르면 앞에 낀 장애물.
    """
    if not target.found or not target.is_centered(0.15):
        return False
    if ultra_cm <= 0 or target.distance_cm <= 0:
        return False
    # 0730 - 12:26 / 추가
    # ★ 중앙 정렬 조건을 넓게 완화 (0.15 -> 0.8): 화면 안에만 있으면 판정 대상
    if abs(target.offset) > 0.8:
        return False
    tolerance = max(10.0, target.distance_cm * TARGET_TOLERANCE_RATIO)
    return abs(target.distance_cm - ultra_cm) < tolerance    

    # 0730 - 12:26 / 원본 
    # tolerance = max(8.0, target.distance_cm * TARGET_TOLERANCE_RATIO)
    # return abs(target.distance_cm - ultra_cm) < tolerance


class Command:
    """
    handled=False -> 기존 라이다 회피 로직 그대로 실행
    handled=True  -> speed/steer 적용. allow_backup=False면 후진 금지
    """
    def __init__(self, handled=False, speed=0, steer=0.0,
                 allow_backup=True, state=SEARCH, reason=""):
        self.handled = handled
        self.speed = speed
        self.steer = steer
        self.allow_backup = allow_backup
        self.state = state
        self.reason = reason

#0730 - 12:28 / 추가
_detour_repeat_count = 0
MAX_DETOUR_REPEAT = 3

def mission_step(ultra_cm, lidar_min):
    # 7030 - 12:31 / 원본
    # global MISSION_STATE, _last_seen_time, _last_offset
    # global _detour_start, _detour_dir, _hit_count

    # 7030 - 12:31 / 추가
    global MISSION_STATE, _last_seen_time, _last_offset
    global _detour_start, _detour_dir, _hit_count, _detour_repeat_count

    if not VISION_ENABLED:
        return Command(handled=False, state=SEARCH, reason="비전 비활성")

    target = get_target()
    now = time.time()

    if target.found and target.is_fresh():
        _last_seen_time = now
        _last_offset = target.offset
        _hit_count = min(_hit_count + 1, CONFIRM_HITS)
    else:
        _hit_count = 0

    # ---------- ARRIVED ----------
    if MISSION_STATE == ARRIVED:
        # 로봇팔이 잡는 동안 정지 유지 (재탐색 없음 - 필요시 프로그램 재실행)
        return Command(handled=True, speed=0, steer=0.0, allow_backup=False,
                       state=ARRIVED, reason="목표물 도착 - 정지 유지")

    # ---------- SEARCH ----------
    # 카메라 스윕 없음 - 짐벌은 main()에서 최초 1회 0도로 고정된 채 유지된다.
    if MISSION_STATE == SEARCH:
        if _hit_count >= CONFIRM_HITS:
            print(f"[미션] 목표물 발견(거리≈{target.distance_cm:.0f}cm) -> APPROACH")
            MISSION_STATE = APPROACH
        else:
            return Command(handled=False, state=SEARCH, reason="탐색 중")

    # ---------- DETOUR ----------
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

    # ---------- APPROACH ----------
    # 안전핀: 초근접이면 미션 로직이 뭘 하려 했든 기존 탈출(정지+후진) 로직에 위임한다.
    # (미션 판단으로 handled=True를 계속 반환하면 main()의 1순위 정지/탈출 블록에
    #  아예 도달하지 못해 좁은 공간에 낄 수 있음)
    if 0 < ultra_cm <= VERY_CLOSE_CM:
        return Command(handled=False, state=APPROACH, reason="접근 중 초근접 - 기존 탈출로 위임")

    if not (target.found and target.is_fresh()):
        if 0 < ultra_cm <= ARRIVE_DISTANCE_CM:
            print(f"[미션] 근접 상실 + 초음파 {ultra_cm:.0f}cm -> 도착 처리")
            MISSION_STATE = ARRIVED
            return Command(handled=True, speed=0, steer=0.0, allow_backup=False,
                           state=ARRIVED, reason="도착")
        if now - _last_seen_time > LOST_TIMEOUT:
            print("[미션] 목표물 상실 -> SEARCH 복귀")
            MISSION_STATE = SEARCH
            return Command(handled=False, state=SEARCH, reason="목표물 상실")
        # 잠깐 놓친 경우: 마지막 오프셋으로 관성 주행
        return Command(handled=True, speed=APPROACH_SPEED,
                       steer=_last_offset * TARGET_STEER_GAIN, allow_backup=False,
                       state=APPROACH, reason="일시 상실 - 관성 주행")

    # 1순위: 도착 판정 (거리 + 화면 중앙 정렬)
    if 0 < ultra_cm <= ARRIVE_DISTANCE_CM and target.is_centered(ARRIVE_CENTER_TOL):
        print(f"[미션] 목표물 {ultra_cm:.0f}cm 도달 -> 정지")
        MISSION_STATE = ARRIVED
        return Command(handled=True, speed=0, steer=0.0, allow_backup=False,
                       state=ARRIVED, reason=f"목표물 {ultra_cm:.0f}cm 도착")

    # 2순위: 앞을 막은 게 목표물이 아니면 우회
    blocked = (0 < ultra_cm < 25) or (0 < lidar_min < STOP_DIST_MM)
    if blocked and not is_the_target(target, ultra_cm):
        _detour_repeat_count += 1
        if _detour_repeat_count >= MAX_DETOUR_REPEAT:
            # 같은 자리에서 우회만 반복 중 -> 오판 가능성 높음. 강제 접근 시도
            print(f"[미션] 우회 {_detour_repeat_count}회 반복 -> 강제 접근 전환")
            _detour_repeat_count = 0
        else:
            _detour_dir = 1 if target.offset < 0 else -1
            _detour_start = now
            MISSION_STATE = DETOUR
            print(f"[미션] 경로상 장애물(초음파 {ultra_cm:.0f}cm / 라이다 {lidar_min:.0f}mm) "
                  f"-> {_detour_dir} 방향 우회")
            return Command(handled=True, speed=APPROACH_SPEED,
                           steer=_detour_dir * DETOUR_STEER, allow_backup=True,
                           state=DETOUR, reason="우회 시작")
    else:
        _detour_repeat_count = 0   # 정상 접근 중이면 카운터 리셋


    # 0730 - 12:28 / 원본
    # blocked = (0 < ultra_cm < 25) or (0 < lidar_min < STOP_DIST_MM)
    # if blocked and not is_the_target(target, ultra_cm):
    #     _detour_dir = 1 if target.offset < 0 else -1
    #     _detour_start = now
    #     MISSION_STATE = DETOUR
    #     print(f"[미션] 경로상 장애물(초음파 {ultra_cm:.0f}cm / 라이다 {lidar_min:.0f}mm) "
    #           f"-> {_detour_dir} 방향 우회")
    #     return Command(handled=True, speed=APPROACH_SPEED,
    #                    steer=_detour_dir * DETOUR_STEER, allow_backup=True,
    #                    state=DETOUR, reason="우회 시작")

    # 3순위: 정상 접근 - 화면 오프셋만으로 조향 (정면 정렬은 이 되먹임의 결과)
    # 0730 - 12:16 / 원본
    # steer = max(-STEER_LIMIT, min(STEER_LIMIT, target.offset * TARGET_STEER_GAIN))
    # speed = APPROACH_SPEED
    # if 0 < ultra_cm < 30:
    #     speed = max(12, int(APPROACH_SPEED * 0.6))

    # # 0730 - 12:16 / 추가
    # 3순위: 정상 접근 - 목표물 발견 시 속도 절반으로 감속 + 화면 오프셋으로 조향
    steer = max(-STEER_LIMIT, min(STEER_LIMIT, target.offset * TARGET_STEER_GAIN))
    speed = max(1, int(VELOCITY * 0.7))   # ★ 목표물 발견 -> 항상 절반 속도
    if 0 < ultra_cm < 30:
        speed = max(18, int(APPROACH_SPEED * 0.5))  # 더 가까워지면 한 번 더 감속(선택)

    return Command(handled=True, speed=speed, steer=steer, allow_backup=False,
                   state=APPROACH,
                   reason=f"접근 오프셋={target.offset:+.2f} "
                          f"카메라≈{target.distance_cm:.0f}cm 초음파={ultra_cm:.0f}cm")


def calibrate_vision():
    """
    차량을 움직이지 않고 카메라만 튜닝하는 모드.
      python3 lidar_ultra_vision.py calib
    """
    start_vision()
    start_stream()      # 카메라가 실패해도 안내 화면을 보여주기 위해 항상 시작
    print("캘리브레이션 모드. 브라우저에서 튜닝하세요. Ctrl+C 종료")
    try:
        while True:
            t = get_target()
            update_telemetry(state="CALIB", reason="카메라 튜닝 중",
                             ultra_cm=-1.0, lidar_mm=-1.0, speed=0, steer=0.0)
            if not VISION_ENABLED:
                print(f"카메라 비활성: {VISION_ERROR}")
            elif t.found:
                suggested = (t.width_px * 30.0) / TARGET_REAL_WIDTH_CM
                print(f"폭={t.width_px:.0f}px 면적={t.area:.0f} 오프셋={t.offset:+.2f} "
                      f"| 현재설정 거리≈{t.distance_cm:.0f}cm "
                      f"| 30cm 기준 CAM_FOCAL_PX 추천값={suggested:.0f}")
            else:
                print("목표물 미검출 - 브라우저에서 S/V 하한을 낮추거나 R/B 반전 확인")
            time.sleep(0.5)
    except KeyboardInterrupt:
        stop_stream()
        stop_vision()
        print("종료")

# 로봇팔 픽업 함수
def pick_up_target(shoulder_servo, elbow_servo, grab_servo):
    """ARRIVED 상태에서 한 번 호출: 그립 열고 -> 팔 내리기 -> 그립 닫기 -> 팔 복귀 (부드럽게)"""
    print("[로봇팔] 픽업 시퀀스 시작")

    smooth_servo_move(grab_servo, ARM_GRAB_OPEN, ARM_GRAB_CLOSE)
    time.sleep(ARM_MOVE_DELAY)

    smooth_servo_move(shoulder_servo, ARM_PICK_SHOULDER, ARM_HOME_SHOULDER)
    smooth_servo_move(elbow_servo, ARM_PICK_ELBOW, ARM_HOME_ELBOW)
    time.sleep(ARM_MOVE_DELAY)

    smooth_servo_move(grab_servo, ARM_GRAB_CLOSE, ARM_GRAB_OPEN)
    time.sleep(ARM_MOVE_DELAY)

    smooth_servo_move(shoulder_servo, ARM_HOME_SHOULDER, ARM_PICK_SHOULDER)
    smooth_servo_move(elbow_servo, ARM_HOME_ELBOW, ARM_PICK_ELBOW)
    time.sleep(ARM_MOVE_DELAY)

    print("[로봇팔] 픽업 시퀀스 완료")

# 팔 움직임 부드럽게
def smooth_servo_move(servo, target_angle, current_angle,
                       step_deg=ARM_STEP_DEG, step_delay=ARM_STEP_DELAY):
    """
    서보를 current_angle에서 target_angle까지 조금씩 나눠서 이동.
    끝나면 실제로 도달한 각도(target_angle)를 반환 -> 다음 호출의 current_angle로 사용
    """
    if current_angle < target_angle:
        angle = current_angle
        while angle < target_angle:
            angle = min(angle + step_deg, target_angle)
            servo.angle(angle)
            time.sleep(step_delay)
    else:
        angle = current_angle
        while angle > target_angle:
            angle = max(angle - step_deg, target_angle)
            servo.angle(angle)
            time.sleep(step_delay)
    return target_angle

# =========================================================================
# ===== 기존 라이다 유틸 (로직 변경 없음) =====
# =========================================================================

def normalize_angle(angle):
    angle = angle % 360
    if angle > 180:
        angle -= 360
    return angle


def analyze_scan(scan):
    """가장 트인 방향 + 정면 최소거리(mm)"""
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
    """후방(±sector_deg) 섹터 내 최소거리(cm). 감지 없으면 None"""
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
    """후방 여유거리에 비례해 backSpeed 계산. 0이면 후진 금지."""
    if rear_cm is None:
        return base_backspeed
    available_cm = rear_cm - REAR_SAFETY_MARGIN_CM
    if available_cm <= 0:
        return 0
    ratio = min(1.0, available_cm / DEFAULT_BACK_TARGET_CM)
    return max(1, int(base_backspeed * ratio))


def connect_lidar(port, max_retries=5):
    """
    라이다 연결 + 리셋. 시리얼 스트림이 꼬여있을 수 있으므로
    stop/reset 후 약간의 대기시간을 두고 health를 확인한다.
    """
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
                lidar.reset()   # 내부 버퍼/상태 리셋
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
    loadMonitor = SystemLoadMonitor()          # ===== [과부하 감시 추가] =====

    left_motor = Motor(PWM("P13"), Pin("D4"))
    right_motor = Motor(PWM("P12"), Pin("D5"))
    steer = Servo("P2")

    # 로봇팔 핀번호 
    arm_shoulder = Servo("P5")
    arm_elbow = Servo("P6")
    arm_grab = Servo("P7")
    arm_shoulder.angle(ARM_HOME_SHOULDER)
    arm_elbow.angle(ARM_HOME_ELBOW)
    arm_grab.angle(ARM_GRAB_OPEN)


    sonar = Ultrasonic(Pin("D2"), Pin("D3"))
    lidar = connect_lidar(LIDAR_PORT)

    # ===== [비전 추가] 카메라 시작 + 짐벌 각도 =====
    # [단순화] 팬은 최초 1회 0도(정면)로 고정하고 이후 절대 움직이지 않는다.
    #          (기존 버전의 스윕/추적 관련 서보 제어 코드는 전부 제거됨)
    start_vision()
    try:
        x.set_cam_tilt_angle(CAM_TILT_ANGLE)
        x.set_cam_pan_angle(0)
    except Exception as e:
        print(f"[비전] 카메라 짐벌 제어 실패: {e}")

    # ===== [스트리밍 추가] 웹 서버 시작 =====
    start_stream()

    # ---------- 전진 전용 (robot_hat) ----------
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

    # ---------- 후진 전용 (picarx / WallBackup) ----------
    def back_incre_Move(target):
        global SPEED_BACK
        SPEED_BACK = min(SPEED_BACK + SPEED_STEP, target)
        return SPEED_BACK

    def set_steer(angle):
        angle = max(-STEER_LIMIT, min(STEER_LIMIT, angle))
        if GAIN_REVERSE:
            angle = -angle
        steer.angle(angle + 5)   # 5도 오른쪽 offset

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
          f"경고 기준 {Get_Load_Warn_Threshold():.1f}")   # ===== [과부하 감시 추가] =====
    steer.angle(0)
    time.sleep(1)

    backCnt = 0
    isBack = False
    isBackFlag = False
    BACK_TARGET = VELOCITY * 0.65

    # backSpeed = 20 * (50 / BACK_TARGET)
    backSpeed = 20 * (50 / BACK_TARGET) if BACK_TARGET != 0 else 0
    current_backSpeed = backSpeed
    steel_gain_result = 0

    # 0730 - 1:47 / 추가    
    # mission_done = False
    # try:
    #   while not mission_done:
    _arrived_notified = False

    try:
      while True:
        # ===== 라이다 재연결 루프 =====
        # 시리얼 스트림이 깨지면(RPLidarException) 여기서 잡아서
        # 라이다를 재연결하고 스캔을 이어간다. 프로그램 자체는 죽지 않는다.
        try:
            for scan in lidar.iter_scans(min_len=SCAN_MIN_LEN):
                batteryMonitor.show()
                # ===== [과부하 감시 추가] =====
                # BatteryMonitor와 동일한 패턴: interval마다만 실제로 읽고
                # 출력하므로, 매 스캔마다 호출해도 루프에 부담 없음.
                load1 = loadMonitor.show()
                if load1 is not None:
                    update_telemetry(load_1min=float(load1))
                # ===== [과부하 감시 추가] 끝 =====

                clear_angle, lidar_min = analyze_scan(scan)   # mm
                ultra_cm = read_ultra_cm()                    # cm (-1이면 실패)

                # ===== 후진 구간: picarx(WallBackup) 경로만 사용 =====
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

                # =================================================================
                # ===== [비전 추가] 미션 판단 : 반드시 1순위 정지 블록보다 위 =====
                #   노란 오브젝트는 라이다에도 잡히므로, 30cm(STOP_DIST_MM)에서
                #   기존 1순위가 먼저 발동하면 목표물 앞에서 후진해버림
                #   (단, ultra_cm<=VERY_CLOSE_CM 일 땐 mission_step이 스스로
                #    handled=False를 반환해 아래 탈출 로직으로 위임한다)
                # =================================================================
                cmd = mission_step(ultra_cm, lidar_min)
        
                if cmd.state == ARRIVED:
                    set_decre_Move(0)
                    left_motor.speed(0)
                    right_motor.speed(0)
                    set_steer(0)
                    update_telemetry(state=ARRIVED, reason=cmd.reason,
                                     ultra_cm=float(ultra_cm), lidar_mm=float(lidar_min),
                                     speed=0, steer=0.0)
                    # if not _arrived_notified:
                    #     print(f"★ 미션 완료: {cmd.reason} - 모터 정지 후 대기 중 (Ctrl+C로 종료)")
                    #     _arrived_notified = True
                    # continue
                    if not _arrived_notified:
                        _arrived_notified = True
                        print(f"★ 미션 완료: {cmd.reason} - 로봇팔 픽업 시작")
                        pick_up_target(arm_shoulder, arm_elbow, arm_grab)
                        print("★★★ 픽업 완료 - 대기 중 (Ctrl+C로 종료) ★★★")
                    continue


                if cmd.handled:
                    if not cmd.allow_backup:
                        # 접근 중에는 후진 상태를 강제로 눌러둔다
                        isBack = False
                        backCnt = 0
                        isBackFlag = False
                    steel_gain_result = cmd.steer
                    set_speed(cmd.speed)
                    set_steer(cmd.steer)
                    update_telemetry(state=cmd.state, reason=cmd.reason,
                                     ultra_cm=float(ultra_cm), lidar_mm=float(lidar_min),
                                     speed=cmd.speed, steer=float(cmd.steer))
                    print(f"[{cmd.state}] {cmd.reason}")
                    continue
                # ===== [비전 추가] 끝 - 아래는 기존 로직 그대로 =====

                # ===== 1순위: 정지 (둘 중 하나라도 초근접) =====
                if lidar_min < STOP_DIST_MM or (0 < ultra_cm < Get_Stop_Distance()):
                    set_decre_Move(0)

                    prev_steer = steel_gain_result   # 정지 직전까지 유지되던 조향각(=현재 바퀴 각도)

                    # 초근접 + 조향각이 0이 아니면 -> 직진 후진으로 탈출
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

                # ===== 2순위: 위험 → 감속 + 회피 조향 =====
                if lidar_min < DANGER_DIST_MM or (0 < ultra_cm < Get_Stop_Distance()):
                    set_speed(SPEED_SLOW)
                    steel_gain_result = clear_angle * STEER_GAIN
                    set_steer(steel_gain_result)
                    update_telemetry(state="AVOID", reason="위험거리 회피",
                                     ultra_cm=float(ultra_cm), lidar_mm=float(lidar_min),
                                     speed=SPEED_SLOW, steer=float(steel_gain_result))
                    print(f"[회피] 라이다 {lidar_min:.0f}mm 초음파 {ultra_cm:.0f}cm 트인 {clear_angle:.0f}도")
                    continue

                # ===== 3순위: 안전 → 정상 주행 =====
                if lidar_min < STEER_ACTIVATE_DIST and abs(clear_angle) >= STEER_DEADZONE:
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
            # 시리얼 스트림 손상(New scan flags mismatch, Descriptor length
            # mismatch 등). 차는 안전하게 멈추고 라이다만 재연결한 뒤
            # 바깥 while 루프가 스캔을 다시 시작한다. 프로그램은 죽지 않는다.
            #
            # ===== [과부하 감시 추가] =====
            # 이 순간의 CPU 부하도 같이 남긴다. 라이다 오류가 CPU 과부하와
            # 겹치는지(카메라/스트리밍 처리가 밀려서 시리얼 읽기를 놓친 건
            # 아닌지) 확인하기 위함.
            overload_note = ""
            if loadMonitor.is_overloaded():
                overload_note = (f" (당시 CPU부하 {loadMonitor.last_load1:.2f} - "
                                  f"과부하 상태였음!)")
            print(f"[라이다] 스캔 중 오류 발생: {e}{overload_note} -> 재연결 시도")
            # ===== [과부하 감시 추가] 끝 =====
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
                # 재연결 후 정지 상태에서 재개하도록 후진/조향 상태 초기화
                isBack = False
                backCnt = 0
                isBackFlag = False
                SPEED_FAST = 0
                SPEED_BACK = 0
            except Exception as reconnect_err:
                print(f"[라이다] 재연결 실패: {reconnect_err} - 5초 후 재시도")
                time.sleep(5.0)
            # while not mission_done 루프가 다시 for scan in ... 을 시작함

    except KeyboardInterrupt:
        print("\n종료 중...")
    finally:
        set_speed(0)
        set_steer(0)
        wallBackup.stop(0)

        # ===== [로봇팔 추가] 종료 시 팔을 0도로 초기화 =====
        try:
            print("[로봇팔] 종료 - 초기 위치(0도)로 복귀 중...")
            arm_shoulder.angle(ARM_HOME_SHOULDER)
            arm_elbow.angle(ARM_HOME_ELBOW)
            arm_grab.angle(ARM_GRAB_OPEN)
            time.sleep(ARM_MOVE_DELAY * 1.5)
            print("[로봇팔] 초기 위치 복귀 완료")
        except Exception as e:
            print(f"[로봇팔] 종료 시 초기화 실패: {e}")
        # ===== [로봇팔 추가] 끝 =====

        stop_stream()              # ===== [스트리밍 추가] =====
        stop_vision()              # ===== [비전 추가] =====
        lidar.stop()
        lidar.stop_motor()
        lidar.disconnect()
        # ===== [과부하 감시 추가] =====
        print(f"[CPU부하] 이번 실행 중 최고 부하: {loadMonitor.max_load1:.2f} "
              f"(경고 기준 {Get_Load_Warn_Threshold():.1f})")
        # ===== [과부하 감시 추가] 끝 =====
        print("정지 완료")


if __name__ == "__main__":
    import sys
    if len(sys.argv) > 1 and sys.argv[1] == "calib":
        calibrate_vision()
    else:
        main()
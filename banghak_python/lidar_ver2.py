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
#   [비전 추가]      - 카메라 인식 / 미션 FSM
#   [스트리밍 추가]  - MJPEG 웹 스트리밍 + 실시간 HSV 튜닝 UI

import time
import threading
import json
import socket
from urllib.parse import urlparse, parse_qs
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np

from picamera2 import Picamera2

from rplidar import RPLidar
from robot_hat import Motor, Servo, Pin, PWM, reset_mcu, Ultrasonic

from avoidance_go_back import Get_Stop_Distance, WallBackup, Get_Drive_Duty
from picarx import Picarx

from battery_driving_check import BatteryMonitor

# ===== 설정 =====
LIDAR_PORT = '/dev/ttyUSB0'

# 조향
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
REAR_SAFETY_MARGIN_CM = 15
DEFAULT_BACK_TARGET_CM = 60

# 속도
VELOCITY = 60
SPEED_FAST = 0
SPEED_BACK = 0
SPEED_SLOW = VELOCITY / 2

SPEED_STEP = 3
SCAN_MIN_LEN = 60


# =========================================================================
# ===== [비전 추가] 카메라 / 노란 오브젝트 인식 설정 =====
# =========================================================================

CAM_WIDTH = 320             # 라즈베리파이4에서 라이다와 병행하려면 320x240 권장
CAM_HEIGHT = 240
CAM_FORMAT = "RGB888"       # picamera2에서 이 이름은 실제로 BGR 순서로 나옴(주의)
CAM_TILT_ANGLE = -15
CAM_SWAP_RB = False         # 스트림에서 노란 물체가 파랗게 보이면 웹 UI로 토글

# 노란색 HSV 범위 (OpenCV H는 0~179). 웹 UI 슬라이더로 실시간 변경 가능.
YELLOW_H_MIN, YELLOW_H_MAX = 20, 35
YELLOW_S_MIN, YELLOW_S_MAX = 100, 255
YELLOW_V_MIN, YELLOW_V_MAX = 100, 255

MIN_TARGET_AREA = 400

# 거리 추정 (핀홀 모델): 거리cm = (실제폭cm * focal_px) / 화면폭px
TARGET_REAL_WIDTH_CM = 6.5  # ★ 본인 오브젝트 실제 가로폭으로 수정
CAM_FOCAL_PX = 330.0        # ★ 반드시 캘리브레이션 필요 (해상도 바꾸면 재측정)

ARRIVE_DISTANCE_CM = 10

ARRIVE_HOLD_SEC = 10.0
REARM_COOLDOWN_SEC = 8.0
REARM_MIN_DISTANCE_CM = 40

# ===== 카메라 서보 팬 추적 ==== 추가
PAN_LIMIT = 40.0             # ★ 실제 서보 물리 한계로 조정 필요
CAM_FOV_H_DEG = 25.0         # ★ range_check.py 실측값으로 교체
PAN_TRACK_GAIN = 0.6
PAN_STEP_MAX_DEG = 8.0       # 프레임당 최대 이동각 (모션블러 방지)
PAN_CENTER_TOL_DEG = 5.0
STEER_BEARING_GAIN = 0.9

SWEEP_POSITIONS = [-40, -20, 0, 20, 40, 20, 0, -20]
SWEEP_HOLD_SEC = 0.5

FINAL_ENTRY_CM = 25
FINAL_EXIT_CM = 35 
# ============================ 추가

APPROACH_SPEED = 20
TARGET_STEER_GAIN = 30.0
LOST_TIMEOUT = 2.0
DETOUR_TIME = 1.2
DETOUR_STEER = 25.0
CONFIRM_HITS = 3
TARGET_TOLERANCE_RATIO = 0.35   # 목표물/장애물 판정 오차 허용 계수

# 미션 상태
SEARCH = "SEARCH"
APPROACH = "APPROACH"
DETOUR = "DETOUR"
ARRIVED = "ARRIVED"
# ==========추가
ARRIVED = "ARRIVED"
COOLDOWN = "COOLDOWN"        # 기존에 없었으면 추가
FINAL = "FINAL"              # 신규
# ==========추가


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
    lower = np.array([YELLOW_H_MIN, YELLOW_S_MIN, YELLOW_V_MIN])
    upper = np.array([YELLOW_H_MAX, YELLOW_S_MAX, YELLOW_V_MAX])
    return lower, upper


def grab_frame():
    """
    picamera2에서 프레임을 받아 OpenCV용 BGR 3채널로 정규화해서 반환.
    실패하면 None.

    주의) picamera2의 포맷 이름은 직관과 반대다.
      - "RGB888" 로 설정하면 실제 numpy 배열은 BGR 순서 (OpenCV와 동일)
      - "XBGR8888" 등은 4채널로 나옴
    환경/버전에 따라 달라질 수 있어서 채널 수를 보고 분기하고,
    그래도 색이 뒤집히면 CAM_SWAP_RB 를 웹 UI에서 토글한다.
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
<h3>HSV 범위 (노란색 튜닝)</h3>
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
        global YELLOW_H_MIN, YELLOW_H_MAX, YELLOW_S_MIN
        global YELLOW_S_MAX, YELLOW_V_MIN, YELLOW_V_MAX

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
                    "h_min": YELLOW_H_MIN, "h_max": YELLOW_H_MAX,
                    "s_min": YELLOW_S_MIN, "s_max": YELLOW_S_MAX,
                    "v_min": YELLOW_V_MIN, "v_max": YELLOW_V_MAX,
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

            YELLOW_H_MIN = gv('h_min', YELLOW_H_MIN)
            YELLOW_H_MAX = gv('h_max', YELLOW_H_MAX)
            YELLOW_S_MIN = gv('s_min', YELLOW_S_MIN)
            YELLOW_S_MAX = gv('s_max', YELLOW_S_MAX)
            YELLOW_V_MIN = gv('v_min', YELLOW_V_MIN)
            YELLOW_V_MAX = gv('v_max', YELLOW_V_MAX)

            if 'mask' in q:
                STREAM_SHOW_MASK = not STREAM_SHOW_MASK
            if 'swaprb' in q:
                CAM_SWAP_RB = not CAM_SWAP_RB
                print(f"[비전] CAM_SWAP_RB = {CAM_SWAP_RB}")
            if 'dump' in q:
                print("\n===== 현재 설정 (코드에 반영하세요) =====")
                print(f"CAM_SWAP_RB = {CAM_SWAP_RB}")
                print(f"YELLOW_H_MIN, YELLOW_H_MAX = {YELLOW_H_MIN}, {YELLOW_H_MAX}")
                print(f"YELLOW_S_MIN, YELLOW_S_MAX = {YELLOW_S_MIN}, {YELLOW_S_MAX}")
                print(f"YELLOW_V_MIN, YELLOW_V_MAX = {YELLOW_V_MIN}, {YELLOW_V_MAX}")
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
# ===== [비전 추가] 미션 FSM =====
# =========================================================================

MISSION_STATE = SEARCH
_last_seen_time = 0.0
_last_offset = 0.0
_detour_start = 0.0
_detour_dir = 1
_hit_count = 0
# ====== 추가
_arrive_time = 0.0
_cooldown_start = 0.0
_arrive_count = 0
_pan_angle = 0.0              # 메인 루프가 매 프레임 갱신하는 "현재 팬 각도"
_sweep_index = 0
_sweep_last_change = 0.0


def _enter_arrived():
    global MISSION_STATE, _arrive_time, _arrive_count
    MISSION_STATE = ARRIVED
    _arrive_time = time.time()
    _arrive_count += 1
    print(f"[미션] ★ 도착 #{_arrive_count} - {ARRIVE_HOLD_SEC:.0f}초 정지")


def _sweep_pan(now):
    global _sweep_index, _sweep_last_change
    if now - _sweep_last_change >= SWEEP_HOLD_SEC:
        _sweep_index = (_sweep_index + 1) % len(SWEEP_POSITIONS)
        _sweep_last_change = now
    return SWEEP_POSITIONS[_sweep_index]


def _step_toward(current, target, max_step):
    diff = target - current
    if abs(diff) <= max_step:
        return target
    return current + max_step * (1 if diff > 0 else -1)


def _track_pan(target, current_pan):
    """화면 오프셋으로 목표 팬각을 계산하고, 프레임당 이동량을 제한해서 반환"""
    if not target.found:
        return current_pan
    pan_error = target.offset * (CAM_FOV_H_DEG / 2.0) * PAN_TRACK_GAIN
    desired = max(-PAN_LIMIT, min(PAN_LIMIT, current_pan + pan_error))
    return _step_toward(current_pan, desired, PAN_STEP_MAX_DEG)


def _pan_to_center(current_pan):
    return _step_toward(current_pan, 0.0, PAN_STEP_MAX_DEG)


def compute_bearing(target, pan_angle):
    """차체 기준 목표물 방위각(도) = 팬각 + 화면오프셋을 화각으로 환산한 값"""
    if not target.found:
        return pan_angle
    return pan_angle + target.offset * (CAM_FOV_H_DEG / 2.0)
# ====== 추가

class Command:
    """
    handled=False -> 기존 라이다 회피 로직 그대로 실행
    handled=True  -> speed/steer 적용. allow_backup=False면 후진 금지
    """
    def __init__(self, handled=False, speed=0, steer=0.0,
                 allow_backup=True, state=SEARCH, reason="", pan=0.0):
        self.handled = handled
        self.speed = speed
        self.steer = steer
        self.allow_backup = allow_backup
        self.state = state
        self.reason = reason
        # ===== 추가
        self.pan = pan          # ★ 신규: 이번 프레임에 명령할 팬 각도
        # ===== 추가

def is_the_target(target, ultra_cm, pan_angle):
    """
    전방 물체가 목표물인지 장애물인지 판정.
    카메라 추정거리와 초음파 거리가 비슷하면 목표물, 크게 다르면 앞에 낀 장애물.
    """
    # ===== 추가
    if abs(pan_angle) > PAN_CENTER_TOL_DEG:
        return False    # 카메라가 정면이 아니면 초음파와 비교 자체가 무의미
    # ===== 추가
    if not target.found or not target.is_centered(0.15):
        return False
    if ultra_cm <= 0 or target.distance_cm <= 0:
        return False
    tolerance = max(8.0, target.distance_cm * TARGET_TOLERANCE_RATIO)
    return abs(target.distance_cm - ultra_cm) < tolerance


def mission_step(ultra_cm, lidar_min):
    global MISSION_STATE, _last_seen_time, _last_offset
    global _detour_start, _detour_dir, _hit_count, _cooldown_start, _pan_angle

    now = time.time()

    if not VISION_ENABLED:
        return Command(handled=False, state=SEARCH, reason="비전 비활성", pan=0.0)

    target = get_target()

    if target.found and target.is_fresh():
        _last_seen_time = now
        _last_offset = target.offset
        _hit_count = min(_hit_count + 1, CONFIRM_HITS)
    else:
        _hit_count = 0

    # ---------- ARRIVED ----------
    if MISSION_STATE == ARRIVED:
        remain = ARRIVE_HOLD_SEC - (now - _arrive_time)
        if remain > 0:
            return Command(handled=True, speed=0, steer=0.0, allow_backup=False,
                           state=ARRIVED, reason=f"도착 정지 유지 {remain:.1f}초 남음",
                           pan=0.0)
        MISSION_STATE = COOLDOWN
        _cooldown_start = now
        _hit_count = 0
        print("[미션] 정지 해제 -> COOLDOWN")

    # ---------- COOLDOWN ----------
    if MISSION_STATE == COOLDOWN:
        elapsed = now - _cooldown_start
        far_enough = (ultra_cm < 0) or (ultra_cm > REARM_MIN_DISTANCE_CM)
        if elapsed >= REARM_COOLDOWN_SEC and far_enough:
            MISSION_STATE = SEARCH
            _hit_count = 0
            print("[미션] 재탐색 시작 -> SEARCH")
        else:
            return Command(handled=False, state=COOLDOWN,
                           reason=f"쿨다운 {elapsed:.1f}/{REARM_COOLDOWN_SEC:.0f}초 "
                                  f"초음파 {ultra_cm:.0f}cm", pan=0.0)

    # ---------- SEARCH: 팬 스윕, 조향은 기존 회피에 위임 ----------
    if MISSION_STATE == SEARCH:
        sweep_pan = _sweep_pan(now)
        if _hit_count >= CONFIRM_HITS:
            print(f"[미션] 노란 오브젝트 발견(거리≈{target.distance_cm:.0f}cm) -> APPROACH")
            MISSION_STATE = APPROACH
        else:
            return Command(handled=False, state=SEARCH, reason="탐색 중 (팬 스윕)",
                           pan=sweep_pan)

    # ---------- DETOUR ----------
    if MISSION_STATE == DETOUR:
        track_pan = _track_pan(target, _pan_angle) if target.found else _pan_angle
        if now - _detour_start > DETOUR_TIME:
            print("[미션] 우회 종료 -> APPROACH 복귀")
            MISSION_STATE = APPROACH
        else:
            if 0 < ultra_cm < 8 or lidar_min < 200:
                return Command(handled=False, state=DETOUR,
                               reason="우회 중 초근접 - 기존 회피로 위임", pan=track_pan)
            return Command(handled=True, speed=APPROACH_SPEED,
                           steer=_detour_dir * DETOUR_STEER, allow_backup=True,
                           state=DETOUR, reason="장애물 우회 중", pan=track_pan)

    # ---------- FINAL: 팬을 0도로 되돌리며 정밀 접근 ----------
    if MISSION_STATE == FINAL:
        center_pan = _pan_to_center(_pan_angle)
        pan_ready = abs(center_pan) <= PAN_CENTER_TOL_DEG

        if 0 < ultra_cm <= ARRIVE_DISTANCE_CM and pan_ready:
            _enter_arrived()
            return Command(handled=True, speed=0, steer=0.0, allow_backup=False,
                           state=ARRIVED, reason=f"목표물 {ultra_cm:.0f}cm 도착", pan=0.0)

        if ultra_cm > FINAL_EXIT_CM or ultra_cm <= 0:
            print("[미션] 목표물과 재이격 -> APPROACH 복귀")
            MISSION_STATE = APPROACH
        else:
            bearing = compute_bearing(target, center_pan) if target.found else 0.0
            steer = max(-STEER_LIMIT, min(STEER_LIMIT, bearing * STEER_BEARING_GAIN))
            speed = max(10, int(APPROACH_SPEED * 0.5))
            return Command(handled=True, speed=speed, steer=steer,
                           allow_backup=False, state=FINAL,
                           reason=f"정밀접근 초음파={ultra_cm:.0f}cm 팬={center_pan:.0f}도",
                           pan=center_pan)

    # ---------- APPROACH ----------
    track_pan = _track_pan(target, _pan_angle) if target.found else _pan_angle

    if not (target.found and target.is_fresh()):
        if 0 < ultra_cm <= ARRIVE_DISTANCE_CM and abs(_pan_angle) <= PAN_CENTER_TOL_DEG:
            print(f"[미션] 근접 상실 + 초음파 {ultra_cm:.0f}cm -> 도착 처리")
            _enter_arrived()
            return Command(handled=True, speed=0, steer=0.0, allow_backup=False,
                           state=ARRIVED, reason="도착", pan=0.0)
        if now - _last_seen_time > LOST_TIMEOUT:
            print("[미션] 목표물 상실 -> SEARCH 복귀")
            MISSION_STATE = SEARCH
            return Command(handled=False, state=SEARCH, reason="목표물 상실", pan=0.0)
        return Command(handled=True, speed=APPROACH_SPEED,
                       steer=_last_offset * TARGET_STEER_GAIN, allow_backup=False,
                       state=APPROACH, reason="일시 상실 - 관성 주행", pan=track_pan)

    # 정밀접근 구간 진입
    if 0 < ultra_cm <= FINAL_ENTRY_CM:
        print(f"[미션] 정밀접근 구간 진입(초음파 {ultra_cm:.0f}cm) -> FINAL")
        MISSION_STATE = FINAL
        return Command(handled=True, speed=max(10, int(APPROACH_SPEED * 0.5)),
                       steer=0.0, allow_backup=False, state=FINAL,
                       reason="정밀접근 시작", pan=track_pan)

    # 앞을 막은 게 목표물이 아니면 우회
    blocked = (0 < ultra_cm < 25) or (0 < lidar_min < STOP_DIST_MM)
    if blocked and not is_the_target(target, ultra_cm, _pan_angle):
        _detour_dir = 1 if target.offset < 0 else -1
        _detour_start = now
        MISSION_STATE = DETOUR
        print(f"[미션] 경로상 장애물 -> {_detour_dir} 방향 우회")
        return Command(handled=True, speed=APPROACH_SPEED,
                       steer=_detour_dir * DETOUR_STEER, allow_backup=True,
                       state=DETOUR, reason="우회 시작", pan=track_pan)

    # 정상 접근: 팬 추적 + 방위각 기반 조향
    bearing = compute_bearing(target, _pan_angle)
    steer = max(-STEER_LIMIT, min(STEER_LIMIT, bearing * STEER_BEARING_GAIN))
    speed = APPROACH_SPEED
    if 0 < ultra_cm < 40:
        speed = max(12, int(APPROACH_SPEED * 0.6))

    return Command(handled=True, speed=speed, steer=steer, allow_backup=False,
                   state=APPROACH,
                   reason=f"접근 팬={_pan_angle:.0f}도 방위각={bearing:.0f}도 "
                          f"카메라≈{target.distance_cm:.0f}cm 초음파={ultra_cm:.0f}cm",
                   pan=track_pan)
def calibrate_vision():
    """
    차량을 움직이지 않고 카메라만 튜닝하는 모드.
      python3 lidar_ultra_vision.py calib

    순서:
      1) 브라우저 접속 -> 영상이 보이는지 확인
      2) 노란 물체가 파랗게 보이면 'R/B 색상 반전' 클릭
      3) '마스크 보기 전환' 클릭 후 슬라이더로 오브젝트만 하얗게 만들기
      4) 오브젝트를 30cm 앞에 두고 '현재 HSV 값 출력' 클릭
      5) 터미널에 찍힌 값들을 이 파일 상단 상수에 반영
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
                print("노란색 미검출 - 브라우저에서 S/V 하한을 낮추거나 R/B 반전 확인")
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


def main():
    reset_mcu()
    time.sleep(0.5)

    global SPEED_FAST
    global SPEED_BACK
    global VELOCITY
    global STEEL_CMD
    # ===== 추가
    global _pan_angle          # ★ 추가
    # ===== 추가

    x = Picarx()
    wallBackup = WallBackup(x, Get_Drive_Duty())
    batteryMonitor = BatteryMonitor()

    left_motor = Motor(PWM("P13"), Pin("D4"))
    right_motor = Motor(PWM("P12"), Pin("D5"))
    steer = Servo("P2")

    sonar = Ultrasonic(Pin("D2"), Pin("D3"))
    lidar = RPLidar(LIDAR_PORT)

    # ===== [비전 추가] 카메라 시작 + 짐벌 각도 =====
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
    steer.angle(0)
    time.sleep(1)

    backCnt = 0
    isBack = False
    isBackFlag = False
    BACK_TARGET = VELOCITY * 0.65

    backSpeed = 20 * (50 / BACK_TARGET)
    current_backSpeed = backSpeed
    steel_gain_result = 0

    try:
        for scan in lidar.iter_scans(min_len=SCAN_MIN_LEN):
            batteryMonitor.show()

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
            # =================================================================
            cmd = mission_step(ultra_cm, lidar_min)

            # ===== 추가
            # ===== [팬 추적 추가] 서보 하드웨어 반영 =====
            if abs(cmd.pan - _pan_angle) >= 0.5:
                try:
                    x.set_cam_pan_angle(cmd.pan)
                except Exception as e:
                    print(f"[비전] 팬 서보 제어 실패: {e}")
            _pan_angle = cmd.pan
            # ===== 추가


            if cmd.state == ARRIVED:
                set_decre_Move(0)
                left_motor.speed(0)
                right_motor.speed(0)
                set_steer(0)
                update_telemetry(state=ARRIVED, reason=cmd.reason,
                                 ultra_cm=float(ultra_cm), lidar_mm=float(lidar_min),
                                 speed=0, steer=0.0)
                print(f"★ 미션 완료: {cmd.reason}")
                break

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
                steel_gain_result = clear_angle * STEER_GAIN
                set_steer(steel_gain_result)

                rear_cm = get_rear_min_cm(scan)
                current_backSpeed = compute_dynamic_backspeed(rear_cm, backSpeed)

                if current_backSpeed == 0:
                    print(f"[후진 불가] 후방 {rear_cm}cm 이내 근접 - 후진 취소")
                    isBack = False
                else:
                    rear_str = f"{rear_cm:.1f}cm" if rear_cm is not None else "측정불가"
                    print(f"[정지] 라이다 {lidar_min:.0f}mm 초음파 {ultra_cm:.0f}cm 트인 {clear_angle:.0f}도 "
                          f"| 후방 {rear_str} -> backSpeed {current_backSpeed}(기존 {backSpeed:.0f})")
                    isBack = True
                    backCnt = 0

                update_telemetry(state="STOP", reason="초근접 정지",
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

    except KeyboardInterrupt:
        print("\n종료 중...")
    finally:
        set_speed(0)
        set_steer(0)
        wallBackup.stop(0)
        stop_stream()              # ===== [스트리밍 추가] =====
        stop_vision()              # ===== [비전 추가] =====
        lidar.stop()
        lidar.stop_motor()
        lidar.disconnect()
        print("정지 완료")


if __name__ == "__main__":
    import sys
    if len(sys.argv) > 1 and sys.argv[1] == "calib":
        calibrate_vision()
    else:
        main()
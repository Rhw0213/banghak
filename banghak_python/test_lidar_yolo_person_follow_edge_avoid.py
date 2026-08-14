# test_lidar_yolo_person_follow_edge_avoid.py
# [테스트용] 기존 라이다+초음파 자율주행 + YOLO 사람추적에
# "화면에 뭔가 복잡하게 보이면(엣지가 많으면) = 장애물"로 판단하는
# 카메라 기반 회피 로직을 추가한 버전.
#
# 바닥색 캘리브레이션 불필요 (이전 버전과 차이점). 그냥 화면 하단 영역이
# 평소보다 엣지(윤곽선)가 많아지면 "뭔가 가까이 있다"고 보고 피함.
# 물체가 뭔지 몰라도 되고, 라이다/초음파가 놓치기 쉬운 얇은 물체(의자다리 등)도
# 카메라 화면에 윤곽으로는 잡히므로 보완 효과가 있음.
#
# 안전 우선순위:
#   1순위: 라이다/초음파 초근접 -> 정지+후진 (기존, 즉각 반응)
#   2순위: 라이다/초음파 위험거리 -> 감속+회피 (기존, 즉각 반응)
#   2.5순위: [신규] 카메라 엣지분석 -> 장애물 있으면 회피 (로컬, 즉각 반응)
#   3순위: 사람이 보이면 -> 그 방향으로 저속 접근 (YOLO, 지연 있음)
#   4순위: 사람도 없으면 -> 기존 라이다 자유주행
#
# 실행 전 준비:
#   1. PC(서버)에서 server.py를 먼저 실행해서 "대기 중..." 상태로 켜둘 것
#   2. 아래 YOLO_SERVER_IP를 PC의 실제 IP로 맞출 것
#
# 실행: python3 test_lidar_yolo_person_follow_edge_avoid.py

import os
import time
import threading
import socket
import pickle
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np
from picamera2 import Picamera2

from rplidar import RPLidar, RPLidarException
from robot_hat import Motor, Servo, Pin, PWM, reset_mcu, Ultrasonic

from avoidance_go_back import Get_Stop_Distance, WallBackup, Get_Drive_Duty
from picarx import Picarx

from battery_driving_check import BatteryMonitor

# ===== 기존 설정 (lidar_ultra_avoidance.py와 동일) =====
LIDAR_PORT = '/dev/ttyUSB0'

STEER_GAIN = 0.3
STEER_DEADZONE = 0
STEER_ACTIVATE_DIST = 800
STEER_LIMIT = 35
GAIN_REVERSE = False

DANGER_DIST_MM = 500
STOP_DIST_MM = 300

REAR_SECTOR_DEG = 25
REAR_SAFETY_MARGIN_CM = 15
DEFAULT_BACK_TARGET_CM = 60

VELOCITY = 60
SPEED_FAST = 0
SPEED_BACK = 0
SPEED_SLOW = VELOCITY / 2

SPEED_STEP = 3
SCAN_MIN_LEN = 60

# ===== 카메라 각도 =====
CAM_TILT_ANGLE = 10   # 처음에 요청하셨던 원래 값으로 복원


# =========================================================================
# ===== YOLO 오프로딩 + 사람 추적 설정 (기존과 동일) =====
# =========================================================================

YOLO_SERVER_IP = '192.168.0.121'   # PC(서버)의 실제 IP로 수정
YOLO_SERVER_PORT = 5051

CAM_WIDTH = 480
CAM_HEIGHT = 360
CAM_FORMAT = "RGB888"       # picamera2에서 이 이름은 실제로 BGR 순서로 나옴

YOLO_SEND_INTERVAL_SEC = 0.5
JPEG_QUALITY = 70

YOLO_TARGET_CLASS = 'person'
PERSON_FOLLOW_SPEED = 32   # 기존 25에서 상향
PERSON_STEER_GAIN = 30.0
PERSON_LOST_TIMEOUT = 1.5
PERSON_CONF_MIN = 0.5

_cam_running = False
_cam_thread = None
_picam2 = None

_yolo_sock = None
_yolo_connected = False

_person_lock = threading.Lock()
_person_offset = 0.0
_person_last_seen = 0.0
_person_found = False


# =========================================================================
# ===== [신규] 엣지 밀도 기반 장애물(뭐든 상관없음) 감지 설정 =====
# =========================================================================

# 장애물 탐색 영역 (ROI): 화면의 이 구간만 검사 (하단=바닥 근처, 중앙=주행 경로)
# ===== [민감도/거리 조정] Y 시작점을 낮춰서(위로 넓혀서) 더 먼 거리도 탐색하도록 변경 =====
OBSTACLE_ROI_X_RANGE = (0.30, 0.70)   # 가로 30%~70% (35~65에서 살짝 넓힘)
OBSTACLE_ROI_Y_RANGE = (0.58, 1.0)    # 세로 58%~100% (65에서 살짝 넓힘)

# Canny 엣지 검출 임계값 (낮을수록 약한 윤곽선도 잡음)
# ===== [바닥 질감 노이즈 개선] 약한 엣지(바닥 매트/장판 질감)는 무시하도록 상향 =====
EDGE_CANNY_LOW = 80
EDGE_CANNY_HIGH = 200

# ROI 안에서 엣지 픽셀이 차지하는 비율이 이 값을 넘으면 "장애물 있음"으로 판단
# ★ 처음 실행 후 조정 필요. 평평한 바닥만 있을 때 이 값이 얼마나 나오는지 로그로 확인 후
#   그보다 확실히 높은 값으로 설정
EDGE_OBSTACLE_THRESHOLD = 0.033   # 실측(장애물 0.037) 기준으로 살짝 아래로 조정

# ===== [추세 판단 추가] =====
# 임계값을 넘었어도, 그게 "계속 잡히지만 값은 그대로인 배경 클러터"인지
# "점점 커지는(다가오는) 진짜 장애물"인지 구분하기 위한 설정
OBSTACLE_HISTORY_SEC = 1.0        # 최근 이만큼의 엣지비율 기록을 보관
OBSTACLE_RISE_MARGIN = 0.012      # 최근 절반 평균이 이전 절반 평균보다 이만큼 이상 커야 "다가오는 중"으로 인정
OBSTACLE_ABS_SAFETY = 0.13        # 이 값 이상이면 추세 상관없이 무조건 장애물 (너무 크고 가까운 경우 안전장치)

OBSTACLE_FRESH_SEC = 0.3          # 이 시간 안에 갱신된 정보만 "최신"으로 인정
OBSTACLE_AVOID_SPEED = 22         # 장애물 회피 중 속도 (15는 너무 느려서 진행이 안 됨 -> 22로 상향)
OBSTACLE_AVOID_STEER = 28.0       # 장애물 반대 방향으로 꺾는 고정 각도

# ===== [반복회피 탈출 추가] =====
# lidar_ultra_vision.py의 DETOUR 로직(MAX_DETOUR_REPEAT)과 동일한 이름/패턴 사용
# (나중에 합칠 때 변수명 통일용). 좁은 공간에서 회피만 계속 반복하며 제자리
# 걸음하는 것을 막기 위해, 연속으로 이 횟수 이상 회피하면 후진으로 전환한다.
MAX_OBSTACLE_REPEAT = 3

# ===== [웹 스트리밍 추가] =====
# 브라우저에서 http://라즈베리파이IP:8000 접속하면 엣지 탐지 결과를 실시간으로 볼 수 있음.
# SSH 원격 환경에서도 문제없이 동작함 (cv2.imshow와 달리 모니터 직결 불필요).
STREAM_ENABLED = True
STREAM_PORT = 8000
STREAM_FPS = 10             # 스트리밍 프레임 속도 (높이면 부하 늘어남)
STREAM_QUALITY = 60         # JPEG 품질 (낮을수록 가벼움)

_stream_lock = threading.Lock()
_stream_jpeg = None
_stream_clients = 0         # 접속자 수. 0이면 인코딩 자체를 건너뜀 (부하 절약)
_stream_server = None
_stream_running = True

_obstacle_lock = threading.Lock()
_obstacle_found = False
_obstacle_offset = 0.0    # -1.0(왼쪽) ~ +1.0(오른쪽), 장애물이 화면 어디 있는지
_obstacle_last_seen = 0.0
_obstacle_debug_ratio = 0.0   # 디버깅용: 방금 측정된 엣지 비율 (튜닝할 때 참고)


# ===== CPU 부하 확인용 =====
LOAD_CHECK_INTERVAL_SEC = 2.0


class SystemLoadMonitor:
    def __init__(self, interval=LOAD_CHECK_INTERVAL_SEC):
        self.interval = interval
        self.last_time = 0
        self.cores = os.cpu_count() or 1
        self.max_load1 = 0.0

    def show(self):
        now = time.time()
        if now - self.last_time < self.interval:
            return
        self.last_time = now
        try:
            load1, _, _ = os.getloadavg()
        except Exception:
            return
        if load1 > self.max_load1:
            self.max_load1 = load1
        print(f"##[CPU부하] {load1:.2f} (코어 {self.cores}개, 최고 {self.max_load1:.2f})##")


# =========================================================================
# ===== [신규] 엣지 밀도 기반 장애물 감지 함수 =====
# 캘리브레이션 불필요. 화면 하단 영역에 윤곽선(엣지)이 얼마나 있는지만 봄.
# 빈 바닥 = 엣지 거의 없음 / 물체(뭐든) = 엣지 많음
# =========================================================================

def _detect_obstacle_edge(frame):
    """
    ROI 안의 엣지 비율을 계산해서 장애물 후보를 찾는다.
    반환: (found, offset, edge_ratio, edges, roi_box)
    edges: ROI 안의 엣지 흑백 이미지 (화면표시용)
    roi_box: (rx1, ry1, rx2, ry2) ROI 좌표 (화면표시용)
    """
    fh, fw = frame.shape[:2]
    rx1 = int(fw * OBSTACLE_ROI_X_RANGE[0])
    rx2 = int(fw * OBSTACLE_ROI_X_RANGE[1])
    ry1 = int(fh * OBSTACLE_ROI_Y_RANGE[0])
    ry2 = int(fh * OBSTACLE_ROI_Y_RANGE[1])
    roi_box = (rx1, ry1, rx2, ry2)

    roi = frame[ry1:ry2, rx1:rx2]
    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    edges = cv2.Canny(blurred, EDGE_CANNY_LOW, EDGE_CANNY_HIGH)
    # ===== [노이즈 개선 롤백] MORPH_OPEN이 1픽셀 두께의 Canny 엣지 자체를 거의
    # 다 지워버려서(의자다리 등 진짜 엣지까지 삭제) 제거함. Canny 임계값
    # 상향(80/200)만으로 바닥 질감 노이즈를 줄이고, 엣지 자체는 보존함.

    edge_ratio = np.count_nonzero(edges) / edges.size

    if edge_ratio < EDGE_OBSTACLE_THRESHOLD:
        return False, 0.0, edge_ratio, edges, roi_box

    # 엣지 픽셀들의 좌우 분포를 보고 물체가 화면 어느 쪽에 몰려있는지 계산
    ys, xs = np.nonzero(edges)
    if len(xs) == 0:
        return False, 0.0, edge_ratio, edges, roi_box

    roi_w = rx2 - rx1
    mean_x = float(np.mean(xs))
    offset = (mean_x - roi_w / 2.0) / (roi_w / 2.0)   # -1.0(왼쪽) ~ +1.0(오른쪽)

    return True, offset, edge_ratio, edges, roi_box


def get_obstacle_status():
    with _obstacle_lock:
        fresh = (time.time() - _obstacle_last_seen) < OBSTACLE_FRESH_SEC
        return _obstacle_found and fresh, _obstacle_offset


# =========================================================================
# ===== [웹 스트리밍 추가] MJPEG 웹 서버 =====
# =========================================================================

STREAM_PAGE_HTML = b"""<!DOCTYPE html>
<html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Edge Avoidance Debug</title>
<style>
 body{background:#111;margin:0;padding:12px;font-family:sans-serif}
 img{width:100%;max-width:640px;border:1px solid #444}
</style></head><body>
<img src="/stream.mjpg">
</body></html>"""


def _publish_frame(vis):
    """엣지 표시가 그려진 프레임을 JPEG으로 인코딩해서 스트리밍 버퍼에 저장"""
    global _stream_jpeg
    ok, buf = cv2.imencode('.jpg', vis, [int(cv2.IMWRITE_JPEG_QUALITY), STREAM_QUALITY])
    if ok:
        with _stream_lock:
            _stream_jpeg = buf.tobytes()


class _StreamHandler(BaseHTTPRequestHandler):
    def log_message(self, *args):
        pass    # 요청 로그로 터미널 도배 방지

    def do_GET(self):
        global _stream_clients
        if self.path == '/':
            self.send_response(200)
            self.send_header('Content-Type', 'text/html; charset=utf-8')
            self.send_header('Content-Length', str(len(STREAM_PAGE_HTML)))
            self.end_headers()
            self.wfile.write(STREAM_PAGE_HTML)
        elif self.path == '/stream.mjpg':
            self.send_response(200)
            self.send_header('Age', '0')
            self.send_header('Cache-Control', 'no-cache, private')
            self.send_header('Pragma', 'no-cache')
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=FRAME')
            self.end_headers()
            _stream_clients += 1
            try:
                while _stream_running:
                    with _stream_lock:
                        buf = _stream_jpeg
                    if buf is None:
                        time.sleep(0.1)
                        continue
                    self.wfile.write(b'--FRAME\r\n')
                    self.wfile.write(b'Content-Type: image/jpeg\r\n')
                    self.wfile.write(('Content-Length: %d\r\n\r\n' % len(buf)).encode())
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
    if not STREAM_ENABLED:
        return
    try:
        _stream_server = ThreadingHTTPServer(('0.0.0.0', STREAM_PORT), _StreamHandler)
        _stream_server.daemon_threads = True
        threading.Thread(target=_stream_server.serve_forever, daemon=True).start()
        print(f"[스트리밍] 브라우저에서 http://{_get_local_ip()}:{STREAM_PORT} 접속하세요")
    except Exception as e:
        print(f"[스트리밍] 시작 실패: {e}")


def stop_stream():
    global _stream_running
    _stream_running = False
    if _stream_server:
        try:
            _stream_server.shutdown()
        except Exception:
            pass


# =========================================================================
# ===== YOLO 서버 통신 (기존과 동일) =====
# =========================================================================

def _yolo_connect():
    global _yolo_sock, _yolo_connected
    try:
        _yolo_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        _yolo_sock.settimeout(3.0)
        _yolo_sock.connect((YOLO_SERVER_IP, YOLO_SERVER_PORT))
        _yolo_connected = True
        print("[YOLO] 서버 연결 성공")
    except Exception as e:
        _yolo_connected = False
        print(f"[YOLO] 서버 연결 실패: {e} - YOLO 없이 카메라 캡처만 테스트합니다")


def _yolo_send_frame(frame):
    global _yolo_connected
    if not _yolo_connected:
        return []
    try:
        ok, encoded = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
        if not ok:
            return []
        data = encoded.tobytes()

        _yolo_sock.sendall(len(data).to_bytes(4, 'big'))
        _yolo_sock.sendall(data)

        raw_len = _yolo_sock.recv(4)
        if not raw_len:
            raise ConnectionError("서버 응답 없음(연결 끊김)")
        result_len = int.from_bytes(raw_len, 'big')
        result = b''
        while len(result) < result_len:
            chunk = _yolo_sock.recv(4096)
            if not chunk:
                raise ConnectionError("응답 수신 중 끊김")
            result += chunk

        return pickle.loads(result)
    except Exception as e:
        print(f"[YOLO] 전송/수신 실패: {e}")
        _yolo_connected = False
        return []


def _camera_loop():
    """
    별도 스레드. 매 프레임마다:
      - [신규] 엣지분석 장애물 감지 (로컬, 매 프레임 - 빠름, 캘리브레이션 없음)
      - YOLO_SEND_INTERVAL_SEC 간격으로만 사람 추적 (서버 전송 - 느림)
    """
    global _person_offset, _person_last_seen, _person_found
    global _obstacle_found, _obstacle_offset, _obstacle_last_seen, _obstacle_debug_ratio

    _yolo_connect()
    last_send = 0.0
    debug_print_count = 0
    edge_history = []   # [(timestamp, edge_ratio), ...] - 추세 판단용, 이 스레드 안에서만 사용

    while _cam_running:
        try:
            frame = _picam2.capture_array()
        except Exception as e:
            print(f"[카메라] 프레임 캡처 실패: {e}")
            time.sleep(0.1)
            continue

        now = time.time()

        # ===== [신규] 엣지분석 장애물 감지 - 매 프레임 로컬 처리 =====
        raw_found, offset, edge_ratio, edges, roi_box = _detect_obstacle_edge(frame)

        # ===== [추세 판단 추가] 엣지비율 기록 갱신 (found 여부와 상관없이 매 프레임 기록) =====
        edge_history.append((now, edge_ratio))
        cutoff = now - OBSTACLE_HISTORY_SEC
        while edge_history and edge_history[0][0] < cutoff:
            edge_history.pop(0)

        # ===== [추세 판단 롤백] 실제 테스트에서 진짜 장애물도 "정체"로 판정되어
        # 놓치는 경우가 많았음(카메라가 조향으로 홱 돌아가며 갑자기 시야에
        # 들어오는 경우가 많아서 "서서히 커진다"는 가정이 안 맞음).
        # 놓쳐서 부딪히는 게 오탐보다 훨씬 위험하므로 즉시 회피 방식으로 복귀.
        found = raw_found
        trend_info = ""

        with _obstacle_lock:
            _obstacle_found = found
            _obstacle_debug_ratio = edge_ratio
            if found:
                _obstacle_offset = offset
                _obstacle_last_seen = now

        if found:
            print(f"[엣지회피] 장애물 감지 offset={offset:+.2f} 엣지비율={edge_ratio:.3f} {trend_info}")
        elif raw_found:
            # 임계값은 넘었지만 추세상 배경 클러터로 판단되어 무시한 경우
            debug_print_count += 1
            if debug_print_count % 15 == 0:
                print(f"[엣지회피 무시] 임계값 넘었지만 배경 클러터로 판단 엣지비율={edge_ratio:.3f} {trend_info}")
        else:
            # 튜닝 참고용으로 가끔(약 1초에 한 번꼴) 평소 엣지 비율을 출력
            debug_print_count += 1
            if debug_print_count % 30 == 0:
                print(f"[엣지회피 참고] 평소 엣지비율={edge_ratio:.3f} (기준값 {EDGE_OBSTACLE_THRESHOLD})")

        # ===== [웹 스트리밍 추가] 브라우저 접속자가 있을 때만 인코딩 (CPU 절약) =====
        if STREAM_ENABLED and _stream_clients > 0:
            try:
                vis = frame.copy()
                rx1, ry1, rx2, ry2 = roi_box

                # ROI 안의 엣지를 빨간색으로 겹쳐서 표시
                edge_bgr = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)
                edge_bgr[edges > 0] = (0, 0, 255)   # 엣지 픽셀만 빨간색
                blended = cv2.addWeighted(vis[ry1:ry2, rx1:rx2], 0.6, edge_bgr, 0.8, 0)
                vis[ry1:ry2, rx1:rx2] = blended

                # 장애물 판단 여부에 따라 ROI 테두리 색 변경 (빨강=장애물, 초록=클리어)
                box_color = (0, 0, 255) if found else (0, 255, 0)
                cv2.rectangle(vis, (rx1, ry1), (rx2, ry2), box_color, 2)

                status = "OBSTACLE!" if found else "clear"
                cv2.putText(vis, f"{status}  edge={edge_ratio:.3f} thr={EDGE_OBSTACLE_THRESHOLD:.3f}",
                            (5, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, box_color, 2)

                # 사람 추적 상태도 같이 표시
                p_found, p_offset = get_person_status()
                if p_found:
                    cv2.putText(vis, f"person offset={p_offset:+.2f}",
                                (5, vis.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 200, 0), 2)

                _publish_frame(vis)
            except Exception as e:
                print(f"[스트리밍] 프레임 생성 실패: {e}")

        # ===== YOLO 사람 추적 - 주기적으로만 =====
        if now - last_send >= YOLO_SEND_INTERVAL_SEC:
            last_send = now
            detections = _yolo_send_frame(frame)

            best_person = None
            for d in detections:
                if d['label'] != YOLO_TARGET_CLASS:
                    continue
                if d['conf'] < PERSON_CONF_MIN:
                    continue
                if best_person is None or d['conf'] > best_person['conf']:
                    best_person = d

            if best_person:
                x1, y1, x2, y2 = best_person['box']
                cx = (x1 + x2) / 2.0
                frame_w = frame.shape[1]
                p_offset = (cx - frame_w / 2.0) / (frame_w / 2.0)

                with _person_lock:
                    _person_offset = p_offset
                    _person_last_seen = now
                    _person_found = True
                print(f"[사람추적] 발견 offset={p_offset:+.2f} 확신도={best_person['conf']:.2f}")
            else:
                with _person_lock:
                    _person_found = False
                if detections:
                    names = ', '.join([f"{d['label']}({d['conf']:.2f})" for d in detections])
                    print(f"[YOLO] 사람 없음 (다른 인식: {names})")

        time.sleep(0.03)


def get_person_status():
    with _person_lock:
        fresh = (time.time() - _person_last_seen) < PERSON_LOST_TIMEOUT
        return _person_found and fresh, _person_offset


def start_camera_yolo():
    global _picam2, _cam_running, _cam_thread
    try:
        _picam2 = Picamera2()
        config = _picam2.create_preview_configuration(
            main={"size": (CAM_WIDTH, CAM_HEIGHT), "format": CAM_FORMAT})
        _picam2.configure(config)
        _picam2.start()
        time.sleep(1.0)

        _cam_running = True
        _cam_thread = threading.Thread(target=_camera_loop, daemon=True)
        _cam_thread.start()
        print("[카메라] YOLO 오프로딩 + 사람추적 + 엣지분석 스레드 시작 완료")
    except Exception as e:
        print(f"[카메라] 시작 실패: {e} - 카메라/YOLO 없이 주행 로직만 테스트합니다")


def stop_camera_yolo():
    global _cam_running
    _cam_running = False
    if _cam_thread:
        _cam_thread.join(timeout=1.5)
    if _picam2:
        try:
            _picam2.stop()
            _picam2.close()
        except Exception:
            pass
    if _yolo_sock:
        try:
            _yolo_sock.close()
        except Exception:
            pass


# =========================================================================
# ===== 기존 lidar_ultra_avoidance.py 로직 그대로 =====
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
    """
    라이다 연결 + 리셋. 시리얼 스트림이 꼬여있을 수 있으므로
    stop/reset 후 약간의 대기시간을 두고 health를 확인한다.
    (lidar_ultra_vision.py와 동일한 패턴)
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

    sonar = Ultrasonic(Pin("D2"), Pin("D3"))
    lidar = connect_lidar(LIDAR_PORT)

    # 카메라+YOLO+사람추적+엣지분석 스레드 시작
    start_camera_yolo()
    start_stream()   # 웹 스트리밍 서버 시작

    try:
        x.set_cam_tilt_angle(CAM_TILT_ANGLE)
        x.set_cam_pan_angle(0)
        print(f"[카메라] 틸트 각도 {CAM_TILT_ANGLE}도로 설정 완료")
    except Exception as e:
        print(f"[카메라] 짐벌 제어 실패: {e}")

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

    print("라이다+초음파 회피 + YOLO 사람추적 + 엣지분석 장애물회피 테스트 시작 (Ctrl+C 종료)")
    steer.angle(0)
    time.sleep(1)

    backCnt = 0
    isBack = False
    isBackFlag = False
    obstacle_repeat_count = 0   # ===== [반복회피 탈출 추가] =====
    BACK_TARGET = VELOCITY * 1.0   # 후진 토크 부족 문제로 최대치까지 상향 (VELOCITY=60 기준 약 47 -> 60)
                                    # 후방 거리 기반 안전로직(compute_dynamic_backspeed)이 이미
                                    # 후방 여유 없으면 후진량을 스스로 줄이므로 힘을 세게 잡아도 안전함

    backSpeed = 20 * (50 / BACK_TARGET)
    current_backSpeed = backSpeed
    steel_gain_result = 0

    try:
      while True:
        # ===== 라이다 재연결 루프 =====
        # 시리얼 스트림이 깨지면(RPLidarException) 여기서 잡아서
        # 라이다를 재연결하고 스캔을 이어간다. 프로그램 자체는 죽지 않는다.
        try:
          for scan in lidar.iter_scans(min_len=SCAN_MIN_LEN):
            batteryMonitor.show()
            loadMonitor.show()

            clear_angle, lidar_min = analyze_scan(scan)
            ultra_cm = read_ultra_cm()

            # ===== 후진 구간 (기존 그대로, 최우선) =====
            if (isBack and (backCnt <= current_backSpeed)):
                duty = back_incre_Move(BACK_TARGET)
                wallBackup.update(duty)
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

            # ===== 1순위: 정지 (라이다/초음파 즉각 반응, 기존 그대로) =====
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
                continue

            # ===== 2순위: 위험 → 감속 + 회피 조향 (기존 그대로) =====
            if lidar_min < DANGER_DIST_MM or (0 < ultra_cm < Get_Stop_Distance()):
                set_speed(SPEED_SLOW)
                steel_gain_result = clear_angle * STEER_GAIN
                set_steer(steel_gain_result)
                print(f"[회피] 라이다 {lidar_min:.0f}mm 초음파 {ultra_cm:.0f}cm 트인 {clear_angle:.0f}도")
                continue

            # ===== 2.5순위: 카메라 엣지분석 장애물 회피 =====
            obstacle_found, obstacle_offset = get_obstacle_status()

            if obstacle_found:
                avoid_dir = -1 if obstacle_offset > 0 else 1
                steer_cmd = avoid_dir * OBSTACLE_AVOID_STEER
                set_steer(steer_cmd)
                # ===== [즉시감속 추가] 회피는 안전이 걸린 상황이라 서서히 줄이지 않고
                # 그 즉시 목표 속도로 뚝 떨어뜨림 (기존 set_speed는 SPEED_STEP=3씩 천천히 줄어서
                # 감속 다 끝나기 전에 장애물 신호가 사라져 계속 고속으로 회피하는 문제가 있었음)
                SPEED_FAST = OBSTACLE_AVOID_SPEED
                left_motor.speed(-SPEED_FAST)
                right_motor.speed(SPEED_FAST)
                print(f"[엣지회피] 장애물 offset={obstacle_offset:+.2f} -> {avoid_dir}방향 회피 조향={steer_cmd:.0f}도")
                continue

            # ===== 3순위: 사람이 보이면 그 방향으로 저속 접근 =====
            person_found, person_offset = get_person_status()

            if person_found:
                steer_cmd = max(-STEER_LIMIT, min(STEER_LIMIT, person_offset * PERSON_STEER_GAIN))
                set_steer(steer_cmd)
                set_speed(PERSON_FOLLOW_SPEED)
                print(f"[사람추적] offset={person_offset:+.2f} 조향={steer_cmd:.0f}도 -> 접근 중")
                continue

            # ===== 4순위(기존): 아무것도 없을 때 -> 기존 라이다 자유주행 =====
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

            print(f"[주행] 라이다 {lidar_min:.0f}mm 초음파 {ultra_cm:.0f}cm 조향 {steer_cmd:.0f}도")

        except RPLidarException as e:
            # 시리얼 스트림 손상(New scan flags mismatch 등). 차는 안전하게 멈추고
            # 라이다만 재연결한 뒤 바깥 while 루프가 스캔을 다시 시작한다.
            # 프로그램은 죽지 않는다. (lidar_ultra_vision.py와 동일한 패턴)
            print(f"[라이다] 스캔 중 오류 발생: {e} -> 재연결 시도")
            set_decre_Move(0)
            set_steer(0)
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
        lidar.stop()
        lidar.stop_motor()
        lidar.disconnect()
        stop_camera_yolo()
        stop_stream()
        print(f"[CPU부하] 이번 실행 중 최고 부하: {loadMonitor.max_load1:.2f} (코어 {loadMonitor.cores}개)")
        print("정지 완료")


if __name__ == "__main__":
    main()
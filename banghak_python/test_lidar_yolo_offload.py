# test_lidar_yolo_person_follow.py
# [테스트용] 기존 라이다+초음파 자율주행 로직(1,2순위 안전 정지/회피)은 그대로 유지하면서
# YOLO로 "사람(person)"을 인식해서 그 방향으로 저속 접근하는 실험용 스크립트.
#
# 안전 우선순위:
#   1순위: 라이다/초음파 초근접 -> 정지+후진 (기존 로직, 즉각 반응)
#   2순위: 라이다/초음파 위험거리 -> 감속+회피 (기존 로직, 즉각 반응)
#   3순위: 사람이 보이면 -> 그 방향으로 저속 접근 (YOLO, 0.5초 지연 있음)
#   4순위: 사람도 없으면 -> 기존 라이다 자유주행
#
# [카메라 각도 추가] 카메라가 바닥만 보는 문제 해결 -> CAM_TILT_ANGLE 만큼 위로 들어줌
#
# 실행 전 준비:
#   1. PC(서버)에서 server.py를 먼저 실행해서 "대기 중..." 상태로 켜둘 것
#   2. 아래 YOLO_SERVER_IP를 PC의 실제 IP로 맞출 것
#
# 실행: python3 test_lidar_yolo_person_follow.py

import os
import time
import threading
import socket
import pickle

import cv2
from picamera2 import Picamera2

from rplidar import RPLidar
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

# ===== [카메라 각도 추가] =====
# 카메라가 바닥만 보고 있으면 사람을 잘 못 찾으므로 위로 들어줌.
# lidar_ultra_vision.py에서 쓰던 것과 동일한 패턴 (Picarx의 set_cam_tilt_angle 사용).
CAM_TILT_ANGLE = 10   # 위로 10도. 너무 많이 들면 바닥 장애물이 프레임 밖으로 벗어나니 필요시 조정


# =========================================================================
# ===== [YOLO 오프로딩 + 사람 추적 추가] 설정 =====
# =========================================================================

YOLO_SERVER_IP = '192.168.0.121'   # PC(서버)의 실제 IP로 수정
YOLO_SERVER_PORT = 5051

CAM_WIDTH = 320
CAM_HEIGHT = 240
CAM_FORMAT = "RGB888"       # picamera2에서 이 이름은 실제로 BGR 순서로 나옴

YOLO_SEND_INTERVAL_SEC = 0.5   # 사람 추적이라 조금 당김 (0.5초마다 서버 전송)
JPEG_QUALITY = 70

YOLO_TARGET_CLASS = 'person'      # 이 클래스만 추적
PERSON_FOLLOW_SPEED = 50          # 사람 쪽으로 갈 때 속도 (기존 VELOCITY보다 낮게 - 안전)
PERSON_STEER_GAIN = 30.0          # offset(-1~1) -> 조향각 변환 계수
PERSON_LOST_TIMEOUT = 1.5         # 이 시간 이상 못 찾으면 추적 포기
PERSON_CONF_MIN = 0.5             # 이 확신도 이상만 진짜로 인정 (오탐 필터링)

# 카메라 스레드와 메인(주행) 스레드가 공유하는 상태값들
_cam_running = False
_cam_thread = None
_picam2 = None

_yolo_sock = None
_yolo_connected = False

_person_lock = threading.Lock()
_person_offset = 0.0
_person_last_seen = 0.0
_person_found = False

# ===== [CPU 부하 확인용] =====
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


def _yolo_connect():
    """YOLO 서버(PC)에 TCP 연결 시도. 실패해도 프로그램은 계속 진행됨(주행에 영향 없음)."""
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
    """프레임 1장을 서버로 보내고 결과를 받아온다. 실패하면 조용히 빈 리스트 반환."""
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


def _camera_yolo_loop():
    """
    별도 스레드에서 독립적으로 도는 루프.
    - 카메라 프레임을 계속 캡처
    - YOLO_SEND_INTERVAL_SEC 간격으로만 서버에 전송
    - person 클래스만 골라서 화면 오프셋(offset) 계산 후 공유 변수에 저장
    - 이 스레드에서 무슨 일이 일어나도 메인 주행 루프는 절대 기다리지 않음
    """
    global _person_offset, _person_last_seen, _person_found

    _yolo_connect()
    last_send = 0.0

    while _cam_running:
        try:
            frame = _picam2.capture_array()
        except Exception as e:
            print(f"[카메라] 프레임 캡처 실패: {e}")
            time.sleep(0.1)
            continue

        now = time.time()
        if now - last_send >= YOLO_SEND_INTERVAL_SEC:
            last_send = now
            detections = _yolo_send_frame(frame)

            # person 클래스 중 확신도 제일 높은 것 하나만 목표로 삼음
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
                offset = (cx - frame_w / 2.0) / (frame_w / 2.0)   # -1.0(왼쪽) ~ +1.0(오른쪽)

                with _person_lock:
                    _person_offset = offset
                    _person_last_seen = now
                    _person_found = True
                print(f"[사람추적] 발견 offset={offset:+.2f} 확신도={best_person['conf']:.2f}")
            else:
                with _person_lock:
                    _person_found = False
                if detections:
                    names = ', '.join([f"{d['label']}({d['conf']:.2f})" for d in detections])
                    print(f"[YOLO] 사람 없음 (다른 인식: {names})")
                else:
                    print("[YOLO] 인식된 물체 없음")

        time.sleep(0.05)


def get_person_status():
    """메인 주행 루프에서 안전하게 조회하는 함수. (찾았는지, 최신 offset)"""
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
        _cam_thread = threading.Thread(target=_camera_yolo_loop, daemon=True)
        _cam_thread.start()
        print("[카메라] YOLO 오프로딩 + 사람추적 스레드 시작 완료")
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
    lidar = RPLidar(LIDAR_PORT)

    # [추가] 카메라+YOLO+사람추적 스레드 시작 (실패해도 주행 로직엔 영향 없음)
    start_camera_yolo()

    # ===== [카메라 각도 추가] 짐벌을 위로 CAM_TILT_ANGLE 만큼 들어줌 =====
    # lidar_ultra_vision.py와 동일한 방식. 팬(pan)은 정면(0도) 고정.
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

    print("라이다+초음파 회피 주행 + YOLO 사람추적 테스트 시작 (Ctrl+C 종료)")
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

            # ===== 1순위: 정지 (기존 그대로, 라이다/초음파 즉각 반응) =====
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

            # ===== [추가] 3순위: 사람이 보이면 그 방향으로 저속 접근 =====
            # 위의 1,2순위에서 이미 안전하지 않으면 continue로 빠져나갔으므로
            # 여기 도달했다는 것 자체가 "지금은 안전하다"는 뜻 -> 사람 추적 실행해도 안전
            person_found, person_offset = get_person_status()

            if person_found:
                steer_cmd = max(-STEER_LIMIT, min(STEER_LIMIT, person_offset * PERSON_STEER_GAIN))
                set_steer(steer_cmd)
                set_speed(PERSON_FOLLOW_SPEED)
                print(f"[사람추적] offset={person_offset:+.2f} 조향={steer_cmd:.0f}도 -> 접근 중")
                continue

            # ===== 4순위(기존): 사람 없을 때 -> 기존 라이다 자유주행 =====
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
        print(f"[CPU부하] 이번 실행 중 최고 부하: {loadMonitor.max_load1:.2f} (코어 {loadMonitor.cores}개)")
        print("정지 완료")


if __name__ == "__main__":
    main()
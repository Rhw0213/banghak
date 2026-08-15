# arm_visual_servo.py
# 팀원의 arm_setup.py(build_arm) + smooth_servo.py(SmoothJoint/move_all) 위에서
# 그리퍼 끝 빨간 마커 + 타겟 객체(노란색)를 보면서 base -> shoulder -> elbow 순서로
# 시각 피드백 정렬 후 픽업한다.
#
# 전제:
#   - 서보 나사 헐거움은 본드로 고정 완료 -> 이제 명령 각도를 신뢰할 수 있음
#     (이전 버전에 있던 블라인드 호밍/슬립 감지/마커탐색 로직은 더 이상 필요 없어 제거)
#   - 관절은 arm_setup.build_arm(servo_factory)로 생성한 SmoothJoint 4개를 그대로 받아서 사용
#     (min/max 각도 범위, 오프셋은 arm_setup.py에서 팀원이 이미 실측/지정)
#   - 카메라는 고정, 팔 전체 + 물체가 같이 보이는 위치 (eye-to-hand)
#
# 사용법:
#   1) calib 모드로 마커/타겟 검출 + 카메라 틸트 확인
#        python3 arm_visual_servo.py calib 60
#   2) visual_servo_pick(base, shoulder, elbow, gripper, grab_frame_fn) 호출
#      (네 개 모두 arm_setup.build_arm()이 반환한 SmoothJoint 객체)

import time
import threading
import json
import socket
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np


# =========================================================================
# ===== 빨간 마커 HSV 범위 (★ calib 모드로 실측 후 교체) =====
# =========================================================================
RED_H1_MIN, RED_H1_MAX = 0, 8
RED_H2_MIN, RED_H2_MAX = 170, 179
RED_S_MIN, RED_S_MAX = 100, 255
RED_V_MIN, RED_V_MAX = 100, 255
MIN_MARKER_AREA = 30

# ===== 타겟(노란색) HSV 범위 - lidar_ultra_vision.py의 COLOR_* 와 동일값 권장 =====
YELLOW_H_MIN, YELLOW_H_MAX = 20, 35
YELLOW_S_MIN, YELLOW_S_MAX = 100, 255
YELLOW_V_MIN, YELLOW_V_MAX = 100, 255
MIN_TARGET_AREA = 400

CAM_WIDTH = 320
CAM_HEIGHT = 240
CAM_FORMAT = "RGB888"
VS_CAM_TILT_ANGLE = 40          # 실측 완료 - 25도보다 35도가 훨씬 잘 보여서 재조정


class ServoTarget:
    """화면상 위치 정보 (마커/타겟 공용)"""
    def __init__(self, found=False, cx=0.0, cy=0.0, dx=0.0, dy=0.0,
                 width_px=0.0, box=None):
        self.found = found
        self.cx = cx
        self.cy = cy
        self.dx = dx            # -1.0(왼쪽) ~ +1.0(오른쪽) 정규화
        self.dy = dy            # -1.0(위) ~ +1.0(아래) 정규화
        self.width_px = width_px
        self.box = box


def _detect_blob(frame, mask, min_area):
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return ServoTarget(found=False)

    largest = max(contours, key=cv2.contourArea)
    area = cv2.contourArea(largest)
    if area < min_area:
        return ServoTarget(found=False)

    h, w = frame.shape[:2]
    bx, by, bw, bh = cv2.boundingRect(largest)
    cx = bx + bw / 2.0
    cy = by + bh / 2.0
    dx = (cx - w / 2.0) / (w / 2.0)
    dy = (cy - h / 2.0) / (h / 2.0)

    return ServoTarget(found=True, cx=cx, cy=cy, dx=dx, dy=dy,
                        width_px=float(bw), box=(bx, by, bw, bh))


def detect_red_marker(frame):
    """그리퍼 끝 빨간 마커 검출. (ServoTarget, mask) 반환"""
    blurred = cv2.GaussianBlur(frame, (5, 5), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

    mask1 = cv2.inRange(hsv, np.array([RED_H1_MIN, RED_S_MIN, RED_V_MIN]),
                              np.array([RED_H1_MAX, RED_S_MAX, RED_V_MAX]))
    mask2 = cv2.inRange(hsv, np.array([RED_H2_MIN, RED_S_MIN, RED_V_MIN]),
                              np.array([RED_H2_MAX, RED_S_MAX, RED_V_MAX]))
    mask = cv2.bitwise_or(mask1, mask2)

    kernel = np.ones((3, 3), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    info = _detect_blob(frame, mask, MIN_MARKER_AREA)
    return info, mask


def detect_yellow_target(frame):
    """타겟 객체(노란색) 검출. (ServoTarget, mask) 반환"""
    blurred = cv2.GaussianBlur(frame, (5, 5), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, np.array([YELLOW_H_MIN, YELLOW_S_MIN, YELLOW_V_MIN]),
                             np.array([YELLOW_H_MAX, YELLOW_S_MAX, YELLOW_V_MAX]))

    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    info = _detect_blob(frame, mask, MIN_TARGET_AREA)
    return info, mask


def _grab_both(grab_frame_fn):
    """마커+타겟을 같은 프레임에서 동시에 검출. 실패 시 (None, None, None).
    스트리밍이 켜져있고 보는 사람이 있으면, 여기서 자동으로 오버레이 프레임도 발행한다
    (정렬/접근 루프가 전부 이 함수를 거치므로 별도 훅 없이 항상 최신 화면이 나감)."""
    frame = grab_frame_fn()
    if frame is None:
        return None, None, None
    marker, _ = detect_red_marker(frame)
    target, _ = detect_yellow_target(frame)
    if _stream_clients > 0:
        _publish_frame(frame, marker, target)
    return frame, marker, target


# =========================================================================
# ===== 웹 스트리밍 (MJPEG) - lidar_ultra_vision.py와 같은 패턴 =====
#   브라우저에서 http://<라즈베리파이IP>:8001 접속하면 실시간으로
#   마커(빨강 박스)/타겟(노랑 박스) 검출 결과가 오버레이된 카메라 화면을 볼 수 있다.
# =========================================================================
STREAM_PORT = 8001            # lidar 쪽(8000)과 겹치지 않게 다른 포트 사용
STREAM_FPS = 10
STREAM_QUALITY = 60

_stream_lock = threading.Lock()
_stream_jpeg = None
_stream_clients = 0
_stream_server = None
_app_running = True

_telemetry = {"phase": "-", "info": "-"}
_telemetry_lock = threading.Lock()


def update_telemetry(**kwargs):
    with _telemetry_lock:
        _telemetry.update(kwargs)


def _draw_overlay(frame, marker, target):
    h, w = frame.shape[:2]
    out = frame.copy()
    cv2.line(out, (w // 2, 0), (w // 2, h), (200, 200, 200), 1)
    cv2.line(out, (0, h // 2), (w, h // 2), (200, 200, 200), 1)

    if marker is not None and marker.found and marker.box:
        bx, by, bw, bh = marker.box
        cv2.rectangle(out, (bx, by), (bx + bw, by + bh), (0, 0, 255), 2)
        cv2.putText(out, f"MARKER dx={marker.dx:+.2f} dy={marker.dy:+.2f} w={marker.width_px:.0f}px",
                    (bx, max(15, by - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1)

    if target is not None and target.found and target.box:
        bx, by, bw, bh = target.box
        cv2.rectangle(out, (bx, by), (bx + bw, by + bh), (0, 220, 255), 2)
        cv2.putText(out, f"TARGET dx={target.dx:+.2f} dy={target.dy:+.2f} w={target.width_px:.0f}px",
                    (bx, min(h - 5, by + bh + 15)), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 220, 255), 1)

    with _telemetry_lock:
        phase = _telemetry.get("phase", "-")
        info = _telemetry.get("info", "-")
    cv2.putText(out, f"{phase} | {info}", (5, h - 8),
                cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 255, 255), 1)
    return out


def _publish_frame(frame, marker, target):
    global _stream_jpeg
    img = _draw_overlay(frame, marker, target)
    ok, buf = cv2.imencode('.jpg', img, [int(cv2.IMWRITE_JPEG_QUALITY), STREAM_QUALITY])
    if ok:
        with _stream_lock:
            _stream_jpeg = buf.tobytes()


PAGE_HTML = """<!DOCTYPE html>
<html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Arm Visual Servo</title>
<style>
 body{font-family:sans-serif;background:#1a1a1a;color:#eee;margin:0;padding:12px}
 img{width:100%;max-width:640px;image-rendering:pixelated;border:1px solid #444}
 #status{background:#222;padding:8px;border-radius:4px;font-size:13px;margin-top:10px}
</style></head><body>
<img src="/stream.mjpg">
<div id="status">연결 중...</div>
<script>
async function poll(){
  try{
    const r=await fetch('/status');const s=await r.json();
    document.getElementById('status').textContent = `phase: ${s.phase}\\ninfo: ${s.info}`;
  }catch(e){}
}
setInterval(poll,400);poll();
</script></body></html>"""


class _StreamHandler(BaseHTTPRequestHandler):
    def log_message(self, *args):
        pass

    def do_GET(self):
        global _stream_clients
        if self.path == '/':
            body = PAGE_HTML.encode('utf-8')
            self.send_response(200)
            self.send_header('Content-Type', 'text/html; charset=utf-8')
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        elif self.path == '/status':
            with _telemetry_lock:
                data = dict(_telemetry)
            body = json.dumps(data).encode('utf-8')
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        elif self.path == '/stream.mjpg':
            self.send_response(200)
            self.send_header('Age', '0')
            self.send_header('Cache-Control', 'no-cache, private')
            self.send_header('Pragma', 'no-cache')
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=FRAME')
            self.end_headers()
            _stream_clients += 1
            try:
                while _app_running:
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
# ===== 이동 속도 설정 =====
#   base/shoulder/elbow: 본드로 고정한 영점이 어긋나지 않도록 느리게(슬로우)
#   gripper: 물체를 확실히 물어야 하므로 닫을 때는 빠르게(스냅)
# =========================================================================
ALIGN_SPEED = 18.0       # 정렬용 미세 이동 속도 (도/초) - 이전 대비 1.5배
ALIGN_STEP = 0.5         # 정렬용 한 스텝 각도 - 더 촘촘하게
HOME_SPEED = 36.0        # 호밍/복귀 속도 - 토크 부족으로 0도까지 못 가는 문제 있어 2배로 상향 (ALIGN_SPEED는 그대로 유지)
def _snap_gripper(gripper, target_angle):
    """
    그리퍼는 뻑뻑해서 SmoothJoint.move_to()의 스텝별 저속 이동으로는 안 열리고/안 닫힐 수 있음.
    move_to()를 아예 거치지 않고, 실제 서보(gripper.servo)에 목표각을 한 번에 직접
    명령해서 하드웨어가 낼 수 있는 최고 속도로 즉시 움직이게 한다.
    (SmoothJoint가 내부적으로 기억하는 .angle 값도 그대로 갱신해 이후 로직과 안 어긋나게 함)
    """
    gripper.servo.angle(target_angle)
    gripper.angle = target_angle

VS_DX_TOL = 0.05             # base 정렬 완료 판정
VS_DY_TOL = 0.13             # shoulder 정렬 완료 판정
VS_MAX_STEPS_PER_PHASE = 80   # [더 이상 안 씀] 순차 정렬 방식(_align_axis)에서 쓰던 값, 인터리브로 교체하며 미사용

# ★ 실측 완료: 그랩 위치(팔이 물체를 실제로 집을 수 있는 자세)에서
#   "마커(그리퍼)"의 화면상 폭 ≈ 61px.
#   ※ 이전 버전은 target.width_px(타겟 자체의 화면 폭)를 깊이 신호로 썼는데,
#     카메라가 팔에 안 달려있고 고정(eye-to-hand)이라 팔이 움직여도 카메라와
#     타겟 사이 거리는 거의 안 변해 target.width_px가 사실상 안 바뀌는 문제가
#     있었음. 그래서 팔에 붙어 움직이는 "마커"의 화면 폭으로 깊이 신호를 교체.
# ★ 실측 완료(35도 틸트): 그랩 위치에서 marker.width_px ≈ 58px
MARKER_WIDTH_GRAB = 58.0
MARKER_WIDTH_TOL = 6.0       # 이 오차 이내면 도달로 판정

# ★ 실측 완료(35도 틸트): 실제로 집을 수 있는 자세에서
#   (target.dx - marker.dx) ≈ +0.58, (target.dy - marker.dy) ≈ +0.01
#   마커를 그리퍼의 실제 무는 지점이 아니라 옆/뒤쪽에 붙여놨기 때문에,
#   화면상으로는 마커와 타겟이 정확히 안 겹치는 게 "정상"인 상태다.
#   그동안 이 오프셋을 0으로 잘못 가정해서, 실제 집을 수 있는 지점을 지나쳐서
#   계속 밀어붙이는 문제가 있었음 (base_done/shoulder_done 판정에 반영해 수정).
#   ※ 이건 마커-타겟 사이의 상대적 오프셋이라 물체 위치가 바뀌어도 대체로
#     유지될 것으로 예상되지만(카메라 각도/접근 방식이 비슷하면), 물체를
#     많이 다른 위치로 옮기면 한 번씩 재확인해볼 것.
GRASP_DX_OFFSET = 0.58
GRASP_DY_OFFSET = 0.01
OCCLUSION_WIDTH_TOL = 20.0   # 타겟이 안 보일 때 "마커가 가려서 그런 것"으로 볼 폭 오차 허용치 (MARKER_WIDTH_TOL보다 훨씬 넉넉하게)
ELBOW_PROBE_STEP = 1.5       # elbow가 마커를 키우는/줄이는 방향인지 확인용 첫 스텝

# ★ 실측 완료(base) / 미확인(shoulder, elbow): 각 축의 "+각도 명령"이
#   화면상 어느 방향으로 마커를 움직이는지는 하드웨어/카메라 배치마다 다르다.
#   실측 결과 base는 각도를 올릴수록 마커가 화면상 반대(음수) 방향으로 움직이는
#   것으로 확인되어 부호를 반전(-1)했다. shoulder/elbow도 이상하게 발산하면
#   여기 부호를 뒤집어서 테스트할 것.
BASE_ALIGN_SIGN = -1
SHOULDER_ALIGN_SIGN = 1

LOST_MAX_RETRY = 5                   # 마커/타겟 놓쳤을 때 재시도 프레임 수
MARKER_VISIBLE_CONFIRM_FRAMES = 3    # 호밍 후 마커 확인 시 노이즈 방지용 연속 확인 수

# ===== 인터리브 접근(base/shoulder/elbow를 매 스텝 같이 움직이는 방식) 설정 =====
INTERLEAVE_MAX_STEPS = 150      # 전체 인터리브 루프 최대 스텝 (3축을 같이 다루니 순차 방식보다 넉넉하게)
COAST_MAX_STEPS = 15            # 타겟이 안 보일 때, 마지막 오차값으로 계속 진행("관성 진행")을 허용하는 최대 횟수
EDGE_STUCK_MAX = 10             # 화면 경계에 걸려서 한 축의 이동을 계속 건너뛸 수 있는 최대 횟수
OCCLUSION_POSITION_TOL = 0.25   # 마커가 타겟의 마지막 위치에 얼마나 가까워야 "가려짐"으로 인정할지

# 마커가 화면 가장자리에 너무 가까워지면(화면 밖으로 나가기 직전) 그 축은
# 더 이상 움직이지 않고 안전 정지한다. dx/dy 정규화 값 기준(-1.0~+1.0).
FRAME_EDGE_LIMIT = 0.85
EDGE_IMPROVE_EPS = 0.01   # [더 이상 안 씀] 개별축 판단 방식에서 쓰던 값
TOTAL_IMPROVE_EPS = 0.01  # 전체 오차 합이 이 정도 이상 줄어야 "개선 중"으로 인정


def _marker_near_edge(marker):
    return marker is not None and marker.found and \
        (abs(marker.dx) > FRAME_EDGE_LIMIT or abs(marker.dy) > FRAME_EDGE_LIMIT)

# ★ 실측 결과 반영: 기존 가정(open=-20, close=10)이 반대로 동작해서 방향 교체
# ★ 실측 완료: 0도는 팀원이 잡아둔 영점(그리퍼 열림)으로 항상 정확하게 돌아감.
#   닫힘은 arm_setup.py에 정의된 안전범위(-20)보다 훨씬 더 내려간 -50을 일부러
#   명령한다 - 뻑뻑해서 중간값(-25,-30,-40 등)은 매번 실제로 움직이는 양이
#   들쭉날쭉했지만, 기구적 한계를 확실히 넘어서는 값을 주면 정지 마찰 편차와
#   상관없이 항상 물리적 하드스톱까지 밀려서 결과가 일정해짐 (실측 확인됨).
#   ※ 이 값은 _snap_gripper()로만 명령해야 함 - move_to()를 쓰면 arm_setup.py의
#     clamp(min=-20)에 걸려 -50까지 못 감.
# ★ 실측 완료(본드칠 이후): 10=열림, -20=닫힘. 중간값은 여전히 뻑뻑해서
#   못 쓰고, 어차피 완전열림/완전닫힘 두 상태만 쓰므로 이 두 값만 사용.
#   arm_setup.py의 안전범위(min=-30, max=0)와는 다른 값이지만, _snap_gripper()로
#   clamp를 우회해서 명령하므로 이 실측값 그대로 사용 가능.
GRAB_OPEN_ANGLE = 10
GRAB_CLOSE_ANGLE = -20

# 팀원이 arm_setup.py에서 base 0도 오프셋을 카메라 시야에 맞게 이미 잡아줬으므로
# 우리 쪽 추가 보정은 불필요 (0으로 비활성화). 나중에 팀원 오프셋이 바뀌어서
# 다시 시야 밖으로 나가면 이 값을 사용해 임시로 보정할 수 있음.
BASE_HOME_OFFSET = 0.0


def _is_marker_reliably_visible(grab_frame_fn, confirm_frames=MARKER_VISIBLE_CONFIRM_FRAMES,
                                  check_delay=0.1):
    """단일 프레임 판단은 노이즈로 오탐될 수 있어 여러 프레임 확인 후 판단."""
    hits = 0
    for _ in range(confirm_frames):
        frame, marker, _ = _grab_both(grab_frame_fn)
        if frame is not None and marker.found:
            hits += 1
        time.sleep(check_delay)
    return hits > confirm_frames / 2


def home_arm(base, shoulder, elbow, gripper, grab_frame_fn=None, check_marker=True):
    """
    픽업 시퀀스 시작 전(또는 종료 후) 호출.
    서보가 이제 신뢰 가능하므로, 기준 자세(0,0,0)로 천천히(HOME_SPEED) 복귀시킨 뒤,
    카메라 시야 확보를 위해 base만 우리 쪽 보정값(BASE_HOME_OFFSET)만큼 추가로 돌린다.
    (팀원의 base 0도 정의 자체는 건드리지 않고, 우리 비주얼 서보잉 루틴 안에서만 적용)

    check_marker=False로 부르면 마커 안 보임 경고를 안 띄운다. 0도(레이더 안 가리는
    대기 자세)는 카메라 틸트를 낮춘 뒤로는 원래 마커가 안 보이는 게 정상이라서,
    픽업 시작 전 호출에서는 이 경고가 오히려 혼란만 준다 (실제 마커 확인은
    _descend_until_visible()이 담당).

    ※ move_all()을 안 쓰고 관절마다 move_to()를 개별 호출한다. 이유:
      스크립트를 새로 실행하면 SmoothJoint는 실제 위치 확인 없이 무조건
      init_angle=0으로 시작하는데(팀원 파일 move_on_init=False), 팔이 지난
      실행 끝난 위치에 그대로 남아있어도 소프트웨어는 "이미 0도"라고 착각한다.
      move_all()은 이 계산상 이동거리가 0이면 아예 명령을 안 보내고 조용히
      리턴해버려서 실제로는 팔이 하나도 안 움직이는 문제가 있었다.
      반면 move_to()는 거리가 0으로 계산돼도 항상 마지막에 실제 서보 명령을
      보내므로 이 문제가 없다. (동시에 움직이지 않고 base->shoulder->elbow
      순서로 하나씩 움직이는 차이는 있음 - 호밍은 속도가 느려서 문제 없음)
    """
    print("[비주얼서보] 호밍 - 기준 자세(0,0,0)로 복귀")
    update_telemetry(phase="home", info="returning to base pose")
    base.move_to(0, speed=HOME_SPEED, step=ALIGN_STEP)
    shoulder.move_to(0, speed=HOME_SPEED, step=ALIGN_STEP)
    elbow.move_to(0, speed=HOME_SPEED, step=ALIGN_STEP)

    if BASE_HOME_OFFSET != 0:
        print(f"[비주얼서보] base 카메라 시야 보정 {BASE_HOME_OFFSET:+.1f}도")
        base.move_to(base.angle + BASE_HOME_OFFSET, speed=HOME_SPEED, step=ALIGN_STEP)

    _snap_gripper(gripper, GRAB_OPEN_ANGLE)
    time.sleep(0.2)

    if check_marker and grab_frame_fn is not None and not _is_marker_reliably_visible(grab_frame_fn):
        print("[비주얼서보] 경고 - 호밍 후에도 마커가 안 보입니다. "
              "카메라 각도/팔 위치를 확인하세요.")

    print(f"[비주얼서보] 호밍 완료 base={base.angle:.1f} "
          f"shoulder={shoulder.angle:.1f} elbow={elbow.angle:.1f}")


RETURN_SHOULDER_LIFT_FRACTION = 0.7   # 1차 들어올리기: shoulder는 많이
RETURN_ELBOW_LIFT_FRACTION = 0.15     # 1차 들어올리기: elbow는 살짝만 (많이 접으면 뒤로 끌림)


def _return_home_slow(base, shoulder, elbow):
    """
    작업(픽업 시도) 종료 후, 그리퍼는 건드리지 않고 base/shoulder/elbow만
    슬로우스타트로 0도(레이더 안 가리는 대기 자세)로 복귀시킨다.
    (그리퍼는 성공 시 물건을 든 채로 유지, 실패 시엔 이미 각 실패 경로에서
    알아서 열어뒀으므로 여기서는 손대지 않음)

    순서 중요: base부터 돌리면 아직 shoulder/elbow가 낮은 상태라 그리퍼(또는
    집은 물체)가 바닥에 끌릴 수 있다. 그래서 1) shoulder 위주로 먼저 들어올려서
    바닥에서 뗀 다음(elbow는 조금만 - 같이 많이 접으면 오히려 뒤로 끌림),
    2) base를 돌리고, 3) 마저 완전히 들어올린다.
    """
    print("[비주얼서보] 작업 종료 - 대기 자세(0,0,0)로 슬로우스타트 복귀")
    update_telemetry(phase="return", info="lifting before base rotates")

    # 1) shoulder 위주로 먼저 들어올려 바닥에서 뗌 (elbow는 살짝만)
    mid_shoulder = shoulder.angle * (1 - RETURN_SHOULDER_LIFT_FRACTION)
    mid_elbow = elbow.angle * (1 - RETURN_ELBOW_LIFT_FRACTION)
    shoulder.move_to(mid_shoulder, speed=HOME_SPEED, step=ALIGN_STEP)
    elbow.move_to(mid_elbow, speed=HOME_SPEED, step=ALIGN_STEP)

    # 2) 어느 정도 들린 상태에서 base 복귀
    update_telemetry(phase="return", info="rotating base")
    base.move_to(0, speed=HOME_SPEED, step=ALIGN_STEP)
    if BASE_HOME_OFFSET != 0:
        base.move_to(base.angle + BASE_HOME_OFFSET, speed=HOME_SPEED, step=ALIGN_STEP)

    # 3) 나머지 마저 들어올려서 완전히 0도로
    update_telemetry(phase="return", info="finishing lift")
    shoulder.move_to(0, speed=HOME_SPEED, step=ALIGN_STEP)
    elbow.move_to(0, speed=HOME_SPEED, step=ALIGN_STEP)

    print(f"[비주얼서보] 복귀 완료 base={base.angle:.1f} "
          f"shoulder={shoulder.angle:.1f} elbow={elbow.angle:.1f}")


# ===== 하강 탐색 설정 =====
# 대기 자세(0도)는 카메라 틸트를 낮춘 뒤로는 마커가 화면 위쪽 밖에 있어서 안 보임.
# shoulder+elbow를 천천히 같이 뻗어 내리면서 마커가 보일 때까지 진행한다.
DESCEND_STEP_DEG = 1.0
DESCEND_SPEED = 12.0          # 하강 탐색 속도 - 이전 대비 1.5배
DESCEND_MAX_STEPS = 60
# ★ 실측 필요: 이 부호로 shoulder/elbow를 움직이면 실제로 아래(카메라 시야)로
#   내려가는지 첫 실행에서 눈으로 확인. 반대로 더 위로 올라가버리면 -1로 뒤집을 것.
DESCEND_DIRECTION = 1


def _descend_until_visible(shoulder, elbow, grab_frame_fn,
                            direction=DESCEND_DIRECTION, max_steps=DESCEND_MAX_STEPS):
    """
    대기 자세(0도)에서 마커가 화면 밖에 있을 때, shoulder+elbow를 천천히 같이
    뻗어 내리면서 마커가 카메라에 들어올 때까지 진행한다.

    단일 프레임만 보고 "발견"으로 판단하면 노이즈(순간 블러/흔들림)로 오판해서
    실제로는 안정적으로 안 보이는데 너무 일찍 멈춰버릴 수 있다 - 그래서
    _is_marker_reliably_visible()로 여러 프레임 연속 확인 후에만 정지한다.
    """
    print("[비주얼서보] 대기 자세 -> 마커가 보일 때까지 천천히 하강")
    update_telemetry(phase="descend", info="lowering until marker visible")

    if _is_marker_reliably_visible(grab_frame_fn):
        return True, "이미 마커가 보임 - 하강 불필요"

    for step in range(max_steps):
        _paced_step(shoulder, direction, speed=DESCEND_SPEED, step=DESCEND_STEP_DEG)
        _paced_step(elbow, direction, speed=DESCEND_SPEED, step=DESCEND_STEP_DEG)

        if _is_marker_reliably_visible(grab_frame_fn):
            print(f"[비주얼서보] {step + 1}스텝 하강 후 마커 발견 "
                  f"(shoulder={shoulder.angle:.1f} elbow={elbow.angle:.1f})")
            return True, f"{step + 1}스텝 하강 후 마커 발견"

    return False, f"{max_steps}스텝 하강해도 마커를 찾지 못함"





def _paced_step(joint, direction, speed=ALIGN_SPEED, step=ALIGN_STEP):
    """
    joint를 딱 한 스텝(step도)만큼 움직인다.
    SmoothJoint.move_to()는 이동거리가 step 이하면 내부 while 루프(속도 조절 담당)를
    아예 안 돌고 즉시 점프해버려서, 우리처럼 매번 한 스텝씩만 요청하는 방식에서는
    속도 제한이 무시되고 홱홱 움직이는 문제가 생긴다. 그래서 여기서 직접
    step/speed 만큼 재워서 의도한 속도가 지켜지게 한다.
    """
    joint.move_to(joint.angle + direction * step, speed=speed, step=step)
    time.sleep(step / speed)


def _verify_grab(shoulder, elbow, elbow_direction, grab_frame_fn,
                  shoulder_lift_deg=25, elbow_lift_deg=18, settle_time=0.5,
                  target_move_min=0.06, marker_move_min=0.03):
    """
    그랩 성공 여부를 서보 각도가 아니라 카메라로 검증.
    그랩 닫은 직후 무조건 shoulder를 들어올리고 elbow도 접근 때와 반대 방향으로
    빼면서(retract), 들어올리기 전/후 "타겟(물체) 위치"가 바뀌었는지를 1차
    기준으로 판단한다.

    ※ 왜 마커가 아니라 타겟 위치를 1차 기준으로 쓰는가:
      그랩에 성공하면 물체가 그리퍼 손가락 사이에 물리면서 마커(빨간 표시)
      자체가 가려지거나 일부만 보이는 경우가 흔하다. 그러면 "성공했을 때일수록
      마커 데이터가 불안정해지는" 역설이 생겨서, 마커 이동량 기준 판단은
      성공 케이스에서 오히려 자주 "판단 불가"가 나온다. 반대로 타겟(물체)
      위치는 실제로 들렸는지 아닌지를 더 직접적으로 반영한다.

    ※ 3가지 결과("success"/"fail"/"unknown")로 구분한다. 애매한 상황을
      "fail"로 몰아버리면 실제로 잘 집었는데도 그리퍼를 다시 열어 물체를
      놓치는 심각한 문제가 생긴다 (실제로 겪은 버그) - 확실한 반증이 있을
      때만 "fail", 그 외엔 "unknown"으로 분류해 그리퍼를 그대로 둔다.

    Returns:
        (outcome: "success"|"fail"|"unknown", reason: str)
    """
    frame, marker_before, target_before = _grab_both(grab_frame_fn)
    if frame is None:
        return "unknown", "검증 불가 - 카메라 프레임 획득 실패"

    target_before_found = target_before.found
    target_before_dx = target_before.dx if target_before_found else None
    target_before_dy = target_before.dy if target_before_found else None
    marker_before_found = marker_before.found
    marker_before_dy = marker_before.dy if marker_before_found else None

    # shoulder는 들어올리는 방향(+), elbow는 접근 때(elbow_direction)와 반대 방향으로
    # 빼서(retract) 같이 움직인다 - 두 관절이 함께 움직이면 물체 무게로 뻑뻑해도
    # 훨씬 뚜렷한 화면 변화가 생겨 판단이 쉬워진다.
    shoulder.move_to(shoulder.angle + shoulder_lift_deg, speed=ALIGN_SPEED, step=ALIGN_STEP)
    elbow.move_to(elbow.angle - elbow_direction * elbow_lift_deg, speed=ALIGN_SPEED, step=ALIGN_STEP)
    time.sleep(settle_time)

    frame, marker_after, target_after = _grab_both(grab_frame_fn)
    if frame is None:
        return "unknown", "검증 불가 - 들어올린 후 프레임 획득 실패"

    # 1) 들어올리기 전엔 타겟이 보였는데 이후 사라짐 -> 그리퍼에 들려 시야가
    #    가려진 것으로 보고 성공 추정
    if target_before_found and not target_after.found:
        return "success", "검증: 들어올린 후 타겟이 사라짐 (그리퍼에 들려 시야 가려짐 - 그랩 성공 추정)"

    # 2) 들어올리기 전부터 타겟이 안 보였으면 애초에 판단 근거가 없음 - 애매함
    if not target_before_found:
        return "unknown", "검증 불가 - 들어올리기 전부터 타겟이 안 보임"

    # 3) 둘 다 보임 -> 타겟 위치 변화량(1차 기준)
    target_dx_change = target_after.dx - target_before_dx
    target_dy_change = target_after.dy - target_before_dy
    target_move = (target_dx_change ** 2 + target_dy_change ** 2) ** 0.5

    print(f"[검증] 타겟 이동량={target_move:.3f} (dx변화={target_dx_change:+.3f} "
          f"dy변화={target_dy_change:+.3f})")

    if target_move >= target_move_min:
        return "success", (f"검증 성공 - 타겟이 실제로 움직임 "
                           f"(이동량={target_move:.3f}, dx={target_dx_change:+.2f} dy={target_dy_change:+.2f})")

    # 타겟이 거의 안 움직였음 - 마커가 신뢰 가능하면(안 가려졌으면) 그걸로 확실한 실패 판정 시도
    if marker_before_found and marker_after.found:
        marker_dy_change = marker_after.dy - marker_before_dy
        if abs(marker_dy_change) >= marker_move_min:
            # 그리퍼(마커)는 확실히 움직였는데 물체는 그대로 -> 확실한 실패
            return "fail", (f"검증 실패(확실함) - 그리퍼는 움직였는데 타겟이 제자리 "
                            f"(marker_dy={marker_dy_change:+.2f}, target 이동량={target_move:.3f})")

    # 마커도 신뢰 못 하고 타겟도 거의 안 움직임 - 확신 없음
    return "unknown", f"검증 불가 - 타겟도 거의 안 움직이고 마커도 판단 근거 부족 (타겟 이동량={target_move:.3f})"


def _interleaved_approach(base, shoulder, elbow, gripper, grab_frame_fn, elbow_direction):
    """
    base/shoulder/elbow를 순서대로 끝까지 움직이는 대신, 매 스텝마다 세 축을
    조금씩 같이 움직이면서 dx/dy/폭 오차를 동시에 줄여나간다.

    장점:
      - 한 축이 끝까지 움직이는 동안 화면을 오래 가로지르면서 마커/타겟이
        우연히 겹쳐(가려) 보이는 상황 자체가 줄어든다.
      - elbow 접근 로직에 이미 있던 "타겟이 안 보이면 미세보정" 패턴을
        base/shoulder에도 자연스럽게 확장한 형태.

    타겟이 일시적으로 안 보이면(가려짐/노이즈) 무조건 멈추지 않고, 마지막으로
    알고 있던 오차값 기준으로 "관성 진행"하다가 다시 보이면 갱신한다.
    단, 마커가 타겟의 마지막 위치+목표 폭에 충분히 가까우면 "그리퍼에 가려진
    것"으로 보고 바로 그랩을 시도한다 (elbow 단계 전용이 아니라 언제든 해당).
    """
    lost_count = 0     # 마커 자체를 못 찾는 경우 (완전 상실)
    coast_count = 0     # 마커는 보이는데 타겟만 안 보이는 상태가 이어지는 횟수
    stuck_count = 0     # 화면 경계 근처에서 "전체 오차"가 정체되는 상태가 이어지는 횟수

    last_dx_err, last_dy_err, last_width_error = None, None, None
    last_abs_dx, last_abs_dy = None, None   # 참고용 (현재는 total_error 판단에는 안 씀)
    last_total_error = None
    last_target_dx, last_target_dy = None, None

    for step in range(INTERLEAVE_MAX_STEPS):
        frame, marker, target = _grab_both(grab_frame_fn)

        if frame is None or marker is None or not marker.found:
            lost_count += 1
            print(f"[인터리브] 마커 놓침 ({lost_count}/{LOST_MAX_RETRY})")
            if lost_count >= LOST_MAX_RETRY:
                return False, "마커를 완전히 놓쳐서 중단"
            time.sleep(0.2)
            continue
        lost_count = 0

        target_visible = target is not None and target.found

        if target_visible:
            coast_count = 0
            last_target_dx, last_target_dy = target.dx, target.dy
            # 그랩 목표는 (0,0)이 아니라 실측한 오프셋(GRASP_DX/DY_OFFSET) -
            # 마커가 그 오프셋만큼 떨어져 있어야 실제로 집을 수 있는 자세임
            dx_err = (target.dx - marker.dx) - GRASP_DX_OFFSET
            dy_err = (target.dy - marker.dy) - GRASP_DY_OFFSET
            width_error = MARKER_WIDTH_GRAB - marker.width_px
            last_dx_err, last_dy_err, last_width_error = dx_err, dy_err, width_error
        else:
            # 타겟이 안 보임: "가려짐(=거의 다 왔다)"인지 "그냥 일시 상실"인지 판단
            width_close = abs(MARKER_WIDTH_GRAB - marker.width_px) <= OCCLUSION_WIDTH_TOL
            position_close = (
                last_target_dx is not None and
                abs((last_target_dx - marker.dx) - GRASP_DX_OFFSET) <= OCCLUSION_POSITION_TOL and
                abs((last_target_dy - marker.dy) - GRASP_DY_OFFSET) <= OCCLUSION_POSITION_TOL
            )
            if width_close and position_close:
                print(f"[인터리브] 타겟이 마커에 가려진 것으로 판단 "
                      f"(marker dx={marker.dx:+.2f} dy={marker.dy:+.2f} "
                      f"width={marker.width_px:.0f}px) -> 그랩 시도")
                update_telemetry(phase="grab", info="occluded by gripper, closing")
                _snap_gripper(gripper, GRAB_CLOSE_ANGLE)
                time.sleep(0.4)
                outcome, verify_reason = _verify_grab(shoulder, elbow, elbow_direction, grab_frame_fn)
                print(f"[비주얼서보] {verify_reason}")
                if outcome == "fail":
                    _snap_gripper(gripper, GRAB_OPEN_ANGLE)
                    return False, "그랩 실패 (카메라 검증) - " + verify_reason
                # "success" 또는 "unknown"(애매함) 모두 그리퍼를 그대로 유지
                # (판단 불확실하다고 무조건 열면, 실제로는 집었는데 놓치는 사고가 남)
                return True, "픽업 완료 - " + verify_reason

            if last_dx_err is None:
                # 아직 타겟을 한 번도 못 본 상태 - 관성 진행할 기준값이 없음
                lost_count += 1
                print(f"[인터리브] 타겟 상실(기준값 없음) ({lost_count}/{LOST_MAX_RETRY})")
                if lost_count >= LOST_MAX_RETRY:
                    return False, "타겟을 완전히 놓쳐서 중단"
                time.sleep(0.2)
                continue

            coast_count += 1
            print(f"[인터리브] 타겟 일시 상실 - 마지막 오차로 관성 진행 "
                  f"({coast_count}/{COAST_MAX_STEPS})")
            if coast_count >= COAST_MAX_STEPS:
                return False, "타겟 상실 상태가 너무 오래 지속됨 - 중단"
            dx_err, dy_err, width_error = last_dx_err, last_dy_err, last_width_error

        base_done = abs(dx_err) < VS_DX_TOL
        shoulder_done = abs(dy_err) < VS_DY_TOL
        elbow_done = abs(width_error) <= MARKER_WIDTH_TOL

        print(f"[인터리브] step={step} dx={dx_err:+.3f}({'OK' if base_done else '-'}) "
              f"dy={dy_err:+.3f}({'OK' if shoulder_done else '-'}) "
              f"w_err={width_error:+.1f}({'OK' if elbow_done else '-'}) "
              f"{'[관성]' if not target_visible else ''}")
        update_telemetry(phase="interleave",
                         info=f"step={step} dx={dx_err:+.2f} dy={dy_err:+.2f} w_err={width_error:+.1f}")

        if base_done and shoulder_done and elbow_done:
            print("[비주얼서보] 3축 모두 수렴 -> 그랩 닫기 (스냅)")
            update_telemetry(phase="grab", info="closing")
            _snap_gripper(gripper, GRAB_CLOSE_ANGLE)
            time.sleep(0.4)
            outcome, verify_reason = _verify_grab(shoulder, elbow, elbow_direction, grab_frame_fn)
            print(f"[비주얼서보] {verify_reason}")
            if outcome == "fail":
                _snap_gripper(gripper, GRAB_OPEN_ANGLE)
                return False, "그랩 실패 (카메라 검증) - " + verify_reason
            # "success" 또는 "unknown"(애매함) 모두 그리퍼를 그대로 유지
            return True, "픽업 완료 - " + verify_reason

        # 화면 경계 안전장치: 개별 축 하나만 놓고 "이 축이 안 나아진다"고 판단하면
        # 오판이 잦다 - shoulder/elbow처럼 서로 다른 관절이 화면상 같은 방향(dy)에
        # 동시에 영향을 줘서 서로 상쇄되는 경우, 한쪽만 보면 "정체"처럼 보이지만
        # 실제로는 다른 축이 수렴하고 나면 자연히 풀리는 일시적 현상일 수 있다.
        # 그래서 축 개별 판단 대신 "3축 오차 합이 전혀 안 줄어드는지"로 전체
        # 정체 여부를 판단한다 - 이게 진짜 "더 이상 못 나아가는" 상황이다.
        cur_abs_dx = abs(marker.dx)
        cur_abs_dy = abs(marker.dy)
        total_error = abs(dx_err) + abs(dy_err) + abs(width_error) / 30.0  # 폭(px) 단위를 나머지와 비슷한 스케일로 보정
        total_improving = last_total_error is None or total_error < last_total_error - TOTAL_IMPROVE_EPS

        near_edge_dx = cur_abs_dx > FRAME_EDGE_LIMIT
        near_edge_dy = cur_abs_dy > FRAME_EDGE_LIMIT
        if near_edge_dx or near_edge_dy:
            if total_improving:
                stuck_count = 0
            else:
                stuck_count += 1
                print(f"[인터리브] 화면 경계 근처인데 전체 오차도 정체 중 "
                      f"(total_error={total_error:.3f}) ({stuck_count}/{EDGE_STUCK_MAX})")
                if stuck_count >= EDGE_STUCK_MAX:
                    return False, "화면 경계 근처에서 전체 오차가 여러 스텝째 개선되지 않아 중단"
        else:
            stuck_count = 0

        if not base_done:
            direction = BASE_ALIGN_SIGN * (1 if dx_err > 0 else -1)
            _paced_step(base, direction)

        if not shoulder_done:
            direction = SHOULDER_ALIGN_SIGN * (1 if dy_err > 0 else -1)
            _paced_step(shoulder, direction)

        if not elbow_done:
            _paced_step(elbow, elbow_direction)

        last_abs_dx, last_abs_dy = cur_abs_dx, cur_abs_dy
        last_total_error = total_error

    return False, f"인터리브 접근 최대 스텝({INTERLEAVE_MAX_STEPS}) 초과 - 수렴 실패"


def visual_servo_pick(base, shoulder, elbow, gripper, grab_frame_fn):
    """
    호밍(대기 자세) -> 하강 탐색(마커 보일 때까지) -> elbow 방향탐색
    -> 인터리브 접근(base/shoulder/elbow 동시 정렬) -> 그랩+검증
    -> 슬로우스타트로 대기 자세 복귀.

    Args:
        base, shoulder, elbow, gripper: arm_setup.build_arm()이 반환한 SmoothJoint 4개
        grab_frame_fn: 인자 없이 호출하면 BGR numpy 프레임을 반환하는 함수

    Returns:
        (success: bool, reason: str)
    """
    print("[비주얼서보] 픽업 시퀀스 시작")
    # 대기 자세(0도)는 카메라 틸트를 낮춘 뒤로는 마커가 안 보이는 게 정상이라
    # 여기서는 마커 확인 경고를 끔 (실제 확인은 아래 _descend_until_visible이 담당)
    home_arm(base, shoulder, elbow, gripper, grab_frame_fn=None, check_marker=False)

    # ---------- 하강 탐색: 마커가 카메라에 들어올 때까지 shoulder+elbow 천천히 뻗기 ----------
    ok, reason = _descend_until_visible(shoulder, elbow, grab_frame_fn)
    if not ok:
        _return_home_slow(base, shoulder, elbow)
        return False, reason

    # ---------- elbow 방향 탐색 (본격 접근 전 1회) ----------
    # eye-to-hand 구조라 target.width_px는 팔이 움직여도 거의 안 변하므로,
    # 팔에 붙어 움직이는 marker의 화면 폭을 깊이 신호로 사용한다.
    # elbow의 "+각도"가 마커를 키우는 방향인지 줄이는 방향인지 실측이 안 됐으므로,
    # 본격적인 접근 전에 살짝 움직여보고 방향을 스스로 판단한다.
    # (디센드 직후라 아주 짧은 순간의 노이즈로 놓칠 수 있으니 몇 번 재시도)
    marker0 = None
    for _retry in range(5):
        frame, marker0, target0 = _grab_both(grab_frame_fn)
        if frame is not None and marker0.found:
            break
        time.sleep(0.15)
    if marker0 is None or not marker0.found:
        _return_home_slow(base, shoulder, elbow)
        return False, "하강 후에도 마커가 안 보임"
    w0 = marker0.width_px
    width_error0 = MARKER_WIDTH_GRAB - w0

    if abs(width_error0) <= MARKER_WIDTH_TOL:
        elbow_direction = 1   # 이미 도달 - 방향 무의미
    else:
        elbow.move_to(elbow.angle + ELBOW_PROBE_STEP, speed=ALIGN_SPEED, step=ALIGN_STEP)
        time.sleep(0.3)
        frame, marker1, _ = _grab_both(grab_frame_fn)
        w1 = marker1.width_px if (frame is not None and marker1.found) else w0
        moved_toward_goal = (w1 - w0) * width_error0 >= 0
        elbow_direction = 1 if moved_toward_goal else -1
        print(f"[비주얼서보] elbow 방향탐색: w0={w0:.0f}px w1={w1:.0f}px "
              f"-> elbow_direction={elbow_direction:+d}")

    # ---------- 인터리브 접근 ----------
    success, reason = _interleaved_approach(base, shoulder, elbow, gripper, grab_frame_fn, elbow_direction)

    # ---------- 작업 종료 - 슬로우스타트로 대기 자세 복귀 ----------
    # (성공 시 그리퍼는 물건을 든 채로 유지, 실패 시엔 실패 경로에서 이미 열어뒀으므로
    #  여기서는 base/shoulder/elbow만 원위치, 그리퍼는 손대지 않음)
    _return_home_slow(base, shoulder, elbow)

    return success, reason


# =========================================================================
# ===== 캘리브레이션 모드 (서보 없이 카메라 검출값만 확인) =====
#   python3 arm_visual_servo.py calib [틸트각도]
# =========================================================================
def calibrate(tilt_angle=None):
    from picamera2 import Picamera2

    x = None
    if tilt_angle is not None:
        try:
            from picarx import Picarx
            x = Picarx()
            x.set_cam_tilt_angle(tilt_angle)
            x.set_cam_pan_angle(0)
            print(f"[짐벌] 틸트각 {tilt_angle}도로 설정")
        except Exception as e:
            print(f"[짐벌] 틸트 설정 실패({e}) - 현재 각도 그대로 진행")

    picam2 = Picamera2()
    config = picam2.create_preview_configuration(
        main={"size": (CAM_WIDTH, CAM_HEIGHT), "format": CAM_FORMAT})
    picam2.configure(config)
    picam2.start()
    time.sleep(1.0)

    start_stream()
    print("캘리브레이션 모드 (Ctrl+C 종료)")
    print("빨간 마커와 노란 타겟이 잘 잡히는지 확인하세요.")
    try:
        while True:
            arr = picam2.capture_array()
            if arr is None:
                time.sleep(0.1)
                continue
            frame = arr[:, :, :3] if arr.shape[2] == 4 else arr

            marker, _ = detect_red_marker(frame)
            target, _ = detect_yellow_target(frame)

            if marker.found:
                print(f"[마커] dx={marker.dx:+.2f} dy={marker.dy:+.2f} width={marker.width_px:.0f}px")
                marker_info = f"marker dx={marker.dx:+.2f} dy={marker.dy:+.2f} w={marker.width_px:.0f}px"
            else:
                print("[마커] 미검출")
                marker_info = "marker not found"

            if target.found:
                print(f"[타겟] dx={target.dx:+.2f} dy={target.dy:+.2f} "
                      f"width={target.width_px:.0f}px  (참고용, 그랩 판정은 마커 폭 기준 목표={MARKER_WIDTH_GRAB:.0f})")
                target_info = f"target dx={target.dx:+.2f} dy={target.dy:+.2f} w={target.width_px:.0f}px"
            else:
                print("[타겟] 미검출")
                target_info = "target not found"

            update_telemetry(phase="calib", info=f"{marker_info} | {target_info}")
            if _stream_clients > 0:
                _publish_frame(frame, marker, target)

            print("-" * 40)
            time.sleep(0.5)
    except KeyboardInterrupt:
        stop_stream()
        picam2.stop()
        picam2.close()
        print("종료")


if __name__ == "__main__":
    import sys
    if len(sys.argv) > 1 and sys.argv[1] == "calib":
        tilt = None
        if len(sys.argv) > 2:
            try:
                tilt = int(sys.argv[2])
            except ValueError:
                print(f"틸트 각도는 숫자여야 합니다: {sys.argv[2]}")
        calibrate(tilt_angle=tilt)
    else:
        print("이 파일은 단독 실행용이 아닙니다.")
        print("먼저 'python3 arm_visual_servo.py calib 60' 으로 마커/타겟 검출을 확인하세요.")

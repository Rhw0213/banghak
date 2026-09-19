#!/usr/bin/env python3
"""
car_main.py  -  Raspberry Pi 4 (PiCar-X)에서 실행하는 '제어 담당' 프로그램

역할
  1) Vilib으로 카메라 스트림을 켠다 (노트북이 이 영상을 가져가서 YOLO로 인식한다).
  2) 노트북이 UDP로 보내는 검출 결과를 받는다.
  3) 논블로킹 상태 머신으로 조향/주행을 제어한다.
  4) 초음파 긴급 정지, 결과 수신 워치독으로 안전을 지킨다.

상태 흐름
  STANDBY   결과가 연속으로 정상 수신될 때까지 대기 (모터 정지)
  FORWARD   직진하며 장애물 탐색
  DETECT    장애물 후보 확정 (연속 N패킷 확인, 좌/우 방향 결정)
  AVOID_OUT 조향해서 옆으로 이탈
  PASS      조향 0으로 직진, 장애물이 시야에서 사라진 뒤에도 차체 길이만큼 더 직진
  AVOID_BACK 반대로 조향해서 원래 방향으로 복귀
  STRAIGHTEN 조향 0으로 정렬 후 FORWARD
  STOPPED   초음파/워치독/타임아웃으로 정지 (조건이 풀리면 STANDBY로 복귀)

실행
  python3 car_main.py                 # 주행
  python3 car_main.py --no-drive      # 모터는 끄고 조향만 움직임 (바퀴 띄우고 테스트)
  python3 car_main.py --calibrate 50  # 장애물을 50cm 앞에 두고 FOCAL_PX 보정값 출력

회피 궤적 (튜닝 참고)
  차량은 앞바퀴 조향이라 옆으로 평행이동을 못 하고 'S자'로 움직인다.
    OUT(조향 +a, T초) → PASS(직진, 이미 heading이 θ만큼 틀어진 상태) → BACK(조향 -a, T초)
  회전 반경 R = L / tan(a)   (L: 휠베이스, a: 조향각)
  회전각 θ ≈ (v * T) / R
  옆으로 벗어나는 총량 d ≈ 2R(1 - cosθ) + (PASS 직진 거리) * sinθ
  → 장애물 폭(9.5cm) + 차체 반폭 + 여유를 넘도록 AVOID_ANGLE, OUT_TIME_S, PASS_CLEAR_S를 실차에서 조정.
"""
import argparse
import json
import socket
import threading
import time

# =====================================================================
# 파라미터 (여기만 고치면 된다)
# =====================================================================
UDP_PORT = 5005          # 노트북이 결과를 보내는 포트
LOOP_HZ = 20             # 제어 루프 주기

# --- 주행/조향 ---
# SPEED = 25              # 주행 속도 (처음엔 낮게)
SPEED = 0              # 주행 속도 (처음엔 낮게)
AVOID_ANGLE = 25         # 회피 조향각(도). PiCar-X 서보는 대략 ±30도 이내로
STEER_RIGHT_SIGN = +1    # 오른쪽 조향이 + 인지 - 인지. 바퀴 띄운 테스트에서 반드시 확인!
DEFAULT_SIDE = "right"   # 장애물이 정중앙일 때 피할 방향
CAM_TILT_DEG = 0         # 카메라 상하 각도 (바닥 쪽으로 살짝 내리려면 음수 예: -8)

# --- 장애물 실측 크기(cm) : 세로(깊이) 11 / 가로 9.5 / 높이 23 ---
# 거리 추정에는 '높이(23cm)'를 쓴다. 물체가 회전해도 높이는 그대로라서 폭보다 안정적이다.
OBJ_HEIGHT_CM = 23.0
OBJ_WIDTH_CM = 9.5
# 초점거리(픽셀). 640x480 기준 초기 추정값이다. 반드시 --calibrate로 보정할 것.
FOCAL_PX = 640.0

# --- 인식/판정 ---
TRIGGER_CM = 50.0        # 추정 거리가 이 값 이하가 되면 회피 준비(DETECT)
CORRIDOR_FRAC = 0.6      # 화면 가운데 이 비율 안에 걸친 물체만 '내 경로상의 장애물'로 본다
CENTER_DEADZONE = 0.10   # 화면 중앙 ±10% 이내면 좌/우 판단이 애매 → DEFAULT_SIDE
CONFIRM_N = 3            # 연속 N패킷 검출돼야 장애물로 확정
MISS_N = 2               # DETECT 중 N패킷 연속 안 보이면 오검출로 보고 FORWARD 복귀
DETECT_MAX_S = 1.5       # DETECT 최대 시간

# --- 회피 시간 (실차에서 튜닝) ---
OUT_TIME_S = 1.0         # AVOID_OUT 시간
BACK_TIME_S = OUT_TIME_S # AVOID_BACK 시간 (대칭이 기본)
PASS_MIN_S = 0.5         # PASS 최소 시간
PASS_CLEAR_S = 1.0       # 장애물이 시야에서 사라진 뒤 더 직진하는 시간 (차체 길이 + 여유)
PASS_MAX_S = 6.0         # PASS 최대 시간 (넘으면 정지)
STRAIGHTEN_S = 0.5       # 복귀 후 정렬 직진 시간
COOLDOWN_S = 1.0         # 복귀 직후 재트리거 방지 시간

# --- 안전 ---
EMERGENCY_CM = 10.0      # 초음파가 이 값 이하면 즉시 정지
WATCHDOG_S = 0.5         # 노트북 결과가 이 시간 이상 안 오면 정지
READY_N = 10             # STANDBY에서 이 개수만큼 연속 수신되면 출발
STOP_HOLD_S = 1.0        # STOPPED에서 최소 대기 시간
AUTO_RESUME = True       # 정지 원인이 사라지면 자동으로 STANDBY → FORWARD 재개

MOVING_STATES = {"FORWARD", "DETECT", "AVOID_OUT", "PASS", "AVOID_BACK", "STRAIGHTEN"}


# =====================================================================
# 하드웨어 래퍼 (picarx는 여기서만 import → 테스트 시 가짜로 교체 가능)
# =====================================================================
class Car:
    def __init__(self, drive=True):
        from picarx import Picarx
        self.px = Picarx()
        self.drive = drive
        self.px.set_cam_pan_angle(0)
        self.px.set_cam_tilt_angle(CAM_TILT_DEG)
        self.px.set_dir_servo_angle(0)

    def forward(self, speed):
        if self.drive:
            self.px.forward(speed)
        else:
            self.px.stop()

    def steer(self, angle):
        self.px.set_dir_servo_angle(angle)

    def stop(self):
        self.px.stop()

    def us_cm(self):
        """초음파 거리(cm). 측정 실패면 None. (picarx 버전에 따라 read() 방식이 다를 수 있음)"""
        try:
            d = self.px.ultrasonic.read()
        except Exception:
            return None
        if d is None or d <= 0:
            return None
        return float(d)

    def cleanup(self):
        try:
            self.px.stop()
            self.px.set_dir_servo_angle(0)
            self.px.set_cam_pan_angle(0)
            self.px.set_cam_tilt_angle(0)
        except Exception:
            pass


# =====================================================================
# 노트북 결과 수신 (UDP)
# =====================================================================
class Link:
    """노트북이 보낸 최신 결과와 '수신 시각'만 보관한다.

    노트북과 Pi의 시계는 서로 다르므로 타임스탬프를 비교하지 않고,
    Pi가 패킷을 받은 시각(monotonic)으로 신선도를 판단한다.
    """

    def __init__(self, port):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(("0.0.0.0", port))
        self.sock.settimeout(0.5)
        self.lock = threading.Lock()
        self.msg = None
        self.rx_time = None
        self.running = True
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()

    def _loop(self):
        while self.running:
            try:
                data, _ = self.sock.recvfrom(65535)
            except socket.timeout:
                continue
            except OSError:
                break
            try:
                m = json.loads(data.decode("utf-8"))
            except ValueError:
                continue
            with self.lock:
                self.msg = m
                self.rx_time = time.monotonic()

    def snapshot(self, now):
        """(최신 메시지, 마지막 수신 후 경과 초). 아직 못 받았으면 (None, None)."""
        with self.lock:
            if self.msg is None:
                return None, None
            return self.msg, now - self.rx_time

    def close(self):
        self.running = False
        try:
            self.sock.close()
        except OSError:
            pass


# =====================================================================
# 장애물 선택 / 거리 추정
# =====================================================================
def pick_obstacle(msg):
    """내 경로(화면 가운데 띠)에 걸친 검출 중 가장 큰(=가장 가까운) 것을 고른다."""
    if not msg:
        return None
    try:
        W = float(msg["w"])
        H = float(msg["h"])
        dets = msg["dets"]
    except (KeyError, TypeError, ValueError):
        return None

    lo = W * (0.5 - CORRIDOR_FRAC / 2.0)
    hi = W * (0.5 + CORRIDOR_FRAC / 2.0)
    best = None
    for d in dets:
        try:
            x1, y1, x2, y2 = d["xyxy"]
        except (KeyError, TypeError, ValueError):
            continue
        if x2 < lo or x1 > hi:
            continue
        h_px = max(y2 - y1, 1.0)
        # 가까워지면 23cm 높이가 화면 위/아래로 잘려서 bbox 높이가 포화된다.
        # 잘린 경우 실제 거리는 추정값보다 가깝다.
        clipped = (y1 <= 2.0) or (y2 >= H - 2.0)
        dist = FOCAL_PX * OBJ_HEIGHT_CM / h_px
        if clipped:
            dist = min(dist, TRIGGER_CM)
        if best is None or h_px > best["h_px"]:
            best = {
                "cx": (x1 + x2) / 2.0, "W": W, "h_px": h_px, "dist": dist,
                "clipped": clipped, "cls": d.get("cls", "?"), "conf": d.get("conf", 0.0),
            }
    return best


# =====================================================================
# 상태 머신 (시간/센서값을 인자로 받으므로 하드웨어 없이 테스트 가능)
# =====================================================================
class Avoider:
    def __init__(self, car, log=print):
        self.car = car
        self.log = log
        self.state = "STANDBY"
        self.t_enter = 0.0
        self.last_seq = None
        self.ready_cnt = 0
        self.hits = 0
        self.misses = 0
        self.cx_list = []
        self.W = 640.0
        self.sign = 1
        self.last_seen = 0.0
        self.cooldown_until = 0.0
        self.stop_kind = "transient"

    # ---- 전이 + 진입 동작 ----
    def _go(self, now, state, why=""):
        self.log(f"[{now:8.2f}] {self.state:<10} -> {state:<10} {why}")
        self.state = state
        self.t_enter = now
        if state == "FORWARD":
            self.car.steer(0)
            self.car.forward(SPEED)
        elif state == "AVOID_OUT":
            self.car.steer(self.sign * AVOID_ANGLE)
            self.car.forward(SPEED)
        elif state == "PASS":
            self.last_seen = now
            self.car.steer(0)
            self.car.forward(SPEED)
        elif state == "AVOID_BACK":
            self.car.steer(-self.sign * AVOID_ANGLE)
            self.car.forward(SPEED)
        elif state == "STRAIGHTEN":
            self.car.steer(0)
        elif state == "STOPPED":
            self.car.stop()
            self.car.steer(0)
        elif state == "STANDBY":
            self.ready_cnt = 0

    def _stop(self, now, why, kind="transient"):
        # kind="transient": 원인(초음파/링크)이 사라지면 자동 재출발해도 안전.
        # kind="stuck": 원인이 '판단 실패'(예: PASS 타임아웃)라서 그냥 재출발하면
        #   같은 장애물을 다시 만나 DETECT→AVOID_OUT→PASS→타임아웃을 반복할 수 있다.
        #   자동 재출발하지 않고 사람이 개입할 때까지 정지 상태를 유지한다.
        self.stop_kind = kind
        self._go(now, "STOPPED", why)

    # ---- 한 스텝 ----
    def step(self, now, msg, age, us_cm):
        link_ok = msg is not None and age is not None and age <= WATCHDOG_S
        seq = msg.get("seq") if msg else None
        new_pkt = link_ok and seq != self.last_seq
        if new_pkt:
            self.last_seq = seq

        # 최우선 안전 검사: 어떤 이동 상태에서든 적용
        if self.state in MOVING_STATES:
            if us_cm is not None and us_cm <= EMERGENCY_CM:
                self._stop(now, f"초음파 {us_cm:.0f}cm 긴급 정지", kind="transient")
                return
            if not link_ok:
                self._stop(now, "노트북 결과 수신 끊김(워치독)", kind="transient")
                return

        getattr(self, "_st_" + self.state.lower())(now, msg, new_pkt, link_ok, us_cm)

    # ---- 상태별 처리 ----
    def _st_standby(self, now, msg, new_pkt, link_ok, us_cm):
        if not link_ok:
            self.ready_cnt = 0
            return
        if new_pkt:
            self.ready_cnt += 1
        if self.ready_cnt >= READY_N:
            self._go(now, "FORWARD", f"결과 {READY_N}회 연속 수신 확인")

    def _st_stopped(self, now, msg, new_pkt, link_ok, us_cm):
        us_clear = us_cm is None or us_cm > EMERGENCY_CM + 5.0
        if (AUTO_RESUME and self.stop_kind == "transient"
                and now - self.t_enter >= STOP_HOLD_S and link_ok and us_clear):
            self._go(now, "STANDBY", "정지 원인 해소, 재시작 준비")
        # stop_kind == "stuck" (예: PASS 타임아웃)이면 자동 재개하지 않는다.
        # 현재 상태 머신에는 '사람이 재개시키는' 입력이 없으므로, 실제로는
        # 차량이 이 상태에서 멈춰 있게 된다 — 아래 main()의 요청대로 별도
        # 재개 트리거(버튼/명령)를 붙이거나, 최소한 이 상황을 로그/알림으로
        # 알리는 처리가 필요하다.

    def _st_forward(self, now, msg, new_pkt, link_ok, us_cm):
        if now < self.cooldown_until:
            return
        obs = pick_obstacle(msg) if link_ok else None
        if obs and obs["dist"] <= TRIGGER_CM:
            self.hits, self.misses = 1, 0
            self.cx_list = [obs["cx"]]
            self.W = obs["W"]
            self._go(now, "DETECT", f"{obs['cls']} 추정 {obs['dist']:.0f}cm")

    def _st_detect(self, now, msg, new_pkt, link_ok, us_cm):
        if new_pkt:  # 같은 패킷을 두 번 세지 않도록 새 패킷일 때만 집계
            obs = pick_obstacle(msg)
            if obs:
                self.hits += 1
                self.misses = 0
                self.cx_list.append(obs["cx"])
                self.W = obs["W"]
            else:
                self.misses += 1

        if self.hits >= CONFIRM_N:
            cx = sum(self.cx_list) / len(self.cx_list)
            off = (cx - self.W / 2.0) / self.W  # 음수: 장애물이 화면 왼쪽
            if abs(off) < CENTER_DEADZONE:
                steer_right = DEFAULT_SIDE == "right"
            else:
                steer_right = off < 0  # 장애물이 왼쪽이면 오른쪽으로 피한다
            self.sign = STEER_RIGHT_SIGN if steer_right else -STEER_RIGHT_SIGN
            side = "오른쪽" if steer_right else "왼쪽"
            self._go(now, "AVOID_OUT", f"x오프셋 {off:+.2f} → {side}으로 회피")
        elif self.misses >= MISS_N or now - self.t_enter > DETECT_MAX_S:
            self._go(now, "FORWARD", "오검출로 판단, 직진 복귀")

    def _st_avoid_out(self, now, msg, new_pkt, link_ok, us_cm):
        if now - self.t_enter >= OUT_TIME_S:
            self._go(now, "PASS")

    def _st_pass(self, now, msg, new_pkt, link_ok, us_cm):
        seen = bool(msg and msg.get("dets"))
        if seen:
            self.last_seen = now
        el = now - self.t_enter
        if el >= PASS_MIN_S and now - self.last_seen >= PASS_CLEAR_S:
            self._go(now, "AVOID_BACK", f"장애물 {PASS_CLEAR_S:.1f}초간 미검출, 통과로 판단")
        elif el > PASS_MAX_S:
            self._stop(now, "PASS 타임아웃 (장애물이 계속 보임, 사람 개입 필요)", kind="stuck")

    def _st_avoid_back(self, now, msg, new_pkt, link_ok, us_cm):
        if now - self.t_enter >= BACK_TIME_S:
            self._go(now, "STRAIGHTEN")

    def _st_straighten(self, now, msg, new_pkt, link_ok, us_cm):
        if now - self.t_enter >= STRAIGHTEN_S:
            self.cooldown_until = now + COOLDOWN_S
            self._go(now, "FORWARD", "경로 복귀 완료")


# =====================================================================
# 캘리브레이션 (FOCAL_PX 보정)
# =====================================================================
def run_calibrate(link, known_cm):
    print(f"[calib] 장애물을 카메라 정면 {known_cm:.0f}cm 거리에 두세요. (화면 가운데, 잘리지 않게)")
    print("[calib] 출력된 평균 FOCAL_PX 값을 car_main.py 상단에 입력하세요. Ctrl+C로 종료.")
    samples = []
    while True:
        now = time.monotonic()
        msg, age = link.snapshot(now)
        obs = pick_obstacle(msg) if (msg and age is not None and age <= WATCHDOG_S) else None
        if obs and not obs["clipped"]:
            f = obs["h_px"] * known_cm / OBJ_HEIGHT_CM
            samples = (samples + [f])[-20:]
            avg = sum(samples) / len(samples)
            print(f"[calib] bbox 높이 {obs['h_px']:5.1f}px → FOCAL_PX ≈ {f:6.1f}  (최근 {len(samples)}개 평균 {avg:6.1f})")
        else:
            print("[calib] 장애물이 화면 가운데에 안 보이거나, 위/아래로 잘렸습니다.")
        time.sleep(0.5)


# =====================================================================
# main
# =====================================================================
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=UDP_PORT)
    ap.add_argument("--no-drive", action="store_true", help="모터 정지, 조향만 동작 (바퀴 띄우고 테스트)")
    ap.add_argument("--calibrate", type=float, metavar="DIST_CM",
                    help="장애물을 DIST_CM 앞에 두고 FOCAL_PX 보정값 출력")
    args = ap.parse_args()

    from vilib import Vilib
    Vilib.camera_start(vflip=False, hflip=False)
    Vilib.display(local=False, web=True)  # http://<파이IP>:9000/mjpg (버전에 따라 확인)

    link = Link(args.port)
    car = None
    try:
        if args.calibrate is not None:
            run_calibrate(link, args.calibrate)
            return

        car = Car(drive=not args.no_drive)
        av = Avoider(car)
        print(f"[main] 스트림 시작. 노트북에서 yolo_server.py를 실행하세요. (UDP {args.port} 수신 대기)")
        if args.no_drive:
            print("[main] --no-drive: 모터는 움직이지 않습니다.")

        period = 1.0 / LOOP_HZ
        while True:
            t0 = time.monotonic()
            msg, age = link.snapshot(t0)
            av.step(t0, msg, age, car.us_cm())
            time.sleep(max(0.0, period - (time.monotonic() - t0)))
    except KeyboardInterrupt:
        pass
    finally:
        link.close()
        if car:
            car.cleanup()
        try:
            Vilib.camera_close()
        except Exception:
            pass
        print("[main] 종료")


if __name__ == "__main__":
    main()

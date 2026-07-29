#!/usr/bin/env python3
# range_check.py
# 카메라의 '인식 범위'를 실측하는 도구
#
#   python3 range_check.py
#
# 측정하는 것:
#   1) 초점거리(CAM_FOCAL_PX)  - 여러 거리에서 샘플을 찍어 평균 + 편차로 신뢰도 확인
#   2) 최소/최대 인식 거리      - 실제로 검출이 되는 거리 구간
#   3) 수평 시야 폭             - "이 거리에서 좌우 몇 cm까지 보이는가"
#
# 브라우저(http://<IP>:8000)로 영상을 보면서 진행하세요.
#
# ★ lidar_ultra_vision.py 와 HSV / 해상도 설정을 반드시 동일하게 맞출 것.
#   해상도나 오브젝트가 바뀌면 초점거리도 다시 재야 합니다.

import time
import math
import threading
import json
import socket
from urllib.parse import urlparse, parse_qs
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np
from picamera2 import Picamera2

# ===================== lidar_ultra_vision.py 와 동일하게 맞출 것 =====================
CAM_WIDTH = 320
CAM_HEIGHT = 240
CAM_FORMAT = "RGB888"
CAM_SWAP_RB = False

YELLOW_H_MIN, YELLOW_H_MAX = 20, 35
YELLOW_S_MIN, YELLOW_S_MAX = 100, 255
YELLOW_V_MIN, YELLOW_V_MAX = 100, 255

MIN_TARGET_AREA = 400          # 이 면적 미만이면 미검출 -> 최대 인식거리를 결정하는 값
TARGET_REAL_WIDTH_CM = 6.5     # ★ 실제 오브젝트 가로폭
TARGET_REAL_HEIGHT_CM = 6.5    # ★ 실제 오브젝트 세로폭 (예측 계산용)

STREAM_PORT = 8000
STREAM_FPS = 10
STREAM_QUALITY = 60
# ===================================================================================

_picam2 = None
_lock = threading.Lock()
_latest = {"found": False, "w": 0.0, "h": 0.0, "area": 0.0,
           "offset": 0.0, "cx": 0, "cy": 0}
_running = True
_stream_jpeg = None
_stream_lock = threading.Lock()
_show_mask = False

# 측정 샘플: [(실제거리cm, 픽셀폭, 픽셀높이, 면적), ...]
samples = []


# ---------------- 카메라 ----------------

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


def detect(frame):
    lower = np.array([YELLOW_H_MIN, YELLOW_S_MIN, YELLOW_V_MIN])
    upper = np.array([YELLOW_H_MAX, YELLOW_S_MAX, YELLOW_V_MAX])

    blurred = cv2.GaussianBlur(frame, (5, 5), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower, upper)
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None, mask
    largest = max(contours, key=cv2.contourArea)
    area = cv2.contourArea(largest)
    if area < MIN_TARGET_AREA:
        return None, mask
    bx, by, bw, bh = cv2.boundingRect(largest)
    return (bx, by, bw, bh, area), mask


def cam_loop():
    global _latest, _stream_jpeg
    last = 0.0
    while _running:
        frame = grab_frame()
        if frame is None:
            time.sleep(0.05)
            continue

        res, mask = detect(frame)
        h, w = frame.shape[:2]

        if res:
            bx, by, bw, bh, area = res
            cx = bx + bw // 2
            cy = by + bh // 2
            offset = (cx - w / 2.0) / (w / 2.0)
            with _lock:
                _latest = {"found": True, "w": float(bw), "h": float(bh),
                           "area": float(area), "offset": offset,
                           "cx": cx, "cy": cy}
        else:
            with _lock:
                _latest = {"found": False, "w": 0.0, "h": 0.0, "area": 0.0,
                           "offset": 0.0, "cx": 0, "cy": 0}

        now = time.time()
        if now - last >= 1.0 / STREAM_FPS:
            last = now
            if _show_mask:
                img = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
            else:
                img = frame.copy()
                cv2.line(img, (w // 2, 0), (w // 2, h), (200, 200, 200), 1)
                if res:
                    bx, by, bw, bh, area = res
                    cv2.rectangle(img, (bx, by), (bx + bw, by + bh), (0, 255, 0), 2)
                    cv2.putText(img, f"w={bw} a={int(area)}", (bx, max(14, by - 5)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 1)
                else:
                    cv2.putText(img, "NOT DETECTED", (8, h - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
            ok, buf = cv2.imencode('.jpg', img,
                                   [int(cv2.IMWRITE_JPEG_QUALITY), STREAM_QUALITY])
            if ok:
                with _stream_lock:
                    _stream_jpeg = buf.tobytes()

        time.sleep(0.01)


# ---------------- 웹 스트림 (측정 중 눈으로 확인용) ----------------

PAGE = """<!DOCTYPE html><html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Range Check</title>
<style>body{background:#1a1a1a;color:#eee;font-family:sans-serif;padding:10px}
img{width:100%;max-width:640px;border:1px solid #444}
#s{background:#222;padding:8px;border-radius:4px;margin-top:8px;white-space:pre-wrap}
button{padding:8px 14px;background:#356;color:#fff;border:none;border-radius:4px;margin-top:8px}
</style></head><body>
<img src="/stream.mjpg">
<button onclick="fetch('/set?mask=1')">마스크 보기 전환</button>
<div id="s">...</div>
<script>
async function p(){try{const r=await fetch('/status');const d=await r.json();
document.getElementById('s').textContent= d.found
 ? `검출됨\\n픽셀폭 : ${d.w.toFixed(0)} px\\n픽셀높이: ${d.h.toFixed(0)} px\\n면적    : ${d.area.toFixed(0)}\\n오프셋  : ${d.offset.toFixed(2)}`
 : '미검출';}catch(e){}}
setInterval(p,300);p();
</script></body></html>"""


class H(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def do_GET(self):
        global _show_mask
        u = urlparse(self.path)
        if u.path == '/':
            b = PAGE.encode()
            self.send_response(200)
            self.send_header('Content-Type', 'text/html; charset=utf-8')
            self.send_header('Content-Length', str(len(b)))
            self.end_headers()
            self.wfile.write(b)
        elif u.path == '/status':
            with _lock:
                b = json.dumps(_latest).encode()
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Content-Length', str(len(b)))
            self.end_headers()
            self.wfile.write(b)
        elif u.path == '/set':
            _show_mask = not _show_mask
            self.send_response(204)
            self.end_headers()
        elif u.path == '/stream.mjpg':
            self.send_response(200)
            self.send_header('Content-Type',
                             'multipart/x-mixed-replace; boundary=FRAME')
            self.end_headers()
            try:
                while _running:
                    with _stream_lock:
                        buf = _stream_jpeg
                    if buf is None:
                        time.sleep(0.05)
                        continue
                    self.wfile.write(b'--FRAME\r\n')
                    self.wfile.write(b'Content-Type: image/jpeg\r\n')
                    self.wfile.write(('Content-Length: %d\r\n\r\n' % len(buf)).encode())
                    self.wfile.write(buf)
                    self.wfile.write(b'\r\n')
                    time.sleep(1.0 / STREAM_FPS)
            except (BrokenPipeError, ConnectionResetError):
                pass
        else:
            self.send_error(404)


def local_ip():
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(('8.8.8.8', 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except Exception:
        return '127.0.0.1'


# ---------------- 측정 로직 ----------------

def take_sample(actual_cm):
    """지금 화면의 검출 결과를 실제거리와 함께 기록 (10프레임 평균)"""
    ws, hs, areas = [], [], []
    for _ in range(10):
        with _lock:
            d = dict(_latest)
        if d["found"]:
            ws.append(d["w"])
            hs.append(d["h"])
            areas.append(d["area"])
        time.sleep(0.05)

    if len(ws) < 5:
        print(f"  ✗ 검출 불안정 ({len(ws)}/10 프레임만 검출) - 기록하지 않음")
        print("    HSV 범위나 조명을 확인하세요.")
        return

    w = sum(ws) / len(ws)
    h = sum(hs) / len(hs)
    a = sum(areas) / len(areas)
    focal = (w * actual_cm) / TARGET_REAL_WIDTH_CM
    samples.append((actual_cm, w, h, a))
    print(f"  ✓ {actual_cm}cm 기록: 폭={w:.1f}px 높이={h:.1f}px "
          f"면적={a:.0f} -> focal={focal:.1f}")


def scan_range():
    """오브젝트를 천천히 멀리/가까이 움직이는 동안 검출 여부를 추적"""
    print("\n15초간 측정합니다. 오브젝트를 카메라 앞에서 아주 가까운 곳부터")
    print("천천히 멀리 이동시켰다가 다시 가까이 가져오세요.")
    input("준비되면 Enter...")

    t0 = time.time()
    max_w = 0.0
    min_w = 1e9
    detected_frames = 0
    total_frames = 0

    while time.time() - t0 < 15.0:
        with _lock:
            d = dict(_latest)
        total_frames += 1
        if d["found"]:
            detected_frames += 1
            max_w = max(max_w, d["w"])
            min_w = min(min_w, d["w"])
        remain = 15.0 - (time.time() - t0)
        print(f"\r  남은시간 {remain:4.1f}s | "
              f"{'검출' if d['found'] else '미검출'} 폭={d['w']:5.1f}px | "
              f"관측범위 {min_w if min_w < 1e9 else 0:.0f}~{max_w:.0f}px   ",
              end="")
        time.sleep(0.1)

    print()
    if max_w == 0:
        print("  한 번도 검출되지 않았습니다. HSV 설정을 먼저 맞추세요.")
        return

    focal = estimate_focal()
    if focal is None:
        print(f"  관측된 픽셀폭 범위: {min_w:.0f} ~ {max_w:.0f} px")
        print("  (거리 환산하려면 먼저 거리 샘플을 1개 이상 기록하세요)")
        return

    near = (TARGET_REAL_WIDTH_CM * focal) / max_w
    far = (TARGET_REAL_WIDTH_CM * focal) / min_w
    print(f"  검출률: {detected_frames}/{total_frames} "
          f"({100.0 * detected_frames / total_frames:.0f}%)")
    print(f"  관측된 인식 거리 범위: 약 {near:.0f}cm ~ {far:.0f}cm")


def estimate_focal():
    """기록된 샘플들로 초점거리 추정. 샘플 없으면 None"""
    if not samples:
        return None
    fs = [(w * d) / TARGET_REAL_WIDTH_CM for d, w, h, a in samples]
    return sum(fs) / len(fs)


def report():
    if not samples:
        print("\n기록된 샘플이 없습니다. 먼저 거리를 입력해 샘플을 찍으세요.")
        return

    fs = [(w * d) / TARGET_REAL_WIDTH_CM for d, w, h, a in samples]
    focal = sum(fs) / len(fs)
    std = math.sqrt(sum((f - focal) ** 2 for f in fs) / len(fs)) if len(fs) > 1 else 0.0

    print("\n" + "=" * 66)
    print("측정 결과")
    print("=" * 66)
    print(f"{'실제거리':>8} {'픽셀폭':>8} {'픽셀높이':>8} {'면적':>9} {'계산 focal':>11}")
    print("-" * 66)
    for (d, w, h, a), f in zip(samples, fs):
        print(f"{d:>7.0f}cm {w:>8.1f} {h:>8.1f} {a:>9.0f} {f:>11.1f}")
    print("-" * 66)
    print(f"평균 초점거리 : {focal:.1f} px")
    if len(fs) > 1:
        print(f"표준편차      : {std:.1f} px  ({100.0 * std / focal:.1f}%)")
        if std / focal > 0.10:
            print("  ⚠ 편차가 10%를 넘습니다. 거리를 잘못 쟀거나 오브젝트가")
            print("    정면을 향하지 않았을 수 있습니다. 다시 측정하세요.")
        else:
            print("  ✓ 편차가 작습니다. 신뢰할 만한 값입니다.")
    print()
    print(f"  ★ lidar_ultra_vision.py 에 반영:  CAM_FOCAL_PX = {focal:.0f}")

    # ---- 거리 범위 예측 ----
    print()
    print("=" * 66)
    print("인식 거리 범위 (예측)")
    print("=" * 66)

    # 최대: 면적이 MIN_TARGET_AREA 밑으로 떨어지는 거리
    # 면적 ≈ (f*W/d) * (f*H/d) = f^2*W*H/d^2  ->  d = f*sqrt(W*H/area)
    d_max = focal * math.sqrt(
        (TARGET_REAL_WIDTH_CM * TARGET_REAL_HEIGHT_CM) / MIN_TARGET_AREA)
    # 최소: 오브젝트 폭이 화면 폭을 넘어서는 거리 (그 이상 가까우면 잘림)
    d_min = (TARGET_REAL_WIDTH_CM * focal) / CAM_WIDTH

    print(f"최대 인식 거리 : 약 {d_max:.0f} cm")
    print(f"   (이보다 멀면 면적이 MIN_TARGET_AREA={MIN_TARGET_AREA} 밑으로 떨어져 무시됨)")
    print(f"   -> 더 멀리서 잡고 싶으면 MIN_TARGET_AREA 를 낮추세요")
    print(f"      (단, 노이즈 오검출이 늘어납니다)")
    print(f"최소 인식 거리 : 약 {d_min:.0f} cm")
    print(f"   (이보다 가까우면 오브젝트가 화면 폭을 넘어 잘림)")
    print(f"   -> 실제로는 카메라 높이/틸트 때문에 이보다 먼저 화면 아래로 벗어납니다.")
    print(f"      정지 거리 10cm 근처에서 실제로 보이는지 반드시 눈으로 확인하세요.")

    # ---- 수평 시야 폭 ----
    fov_h = 2 * math.degrees(math.atan(CAM_WIDTH / (2 * focal)))
    fov_v = 2 * math.degrees(math.atan(CAM_HEIGHT / (2 * focal)))
    print()
    print("=" * 66)
    print("시야각 및 거리별 가시 범위")
    print("=" * 66)
    print(f"수평 화각: {fov_h:.1f}도   수직 화각: {fov_v:.1f}도")
    print()
    print(f"{'거리':>8} {'가로 가시폭':>14} {'중심 기준 좌우':>16}")
    print("-" * 44)
    for d in (10, 20, 30, 50, 80, 100, 150):
        if d > d_max * 1.3:
            break
        vis_w = CAM_WIDTH * d / focal
        print(f"{d:>7}cm {vis_w:>13.1f}cm {'±' + f'{vis_w / 2:.1f}':>15}cm")
    print()
    print("해석: 목표물이 이 좌우 범위 밖에 있으면 카메라에 아예 안 잡힙니다.")
    print("      차량이 회전하며 탐색해야 하는 이유이고, 회전 각도를 정할 때")
    print(f"      한 번에 {fov_h:.0f}도 이상 돌리면 사각지대가 생깁니다.")
    print("=" * 66)


def main():
    global _picam2, _running

    _picam2 = Picamera2()
    _picam2.configure(_picam2.create_preview_configuration(
        main={"size": (CAM_WIDTH, CAM_HEIGHT), "format": CAM_FORMAT}))
    _picam2.start()
    time.sleep(1.0)
    print(f"[카메라] 시작 완료 (첫 프레임 {_picam2.capture_array().shape})")

    threading.Thread(target=cam_loop, daemon=True).start()

    srv = ThreadingHTTPServer(('0.0.0.0', STREAM_PORT), H)
    srv.daemon_threads = True
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    print(f"[스트리밍] http://{local_ip()}:{STREAM_PORT}")

    print()
    print("=" * 66)
    print("카메라 인식 범위 측정")
    print("=" * 66)
    print("명령:")
    print("  <숫자>  오브젝트를 그 거리(cm)에 두고 입력 -> 샘플 기록")
    print("          예) 20 <Enter>  (20cm 지점에서 측정)")
    print("          최소 3개 이상, 가까운 거리와 먼 거리를 골고루 찍으세요")
    print("          권장: 15, 30, 50, 80 cm")
    print("  r       거리 스윕 측정 (오브젝트를 움직이며 인식 한계 관측)")
    print("  s       결과 요약 출력")
    print("  c       기록 초기화")
    print("  q       종료")
    print("=" * 66)

    try:
        while True:
            with _lock:
                d = dict(_latest)
            live = (f"검출 폭={d['w']:.0f}px 면적={d['area']:.0f}"
                    if d["found"] else "미검출")
            cmd = input(f"[{live}] > ").strip().lower()

            if cmd == 'q':
                break
            elif cmd == 's':
                report()
            elif cmd == 'r':
                scan_range()
            elif cmd == 'c':
                samples.clear()
                print("  기록을 초기화했습니다.")
            elif cmd == '':
                continue
            else:
                try:
                    dist = float(cmd)
                    if dist <= 0:
                        print("  0보다 큰 거리를 입력하세요.")
                        continue
                    take_sample(dist)
                except ValueError:
                    print("  숫자 또는 r / s / c / q 를 입력하세요.")
    except (KeyboardInterrupt, EOFError):
        pass
    finally:
        if samples:
            report()
        _running = False
        time.sleep(0.3)
        try:
            srv.shutdown()
        except Exception:
            pass
        _picam2.stop()
        _picam2.close()
        print("\n종료")


if __name__ == "__main__":
    main()
#!/usr/bin/env python3
"""
capture_dataset.py  -  Raspberry Pi (PiCar-X)에서 실행하는 '학습 데이터 수집' 프로그램

목적: YOLO 커스텀 학습용 사진을 자동으로 모은다.

동작 순서
  1) 장애물과 차량을 원하는 거리(기본 1m)·각도(정면/측면 등)로 사람이 직접 배치
  2) 이 스크립트를 실행하면 차량이 저속으로 전진하며 일정 간격으로 사진을 찍는다
  3) 초음파 거리가 STOP_CM(기본 10cm) 이하가 되면 즉시 정지하고 촬영을 끝낸다
  4) 사진은 --out 폴더(기본 ./pictures) 아래, 실행할 때마다 새 하위 폴더에 저장된다
     예: pictures/20260919_115300_front/shot_0001_d97cm.jpg
     (폴더명 끝의 front/side 등은 --tag로 넘긴 값. 각도를 바꿔 여러 번 실행하면
      폴더가 나뉘어 저장되므로 나중에 라벨링할 때 뒤섞이지 않는다)

사용 예
  python3 capture_dataset.py --tag front            # 정면 배치, 기본 설정으로 촬영
  python3 capture_dataset.py --tag side_blue --speed 12 --interval 0.4
  python3 capture_dataset.py --tag rotated --stop-cm 15 --dry-run   # 모터 없이 카메라만 테스트

  
  python3 capture_dataset.py --tag front       # 정면 배치
  python3 capture_dataset.py --tag side_blue   # 파란 면이 보이는 측면 배치
  python3 capture_dataset.py --tag rotated45   # 45도 회전 배치
  
  --tag만 바꿔서 여러 번 돌리면 폴더가 겹치지 않고 자동으로 나뉩니다.
  
  주요 옵션
    옵션	     기본값	               설명
  --speed	     12	       전진 속도. 흔들림(블러) 사진 줄이려면 낮게 유지
  --interval     0.3초	   촬영 간격
  --stop-cm	     10  	   이 거리(cm) 이하면 정지
  --max-runtime	 20초	   센서 오류 등으로 못 멈추는 경우를 대비한 안전 타임아웃
  --no-drive	  -	       모터는 끄고 카메라만 테스트 (제자리에서 프레임 저장 확인용)
  --dry-run   	  -	       하드웨어 없이 로직만 확인 (지금 이걸로 버그 검증했습니다)
  
주의
  - 반드시 차량 앞에 사람/장애물 외에 걸리적거리는 것이 없는 넓은 공간에서 실행하세요.
  - 촬영 중간에 위험하면 Ctrl+C로 즉시 정지합니다 (모터는 finally에서 반드시 stop됨).
  - 사진 속 장애물 위치에 실제 바운딩 박스를 그리는 건 이 스크립트가 아니라
    Roboflow 같은 라벨링 툴에서 하는 다음 단계입니다. 여기서는 '사진 수집'까지만 합니다.
"""
import argparse
import csv
import os
import time
from datetime import datetime

# =====================================================================
# 기본 파라미터 (필요하면 CLI 인자로 덮어쓸 수 있음)
# =====================================================================
DEFAULT_OUT_DIR = "pictures"
DEFAULT_SPEED = 12          # 저속 권장: 빠르면 사진에 블러(흔들림)가 생긴다
DEFAULT_INTERVAL_S = 0.3    # 촬영 간격(초)
DEFAULT_STOP_CM = 10.0      # 이 거리 이하가 되면 정지 + 촬영 종료
DEFAULT_MAX_RUNTIME_S = 20.0  # 센서 이상 등으로 못 멈출 경우를 대비한 안전 타임아웃
LOOP_HZ = 20

#조향각도 조정
STEER_TRIM_DEG = -3   # 오른쪽으로 쏠리면 음수(왼쪽 보정), 왼쪽으로 쏠리면 양수로 조정
STEER_MIN_DEG = -30   # 최소 각도
STEER_MAX_DEG = 30    # 최대 각도

class Car:
    """picarx 래퍼. --dry-run이면 모터/카메라 없이도 코드 흐름을 테스트할 수 있게 분리."""

    def __init__(self, drive=True):
        from picarx import Picarx
        self.px = Picarx()
        self.drive = drive
        self.px.set_dir_servo_angle(-1) #조향각도 조정코드
        self.px.set_cam_pan_angle(0)

    def forward(self, speed):
        if self.drive:
            self.px.forward(speed)

    def stop(self):
        self.px.stop()

    def us_cm(self):
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
        except Exception:
            pass


class Camera:
    """Vilib 래퍼. 정지 이미지 저장에는 Vilib.img(최신 프레임, numpy 배열)를 사용한다."""

    def __init__(self):
        from vilib import Vilib
        self.Vilib = Vilib
        Vilib.camera_start(vflip=False, hflip=False)
        # 웹으로 계속 보고 싶다면 web=True 유지. 로컬 화면이 없다면 web=True만 있어도 된다.
        Vilib.display(local=False, web=True)
        time.sleep(0.5)  # 카메라 워밍업 (초기 프레임이 비어있을 수 있음)

    def snap(self):
        """현재 프레임을 BGR numpy 배열로 반환. 아직 준비 안 됐으면 None."""
        img = getattr(self.Vilib, "img", None)
        if img is None:
            return None
        return img.copy()

    def close(self):
        try:
            self.Vilib.camera_close()
        except Exception:
            pass


def make_session_dir(out_dir, tag):
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    name = f"{ts}_{tag}" if tag else ts
    path = os.path.join(out_dir, name)
    os.makedirs(path, exist_ok=True)
    return path


def run(args):
    cv2 = None
    if not args.dry_run:
        import cv2  # numpy 배열을 jpg로 저장하는 데만 사용 (dry-run에서는 불필요)

    if not os.path.isdir(args.out):
        # 사용자가 pictures 폴더를 미리 만들어 둔다고 했지만, 없으면 여기서도 만든다.
        os.makedirs(args.out, exist_ok=True)
        print(f"[info] '{args.out}' 폴더가 없어 새로 만들었습니다.")

    session_dir = make_session_dir(args.out, args.tag)
    log_path = os.path.join(session_dir, "log.csv")
    print(f"[info] 저장 위치: {session_dir}")

    cam = None if args.dry_run else Camera()
    car = None if args.dry_run else Car(drive=not args.no_drive)

    n_saved = 0
    last_capture = -999.0
    period = 1.0 / LOOP_HZ

    try:
        with open(log_path, "w", newline="", encoding="utf-8") as f:
            writer = csv.writer(f)
            writer.writerow(["filename", "elapsed_s", "ultrasonic_cm"])

            print("[info] 3초 후 전진을 시작합니다. 장애물/사람과의 거리를 확인하세요...")
            for i in (3, 2, 1):
                print(f"  {i}...")
                time.sleep(1.0)

            if car:
                car.forward(args.speed)
            print("[info] 촬영 시작 (Ctrl+C로 중단 가능)")

            t_start = time.monotonic()  # 카운트다운 이후부터 max_runtime을 잰다
            while True:
                now = time.monotonic()
                elapsed = now - t_start

                dist = None if args.dry_run else car.us_cm()

                # --- 안전/종료 조건 ---
                if elapsed > args.max_runtime:
                    print(f"[info] 최대 시간({args.max_runtime:.0f}초) 도달, 종료합니다.")
                    break
                if dist is not None and dist <= args.stop_cm:
                    print(f"[info] 초음파 {dist:.1f}cm ≤ {args.stop_cm:.0f}cm, 정지 및 촬영 종료.")
                    break

                # --- 촬영 ---
                if now - last_capture >= args.interval:
                    last_capture = now
                    if args.dry_run:
                        n_saved += 1
                        writer.writerow([f"(dry-run {n_saved})", round(elapsed, 2), dist])
                    else:
                        frame = cam.snap()
                        if frame is not None:
                            n_saved += 1
                            d_txt = f"{dist:.0f}cm" if dist is not None else "na"
                            fname = f"shot_{n_saved:04d}_d{d_txt}.jpg"
                            fpath = os.path.join(session_dir, fname)
                            cv2.imwrite(fpath, frame)
                            writer.writerow([fname, round(elapsed, 2), dist])
                            print(f"  [{n_saved:4d}] {fname}  (거리 {d_txt}, 경과 {elapsed:4.1f}s)")
                        else:
                            print("  [warn] 아직 카메라 프레임이 없어 이번 틱은 건너뜀")

                time.sleep(max(0.0, period - (time.monotonic() - now)))
    except KeyboardInterrupt:
        print("\n[info] 사용자 중단 (Ctrl+C)")
    finally:
        if car:
            car.cleanup()
        if cam:
            cam.close()
        print(f"[info] 종료. 총 {n_saved}장 저장됨 → {session_dir}")
        if n_saved == 0:
            print("[warn] 저장된 사진이 없습니다. --dry-run 없이, 카메라/모터 연결을 확인하고 다시 실행하세요.")


def parse_args():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", default=DEFAULT_OUT_DIR, help=f"저장 루트 폴더 (기본: {DEFAULT_OUT_DIR})")
    ap.add_argument("--tag", default="", help="이번 촬영을 구분할 이름 (예: front, side_blue, rotated45)")
    ap.add_argument("--speed", type=int, default=DEFAULT_SPEED, help=f"전진 속도 (기본 {DEFAULT_SPEED}, 저속 권장)")
    ap.add_argument("--interval", type=float, default=DEFAULT_INTERVAL_S, help=f"촬영 간격(초) (기본 {DEFAULT_INTERVAL_S})")
    ap.add_argument("--stop-cm", type=float, default=DEFAULT_STOP_CM, help=f"이 거리(cm) 이하면 정지 (기본 {DEFAULT_STOP_CM})")
    ap.add_argument("--max-runtime", type=float, default=DEFAULT_MAX_RUNTIME_S, help="안전 타임아웃(초)")
    ap.add_argument("--no-drive", action="store_true", help="모터 끄고 제자리에서 카메라만 테스트")
    ap.add_argument("--dry-run", action="store_true", help="하드웨어(picarx/vilib) 없이 로직만 테스트")
    return ap.parse_args()


if __name__ == "__main__":
    run(parse_args())

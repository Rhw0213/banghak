# blue_avoid_demo.py
# 역할: lidar_ver5.py의 카메라 인식(파란 장애물 HSV 검출) + 모터/조향 제어
#       패턴을 재사용한 단독 테스트 스크립트.
#
# 시나리오:
#   1) 카메라에 파란 장애물이 보이는 동안 -> 그 반대쪽으로 조향하며 회피 전진
#      (회피 중 쓴 조향각을 계속 last_steer에 갱신 저장)
#   2) 카메라에 파란색이 "더 이상 안 보이는" 순간 -> 완전히 회피했다고 판단
#      (단, 그 전에 회피 중이었던 경우에만 - 애초에 안 보이던 건 탈출이 아님)
#   3) 저장해둔 last_steer에 -1을 곱해서 반대 방향으로 꺾고, 2초간 전진
#   4) 정지 후 프로그램 종료
#
# 실행: python3 blue_avoid_demo.py (Ctrl+C로도 안전하게 정지 후 종료됨)

import time
from robot_hat import Motor, Servo, Pin, PWM, device
from picamera2 import Picamera2
import cv2
import numpy as np

# ===== 카메라 설정 (lidar_ver5.py와 동일 패턴) =====
CAM_WIDTH = 320
CAM_HEIGHT = 240
CAM_FORMAT = "RGB888"

# ===== 파란 장애물 HSV 범위 (lidar_ver5.py 값과 동일 - 실측 후 조정 필요) =====
BLUE_H_MIN, BLUE_H_MAX = 100, 125
BLUE_S_MIN, BLUE_S_MAX = 60, 255
BLUE_V_MIN, BLUE_V_MAX = 40, 255
MIN_OBSTACLE_AREA = 400

# ===== 주행/조향 설정 =====
STEER_LIMIT = 35
OBSTACLE_STEER_GAIN = 25.0    # 회피 조향 강도(도) - 부호는 실차 검증 필요
AVOID_SPEED = 24              # 회피 중 전진 속도
RECOVER_SPEED = 24            # 반대조향 전진 속도
RECOVER_HOLD_SEC = 2.0        # 반대조향 유지하며 전진하는 시간(초)


def get_blue_hsv_range():
    lower = np.array([BLUE_H_MIN, BLUE_S_MIN, BLUE_V_MIN])
    upper = np.array([BLUE_H_MAX, BLUE_S_MAX, BLUE_V_MAX])
    return lower, upper


def detect_blue(frame):
    """
    프레임에서 파란 장애물을 찾아 (found, offset) 반환.
    offset: -1.0(화면 맨왼쪽) ~ +1.0(화면 맨오른쪽), 못 찾으면 0.0
    """
    h, w = frame.shape[:2]
    lower, upper = get_blue_hsv_range()

    blurred = cv2.GaussianBlur(frame, (5, 5), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower, upper)

    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return False, 0.0

    largest = max(contours, key=cv2.contourArea)
    if cv2.contourArea(largest) < MIN_OBSTACLE_AREA:
        return False, 0.0

    bx, by, bw, bh = cv2.boundingRect(largest)
    cx = bx + bw / 2.0
    offset = (cx - w / 2.0) / (w / 2.0)
    return True, offset


def grab_frame(picam2):
    """picamera2 프레임을 OpenCV BGR로 정규화. 실패하면 None."""
    try:
        arr = picam2.capture_array()
    except Exception:
        return None
    if arr is None:
        return None
    if arr.ndim == 3 and arr.shape[2] == 4:
        return cv2.cvtColor(arr, cv2.COLOR_BGRA2BGR)
    if arr.ndim == 3 and arr.shape[2] == 3:
        return arr
    return None


def main():
    device.reset_mcu()
    time.sleep(0.5)

    left_motor = Motor(PWM("P13"), Pin("D4"))
    right_motor = Motor(PWM("P12"), Pin("D5"))
    steer = Servo("P2")

    def set_speed(v):
        left_motor.speed(-v)
        right_motor.speed(v)

    def set_steer(angle):
        angle = max(-STEER_LIMIT, min(STEER_LIMIT, angle))
        steer.angle(angle)

    def stop_all():
        set_speed(0)
        set_steer(0)

    picam2 = Picamera2()
    config = picam2.create_preview_configuration(
        main={"size": (CAM_WIDTH, CAM_HEIGHT), "format": CAM_FORMAT})
    picam2.configure(config)
    picam2.start()
    time.sleep(1.0)   # 센서 안정화

    steer.angle(0)
    time.sleep(0.5)

    was_avoiding = False
    last_steer = 0.0

    print("[시작] 파란 장애물 감시 중 (Ctrl+C로 언제든 안전 종료 가능)")

    try:
        while True:
            frame = grab_frame(picam2)
            if frame is None:
                time.sleep(0.05)
                continue

            found, offset = detect_blue(frame)

            if found:
                # 장애물이 화면 왼쪽(offset<0)에 보이면 오른쪽(+)으로 회피
                # ※ 부호가 반대로 느껴지면 avoid_dir 계산을 뒤집을 것 (실차 검증 필요)
                avoid_dir = -1 if offset < 0 else 1
                steer_val = avoid_dir * OBSTACLE_STEER_GAIN

                set_steer(steer_val)
                set_speed(AVOID_SPEED)

                last_steer = steer_val
                was_avoiding = True
                print(f"[회피] 파란 장애물 offset={offset:+.2f} -> 조향={steer_val:.0f}도")

            else:
                if was_avoiding:
                    # [탈출 판정] 회피 중이었는데 이번 프레임엔 파란색이 안 보임
                    # -> 완전히 벗어났다고 판단
                    print("[탈출] 카메라에서 파란 장애물 사라짐 확인 - 반대조향 전진 시작")
                    break
                else:
                    # 아직 장애물을 만난 적 없음 - 그냥 직진
                    set_steer(0)
                    set_speed(AVOID_SPEED)

            time.sleep(0.05)

        # ===== 탈출 처리: 저장된 조향값의 반대로 2초간 전진 =====
        recovery_steer = -last_steer
        print(f"[탈출] 저장된 조향 {last_steer:.0f}도의 반대인 {recovery_steer:.0f}도로 "
              f"{RECOVER_HOLD_SEC:.0f}초간 전진합니다...")
        set_steer(recovery_steer)
        set_speed(RECOVER_SPEED)
        time.sleep(RECOVER_HOLD_SEC)

        print("[종료] 정지합니다")
        stop_all()

    except KeyboardInterrupt:
        print("\n[중단] Ctrl+C 감지 - 정지 중...")
    finally:
        stop_all()
        try:
            picam2.stop()
            picam2.close()
        except Exception:
            pass
        print("[종료] 프로그램 완전히 종료")


if __name__ == "__main__":
    main()
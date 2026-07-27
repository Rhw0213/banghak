# lidar_avoidance.py
# 라이다(RPLIDAR A1M8) 기반 장애물 회피 주행 - 맨 위 장착(360도)
import time
from rplidar import RPLidar
from robot_hat import Motor, Servo, Pin, PWM, reset_mcu

# ===== 설정 =====
LIDAR_PORT = '/dev/ttyUSB0'

# 라이다 장착 보정
LIDAR_OFFSET = 90          # 라이다가 90도 틀어져 장착됨 (방향 안 맞으면 +90으로)

# 조향 파라미터
STEER_GAIN = 0.3            # 라이다 각도 → 조향각 비율 (낮을수록 둔감)
STEER_DEADZONE = 10         # ±15도 안쪽은 무시 (미세 흔들림 방지)
STEER_ACTIVATE_DIST = 800   # 정면 장애물이 이 거리 안에 있을 때만 조향 (mm)
STEER_LIMIT = 35            # 서보 최대 조향각
GAIN_REVERSE = False        # 조향 방향 반대면 True

# 거리 임계값 (mm)
STOP_DIST = 300             # 50cm 이내 → 정지
SLOW_DIST = 500             # 1m 이내 → 감속
SPEED_FAST = 0 
SPEED_SLOW = 0 

# 스캔 반응성
SCAN_MIN_LEN = 60


def normalize_angle(angle):
    """라이다 0~360 → -180~180 변환 (+ 장착 오프셋 보정)"""
    angle += LIDAR_OFFSET
    angle = angle % 360
    if angle > 180:
        angle -= 360
    return angle


def analyze_scan(scan):
    """
    스캔 데이터 분석
    scan: [(quality, angle, distance), ...]
    반환: (가장 트인 방향 각도, 전방 최소거리)
    """
    best_angle = 0
    best_distance = 0
    front_min = 99999

    for quality, angle, distance in scan:
        if distance <= 0:               # 측정 실패 무시
            continue

        norm = normalize_angle(angle)

        # 전방 ±90도 중 가장 먼(트인) 방향 찾기
        if -90 <= norm <= 90:
            if distance > best_distance:
                best_distance = distance
                best_angle = norm

            # 정면 ±25도의 최소거리 (정면 장애물 감지)
            if -25 <= norm <= 25:
                if distance < front_min:
                    front_min = distance

    return best_angle, front_min


def main():
    reset_mcu()
    time.sleep(0.5)

    # ===== 모터 초기화 (C 코드 기준) =====
    # 모터1: PWM=P13, 방향=D4(GPIO23)
    # 모터2: PWM=P12, 방향=D5(GPIO24)
    left_motor = Motor(PWM("P13"), Pin("D4"))
    right_motor = Motor(PWM("P12"), Pin("D5"))
    steer = Servo("P2")

    lidar = RPLidar(LIDAR_PORT)

    # 라이다 회전 속도 (버전에 메서드 있으면 적용, 없으면 건너뜀)
    try:
        lidar.motor_speed = 660
    except Exception:
        pass

    def set_speed(v):
        left_motor.speed(-v)       # 왼쪽이 반대로 도는 것 보정
        right_motor.speed(v)

    def set_steer(angle):
        angle = max(-STEER_LIMIT, min(STEER_LIMIT, angle))
        steer.angle(angle)

    print("라이다 회피 주행 시작 (Ctrl+C로 종료)")
    steer.angle(0)
    time.sleep(1)

    try:
        for scan in lidar.iter_scans(min_len=SCAN_MIN_LEN):
            clear_angle, front_min = analyze_scan(scan)

            # --- 조향: 정면에 장애물이 가까울 때만 작동 ---
            if front_min < STEER_ACTIVATE_DIST and abs(clear_angle) >= STEER_DEADZONE:
                steer_cmd = clear_angle * STEER_GAIN

                # 정면이 아주 가까울수록 조향 증폭
                if front_min < 700:
                    urgency = (700 - front_min) / 700    # 0~1
                    steer_cmd *= (1 + urgency)            # 최대 2배
            else:
                steer_cmd = 0        # 장애물이 멀거나 정면 근처면 직진

            if GAIN_REVERSE:
                steer_cmd = -steer_cmd
            set_steer(steer_cmd)

            # --- 속도: 전방 거리에 따라 ---
            if front_min < STOP_DIST:
                set_speed(0)              # 너무 가까움 → 정지
            elif front_min < SLOW_DIST:
                set_speed(SPEED_SLOW)     # 가까움 → 감속
            else:
                set_speed(SPEED_FAST)     # 트임 → 정상 주행

            print(f"트인방향={clear_angle:6.1f}도  "
                  f"조향={steer_cmd:6.1f}  전방최소={front_min:5.0f}mm")

    except KeyboardInterrupt:
        print("\n종료 중...")
    finally:
        set_speed(0)
        set_steer(0)
        lidar.stop()
        lidar.stop_motor()
        lidar.disconnect()
        print("정지 완료")


if __name__ == "__main__":
    main()

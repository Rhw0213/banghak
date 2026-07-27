# lidar_ultra_avoidance.py
# 라이다 + 초음파 센서 융합 장애물 회피 주행
import time
from rplidar import RPLidar
from robot_hat import Motor, Servo, Pin, PWM, reset_mcu, Ultrasonic

from avoidance_go_back import Get_Stop_Distance, WallBackup, Get_Drive_Duty
from picarx import Picarx

from battery_driving_check import BatteryMonitor

# ===== 설정 =====
LIDAR_PORT = '/dev/ttyUSB0'
# LIDAR_OFFSET = 90          # 라이다 90도 틀어짐 보정

# 조향
STEER_GAIN = 0.3           # 라이다 각도 → 조향각 비율
STEER_DEADZONE = 0         # ±15도 안쪽은 직진
STEER_ACTIVATE_DIST = 800  # 정면 이 거리(mm) 안일 때만 조향
STEER_LIMIT = 35           # 서보 최대 조향각
GAIN_REVERSE = False       # 조향 방향 반대면 True

# 거리 임계값 (라이다=mm, 초음파=cm)
DANGER_DIST_MM = 500       # 라이다 50cm 이내 위험
STOP_DIST_MM = 350         # 라이다 35cm 이내 정지
ULTRA_DANGER_CM = 40       # 초음파 40cm 이내 위험
#ULTRA_STOP_CM = 25        # 초음파 25cm 이내 정지

# 속도
VELOCITY = 60 
SPEED_FAST = 0             # 전진용 슬로우스타터 (robot_hat)
SPEED_BACK = 0              # 후진용 슬로우스타터 (picarx, wallBackup에 duty로 전달)
SPEED_SLOW = VELOCITY / 2

SPEED_STEP = 3              # 가감속 스텝

SCAN_MIN_LEN = 60


def normalize_angle(angle):
    """라이다 0~360 → -180~180 (+ 오프셋 보정)"""
    # angle += LIDAR_OFFSET
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


def main():
    reset_mcu()
    time.sleep(0.5)

    global SPEED_FAST
    global SPEED_BACK
    global VELOCITY
    global STEEL_CMD 

    x = Picarx()
    wallBackup = WallBackup(x, Get_Drive_Duty())
    batteryMonitor = BatteryMonitor()

    # 모터/서보 (모터1=P13/D4, 모터2=P12/D5, 조향=P2)
    left_motor = Motor(PWM("P13"), Pin("D4"))
    right_motor = Motor(PWM("P12"), Pin("D5"))
    steer = Servo("P2")

    # 초음파 (trig=D3, echo=D2)
    sonar = Ultrasonic(Pin("D2"), Pin("D3"))

    # 라이다
    lidar = RPLidar(LIDAR_PORT)

    # ---------- 전진 전용 (robot_hat) ----------
    def set_incre_Move(target):
        """전진 속도를 SPEED_STEP만큼 증가 (target을 넘지 않게)"""
        global SPEED_FAST
        SPEED_FAST = min(SPEED_FAST + SPEED_STEP, target)
        left_motor.speed(-SPEED_FAST)   # 왼쪽 반대로 도는 것 보정
        right_motor.speed(SPEED_FAST)
        return SPEED_FAST

    def set_decre_Move(target=0):
        """전진 속도를 SPEED_STEP만큼 감소 (target 밑으로 안 내려가게)"""
        global SPEED_FAST
        SPEED_FAST = max(SPEED_FAST - SPEED_STEP, target)
        left_motor.speed(-SPEED_FAST)
        right_motor.speed(SPEED_FAST)
        if SPEED_FAST == 0:
            left_motor.speed(0)
            right_motor.speed(0)
        return SPEED_FAST


    def set_speed(v):
        """목표 속도 v를 향해 전진 속도를 한 스텝 조정 (robot_hat 경로 전용)"""
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
        """후진 duty를 SPEED_STEP만큼 증가시켜 WallBackup에 전달할 값 반환"""
        global SPEED_BACK
        SPEED_BACK = min(SPEED_BACK + SPEED_STEP, target)
        return SPEED_BACK

    def back_decre_Move(target=0):
        """후진 duty를 SPEED_STEP만큼 감소"""
        global SPEED_BACK
        SPEED_BACK = max(SPEED_BACK - SPEED_STEP, target)
        return SPEED_BACK

    def set_steer(angle):
        angle = max(-STEER_LIMIT, min(STEER_LIMIT, angle))
        if GAIN_REVERSE:
            angle = -angle
        # 5도 오른쪽 offset
        steer.angle(angle + 5)

    def read_ultra_cm():
        try:
            d = sonar.read()
            if d is None or d < 0:
                return -1
            return d
        except Exception:
            return -1

    print("라이다+초음파 회피 주행 시작 (Ctrl+C 종료)")
    steer.angle(0)
    time.sleep(1)

    backCnt = 0
    isBack = False
    isBackFlag = False
    #BACK_TARGET = Get_Drive_Duty()   # 후진 목표 duty
    BACK_TARGET = VELOCITY * 0.65     # 후진 목표 duty

    backSpeed = 20 * (50 / BACK_TARGET) # 후진시  

    steel_gain_result = 0

    try:
        for scan in lidar.iter_scans(min_len=SCAN_MIN_LEN):
            batteryMonitor.show()

            clear_angle, lidar_min = analyze_scan(scan)   # mm
            ultra_cm = read_ultra_cm()                    # cm (-1이면 실패)

            # ===== 후진 구간: picarx(WallBackup) 경로만 사용 =====
            if (isBack and (backCnt <= backSpeed)):
                duty = back_incre_Move(BACK_TARGET)
                wallBackup.update(duty)
                print(f"[후진] duty={duty} ({backCnt}/{backSpeed})")
                backCnt += 1
                continue
            else:
                if (isBack):
                    print("후진 종료, 조향 :0")
                    wallBackup.stop(steel_gain_result)
                    SPEED_BACK = 0     # 후진 상태 리셋
                    SPEED_FAST = 0     # 전진 재개 시 슬로우스타터부터 다시 시작
                    isBackFlag = True
                isBack = False
                backCnt = 0

            # ===== 1순위: 정지 (둘 중 하나라도 초근접) =====
            if lidar_min < STOP_DIST_MM or (0 < ultra_cm < Get_Stop_Distance()):
                set_decre_Move(0)   # 전진 속도를 서서히 0으로
                steel_gain_result = clear_angle * STEER_GAIN;
                set_steer(steel_gain_result)
                print(f"[정지] 라이다 {lidar_min:.0f}mm 초음파 {ultra_cm:.0f}cm 트인 {clear_angle:.0f}도")
                isBack = True
                continue

            # ===== 2순위: 위험 → 감속 + 회피 조향 =====
            if lidar_min < DANGER_DIST_MM or (0 < ultra_cm < Get_Stop_Distance()):
                set_speed(SPEED_SLOW)
                steel_gain_result = clear_angle * STEER_GAIN;
                set_steer(steel_gain_result)
                print(f"[회피] 라이다 {lidar_min:.0f}mm 초음파 {ultra_cm:.0f}cm 트인 {clear_angle:.0f}도")
                continue

            # ===== 3순위: 안전 → 정상 주행 =====
            if lidar_min < STEER_ACTIVATE_DIST and abs(clear_angle) >= STEER_DEADZONE:
                steer_cmd = clear_angle * STEER_GAIN
                steel_gain_result = steer_cmd
            else:
                steer_cmd = 0        # 멀거나 정면이면 직진

            if (not isBackFlag):
                set_steer(steer_cmd)
                print("ㅁ면면정ㅈ직직직ㄴ")

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
        print("정지 완료")


if __name__ == "__main__":
    main()

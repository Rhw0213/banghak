# 라이다(RPLIDAR A1M8) 기반 벽 장애물 회피 + 모서리 갇힘 탈출(후진) 주행 코드
#
# ---------------------------------------------------------------------------
# [단순화된 후진 판단 로직]
#   - 평소 주행(DRIVE)은 기존 안정 버전 로직 그대로 사용 (활성화거리/데드존/urgency 조향)
#   - "후진이 필요한 상황"을 딱 두 가지 조건으로 단순화:
#       1) 전방 180도(-90~90도) 안에 FRONT_TRAP_DIST(5cm) 이내로 초근접한 장애물이 있다
#          -> 더 이상 전진할 수 없는 "갇힘" 상태로 판단
#       2) 후방 180도(90~270도, 즉 뒤쪽 반원) 전체에 REAR_CLEAR_DIST(50cm) 이내
#          장애물이 하나도 없다 -> 뒤로는 안전하게 빠질 수 있다고 판단
#     두 조건이 동시에 만족될 때만 후진을 실행합니다.
#   - 복잡한 갭(gap) 각도 계산이 필요 없으므로, 후진 시 조향은 그냥 0(정면 유지)로 두고
#     똑바로 뒤로 빠지는 방식입니다.
#
# ---------------------------------------------------------------------------
# [상태 구조]
#
#   1) DRIVE    : 평소 주행. 트인 방향으로 조향 + 거리 기반 속도 조절.
#                 전방 180도 최소거리가 FRONT_TRAP_DIST 이하인 상황이
#                 STUCK_FRAMES번 연속되면 -> TRAPPED로 전환.
#
#   2) TRAPPED  : 차량 완전 정지. 라이다는 계속 회전하며 매 스캔마다 재판단.
#                 - 전방이 다시 열리면(FRONT_TRAP_DIST보다 멀어지면) -> DRIVE 복귀
#                 - 전방은 여전히 막혔지만 후방 180도가 REAR_CLEAR_DIST 이상 비어있으면
#                   -> REVERSING으로 전환
#                 - 둘 다 아니면 계속 정지 상태 유지하며 다음 스캔에서 재확인
#
#   3) REVERSING: 조향 0(직진 유지)으로 저속 후진.
#                 - 전방 거리가 충분히 회복되면(더 이상 근접 위험 없으면) -> DRIVE 복귀
#                 - 후방이 다시 가까워지면(REAR_STOP_DIST 이내) -> 즉시 정지 후 TRAPPED로 복귀 (안전)
#                 - 시간 초과 시에도 TRAPPED로 복귀해 재판단
#
# 상태 전이 그림:
#
#   [DRIVE] --(전방 5cm 이내 연속감지)--> [TRAPPED] --(후방 50cm 이상 확보)--> [REVERSING]
#      ^                                     |   ^                                |
#      |                                     |   |--(후방도 막힘: 대기 재판단)------|
#      |----------------(전방 다시 열림)-------------------------------------------|
#
# ---------------------------------------------------------------------------
# [라이다 좌표계 & 장착 오프셋]
# angle: 0~360도 라이다 기준. LIDAR_OFFSET을 더해 차량 정면=0도 기준으로 재정렬.
#   전방 180도  : -90도 ~ 90도
#   후방 180도  : 90도 ~ 180도, 그리고 -180도 ~ -90도 (두 구간을 합친 것)
# =====================================================================================

import time
from rplidar import RPLidar
from robot_hat import Motor, Servo, Pin, PWM, reset_mcu

# ======================================================================
# ===== 설정값 =====
# ======================================================================
LIDAR_PORT = '/dev/ttyUSB0'

# --- 라이다 장착 보정 ---
LIDAR_OFFSET = 90   # 라이다가 정면 기준 90도 틀어져 장착됨. 실측해서 보정할 것.

# --- 조향 관련 (평소 주행용, 검증된 파라미터) ---
STEER_GAIN = 0.3
STEER_DEADZONE = 15         # ±15도 이내 트인방향은 무시(직진) -> 미세 흔들림 방지
STEER_ACTIVATE_DIST = 800   # 정면 최소거리가 이보다 가까울 때만 조향 개입
STEER_LIMIT = 35
GAIN_REVERSE = False

# --- 평소 주행 속도 제어용 거리 임계값 (mm) ---
STOP_DIST = 300
SLOW_DIST = 500
SPEED_FAST = 60
SPEED_SLOW = 45

# --- 후진(갇힘) 판단 임계값 (mm) : 사용자가 요청한 단순 기준 ---
FRONT_TRAP_DIST = 50        # 전방 180도 안에 이 거리(5cm) 이내 근접 장애물이 있으면 "갇힘" 후보
REAR_CLEAR_DIST = 500       # 후방 180도 전체가 이 거리(50cm) 이상 비어 있어야 후진 가능 판단
REAR_STOP_DIST = 150        # 후진 중 후방이 이 거리(15cm) 이내로 가까워지면 즉시 정지 (안전장치)
FRONT_RECOVER_DIST = 300    # 후진 중 전방이 이 거리 이상 회복되면 DRIVE로 복귀

# --- 상태 전환 조건 ---
STUCK_FRAMES = 5            # FRONT_TRAP_DIST 이하가 이 횟수만큼 연속돼야 TRAPPED로 전환 (디바운스)
REVERSE_SPEED = 25          # 후진 시 사용하는 저속
REVERSE_TIMEOUT = 5.0        # 후진을 최대 몇 초까지 유지할지 (넘으면 정지 후 재판단)

# --- 라이다 스캔 옵션 ---
SCAN_MIN_LEN = 60


# ======================================================================
# ===== 유틸 함수들 =====
# ======================================================================
def normalize_angle(angle):
    """라이다 0~360도 -> 차량 기준 -180~180도(정면=0) 변환. LIDAR_OFFSET으로 장착각 보정."""
    angle += LIDAR_OFFSET
    angle = angle % 360
    if angle > 180:
        angle -= 360
    return angle


def analyze_scan(scan):
    """
    [DRIVE 상태에서 사용]
    scan: [(quality, angle, distance), ...]

    전방 ±90도 안에서 "가장 트인 방향"과 "정면 ±25도 최소거리"를 계산.
    (평소 주행용 조향/속도 판단에 사용, 기존 안정 버전과 동일)
    """
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
            if -25 <= norm <= 25:
                if distance < front_min:
                    front_min = distance

    return best_angle, front_min


def compute_steer_command(clear_angle, front_min):
    """
    [DRIVE 전용 안정 조향 계산]
    - 장애물이 멀면(STEER_ACTIVATE_DIST 밖) 조향 안 함 -> 직진
    - 트인 방향이 데드존(±15도) 이내면 조향 안 함 -> 직진
    - 그 외에는 STEER_GAIN 적용 + 700mm 이내로 가까울수록 최대 2배 증폭
    """
    if front_min < STEER_ACTIVATE_DIST and abs(clear_angle) >= STEER_DEADZONE:
        steer_cmd = clear_angle * STEER_GAIN
        if front_min < 700:
            urgency = (700 - front_min) / 700
            steer_cmd *= (1 + urgency)
    else:
        steer_cmd = 0

    if GAIN_REVERSE:
        steer_cmd = -steer_cmd

    return steer_cmd


def analyze_front_rear_min(scan):
    """
    [TRAPPED / REVERSING 상태에서 사용]
    scan: [(quality, angle, distance), ...]

    전방 180도(-90~90)와 후방 180도(90~180, -180~-90 합산) 각각의 최소거리를 구한다.
    - front_min : 전방 180도 안에서 가장 가까운 장애물까지의 거리
    - rear_min  : 후방 180도 안에서 가장 가까운 장애물까지의 거리
                  (해당 범위에 데이터가 전혀 없으면 "모른다"이므로 안전하지 않다고 보고
                   0을 반환해서 후진 조건을 만족하지 못하게 함)

    이 함수는 gap 각도 계산 없이 "전방/후방 각각 최악의 경우(가장 가까운 값)"만 보는
    단순한 방식이라, 별도의 bin/히스토그램 구조가 필요 없다.
    """
    front_min = 99999
    rear_min = 99999
    rear_has_data = False

    for quality, angle, distance in scan:
        if distance <= 0:
            continue
        norm = normalize_angle(angle)

        if -90 <= norm <= 90:
            # 전방 180도
            if distance < front_min:
                front_min = distance
        else:
            # 후방 180도 (90도 초과 ~ 180도, 그리고 -180도 ~ -90도 미만)
            rear_has_data = True
            if distance < rear_min:
                rear_min = distance

    if not rear_has_data:
        rear_min = 0  # 후방 데이터가 전혀 없으면 "모른다" -> 안전하지 않은 것으로 취급

    return front_min, rear_min


# ======================================================================
# ===== 메인 로직 =====
# ======================================================================
def main():
    reset_mcu()
    time.sleep(0.5)

    left_motor = Motor(PWM("P13"), Pin("D4"))
    right_motor = Motor(PWM("P12"), Pin("D5"))
    steer = Servo("P2")

    lidar = RPLidar(LIDAR_PORT)
    try:
        lidar.motor_speed = 660
    except Exception:
        pass

    def set_speed(v):
        left_motor.speed(-v)
        right_motor.speed(v)

    def set_steer(angle):
        angle = max(-STEER_LIMIT, min(STEER_LIMIT, angle))
        steer.angle(angle)

    print("라이다 회피/후진탈출 주행 시작 (Ctrl+C로 종료)")
    steer.angle(0)
    time.sleep(1)

    # ---- 상태머신 변수 ----
    state = "DRIVE"
    stuck_count = 0            # 전방 초근접이 연속으로 감지된 횟수
    reverse_start_time = 0.0    # REVERSING 진입 시각 (타임아웃 계산용)

    try:
        scan_iter = lidar.iter_scans(min_len=SCAN_MIN_LEN)

        for scan in scan_iter:

            # =========================================================
            # 상태 1: DRIVE (평소 주행)
            # =========================================================
            if state == "DRIVE":
                clear_angle, front_min = analyze_scan(scan)

                steer_cmd = compute_steer_command(clear_angle, front_min)
                set_steer(steer_cmd)

                if front_min < STOP_DIST:
                    set_speed(0)
                elif front_min < SLOW_DIST:
                    set_speed(SPEED_SLOW)
                else:
                    set_speed(SPEED_FAST)

                print(f"[DRIVE] 트인방향={clear_angle:6.1f}도  "
                      f"조향={steer_cmd:6.1f}  전방최소={front_min:5.0f}mm")

                # 전방 180도 기준 초근접(FRONT_TRAP_DIST) 판단은
                # analyze_scan의 front_min(정면 ±25도)이 아니라 전방 180도 전체 기준으로
                # 별도 확인해야 하므로, 여기서 analyze_front_rear_min을 함께 사용한다.
                front180_min, _ = analyze_front_rear_min(scan)

                if front180_min <= FRONT_TRAP_DIST:
                    stuck_count += 1
                else:
                    stuck_count = 0

                if stuck_count >= STUCK_FRAMES:
                    print(f">>> 전방 {FRONT_TRAP_DIST}mm 이내 초근접 지속: "
                          f"TRAPPED 상태로 전환 (차량 정지)")
                    set_speed(0)
                    state = "TRAPPED"

            # =========================================================
            # 상태 2: TRAPPED (정지 후 전방/후방 재판단)
            # =========================================================
            elif state == "TRAPPED":
                set_speed(0)   # 판단 전까지는 항상 정지 유지
                set_steer(0)    # 바퀴도 정면으로 정렬해두면 이후 후진이 더 예측 가능해짐

                front_min, rear_min = analyze_front_rear_min(scan)
                print(f"[TRAPPED] 전방최소={front_min:5.0f}mm  후방최소={rear_min:5.0f}mm")

                if front_min > FRONT_TRAP_DIST:
                    # 전방이 다시 열렸다면(예: 차가 살짝 튀어서 각도가 바뀌었거나,
                    # 장애물이 이동한 경우 등) 굳이 후진할 필요 없이 바로 평소 주행 복귀
                    print(">>> 전방 재확보됨: DRIVE 모드로 복귀")
                    state = "DRIVE"
                    stuck_count = 0

                elif rear_min >= REAR_CLEAR_DIST:
                    # 전방은 여전히 막혔지만 후방 180도 전체가 충분히 비어있음 -> 후진 시작
                    print(f">>> 후방 {REAR_CLEAR_DIST}mm 이상 확보됨: REVERSING 상태로 전환")
                    state = "REVERSING"
                    reverse_start_time = time.time()

                else:
                    # 전방도 막히고 후방도 막힘 -> 진짜 완전히 갇힌 상태.
                    # 이 코드에서는 무리하게 움직이지 않고 계속 정지한 채 다음 스캔을 기다림.
                    print(">>> 전방/후방 모두 막힘: 정지 유지, 재판단 대기")

            # =========================================================
            # 상태 3: REVERSING (직진 후진으로 탈출)
            # =========================================================
            elif state == "REVERSING":
                front_min, rear_min = analyze_front_rear_min(scan)
                elapsed = time.time() - reverse_start_time

                # 방향 계산이 필요 없는 단순 로직이므로 조향은 0(정면) 유지, 속도만 음수(후진)
                set_steer(0)
                set_speed(-REVERSE_SPEED)

                print(f"[REVERSING] 전방최소={front_min:5.0f}mm  "
                      f"후방최소={rear_min:5.0f}mm  경과={elapsed:4.1f}s")

                if rear_min <= REAR_STOP_DIST:
                    # 후진 중 뒤쪽에 뭔가 가까워짐 -> 안전을 위해 즉시 정지하고 재판단
                    print(f">>> 후진 중 후방 {REAR_STOP_DIST}mm 이내 근접: 정지 후 TRAPPED로 복귀")
                    set_speed(0)
                    state = "TRAPPED"

                elif front_min >= FRONT_RECOVER_DIST:
                    # 후진으로 전방 여유가 충분히 확보됨 -> 평소 주행 복귀
                    print(">>> 전방 여유 확보됨: DRIVE 모드로 복귀")
                    set_speed(0)
                    state = "DRIVE"
                    stuck_count = 0

                elif elapsed >= REVERSE_TIMEOUT:
                    # 너무 오래 후진해도 상황이 안 풀리면 정지하고 재판단
                    print(">>> 후진 시간 초과: 정지 후 TRAPPED로 복귀")
                    set_speed(0)
                    state = "TRAPPED"

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
"""
main.py

전체 흐름:
1. 초음파 센서로 전방 거리 측정
2. 정상거리(danger 아님, OBSTACLE_DISTANCE 이상)면 그대로 직진
3. 위험(danger) 또는 OBSTACLE_DISTANCE 이내 감지되면 AVOIDING 상태로 전환,
   앞바퀴를 TURN_ANGLE만큼 꺾은 채로 저속 이동하며 회피
4. CLEAR_DISTANCE보다 멀어지고 danger도 아니면 즉시 NORMAL로 복귀
   (또는 MAX_AVOID_DURATION 초과 시 타임아웃으로 강제 복귀)
"""

from picarx import Picarx
from movement import move_forward, stop, DUTY
import time

px = Picarx()

# --- 거리 기준값 ---
OBSTACLE_DISTANCE = 40   # 이 거리(cm) 이내로 들어오면 회피 시작
CLEAR_DISTANCE = 55      # 이 거리(cm)보다 멀어지면 회피 종료(탈출) 후보로 판단
TURN_ANGLE = 40           # 회피 시 조향각(도)

# --- 속도 설정 ---
# 주의: 현재 두 값이 동일(30)해서, 8cm 이내 근접 시 더 감속하는 CRAWL 로직이
#       사실상 동작하지 않고 있음(항상 30으로 움직임). 필요 시 CRAWL_DUTY만 낮추면
#       근접 구간에서 다시 감속 효과를 줄 수 있음.
AVOID_DUTY = 30           # 일반 회피 상태에서 사용하는 속도(duty %)
CRAWL_DUTY = 30           # CRAWL_DISTANCE 이내로 근접했을 때 사용하는 속도(duty %)
CRAWL_DISTANCE = 8        # 이 거리(cm) 이내면 CRAWL_DUTY로 전환(더 조심스럽게)

# --- 노이즈/오탐 방지 파라미터 ---
INVALID_READINGS = (-1, -2)   # 초음파 에코 소실 시 나오는 무효값들
MAX_INVALID_STREAK = 3         # 무효값이 이만큼 연속되면 "신호 소실로 인한 위험"으로 간주할 후보
DROP_THRESHOLD = 20             # 직전 유효거리 대비 이만큼(cm) 급감하면 위험 후보로 간주
NEAR_ZONE = 50                   # 이 거리(cm) 이내일 때만 위 급감 판정을 적용 (먼 거리 노이즈 무시)
DANGER_CONFIRM = 2                # raw_danger가 이 횟수만큼 "연속"으로 떠야 실제 위험으로 확정 (디바운스)

# --- 회피 상태 지속시간 제어 ---
MAX_AVOID_DURATION = 3.0   # 회피 상태가 이 시간(초)을 넘기면 무조건 NORMAL로 복귀 (무한 회전 방지용 안전장치)
MIN_AVOID_DURATION = 0.3   # 회피 진입 후 최소 이 시간(초) 동안은 무조건 회전 유지 (너무 빨리 복귀해 덜 꺾이는 것 방지)

# --- 센서 판독 상태를 프레임 간 기억하기 위한 전역 변수 ---
prev_distance = None    # 직전 사이클의 "유효했던" 거리값
prev_was_valid = False  # 직전 사이클 판독이 유효했는지 여부
invalid_streak = 0      # 무효값(-1,-2)이 연속된 횟수
danger_streak = 0       # raw_danger가 연속된 횟수 (디바운스 카운터)


def turn_right(px, angle=TURN_ANGLE, duty=AVOID_DUTY):
    """앞바퀴를 angle만큼 꺾은 상태로 duty 속도로 전진(우회전 회피 동작).
    모터 방향 핀 제어는 movement.py의 move_forward()와 동일한 검증된 방식."""
    px.set_dir_servo_angle(angle)
    px.motor_direction_pins[0].low()
    px.motor_direction_pins[1].high()
    duty = max(0, min(100, duty))  # duty 값을 0~100 범위로 안전하게 clamp
    px.motor_speed_pins[0].pulse_width_percent(duty)
    px.motor_speed_pins[1].pulse_width_percent(duty)


def read_safe_distance(px):
    """초음파 원시값을 읽고, 노이즈/정반사로 인한 오탐을 걸러낸 뒤
    '확정된 위험 여부(confirmed_danger)'를 함께 반환한다.

    처리 순서:
    1) 원시 거리값 읽기, 유효값인지(무효값 -1/-2가 아닌지) 판별
    2) 유효값이면: NEAR_ZONE 이내에서만 직전값 대비 급감(DROP_THRESHOLD 초과) 여부 확인
       무효값이면: 무효 연속 횟수를 누적하고, 직전이 NEAR_ZONE 이내로 가까웠을 때만
                  '신호 소실 = 위험'으로 후보 처리 (먼 거리에서의 무효값은 그냥 노이즈로 무시)
    3) 위 조건(raw_danger)이 DANGER_CONFIRM 횟수만큼 연속돼야 최종 위험으로 확정 (디바운스)
       -> 단발성 노이즈로 인한 오조향을 방지하기 위함
    """
    global prev_distance, prev_was_valid, invalid_streak, danger_streak

    distance = px.ultrasonic.read()
    is_valid = (distance not in INVALID_READINGS) and distance > 0
    raw_danger = False

    if is_valid:
        # 직전 사이클도 유효했고, 현재 거리가 NEAR_ZONE 이내일 때만 급감 판정 적용
        # (먼 거리에서는 값이 크게 튀는 게 정상 노이즈라 오탐 방지 차원에서 제외)
        if prev_was_valid and prev_distance is not None and distance < NEAR_ZONE:
            delta = prev_distance - distance
            if delta > DROP_THRESHOLD:
                raw_danger = True
        invalid_streak = 0
    else:
        invalid_streak += 1
        # 직전 유효거리가 가까웠는데(NEAR_ZONE 이내) 신호가 MAX_INVALID_STREAK번 이상
        # 연속으로 끊기면, "각도 때문에 반사파를 못 받는 상황"으로 간주해 위험 후보로 처리
        if prev_distance is not None and prev_distance < NEAR_ZONE \
           and invalid_streak >= MAX_INVALID_STREAK:
            raw_danger = True

    if is_valid:
        prev_distance = distance  # 유효값일 때만 "직전 유효거리"를 갱신
    prev_was_valid = is_valid

    # 디바운스: raw_danger가 DANGER_CONFIRM번 연속으로 떠야만 최종 위험으로 확정
    if raw_danger:
        danger_streak += 1
    else:
        danger_streak = 0
    confirmed_danger = danger_streak >= DANGER_CONFIRM

    return distance, confirmed_danger


# --- 상태 정의 ---
STATE_NORMAL = "NORMAL"      # 정상 직진 상태
STATE_AVOIDING = "AVOIDING"  # 장애물 회피(회전) 상태
state = STATE_NORMAL
avoid_start_time = 0.0        # AVOIDING 상태에 진입한 시각 (지속시간 계산용)

try:
    while True:
        now = time.time()
        distance, danger = read_safe_distance(px)

        # 근접 거리(CRAWL_DISTANCE 이내)면 CRAWL_DUTY, 아니면 AVOID_DUTY 사용
        # (현재 두 값이 같아서 실질적 차이는 없음 - 위 설명 참고)
        if 0 < distance < CRAWL_DISTANCE:
            current_duty = CRAWL_DUTY
        else:
            current_duty = AVOID_DUTY

        print(f"거리: {distance} cm | 위험판단: {danger} | 소실연속: {invalid_streak} "
              f"| 상태: {state} | 속도: {current_duty}")

        if state == STATE_NORMAL:
            # 위험 확정 또는 OBSTACLE_DISTANCE 이내 감지 시 회피 진입
            if danger or (0 < distance < OBSTACLE_DISTANCE):
                turn_right(px, TURN_ANGLE, current_duty)
                state = STATE_AVOIDING
                avoid_start_time = now
            else:
                move_forward(px)  # 정상 상황: 앞바퀴 정면 유지한 채 직진

        elif state == STATE_AVOIDING:
            elapsed = now - avoid_start_time

            # 탈출 조건: 현재 거리가 CLEAR_DISTANCE보다 멀고, danger도 아닌 경우
            # (주의: 이 조건은 디바운스 없이 단 한 프레임만 만족해도 즉시 True가 됨 ->
            #  비스듬한 벽에서 노이즈로 한 프레임 튀면 조기 복귀 후 충돌할 수 있는 지점)
            escaped = (distance > CLEAR_DISTANCE) and not danger

            # 회피가 너무 오래 지속되는 것을 막는 안전장치(타임아웃)
            timed_out = elapsed > MAX_AVOID_DURATION

            if elapsed < MIN_AVOID_DURATION:
                # 최소 유지시간 동안은 무조건 회전 지속 (조기 복귀 방지)
                turn_right(px, TURN_ANGLE, current_duty)
            elif escaped or timed_out:
                # 탈출 조건 충족 또는 타임아웃 -> 다음 루프에서 NORMAL로 재평가됨
                state = STATE_NORMAL
            else:
                # 아직 탈출 못했으면 계속 회전 유지
                turn_right(px, TURN_ANGLE, current_duty)

        time.sleep(0.1)  # 센서 폴링/제어 주기 (초)

except KeyboardInterrupt:
    stop(px)
    print("종료")
"""
main.py
방법 1: 전방 40cm 장애물 감지 -> 감속 + 앞바퀴 45도 우회전하며 회피 -> 
        장애물 탈출(거리 회복) 시 다시 정바퀴 직진
        + 원거리 노이즈로 인한 오탐 방지
        + 근접 시 상태 무관 긴급정지
"""

from picarx import Picarx
from movement import move_forward, stop, DUTY
import time

px = Picarx()

OBSTACLE_DISTANCE = 40      # 회피 진입 기준(cm) - 기존 30에서 상향, 반응 여유 확보
CLEAR_DISTANCE = 55         # 회피 종료(탈출 인정) 기준(cm)
TURN_ANGLE = 40
AVOID_DUTY = 15             # 회피 중엔 저속으로 (기존 DUTY보다 낮게 - 회전반경 확보)
EMERGENCY_DISTANCE = 8      # 이 이하로 들어오면 상태 무관 즉시 정지

INVALID_READINGS = (-1, -2)
MAX_INVALID_STREAK = 3
DROP_THRESHOLD = 20
NEAR_ZONE = 80               # 이 거리 이내일 때만 델타(급감) 판정을 적용 (원거리 노이즈 무시)
MAX_AVOID_DURATION = 2.0
MIN_AVOID_DURATION = 0.3

prev_distance = None
prev_was_valid = False
invalid_streak = 0


def turn_right(px, angle=TURN_ANGLE, duty=AVOID_DUTY):
    px.set_dir_servo_angle(angle)
    px.motor_direction_pins[0].low()
    px.motor_direction_pins[1].high()
    duty = max(0, min(100, duty))
    px.motor_speed_pins[0].pulse_width_percent(duty)
    px.motor_speed_pins[1].pulse_width_percent(duty)


def read_safe_distance(px):
    global prev_distance, prev_was_valid, invalid_streak

    distance = px.ultrasonic.read()
    is_valid = (distance not in INVALID_READINGS) and distance > 0
    danger = False

    if is_valid:
        # 델타 판정은 '이미 가까운 거리'일 때만 의미있게 적용 (원거리 노이즈 무시)
        if prev_was_valid and prev_distance is not None and distance < NEAR_ZONE:
            delta = prev_distance - distance
            if delta > DROP_THRESHOLD:
                danger = True
        invalid_streak = 0
    else:
        invalid_streak += 1
        if prev_distance is not None and prev_distance < NEAR_ZONE \
           and invalid_streak >= MAX_INVALID_STREAK:
            danger = True

    if is_valid:
        prev_distance = distance
    prev_was_valid = is_valid

    return distance, danger


STATE_NORMAL = "NORMAL"
STATE_AVOIDING = "AVOIDING"
state = STATE_NORMAL
avoid_start_time = 0.0

try:
    while True:
        now = time.time()
        distance, danger = read_safe_distance(px)
        print(f"거리: {distance} cm | 위험판단: {danger} | 소실연속: {invalid_streak} | 상태: {state}")

        # 최우선: 상태와 무관하게 물리적 충돌 임박 시 즉시 정지
        if 0 < distance < EMERGENCY_DISTANCE:
            stop(px)
            print(">>> 긴급정지 (근접 위험) <<<")
            time.sleep(0.1)
            continue

        if state == STATE_NORMAL:
            if danger or (0 < distance < OBSTACLE_DISTANCE):
                turn_right(px, TURN_ANGLE, AVOID_DUTY)
                state = STATE_AVOIDING
                avoid_start_time = now
            else:
                move_forward(px)

        elif state == STATE_AVOIDING:
            elapsed = now - avoid_start_time
            escaped = (distance > CLEAR_DISTANCE) and not danger
            timed_out = elapsed > MAX_AVOID_DURATION

            if elapsed < MIN_AVOID_DURATION:
                turn_right(px, TURN_ANGLE, AVOID_DUTY)
            elif escaped or timed_out:
                state = STATE_NORMAL
            else:
                turn_right(px, TURN_ANGLE, AVOID_DUTY)

        time.sleep(0.1)

except KeyboardInterrupt:
    stop(px)
    print("종료")
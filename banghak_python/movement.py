"""
movement.py
역할: 연속 저속 전진 (킥스타트로 정지 마찰 극복 후 저속 유지)
"""

from picarx import Picarx
import time

SPEED_TABLE = {
    10: 3.0,
    20: 7.5,
    30: 16.0,
    40: 23.5,
    50: 30.5,
}

DUTY = 13           # 유지할 저속 듀티
KICK_DUTY = 35       # 시작할 때 순간적으로 밀어줄 듀티 (정지 마찰 극복용)
KICK_TIME = 0.08     # 킥스타트 지속 시간 (초) - 너무 길면 그냥 빨라짐


def move_forward(px, duty=DUTY, kick=True):
    px.set_dir_servo_angle(0)

    px.motor_direction_pins[0].low()
    px.motor_direction_pins[1].high()

    if kick:
        # 1단계: 짧게 강하게 밀어서 정지 마찰 극복
        px.motor_speed_pins[0].pulse_width_percent(KICK_DUTY)
        px.motor_speed_pins[1].pulse_width_percent(KICK_DUTY)
        time.sleep(KICK_TIME)

    # 2단계: 원하는 저속으로 낮춰서 유지 (이미 구르는 중이라 낮은 듀티로도 유지됨)
    duty = max(0, min(100, duty))
    px.motor_speed_pins[0].pulse_width_percent(duty)
    px.motor_speed_pins[1].pulse_width_percent(duty)


def stop(px):
    px.motor_speed_pins[0].pulse_width_percent(0)
    px.motor_speed_pins[1].pulse_width_percent(0)


if __name__ == "__main__":
    px = Picarx()
    try:
        print(f"전진 테스트 시작 (kick={KICK_DUTY}% -> duty={DUTY}%)")
        move_forward(px, duty=DUTY)
        time.sleep(2)
        print("전진 테스트 종료")
    finally:
        stop(px)

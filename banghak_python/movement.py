"""
movement.py
역할: 연속 저속 전진
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

DUTY = 50  # 유지할 저속 듀티


def move_forward(px, duty=DUTY):
    px.set_dir_servo_angle(0)
    px.motor_direction_pins[0].low()
    px.motor_direction_pins[1].high()

    duty = max(0, min(100, duty))
    px.motor_speed_pins[0].pulse_width_percent(duty)
    px.motor_speed_pins[1].pulse_width_percent(duty)


def stop(px):
    px.motor_speed_pins[0].pulse_width_percent(0)
    px.motor_speed_pins[1].pulse_width_percent(0)


if __name__ == "__main__":
    px = Picarx()
    try:
        print(f"전진 테스트 시작 (duty={DUTY}%)")
        move_forward(px, duty=DUTY)
        time.sleep(2)
        print("전진 테스트 종료")
    finally:
        stop(px)

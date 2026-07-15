"""
movement.py
역할: 차량의 기본 움직임(전진/후진/정지/주차)만 담당
"""

from picarx import Picarx
import time

DUTY = 50
ANGLE = 35


def move_forward(px, duty=DUTY, angle=0):
    px.set_dir_servo_angle(angle)
    px.motor_direction_pins[0].low()
    px.motor_direction_pins[1].high()
    px.motor_speed_pins[0].pulse_width_percent(duty)
    px.motor_speed_pins[1].pulse_width_percent(duty)


def move_backward(px, duty=DUTY, angle=0):
    px.set_dir_servo_angle(angle)
    px.motor_direction_pins[0].high()
    px.motor_direction_pins[1].low()
    px.motor_speed_pins[0].pulse_width_percent(duty)
    px.motor_speed_pins[1].pulse_width_percent(duty)


def stop(px):
    px.motor_speed_pins[0].pulse_width_percent(0)
    px.motor_speed_pins[1].pulse_width_percent(0)


def park(px):
    px.set_dir_servo_angle(0)
    stop(px)


if __name__ == "__main__":
    px = Picarx()
    try:
        print(f"전진 테스트 시작 (duty={DUTY}%)")
        move_forward(px, DUTY)
        time.sleep(2)

        print(f"회피각 조향 테스트 시작 (angle={ANGLE}도)")
        move_forward(px, DUTY, ANGLE)
        time.sleep(2)

        print("후진 테스트 시작")
        move_backward(px, DUTY)
        time.sleep(2)
    finally:
        park(px)
        print("테스트 종료, 주차 완료")

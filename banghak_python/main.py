"""
main.py
역할: 초음파 센서로 장애물 감지 -> 우측 30도 회전 -> 장애물 없으면 다시 직진
"""

from picarx import Picarx
from movement import move_forward, stop, DUTY
import time

px = Picarx()

OBSTACLE_DISTANCE = 30
TURN_ANGLE = 30


def turn_right(px, angle=TURN_ANGLE, duty=DUTY):
    px.set_dir_servo_angle(angle)
    px.motor_direction_pins[0].low()
    px.motor_direction_pins[1].high()

    duty = max(0, min(100, duty))
    px.motor_speed_pins[0].pulse_width_percent(duty)
    px.motor_speed_pins[1].pulse_width_percent(duty)


try:
    while True:
        distance = px.ultrasonic.read()
        print(f"거리: {distance} cm")

        if 0 < distance < OBSTACLE_DISTANCE:
            turn_right(px, TURN_ANGLE)
        else:
            move_forward(px)

        time.sleep(0.1)

except KeyboardInterrupt:
    stop(px)
    print("종료")

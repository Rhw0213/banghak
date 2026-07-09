"""
main.py
역할: 초음파 센서로 장애물 감지 -> 우측 30도 회전 -> 장애물 없으면 다시 직진
      전진 = movement.py의 move_forward() 사용
      회전 = main.py 자체 구현 (movement.py 책임 밖의 기능)
"""

from picarx import Picarx
from movement import move_forward, stop, DUTY, KICK_DUTY, KICK_TIME
import time

px = Picarx()

OBSTACLE_DISTANCE = 10
TURN_ANGLE = 30

was_stopped = True


def turn_right(px, angle=TURN_ANGLE, duty=DUTY, kick=True):
    """회전 전용 함수 - main.py 소관"""
    px.set_dir_servo_angle(angle)
    px.motor_direction_pins[0].low()
    px.motor_direction_pins[1].high()

    if kick:
        px.motor_speed_pins[0].pulse_width_percent(KICK_DUTY)
        px.motor_speed_pins[1].pulse_width_percent(KICK_DUTY)
        time.sleep(KICK_TIME)

    duty = max(0, min(100, duty))
    px.motor_speed_pins[0].pulse_width_percent(duty)
    px.motor_speed_pins[1].pulse_width_percent(duty)


try:
    while True:
        distance = px.ultrasonic.read()
        print(f"거리: {distance} cm")

        if 0 < distance < OBSTACLE_DISTANCE:
            stop(px)
            was_stopped = True
            time.sleep(0.1)
            turn_right(px, TURN_ANGLE, kick=True)
            was_stopped = False
        else:
            move_forward(px, kick=was_stopped)
            was_stopped = False

        time.sleep(0.1)

except KeyboardInterrupt:
    stop(px)
    print("종료")

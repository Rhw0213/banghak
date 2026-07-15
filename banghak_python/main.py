"""
main.py
"""

from picarx import Picarx
from movement import move_forward, park, DUTY, ANGLE
from avoidance import is_obstacle_ahead, check_escaped

px = Picarx()

try:
    while(True):
        distance = px.ultrasonic.read()

        obstacle = is_obstacle_ahead(distance) 
        escaped = check_escaped(distance)

        if obstacle:
            move_forward(px, angle=ANGLE)       # 장애물 있음 -> 회전
        elif escaped:
            move_forward(px)                    # 탈출 확정됨 -> 직진
        else:
            move_forward(px, angle=ANGLE)       # 아직 대기 중 -> 회전 유지

except KeyboardInterrupt:
    park(px)

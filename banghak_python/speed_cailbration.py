"""
speed_calibration.py
목적: 특정 듀티 사이클(duty)로 몇 초간 구동했을 때 실제로 몇 cm 이동하는지 측정.
      이 값을 알아야 "속도를 낮춰도 목표 거리만큼 가는" 기능을 만들 수 있음.

사용법:
    1) 바닥에 자(줄자)를 펴 놓고, 차 앞바퀴 위치에 시작선을 표시
    2) 아래 DUTY_TO_TEST, RUN_TIME 값을 원하는 대로 바꿔서 실행
    3) 차가 멈추면, 실제로 이동한 거리(cm)를 자로 재서 기록
    4) 여러 duty 값(예: 20, 30, 40, 50)에 대해 반복 측정
    5) 측정한 결과를 distance_move.py의 SPEED_TABLE에 입력
"""
from picarx import Picarx
import time

DUTY_TO_TEST = 50      # 테스트할 듀티 사이클(%)
RUN_TIME = 2.0         # 구동 시간(초)


def move_raw(px, duty, direction='forward'):
    duty = max(0, min(100, duty))
    if direction == 'forward':
        px.motor_direction_pins[0].low()
        px.motor_direction_pins[1].high()
    else:
        px.motor_direction_pins[0].high()
        px.motor_direction_pins[1].low()
    px.motor_speed_pins[0].pulse_width_percent(duty)
    px.motor_speed_pins[1].pulse_width_percent(duty)


px = Picarx()

try:
    print(f"duty={DUTY_TO_TEST}%로 {RUN_TIME}초간 전진합니다. 이동 거리를 자로 재보세요.")
    input("준비되면 Enter를 눌러 시작...")

    move_raw(px, DUTY_TO_TEST, 'forward')
    time.sleep(RUN_TIME)
    px.stop()

    print("측정 완료. 실제 이동 거리(cm)를 자로 잰 뒤 기록해두세요.")
    print(f"속도 = 이동거리(cm) / {RUN_TIME}초 = ??? cm/s")

finally:
    px.stop()

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
speed_calibration.py
================
teleop_slam.py의 SPEED_MM_PER_SEC_AT_MAX 값을 실측하기 위한 전용 스크립트.

teleop_slam.py의 set_speed()는 SPEED_STEP만큼씩 서서히 속도를 올리고
내리는 "슬로우 스타트 / 슬로우 스탑" 구조입니다. 그런데 실측을 위해서는
"최고속도(50)로 정확히 1초 동안만 움직인 거리"가 필요하므로, 여기서는
슬로우 스타트/스탑 없이:

    1. 모터에 곧바로 최고속도(FULL_SPEED=50)를 넣고
    2. 정확히 RUN_SECONDS(기본 1초) 동안 그 상태를 유지한 뒤
    3. 즉시 속도 0으로 정지

시킵니다. 서보(조향)는 실측에 영향 없도록 정면(0도) 고정입니다.

사용법:
    1. 바닥에 출발선을 테이프로 표시
    2. python3 speed_calibration.py 실행 (실행 즉시 차가 출발하니 주의)
    3. 차가 멈추면 이동한 거리(mm)를 줄자로 측정
    4. teleop_slam.py의 SPEED_MM_PER_SEC_AT_MAX에
       "측정한 거리(mm) ÷ RUN_SECONDS" 값을 넣으면 됩니다.
       (RUN_SECONDS가 1초 기본값이면 잰 거리(mm) 값을 그대로 넣으면 됩니다.)

주의:
    - 차가 갑자기 최고속도로 튀어나가므로, 충분히 넓고 장애물 없는 공간에서
      실행하세요.
    - 정지도 슬로우 스탑 없이 즉시 이루어지므로, 관성으로 살짝 더 밀려날 수
      있습니다. 그 밀림까지 포함해서 측정하는 게 오히려 실제 주행 특성에
      더 가까운 값이 됩니다 (완벽한 실험실 값보다 실전에 가까운 값이 목적).
"""

import time

from robot_hat import Motor, Servo, Pin, PWM, reset_mcu

# ============================================================
# 설정값
# ============================================================
FULL_SPEED = 50      # teleop_slam.py의 MAX_SPEED와 동일한 값 (최고속도)
RUN_SECONDS = 1.0    # 최고속도로 유지할 시간 (초)


def main():
    reset_mcu()
    time.sleep(0.5)

    left_motor = Motor(PWM("P13"), Pin("D4"))
    right_motor = Motor(PWM("P12"), Pin("D5"))
    steer_servo = Servo("P2")

    # 조향은 정면 고정 (직진 거리만 측정하기 위함)
    steer_servo.angle(0)
    time.sleep(0.5)

    print(f"[안내] {RUN_SECONDS}초 뒤 즉시 정지합니다. 출발선을 확인하세요.")
    print("[안내] 3초 후 출발합니다...")
    time.sleep(3)

    print("[출발] 최고속도로 즉시 가속 없이 바로 이동합니다.")

    # 슬로우 스타트 없이 곧바로 최고속도 명령 (teleop_slam.py의 set_speed()와
    # 다르게 SPEED_STEP만큼 서서히 올리지 않고 한 번에 FULL_SPEED를 넣음)
    left_motor.speed(-FULL_SPEED)
    right_motor.speed(FULL_SPEED)

    time.sleep(RUN_SECONDS)

    # 슬로우 스탑 없이 곧바로 정지 (0으로 서서히 낮추지 않고 한 번에 0)
    left_motor.speed(0)
    right_motor.speed(0)

    print("[정지] 완료. 실제 이동한 거리(mm)를 줄자로 측정하세요.")
    print(
        f"[계산 안내] 잰 거리(mm) ÷ {RUN_SECONDS}초 = teleop_slam.py의 "
        "SPEED_MM_PER_SEC_AT_MAX 에 넣을 값입니다."
    )


if __name__ == "__main__":
    main()

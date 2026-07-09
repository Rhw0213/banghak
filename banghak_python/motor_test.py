"""
motor_test.py
목적: PiCar-X의 모터(전진/후진, 속도-거리 독립 제어)와 조향 서보가 정상 동작하는지 확인
"""
from picarx import Picarx
import time

# ---- speed_calibration.py로 실측한 값 (duty별 2초간 이동거리 ÷ 2) ----
SPEED_TABLE = {
    20: 7.5,    # 2초간 15cm 이동 -> 7.5 cm/s
    30: 16.0,   # 2초간 32cm 이동 -> 16.0 cm/s
    40: 23.5,   # 2초간 47cm 이동 -> 23.5 cm/s
    50: 30.5,   # 2초간 61cm 이동 -> 30.5 cm/s
}
# ------------------------------------------------------------


def move_raw(px, duty, direction='forward'):
    """duty(%)를 모터 PWM에 그대로 적용 (forward()/backward()의 50~100% 제한 우회)"""
    duty = max(0, min(100, duty))
    if direction == 'forward':
        px.motor_direction_pins[0].low()
        px.motor_direction_pins[1].high()
    else:
        px.motor_direction_pins[0].high()
        px.motor_direction_pins[1].low()
    px.motor_speed_pins[0].pulse_width_percent(duty)
    px.motor_speed_pins[1].pulse_width_percent(duty)


def get_speed_cm_s(duty):
    """SPEED_TABLE에 없는 duty는 가장 가까운 측정값 사이를 선형 보간해서 추정"""
    keys = sorted(SPEED_TABLE.keys())
    if duty <= keys[0]:
        return SPEED_TABLE[keys[0]]
    if duty >= keys[-1]:
        return SPEED_TABLE[keys[-1]]
    for i in range(len(keys) - 1):
        low, high = keys[i], keys[i + 1]
        if low <= duty <= high:
            ratio = (duty - low) / (high - low)
            return SPEED_TABLE[low] + ratio * (SPEED_TABLE[high] - SPEED_TABLE[low])


def move_distance(px, duty, distance_cm, direction='forward'):
    """
    duty        : 속도 (듀티 사이클 %) -> 이 값만 바꿔도 이동 거리는 유지됨
    distance_cm : 목표 이동 거리 (cm) -> 이 값만 바꿔도 속도는 유지됨
    direction   : 'forward' 또는 'backward'
    """
    speed_cm_s = get_speed_cm_s(duty)
    run_time = distance_cm / speed_cm_s
    print(f"duty={duty}% (추정 속도 {speed_cm_s:.1f}cm/s) -> "
          f"{distance_cm}cm 이동을 위해 {run_time:.2f}초 구동")

    move_raw(px, duty, direction)
    time.sleep(run_time)
    px.stop()


# Picarx 객체 생성 - 이 객체를 통해 모터, 서보, 센서를 모두 제어함
px = Picarx()

try:
    # ---- 전진 테스트 (속도 20%, 목표 거리 30cm) ----
    move_distance(px, duty=35, distance_cm=10, direction='forward')
    time.sleep(0.5)

    # ---- 후진 테스트 (속도 40%로 바꿔도 목표 거리는 동일하게 30cm) ----
    move_distance(px, duty=35, distance_cm=10, direction='backward')
    time.sleep(0.5)

    # ---- 조향 테스트 ----
    px.set_dir_servo_angle(20)     # 앞바퀴를 오른쪽으로 20도 꺾음
    time.sleep(0.5)
    px.set_dir_servo_angle(-20)    # 앞바퀴를 왼쪽으로 20도 꺾음
    time.sleep(0.5)
    px.set_dir_servo_angle(0)      # 다시 정중앙(캘리브레이션된 0도)으로 복귀

finally:
    # 에러가 나든 정상 종료하든 항상 모터를 멈춰서 폭주 방지
    px.stop()

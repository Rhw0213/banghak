"""
distance_move.py
목적: 속도(duty)를 낮춰도 목표 거리(distance_cm)만큼 정확히 이동하도록
      "속도"와 "거리"를 서로 독립적인 파라미터로 다루는 함수.

전제조건:
    엔코더가 없으므로 실제 거리는 측정할 수 없고, 대신
    "이 duty에서는 초당 몇 cm 이동한다"는 사전 측정값(SPEED_TABLE)을 기반으로
    필요한 구동 시간을 역산하는 방식(open-loop, 추정치)입니다.
    -> speed_calibration.py로 먼저 측정해서 아래 표를 채워주세요.
"""
from picarx import Picarx
import time

# ---- 여기를 speed_calibration.py로 측정한 실제 값으로 채워주세요 ----
# key: 듀티 사이클(%), value: 그 듀티에서 실제 속도(cm/s)
# 값이 많을수록(예: 15, 20, 25, 30 ... 5단위) 아래 보간(interpolation)이 정확해집니다.
SPEED_TABLE = {
    20: 15/2,   # 예시 값입니다. 반드시 실제 측정값으로 교체하세요.
    30: 32/2,
    40: 47/2,
    50: 61/2,
}
# --------------------------------------------------------------


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


def get_speed_cm_s(duty):
    """
    SPEED_TABLE에 없는 duty 값이 들어오면, 가장 가까운 두 측정값 사이를
    직선 보간(linear interpolation)해서 속도를 추정합니다.
    """
    keys = sorted(SPEED_TABLE.keys())

    if duty <= keys[0]:
        return SPEED_TABLE[keys[0]]
    if duty >= keys[-1]:
        return SPEED_TABLE[keys[-1]]

    for i in range(len(keys) - 1):
        low, high = keys[i], keys[i + 1]
        if low <= duty <= high:
            low_speed, high_speed = SPEED_TABLE[low], SPEED_TABLE[high]
            ratio = (duty - low) / (high - low)
            return low_speed + ratio * (high_speed - low_speed)


def move_distance(px, duty, distance_cm, direction='forward'):
    """
    duty        : 속도 (듀티 사이클 %) - 이것만 바꿔도 아래 거리는 그대로 유지됨
    distance_cm : 목표 이동 거리 (cm) - 이것만 바꿔도 위 속도는 그대로 유지됨
    direction   : 'forward' 또는 'backward'
    """
    speed_cm_s = get_speed_cm_s(duty)
    run_time = distance_cm / speed_cm_s

    print(f"duty={duty}% (추정 속도 {speed_cm_s:.1f}cm/s)로 "
          f"{distance_cm}cm 이동 -> 구동 시간 {run_time:.2f}초")

    move_raw(px, duty, direction)
    time.sleep(run_time)
    px.stop()


if __name__ == "__main__":
    px = Picarx()
    try:
        # 예시: 속도만 다르게, 거리는 동일하게(30cm) 이동
        move_distance(px, duty=20, distance_cm=30, direction='forward')
        time.sleep(1)
        move_distance(px, duty=40, distance_cm=30, direction='forward')
    finally:
        px.stop()

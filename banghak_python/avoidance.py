"""
avoidance.py
역할: 초음파 센서로 전방에 장애물 감지 + 비스듬한 벽 탈출 판단
"""
import time

OBSTACLE_DISTANCE = 30        # 이 거리(cm)보다 가까우면 장애물로 판단
ESCAPE_TRIGGER_DISTANCE = 80  # -2 -> 80 값이 튀면 탈출신호
ESCAPE_DELAY = 2.0            # 탈출신호후 이만큼(초) 기다렸다 직진 전환


def is_obstacle_ahead(distance, limit=OBSTACLE_DISTANCE):
    return distance < limit


# 탈출 판단을 위한 상태 (여러 번 호출 사이에 기억해야 하므로 전역 변수)
prev_distance = None
escape_signal_time = None  # 탈출 신호를 처음 감지한 시각 (아직 없으면 None)


def check_escaped(distance):
    """
    비스듬한 벽을 완전히 벗어났는지 판단.
    - 직전 값이 -2였다가 지금 값이 ESCAPE_TRIGGER_DISTANCE 이상으로 튀면 "탈출 신호"로 기록
    - 탈출 신호가 기록된 시점부터 ESCAPE_DELAY(2초)가 지나야 최종적으로 True 반환
    """
    global prev_distance, escape_signal_time

    if prev_distance == -2 and distance >= ESCAPE_TRIGGER_DISTANCE:
        if escape_signal_time is None:
            escape_signal_time = time.time()

    prev_distance = distance

    if escape_signal_time is None:
        return False

    elapsed = time.time() - escape_signal_time
    if elapsed >= ESCAPE_DELAY:
        escape_signal_time = None
        return True

    return False

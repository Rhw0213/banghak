# smooth_servo.py
# 역할: 서보 하나(관절 하나)를 부드럽게, 정해진 속도/각도 범위 안에서 움직이는 로직만 담당
# robot_hat을 직접 import하지 않음 -> 어떤 환경(윈도우 포함)에서도 이 파일은 그대로 동작함
import time


class SmoothJoint:
    """현재 각도를 기억하면서 속도 제어가 가능한 서보 관절"""

    def __init__(self, servo_driver, init_angle, min_angle, max_angle, move_on_init=True):
        """
        servo_driver: angle(값) 메서드를 가진 서보 객체 (실제 Servo든 MockServo든 바깥에서 주입받음)
        init_angle: 시작 각도
        min_angle, max_angle: 안전 이동 범위 (필수 입력값, 관절마다 반드시 지정)
        """
        self.servo = servo_driver
        self.min_angle = min_angle
        self.max_angle = max_angle
        self.angle = init_angle
        if move_on_init:
            self.servo.angle(init_angle)

    def move_to(self, target, speed=30.0, step=1.0, max_speed=40.0):
        """
        target: 목표 각도
        speed: 초당 몇 도로 움직일지 (도/초)
        step: 한 번에 움직이는 각도
        max_speed: speed가 아무리 커도 이 값을 못 넘게 막는 상한선
        """
        speed = min(speed, max_speed)                              # 속도 상한 제한
        target = max(self.min_angle, min(self.max_angle, target))  # 각도 범위 제한
        delay = step / speed

        direction = 1.0 if target > self.angle else -1.0

        while abs(target - self.angle) > step:
            self.angle += step * direction
            self.servo.angle(self.angle)
            time.sleep(delay)

        self.angle = target
        self.servo.angle(self.angle)
        return self.angle


def move_all(joints, targets, max_speed=15.0, step_deg=1.0):
    """
    여러 관절을 동시에 움직임 (같이 출발해서 같이 도착)
    max_speed: 가장 많이 움직이는 관절 기준, 초당 최대 이동 각도
    step_deg: 한 조각당 이동 각도 (거리에 비례해 자동으로 조각 수 계산)
    """
    starts = [j.angle for j in joints]
    max_distance = max(abs(e - s) for s, e in zip(starts, targets))

    if max_distance == 0:
        return

    duration = max_distance / max_speed
    steps = max(1, int(max_distance / step_deg))
    delay = duration / steps

    for i in range(steps + 1):
        t = i / steps
        for j, s, e in zip(joints, starts, targets):
            a = s + (e - s) * t
            a = max(j.min_angle, min(j.max_angle, a))
            j.angle = a
            j.servo.angle(a)
        time.sleep(delay)
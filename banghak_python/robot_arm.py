"""
arm.py
역할: 로봇팔 4개 서보를 입력받은 각도로 즉시 움직이는 수동 제어 도구.
      arm_setup.build_arm()을 재사용해서 관절별 안전 범위(min/max)는
      그대로 지키되, 슬로우스타터(단계별 이동) 없이 명령받은 즉시 이동한다.
"""

import time
from robot_hat import Servo, device
from arm_setup import build_arm

device.reset_mcu()
time.sleep(0.5)

base, shoulder, elbow, gripper = build_arm(servo_factory=Servo)


def _set_instant(joint, target):
    """
    [슬로우스타터 제외] SmoothJoint.move_to()의 단계별 이동(while 루프)을
    거치지 않고, 범위 클램프만 재사용해서 서보에 즉시 명령을 보낸다.
    joint.angle(소프트웨어가 기억하는 현재 각도)도 같이 갱신해서
    이후 다른 호출과 상태가 어긋나지 않게 한다.
    """
    target = max(joint.min_angle, min(joint.max_angle, target))
    joint.angle = target
    joint.servo.angle(target)
    return target


def set_arm_angles(base_angle=0, shoulder_angle=0, elbow_angle=0, gripper_angle=0):
    """
    4관절을 각각 지정한 각도로 즉시 이동시킨다 (슬로우스타터 없음).
    각 관절의 min/max 범위(arm_setup.py에 정의됨)를 벗어나는 값은
    자동으로 그 범위 안으로 잘린다.
    """
    _set_instant(base, base_angle)
    _set_instant(shoulder, shoulder_angle)
    _set_instant(elbow, elbow_angle)
    _set_instant(gripper, gripper_angle)
    print(f"[로봇팔] base={base.angle:.1f} shoulder={shoulder.angle:.1f} "
          f"elbow={elbow.angle:.1f} gripper={gripper.angle:.1f} (실제 적용된 각도)")


def _parse_line(line):
    """'base shoulder elbow gripper' 형태의 한 줄을 4개 float로 파싱. 실패 시 None."""
    parts = line.strip().split()
    if len(parts) != 4:
        return None
    try:
        return [float(p) for p in parts]
    except ValueError:
        return None


if __name__ == "__main__":
    print("4관절 각도 입력 도구 (즉시 이동, 슬로우스타터 없음)")
    print("형식: base shoulder elbow gripper  (예: -10 20 5 0)")
    print("종료: q 또는 Ctrl+C")
    print(f"허용 범위 - base:[{base.min_angle},{base.max_angle}] "
          f"shoulder:[{shoulder.min_angle},{shoulder.max_angle}] "
          f"elbow:[{elbow.min_angle},{elbow.max_angle}] "
          f"gripper:[{gripper.min_angle},{gripper.max_angle}]")

    try:
        while True:
            raw = input("각도 입력> ")
            if raw.strip().lower() == 'q':
                break

            values = _parse_line(raw)
            if values is None:
                print("★ 형식 오류 - 숫자 4개를 공백으로 구분해서 입력하세요 (예: -10 20 5 0)")
                continue

            set_arm_angles(*values)
    except KeyboardInterrupt:
        print("\n종료 중...")
    finally:
        # 종료 시 4관절 모두 즉시 0도 복귀
        print("[로봇팔] 0도로 복귀 중...")
        for j in (base, shoulder, elbow, gripper):
            _set_instant(j, 0)
        print("[로봇팔] 종료 완료")
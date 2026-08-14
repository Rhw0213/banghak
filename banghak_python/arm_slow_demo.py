"""
arm_slow_demo.py
역할: 슬로우스타터로 로봇팔을 움직이는 데모.
  1) 프로그램 시작  -> 4관절 모두 0도에서 10도까지 슬로우스타터로 이동
  2) 사용자 종료(q 또는 Ctrl+C) -> 4관절 모두 10도에서 0도까지 슬로우스타터로 복귀

arm_setup.build_arm()이 모든 관절을 init_angle=0으로 만들어주기 때문에:
  - 시작 시 move_to(10) -> self.angle(0)과 target(10)이 달라서 실제로 스텝 이동 발생
  - 종료 시 move_to(0)  -> self.angle(10)과 target(0)이 달라서 이번에도 스텝 이동 발생
지난번 겪었던 "이미 목표각과 같아서 순간이동처럼 동작" 문제가 여기선 생기지 않는다.
"""

import time
from robot_hat import Servo, device
from arm_setup import build_arm

ARM_MOVE_SPEED = 30.0   # 도/초
ARM_TEST_ANGLE = 10     # 시작 시 이동할 목표 각도

device.reset_mcu()
time.sleep(0.5)

base, shoulder, elbow, gripper = build_arm(servo_factory=Servo)
joints = (base, shoulder, elbow, gripper)


def main():
    print(f"[로봇팔] 시작 - 4관절 모두 {ARM_TEST_ANGLE}도까지 슬로우스타터로 이동 중...")
    for j in joints:
        j.move_to(ARM_TEST_ANGLE, speed=ARM_MOVE_SPEED)
    print(f"[로봇팔] 이동 완료. 현재 각도: "
          f"base={base.angle:.1f} shoulder={shoulder.angle:.1f} "
          f"elbow={elbow.angle:.1f} gripper={gripper.angle:.1f}")
    print("종료하려면 Ctrl+C를 누르세요.")

    try:
        while True:
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("\n[로봇팔] 종료 요청 감지 - 0도로 슬로우스타터 복귀 중...")
    finally:
        for j in joints:
            j.move_to(0, speed=ARM_MOVE_SPEED)
        print("[로봇팔] 복귀 완료")


if __name__ == "__main__":
    main()
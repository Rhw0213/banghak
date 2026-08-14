# base_only_test.py
# 카메라/비주얼서보 로직 다 빼고, arm_setup.build_arm()으로 만든 base 서보
# 하나만 실제로 물리적으로 움직이는지 확인하는 최소 테스트.
#
# 사용법:
#   python3 base_only_test.py

import time
from robot_hat import Servo, reset_mcu
from arm_setup import build_arm


def main():
    reset_mcu()
    time.sleep(0.3)

    base, shoulder, elbow, gripper = build_arm(Servo)

    print(f"시작 각도: base={base.angle}")
    input("이 상태를 눈으로 보고 Enter를 누르세요 (base 서보 위치 확인)...")

    print("base를 +30도로 이동 명령...")
    base.move_to(base.angle + 30, speed=15, step=1)
    time.sleep(0.5)

    print(f"이동 후 소프트웨어 각도: base={base.angle}")
    input("팔이 실제로 물리적으로 돌아갔는지 눈으로 확인하고 Enter...")

    print("원위치로 복귀...")
    base.move_to(base.angle - 30, speed=15, step=1)

    print(f"최종 각도: base={base.angle}")
    print("실제로 팔이 두 번(가고, 돌아오고) 움직이는 걸 봤나요?")


if __name__ == "__main__":
    main()

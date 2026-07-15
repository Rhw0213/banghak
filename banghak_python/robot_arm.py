"""
arm.py
역할: 로봇팔 4개 서보 제어 (movement.py와 같은 원칙 - "어떻게 움직이는가"만 담당)
"""

from robot_hat import Servo, reset_mcu
import time

reset_mcu()
time.sleep(0.2)
# 채널은 실제 배선한 P번호에 맞춰 수정
base = Servo("P3")
#shoulder = Servo("P5")
#elbow = Servo("P6")
#gripper = Servo("P7")


def set_arm_angles(base_angle=0, shoulder_angle=0, elbow_angle=0, gripper_angle=0):
    base.angle(base_angle)
    # shoulder.angle(shoulder_angle)
    # elbow.angle(elbow_angle)
    # gripper.angle(gripper_angle)


if __name__ == "__main__":
    print("run")
    set_arm_angles(20, 0, 0, 0)   # 모든 관절 중앙(0도)으로
    time.sleep(1)

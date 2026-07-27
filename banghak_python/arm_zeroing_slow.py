import time
from robot_hat import Servo, reset_mcu
from smooth_servo import SmoothJoint
from arm_setup import build_arm

reset_mcu()          # ★ 주석 풀기
time.sleep(0.5)

base, shoulder, elbow, gripper = build_arm(servo_factory=Servo)

# 실제 팔의 현재 물리적 위치에 맞게 (처음 조립상태 0도라고 가정)
base.angle = 45 
shoulder.angle = 0
elbow.angle = 0
gripper.angle = 0

try:
    base.move_to(90, speed=10)   # 0→20 부드럽게
    time.sleep(1)
    #base.move_to(20, speed=10)    # 20→0 부드럽게
    #time.sleep(1)
    #base.move_to(10, speed=10)    # 20→0 부드럽게
    #time.sleep(1)
    #base.move_to(0, speed=10)    # 20→0 부드럽게
    #time.sleep(1)
except KeyboardInterrupt:
    pass

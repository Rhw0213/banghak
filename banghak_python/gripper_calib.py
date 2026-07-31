"""
gripper_calib.py
역할: 그리퍼 서보의 실제 "완전히 닫힘" 각도를 찾기 위한 테스트 스크립트

사용법:
    python3 gripper_calib.py

각도를 입력하면 그 즉시 그리퍼가 그 각도로 움직입니다.
물건을 손에 쥐고 여러 각도를 입력해보면서, 물건을 실제로 꽉 잡는 각도를
찾아서 robot_pick.py의 GRIPPER_CLOSE 값으로 넣어주세요.

종료: 'q' + Enter
"""

from robot_hat import Servo, device
import time

device.reset_mcu()
time.sleep(0.2)

gripper = Servo("P7")  # 실제 배선한 채널로 수정

print("그리퍼 캘리브레이션을 시작합니다.")
print("각도(-90~90 사이 숫자)를 입력하면 그리퍼가 그 각도로 움직입니다.")
print("물건을 손에 쥔 채로 여러 각도를 시도해서, 물건을 놓치지 않고")
print("꽉 잡는 각도를 찾아주세요. 종료하려면 'q' + Enter.")
print()
print("참고용 시작값: 0(열림 추정) / 60(닫힘 추정)")

while True:
    cmd = input("각도 입력 (또는 q): ").strip()
    if cmd.lower() == "q":
        print("종료합니다.")
        break

    try:
        angle = float(cmd)
    except ValueError:
        print("숫자를 입력해주세요.")
        continue

    gripper.angle(angle)
    print(f"-> 그리퍼를 {angle}도로 이동시켰습니다. 물건을 쥐고 있는지 확인해보세요.")

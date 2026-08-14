# servo_home_test.py
# 서보 4개(base/shoulder/elbow/grab) 각각의 하드스톱 각도를 직접 찾기 위한 테스트 스크립트.
#
# 사용법:
#   python3 servo_home_test.py
#
# 동작:
#   - 서보 하나를 골라서, 각도를 조금씩(기본 5도씩) 바꿔가며 이동시킨다.
#   - 매 이동마다 Enter로 다음 스텝 진행, 'q'로 해당 서보 테스트 종료.
#   - "여기서 더 이상 안 움직이는 것 같다" 싶은 지점의 각도를 메모해두면
#     그게 그 축의 HOME_STALL_ANGLE 후보다.
#   - 절대 한 번에 큰 각도로 점프하지 않음 (기구 보호) - 항상 작은 스텝만 이동.

import time

try:
    from robot_hat import Servo, reset_mcu
except ImportError:
    print("robot_hat 모듈을 찾을 수 없습니다. 라즈베리파이에서 실행하세요.")
    raise

STEP_DEG = 5           # 한 번에 움직이는 각도 - 작게 유지 (기구 보호)
START_ANGLE = 90        # 테스트 시작 각도 (일단 중앙 근처에서 시작 권장)

PIN_MAP = {
    "base": "P4",
    "shoulder": "P5",
    "elbow": "P6",
    "grab": "P7",
}


def test_one_servo(name, pin):
    print(f"\n===== {name} 서보 테스트 (핀 {pin}) =====")
    servo = Servo(pin)
    angle = START_ANGLE
    servo.angle(angle)
    print(f"시작 각도: {angle}도 (여기서부터 조금씩 움직입니다)")
    time.sleep(0.5)

    while True:
        cmd = input(
            f"[{name}] 현재 명령각={angle}도 | "
            f"'+'=+{STEP_DEG}도, '-'=-{STEP_DEG}도, "
            f"숫자 직접입력 가능, 'q'=종료: "
        ).strip()

        if cmd == 'q':
            break
        elif cmd == '+':
            angle = min(180, angle + STEP_DEG)
        elif cmd == '-':
            angle = max(0, angle - STEP_DEG)
        else:
            try:
                angle = max(0, min(180, int(cmd)))
            except ValueError:
                print("이해 못한 입력입니다. +, -, 숫자, q 중 하나를 입력하세요.")
                continue

        servo.angle(angle)
        time.sleep(0.3)
        print(f"  -> {angle}도로 이동 명령 (실제로 움직였는지, "
              f"소리/부하가 이상하지 않은지 확인하세요)")

    print(f"[{name}] 테스트 종료. 마지막 각도: {angle}도")
    print(f"  -> 이 지점이 하드스톱이라고 판단되면 이 값을 "
          f"{name.upper()}_HOME_STALL_ANGLE 로 기록하세요.")


def main():
    reset_mcu()
    time.sleep(0.3)

    print("서보 하드스톱 확인용 테스트 스크립트")
    print("★ 주의: 처음엔 STEP_DEG(5도)씩만 움직이면서 눈으로 확인하세요.")
    print("★ 이상한 소리나 진동이 심하면 즉시 'q'로 종료하고 반대 방향으로 확인하세요.\n")

    order = ["base", "shoulder", "elbow", "grab"]
    for name in order:
        ans = input(f"'{name}' 서보(핀 {PIN_MAP[name]})를 테스트할까요? (y/n/skip all): ").strip().lower()
        if ans == 'skip all':
            break
        if ans != 'y':
            continue
        test_one_servo(name, PIN_MAP[name])

    print("\n모든 테스트 종료.")


if __name__ == "__main__":
    main()

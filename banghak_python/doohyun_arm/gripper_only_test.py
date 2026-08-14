# gripper_only_test.py
# 카메라/비주얼서보 로직 다 빼고, 그리퍼 서보 하나만 -20 <-> 10 두 극단 각도로
# 바로 왕복시켜서 실제로 움직이는지(닫히는지/열리는지) 확인하는 테스트.
# + 스냅 직전/직후 배터리 전압을 찍어서 순간 전압 강하가 있는지도 같이 확인.
#
# 사용법:
#   python3 gripper_only_test.py

import time
from robot_hat import Servo, reset_mcu
from arm_setup import build_arm

try:
    from battery_driving_check import BatteryMonitor
    _battery_available = True
except ImportError:
    _battery_available = False
    print("[경고] battery_driving_check.py를 못 찾음 - 전압 체크 없이 진행합니다.")
    print("       (필요하면 doohyun_arm 폴더로 복사해오면 전압 체크가 활성화됩니다)")


def _read_voltage(monitor):
    """가능하면 지금 배터리 전압을 읽어서 반환, 안 되면 None."""
    if not _battery_available or monitor is None:
        return None
    try:
        return monitor.read()
    except Exception as e:
        print(f"[전압] 읽기 실패: {e}")
        return None


def _snap_and_check(gripper, target_angle, monitor, label):
    """
    그리퍼를 target_angle로 즉시 스냅시키면서, 스냅 직전/직후 전압을 비교해
    순간 강하가 있었는지 출력한다.
    """
    v_before = _read_voltage(monitor)

    gripper.servo.angle(target_angle)
    gripper.angle = target_angle

    v_after = _read_voltage(monitor)

    print(f"\n[{label}] {target_angle}도로 스냅")
    if v_before is not None and v_after is not None:
        drop = v_before - v_after
        print(f"  전압: 직전={v_before:.2f}V -> 직후={v_after:.2f}V (강하={drop:+.2f}V)")
        if drop > 0.3:   # ★ 임계값은 실측하며 조정 - 우선 0.3V로 시작
            print("  -> 눈에 띄는 전압 강하 감지됨 (전기적 원인 가능성 높음)")
        else:
            print("  -> 전압은 크게 안 흔들림 (전기적 원인 가능성 낮음)")
    else:
        print("  전압 측정 불가 (battery_driving_check.py 확인 필요)")


def main():
    reset_mcu()
    time.sleep(0.3)

    base, shoulder, elbow, gripper = build_arm(Servo)
    monitor = BatteryMonitor() if _battery_available else None

    print(f"시작 각도: gripper={gripper.angle} (범위: {gripper.min_angle}~{gripper.max_angle})")
    print("다른 관절(base/shoulder/elbow)이 실제로 흔들리는지는 옆에서 직접 지켜봐주세요")
    print("(전압 강하가 아니라 순수 기구적 진동이면 소프트웨어로는 감지가 안 됩니다).")
    input("이 상태(그리퍼 모양)를 눈으로 보고 Enter...")

    _snap_and_check(gripper, -20, monitor, "1차")
    time.sleep(0.5)
    input("모양/흔들림 확인하고 Enter...")

    _snap_and_check(gripper, 10, monitor, "2차")
    time.sleep(0.5)
    input("모양/흔들림 확인하고 Enter...")

    _snap_and_check(gripper, -20, monitor, "3차")
    time.sleep(0.5)
    input("한 번 더 확인하고 Enter...")

    print(f"\n최종 각도: gripper={gripper.angle}")
    print("전압 강하가 매번 크게 나왔다면 전기적 원인(전원 공유) 쪽으로 의심하면 됩니다.")
    print("전압은 안정적인데도 다른 관절이 흔들려 보였다면, 순수 기구적 진동일 가능성이 높습니다.")

    # ===== 자유 입력 모드 =====
    # -20이 arm_setup.py에 정의된 안전범위 최솟값인데, _snap_gripper는 move_to()를
    # 거치지 않아서 그 clamp도 우회한다. 그래서 -20보다 더 내려간 값도 명령은 가능하지만,
    # 팀원이 정한 안전범위 밖이므로 한 번에 크게 넘기지 말고 5도씩 정도만 조금씩 시도할 것.
    print("\n===== 자유 입력 테스트 =====")
    print("원하는 각도를 직접 입력해서 테스트할 수 있습니다 (-20보다 더 내려가는 값도 가능,")
    print("단 조금씩만 - 예: -25, -30 순으로. 기구가 걸리는 느낌 있으면 즉시 q로 중단)")
    while True:
        cmd = input(f"\n현재 gripper={gripper.angle} | 다음 각도 입력 (q=종료): ").strip()
        if cmd.lower() == 'q':
            break
        try:
            target = float(cmd)
        except ValueError:
            print("숫자를 입력하세요.")
            continue
        _snap_and_check(gripper, target, monitor, "자유입력")
        time.sleep(0.5)
        print("모양/걸림 여부 확인하세요.")


if __name__ == "__main__":
    main()
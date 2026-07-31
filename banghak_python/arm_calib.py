# arm_calib.py
# 로봇팔(베이스/어깨/팔꿈치/그립) 각도를 하나씩 테스트해서
# 실제로 목표물을 잘 집는 각도를 찾는 스크립트.
# 차량 주행 로직과 무관하게 팔만 단독으로 움직여봅니다.
#
# 사용법:
#   python3 arm_calib.py
#   숫자 입력하면 해당 서보가 그 각도로 이동합니다.

from robot_hat import Servo
import time

# ===== 핀 배정 =====
base = Servo("P4")      # 베이스(좌우 회전)
shoulder = Servo("P5")  # 어깨
elbow = Servo("P6")     # 팔꿈치
grab = Servo("P7")      # 그립(집게)

SERVOS = {
    "1": ("베이스", base),
    "2": ("어깨", shoulder),
    "3": ("팔꿈치", elbow),
    "4": ("그립", grab),
}


def home_all():
    """모든 서보를 0도(중립)로"""
    for _, (name, s) in SERVOS.items():
        s.angle(0)
    print("전체 0도(중립) 이동 완료")


def main():
    print("===== 로봇팔 캘리브레이션 =====")
    print("먼저 전체 0도로 정렬합니다...")
    home_all()
    time.sleep(1)

    print("""
사용법:
  1 <각도>  -> 베이스 각도 설정   (예: 1 30)
  2 <각도>  -> 어깨 각도 설정     (예: 2 -40)
  3 <각도>  -> 팔꿈치 각도 설정   (예: 3 20)
  4 <각도>  -> 그립 각도 설정     (예: 4 60  # 닫힘 방향)
  home       -> 전체 0도로 복귀
  q          -> 종료

목표: 목표물(노란/파란 오브젝트)을 실제로 자연스럽게 집는
      "어깨/팔꿈치" 각도와, 집게가 완전히 닫히는 "그립" 각도를 찾으세요.
      찾은 값을 lidar_ultra_vision.py의 ARM_PICK_SHOULDER,
      ARM_PICK_ELBOW, ARM_GRAB_CLOSE 에 반영하시면 됩니다.
""")

    try:
        while True:
            cmd = input(">> ").strip().lower()
            if cmd == "q":
                break
            if cmd == "home":
                home_all()
                continue

            parts = cmd.split()
            if len(parts) != 2 or parts[0] not in SERVOS:
                print("입력 형식이 올바르지 않습니다. 예: 2 -40")
                continue

            try:
                angle = float(parts[1])
            except ValueError:
                print("각도는 숫자로 입력하세요.")
                continue

            name, servo = SERVOS[parts[0]]
            servo.angle(angle)
            print(f"[{name}] {angle}도로 이동")

    except KeyboardInterrupt:
        pass
    finally:
        print("\n종료 전 안전 위치(0도)로 복귀합니다...")
        home_all()
        print("종료 완료")


if __name__ == "__main__":
    main()
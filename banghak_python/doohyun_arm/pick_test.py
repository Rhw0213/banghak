# pick_test.py
# arm_setup.build_arm()으로 관절을 만들고, visual_servo_pick() 전체 시퀀스를
# 처음부터 끝까지 한 번 실행하는 테스트 스크립트.
#
# 사용법:
#   python3 pick_test.py

import time
from robot_hat import Servo, reset_mcu
from picamera2 import Picamera2

from arm_setup import build_arm
from arm_visual_servo import (
    visual_servo_pick, CAM_WIDTH, CAM_HEIGHT, CAM_FORMAT, VS_CAM_TILT_ANGLE,
    start_stream, stop_stream
)


def main():
    reset_mcu()
    time.sleep(0.3)

    # ===== 카메라 틸트 세팅 (picarx 짐벌) - build_arm보다 먼저 초기화 =====
    # Picarx()가 내부적으로 reset_mcu/PWM 초기화를 다시 수행할 수 있어서,
    # build_arm()으로 만든 서보보다 나중에 생성하면 서보 상태가 꼬여
    # 명령은 가는데 실제로 안 움직이는 문제가 생길 수 있다. 순서 반드시 유지.
    try:
        from picarx import Picarx
        x = Picarx()
        x.set_cam_tilt_angle(VS_CAM_TILT_ANGLE)
        x.set_cam_pan_angle(0)
        print(f"[짐벌] 틸트 {VS_CAM_TILT_ANGLE}도로 설정")
    except Exception as e:
        print(f"[짐벌] 설정 실패({e}) - 현재 각도 그대로 진행")

    # arm_setup.py에 정의된 min/max 각도 범위, 오프셋을 그대로 사용
    base, shoulder, elbow, gripper = build_arm(Servo)

    picam2 = Picamera2()
    config = picam2.create_preview_configuration(
        main={"size": (CAM_WIDTH, CAM_HEIGHT), "format": CAM_FORMAT})
    picam2.configure(config)
    picam2.start()
    time.sleep(1.0)

    def grab_frame_fn():
        arr = picam2.capture_array()
        if arr is None:
            return None
        return arr[:, :, :3] if arr.shape[2] == 4 else arr

    print("=" * 50)
    print("전체 픽업 시퀀스 테스트")
    print("팔 주변, 특히 이동 경로 위에 손/장애물 없는지 확인하세요.")
    print("이상하면 언제든 Ctrl+C로 즉시 중단할 수 있습니다.")
    print("=" * 50)
    start_stream()
    input("준비되면 Enter를 누르세요...")

    try:
        success, reason = visual_servo_pick(base, shoulder, elbow, gripper, grab_frame_fn)

        print("\n" + "=" * 50)
        print(f"결과: {'성공' if success else '실패'}")
        print(f"사유: {reason}")
        print(f"최종 각도: base={base.angle:.1f} shoulder={shoulder.angle:.1f} elbow={elbow.angle:.1f}")
        print("=" * 50)

    except KeyboardInterrupt:
        print("\n사용자에 의해 중단됨 - 서보를 그 자리에 정지시킵니다.")

    finally:
        stop_stream()
        picam2.stop()
        picam2.close()


if __name__ == "__main__":
    main()
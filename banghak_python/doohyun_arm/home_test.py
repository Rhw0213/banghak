# home_test.py
# arm_setup.build_arm()으로 관절을 만들고, home_arm()만 단독 실행해서
# 기준 자세(0,0,0) 복귀가 잘 되는지 확인하는 스크립트.
#
# 사용법:
#   python3 home_test.py

import time
from robot_hat import Servo, reset_mcu
from picamera2 import Picamera2

from arm_setup import build_arm
from arm_visual_servo import (
    home_arm, CAM_WIDTH, CAM_HEIGHT, CAM_FORMAT, VS_CAM_TILT_ANGLE,
    start_stream, stop_stream
)


def main():
    reset_mcu()
    time.sleep(0.3)

    # ===== 카메라 틸트 세팅 - build_arm보다 먼저 초기화 (순서 중요, 이유는 pick_test.py 주석 참고) =====
    try:
        from picarx import Picarx
        x = Picarx()
        x.set_cam_tilt_angle(VS_CAM_TILT_ANGLE)
        x.set_cam_pan_angle(0)
        print(f"[짐벌] 틸트 {VS_CAM_TILT_ANGLE}도로 설정")
    except Exception as e:
        print(f"[짐벌] 설정 실패({e}) - 현재 각도 그대로 진행")

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

    print("호밍 실행 전 - 팔 주변에 손/물건 없는지 확인하세요.")
    start_stream()
    input("준비되면 Enter를 누르세요...")

    home_arm(base, shoulder, elbow, gripper, grab_frame_fn=grab_frame_fn)

    print(f"\n결과: base={base.angle:.1f} shoulder={shoulder.angle:.1f} elbow={elbow.angle:.1f}")
    print("사진 속 기준 자세(그립 열림/base 살짝왼쪽/shoulder+elbow 반쯤 편 자세)로")
    print("돌아왔는지 눈으로 확인하세요.")

    picam2.stop()
    picam2.close()
    stop_stream()


if __name__ == "__main__":
    main()
# arm_setup.py
# 역할: 로봇팔 하드웨어 스펙(안전 각도 범위)을 한 곳에서 정의
# servo_factory를 인자로 받아서, 실제 서보를 쓸지 가짜 서보를 쓸지는 호출하는 쪽에서 결정
from smooth_servo import SmoothJoint


def build_arm(servo_factory):
    """
    servo_factory: 포트 문자열을 받아서 서보 객체를 만들어주는 함수 또는 클래스
                   예: robot_hat.Servo (실제 하드웨어) 또는 MockServo (디버그용)
    """
    base = SmoothJoint(
        servo_factory("P4"), init_angle=0,
        min_angle=-90, max_angle=90,      # 베이스: 좌우 풀회전 허용
        move_on_init=False
    )
    shoulder = SmoothJoint(
        servo_factory("P5"), init_angle=0,
        min_angle=-60, max_angle=0,       # 어깨: 토크 부족 구간 고려해 좁게 제한
        move_on_init=False
    )
    elbow = SmoothJoint(
        servo_factory("P6"), init_angle=0,
        min_angle=-45, max_angle=45,      # 팔꿈치
        move_on_init=False
    )
    gripper = SmoothJoint(
        servo_factory("P7"), init_angle=0,
        min_angle=-25, max_angle=0,       # 그리퍼: 완전히 닫히는 각도까지만
        move_on_init=False
    )

    return base, shoulder, elbow, gripper

"""
avoidance_go_back.py
역할: 초음파 센서로 전방 벽을 감지해서 전진/후진을 제어하는 '부품' 모듈

다른 메인 프로그램에서 이렇게 불러다 씁니다:
    from wall_backup import WallBackup, Get_Stop_Distance
    from picarx import Picarx
    import time

    px = Picarx()
    wb = WallBackup(px, debug=True)   # debug=True면 거리/상태 로그 출력
    while True:
        wb.update()
        time.sleep(0.05)

동작:
    - 평소: 전진
    - 벽과 거리 <= Get_Stop_Distance()(10cm) -> 후진으로 전환, 계속 뒤로 감
"""


def Get_Stop_Distance():
    """초음파 센서 전방 장애물 감지 범위
       후진으로 전환할 기준 거리(cm)를 반환. 이 숫자만 바꾸면 기준이 바뀜."""
    return 10


def Get_Drive_Duty():
    """주행 속도(듀티 사이클 %)를 반환. 이 숫자만 바꾸면 전진·후진 속도가 바뀜."""
    return 30


def move_raw(px, duty, direction="forward"):
    """duty(0~100%)를 모터에 그대로 적용 (forward()의 50% 하한 우회)"""
    duty = max(0, min(100, duty))
    if direction == "forward":
        px.motor_direction_pins[0].low()
        px.motor_direction_pins[1].high()
    else:
        px.motor_direction_pins[0].high()
        px.motor_direction_pins[1].low()
    px.motor_speed_pins[0].pulse_width_percent(duty)
    px.motor_speed_pins[1].pulse_width_percent(duty)


class WallBackup:
    """
    전방 벽 감지 -> 전진/후진 제어를 담당하는 부품.
    메인 프로그램에서 update()를 반복 호출하면 상태에 따라 차를 움직인다.
    """

    def __init__(self, px, duty=None, debug=False):
        self.px = px
        # duty를 안 넘기면 Get_Drive_Duty()의 값을 기본으로 사용
        self.duty = Get_Drive_Duty() if duty is None else duty
        self.debug = debug            # True면 거리/상태를 매번 출력
        self.state = "forward"        # "forward" 또는 "backward"
        self.px.set_dir_servo_angle(0)  # 앞바퀴 정면 고정

    def update(self, backSpeed):
        """
        한 번 호출할 때마다 거리를 확인하고 전진/후진을 판단.
        반환값: 현재 상태 문자열 ("forward" / "backward")
        """
        move_raw(self.px, backSpeed, "backward")	


    def stop(self, steerAngle):
        """모터 정지 + 앞바퀴 정면 복귀 (메인 프로그램 종료 시 호출)"""
        print(f"모터 정지 반대방향 조향 : {steerAngle}")
        self.px.set_dir_servo_angle(-steerAngle)
        self.px.stop()


# ==================== 단독 실행 테스트용 ====================
if __name__ == "__main__":
    from picarx import Picarx
    import time

    px = Picarx()
    wb = WallBackup(px, debug=True)    # 테스트 시 로그 켬

    print(f"시작 (후진 기준 거리: {Get_Stop_Distance()}cm, Ctrl+C 로 종료)\n")

    try:
        while True:
            wb.update()
            time.sleep(0.05)
    except KeyboardInterrupt:
        print("\n중단")
    finally:
        wb.stop()
        print("정지 완료")

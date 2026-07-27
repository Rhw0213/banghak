"""
teleop_keyboard.py
역할: SSH 터미널에서 키보드로 픽시카를 직접 조종하는 모듈 (1단계: Teleop)

목적:
    나중에 2D 라이다로 지도(SLAM)를 만들려면, 먼저 사람이 차를 직접 몰면서
    한 바퀴 돌아줘야 합니다. 이 파일은 그 "직접 모는" 기능만 담당합니다.
    아직 라이다/지도 관련 기능은 전혀 없습니다. (그건 2단계에서 따로 추가)

조작 방법 (터미널에 포커스가 있어야 키가 먹힙니다):
    w : 전진 (누를 때마다 목표 속도가 조금씩 올라감)
    s : 후진 (누를 때마다 목표 속도가 조금씩 내려감 = 뒤로 가는 방향)
    a : 왼쪽으로 조향
    d : 오른쪽으로 조향
    x : 조향을 정면(0도)으로 되돌림
    space (스페이스바) : 즉시 정지 (목표 속도를 0으로)
    q : 프로그램 종료 (자동으로 정지 후 종료)

주의:
    - 이 파일은 기존 lidar_ultra_avoidance.py 를 건드리지 않습니다. 완전히 별도 파일입니다.
    - 모터 핀 번호(P13/D4, P12/D5)와 조향 서보 핀(P2)은 기존 코드와 동일하게 맞췄습니다.
      -> 나중에 이 파일의 set_speed / set_steer 함수를 그대로 복사해서
         라이다 코드에 붙여넣기만 하면 되도록 설계했습니다.
"""

import curses          # 터미널에서 키보드 입력을 "누르는 즉시" 감지하기 위한 표준 라이브러리
import time

from robot_hat import Motor, Servo, Pin, PWM, reset_mcu


# ============================================================
# 설정값 (전역 상수)
# 나중에 라이다 코드와 합칠 때, 이 이름들이 겹치지 않는지 꼭 확인하세요.
# ============================================================

MAX_SPEED = 50        # 낼 수 있는 최대 속도 (기존 lidar 코드의 VELOCITY=50과 동일하게 맞춤)
SPEED_STEP = 3         # 한 번 루프 돌 때마다 속도를 몇씩 올리고/내릴지 (기존 SPEED_STEP=3과 동일)

STEER_LIMIT = 35       # 조향 서보가 꺾을 수 있는 최대 각도 (기존 STEER_LIMIT=35와 동일)
STEER_STEP = 5          # 키 한 번 누를 때 조향각을 몇 도씩 바꿀지

KEY_REPEAT_TIMEOUT_MS = 100   # curses가 키 입력을 기다리는 시간(ms). 이 시간 안에 키가 없으면
                               # "키 없음"으로 처리하고 루프를 계속 돕니다. (너무 길면 반응이 느려짐)


# ============================================================
# 전진/후진 속도 조절용 전역 변수
# (기존 lidar_ultra_avoidance.py 의 SPEED_FAST 와 완전히 같은 역할입니다)
# ============================================================

SPEED_FAST = 0          # 지금 실제로 모터에 걸려있는 "현재 속도" (슬로우스타트로 서서히 변함)
                         # 양수 = 전진, 음수 = 후진 방향으로 사용합니다.

TARGET_SPEED = 0        # 사용자가 키보드로 원하는 "목표 속도"
                         # SPEED_FAST는 매 루프마다 이 TARGET_SPEED를 향해 SPEED_STEP만큼씩 다가갑니다.

TARGET_STEER = 0        # 목표 조향각 (즉시 반영되므로 슬로우스타트 없음, 기존 코드와 동일)


def set_speed(left_motor, right_motor, target):
    """
    현재 속도(SPEED_FAST)를 target(목표 속도)을 향해 한 스텝(SPEED_STEP)만큼만 움직입니다.
    -> 이게 바로 "슬로우스타트/슬로우스탑" 입니다.
       예: 지금 속도 0, 목표 50 이면 -> 3, 6, 9, ... 이렇게 천천히 올라갑니다.
       예: 지금 속도 30, 목표 0  이면 -> 27, 24, 21, ... 이렇게 천천히 내려갑니다.
    이 함수를 매 루프마다 계속 호출해줘야 속도가 서서히 변합니다. (한 번만 부르면 딱 3만큼만 변함)

    target이 음수면 후진을 의미합니다.
    """
    global SPEED_FAST

    if SPEED_FAST < target:
        # 목표가 더 크면(더 빨리 가야 하면) 속도를 올림. 단, target을 넘지 않게 min으로 막음
        SPEED_FAST = min(SPEED_FAST + SPEED_STEP, target)
    elif SPEED_FAST > target:
        # 목표가 더 작으면(느려지거나 멈추거나 후진해야 하면) 속도를 내림. target 밑으로 안 내려가게 max로 막음
        SPEED_FAST = max(SPEED_FAST - SPEED_STEP, target)
    # SPEED_FAST == target 이면 아무것도 안 바꾸고 그대로 유지

    # 실제 모터에 속도 반영
    # 기존 코드처럼 왼쪽 모터는 부호를 반대로 줘서 두 바퀴가 같은 방향으로 굴러가게 보정합니다.
    left_motor.speed(-SPEED_FAST)
    right_motor.speed(SPEED_FAST)

    # 완전히 멈췄을 때는 확실하게 0을 한 번 더 박아줘서 모터가 미세하게 떨리는 걸 방지
    if SPEED_FAST == 0:
        left_motor.speed(0)
        right_motor.speed(0)

    return SPEED_FAST


def set_steer(steer_servo, angle):
    """
    조향 서보를 원하는 각도로 즉시 돌립니다. (조향은 슬로우스타트 없이 즉시 반영 - 기존 코드와 동일)
    angle이 STEER_LIMIT(35도)을 넘지 않도록 안전하게 잘라줍니다.
    """
    angle = max(-STEER_LIMIT, min(STEER_LIMIT, angle))
    steer_servo.angle(angle)
    return angle


def handle_key(key):
    """
    눌린 키(key) 하나를 보고, TARGET_SPEED와 TARGET_STEER 값을 어떻게 바꿀지 결정합니다.
    실제로 모터를 움직이는 건 여기서 안 하고, "목표값"만 바꿔둡니다.
    (진짜 모터 제어는 메인 루프에서 set_speed/set_steer가 매번 담당)
    """
    global TARGET_SPEED, TARGET_STEER

    if key == ord('w'):
        # 전진 목표 속도를 최대치까지 올림 (실제로는 슬로우스타트로 서서히 도달)
        TARGET_SPEED = MAX_SPEED

    elif key == ord('s'):
        # 후진 목표 속도 (음수 방향)
        TARGET_SPEED = -MAX_SPEED

    elif key == ord(' '):
        # 스페이스바: 즉시 정지 목표로 설정 (슬로우스탑으로 서서히 0에 도달 -> 급정거 충격 방지)
        TARGET_SPEED = 0

    elif key == ord('a'):
        # 왼쪽 조향 (각도를 음수 방향으로 이동, STEER_LIMIT을 넘지 않게 set_steer에서 알아서 잘라줌)
        TARGET_STEER -= STEER_STEP

    elif key == ord('d'):
        # 오른쪽 조향
        TARGET_STEER += STEER_STEP

    elif key == ord('x'):
        # 조향을 정면(0도)으로 리셋
        TARGET_STEER = 0

    # 조향각이 범위를 벗어나지 않도록 여기서도 한 번 더 안전하게 잘라줌
    TARGET_STEER = max(-STEER_LIMIT, min(STEER_LIMIT, TARGET_STEER))


def main(stdscr):
    """
    curses 라이브러리가 제공하는 메인 함수.
    stdscr 은 curses가 자동으로 만들어주는 "터미널 화면 객체"입니다. 우리가 직접 만들 필요 없습니다.
    """
    global SPEED_FAST, TARGET_SPEED, TARGET_STEER

    # ---------- curses 화면 설정 ----------
    curses.curs_set(0)                       # 터미널 커서(깜빡이는 막대) 숨기기
    stdscr.nodelay(True)                     # getch()가 키가 없어도 기다리지 않고 바로 리턴하게 함 (논블로킹)
    stdscr.timeout(KEY_REPEAT_TIMEOUT_MS)    # 키 입력을 몇 ms 동안 기다릴지 설정

    # ---------- 하드웨어 초기화 ----------
    # 기존 lidar_ultra_avoidance.py 와 완전히 동일한 핀 배치를 사용합니다.
    reset_mcu()
    time.sleep(0.5)

    left_motor = Motor(PWM("P13"), Pin("D4"))
    right_motor = Motor(PWM("P12"), Pin("D5"))
    steer_servo = Servo("P2")

    steer_servo.angle(0)   # 시작할 때 바퀴를 정면으로 정렬
    time.sleep(0.5)

    # ---------- 화면에 조작법 안내 출력 ----------
    stdscr.addstr(0, 0, "=== 픽시카 키보드 조종 (Teleop) ===")
    stdscr.addstr(1, 0, "w: 전진  s: 후진  a: 좌회전  d: 우회전")
    stdscr.addstr(2, 0, "x: 조향 정면복귀  space: 정지  q: 종료")
    stdscr.addstr(3, 0, "-" * 40)

    try:
        while True:
            # ---------- 1. 키 입력 확인 ----------
            key = stdscr.getch()   # 키가 눌렸으면 그 키 코드를, 안 눌렸으면 -1을 반환

            if key == ord('q'):
                break               # q를 누르면 while 루프를 빠져나가서 종료 처리로 감

            if key != -1:
                handle_key(key)     # 뭔가 눌렸으면 목표 속도/조향값 갱신

            # ---------- 2. 실제 모터 제어 (매 루프마다 반드시 호출) ----------
            # 키를 누르지 않은 순간에도 이 두 줄은 계속 실행돼야 슬로우스타트가 부드럽게 이어집니다.
            set_speed(left_motor, right_motor, TARGET_SPEED)
            set_steer(steer_servo, TARGET_STEER)

            # ---------- 3. 현재 상태를 화면에 표시 (디버깅용) ----------
            stdscr.addstr(5, 0, f"현재속도(SPEED_FAST): {SPEED_FAST:4d}   ")
            stdscr.addstr(6, 0, f"목표속도(TARGET_SPEED): {TARGET_SPEED:4d}   ")
            stdscr.addstr(7, 0, f"조향각(TARGET_STEER): {TARGET_STEER:4d}   ")
            stdscr.refresh()   # 화면 갱신 (이걸 안 하면 위 addstr 내용이 안 보임)

            # curses의 timeout()이 위 getch()에서 이미 100ms 정도 대기해주므로
            # 여기서 추가로 time.sleep()을 넣지 않아도 루프가 너무 빨리 돌지 않습니다.

    finally:
        # ---------- 종료 처리 ----------
        # q를 누르거나 에러가 나거나, 어떤 경우든 반드시 아래 정지 코드가 실행되도록
        # try/finally 구조를 씁니다. (급정거 대신 슬로우스탑으로 안전하게 멈춤)
        TARGET_SPEED = 0
        while SPEED_FAST != 0:
            set_speed(left_motor, right_motor, 0)
            time.sleep(0.05)

        set_steer(steer_servo, 0)
        left_motor.speed(0)
        right_motor.speed(0)


if __name__ == "__main__":
    # curses.wrapper()를 쓰면 프로그램이 에러로 죽거나 종료될 때
    # 터미널 화면을 자동으로 원래 상태로 복구해줍니다. (안 쓰면 터미널이 깨질 수 있음)
    curses.wrapper(main)
    print("조종 종료. 정지 완료.")

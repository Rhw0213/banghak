import curses
import time
from smooth_servo import SmoothJoint, reset_mcu   # 기존 클래스 재사용

STEP = 3    # 키 한 번 누를 때마다 움직이는 각도
STEP_UP = 10
def main(stdscr):
    curses.cbreak()
    stdscr.nodelay(True)     # 키 입력 없어도 멈추지 않고 계속 진행
    stdscr.keypad(True)      # 방향키를 특수 코드로 인식

    reset_mcu()
    time.sleep(0.5)

    base = SmoothJoint("P4", init_angle=0, move_on_init=True)
    shoulder = SmoothJoint("P5", init_angle=0, move_on_init=True)
    elbow = SmoothJoint("P6", init_angle=0, move_on_init=True)
    gripper = SmoothJoint("P7", init_angle=0, move_on_init=True)

    stdscr.addstr(0, 0, "방향키: 베이스/어깨 | W/S: 팔꿈치 | A/D: 그리퍼 | Q: 종료")

    last_key_time = 0
    KEY_COOLDOWN = 0.15   # 같은 키가 이 시간(초) 안에 다시 눌리면 무시
    
    while True:
        key = stdscr.getch()
        now = time.time()

        if key != -1 and (now - last_key_time) > KEY_COOLDOWN:
            last_key_time = now

            if key == curses.KEY_LEFT:
                base.move_to(base.angle + STEP, speed=60)
            elif key == curses.KEY_RIGHT:
                base.move_to(base.angle - STEP, speed=60)
            elif key == curses.KEY_UP:
                shoulder.move_to(shoulder.angle + STEP, speed=60)
            elif key == curses.KEY_DOWN:
                shoulder.move_to(shoulder.angle - STEP_UP, speed=60)
            elif key == ord('w'):
                elbow.move_to(elbow.angle + STEP, speed=60)
            elif key == ord('s'):
                elbow.move_to(elbow.angle - STEP, speed=60)
            elif key == ord('a'):
                gripper.move_to(gripper.angle - STEP, speed=60)
            elif key == ord('d'):
                gripper.move_to(gripper.angle + STEP, speed=60)
            elif key == ord('q'):
                break

    time.sleep(0.02)

    # 종료 시 안전하게 홈으로
    # for j, target in zip([base, shoulder, elbow, gripper], [0, -30, 0, 0]):
    # for j, target in zip([base, shoulder, elbow, gripper], [0, -30, 0, 0]):
        # j.move_to(target, speed=30)


if __name__ == "__main__":
    curses.wrapper(main)
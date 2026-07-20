# control_arm.py
# 역할: 키보드(방향키) 입력을 받아서 실제 로봇팔을 조작
# robot_hat, curses를 쓰므로 라즈베리파이(리눅스)에서만 실행 가능
import curses
import time
from robot_hat import Servo, reset_mcu
from arm_setup import build_arm

STEP = 3
STEP_UP = 10
KEY_COOLDOWN = 0.15


def main(stdscr):
    curses.cbreak()
    stdscr.nodelay(True)
    stdscr.keypad(True)

    reset_mcu()
    time.sleep(0.5)

    base, shoulder, elbow, gripper = build_arm(servo_factory=Servo)

    key_actions = {
        curses.KEY_LEFT:  lambda: base.move_to(base.angle + STEP, speed=30),
        curses.KEY_RIGHT: lambda: base.move_to(base.angle - STEP, speed=30),
        curses.KEY_UP:    lambda: shoulder.move_to(shoulder.angle + STEP, speed=30),
        curses.KEY_DOWN:  lambda: shoulder.move_to(shoulder.angle - STEP_UP, speed=30),
        ord('w'): lambda: elbow.move_to(elbow.angle + STEP, speed=30),
        ord('s'): lambda: elbow.move_to(elbow.angle - STEP, speed=30),
        ord('a'): lambda: gripper.move_to(gripper.angle - STEP, speed=30),
        ord('d'): lambda: gripper.move_to(gripper.angle + STEP, speed=30),
    }

    stdscr.addstr(0, 0, "방향키: 베이스/어깨 | W/S: 팔꿈치 | A/D: 그리퍼 | Q: 종료")

    last_key_time = 0

    while True:
        key = stdscr.getch()
        now = time.time()

        if key != -1 and (now - last_key_time) > KEY_COOLDOWN:
            last_key_time = now

            if key == ord('q'):
                break

            action = key_actions.get(key)
            if action:
                action()

        time.sleep(0.02)

    for j, target in zip([base, shoulder, elbow, gripper], [0, -30, 0, 0]):
        j.move_to(target, speed=30)


if __name__ == "__main__":
    curses.wrapper(main)

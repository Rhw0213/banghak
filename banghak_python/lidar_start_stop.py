"""
lidar_start_stop.py
목적: 키보드로 라이다만 켜고 끄기 + 스캔 로그 + 배터리 표시 (Enter 불필요)
      (차량 움직임에는 관여하지 않음)

조작법 (Enter 없이 키 하나로 즉시 반응):
    q : 배터리 표시 -> 라이다 켜기(모터 회전 + 스캔) -> 감지 로그 표시
    w : 모터 정지(배터리 절약) -> 남은 배터리 표시 (로그 중단)
    e : 종료

* raw 모드를 프로그램 내내 '한 번만' 켜서 유지하고, 키는 논블로킹으로 확인.
  (이전처럼 raw 모드를 껐다 켰다 반복하면 로그 스레드와 충돌해서 먹통이 됨)
* raw 모드에서는 \n이 줄 맨 앞으로 안 돌아가므로 로그에 \r\n을 사용.

실행:
    sudo python3 lidar_start_stop.py
"""
from rplidar import RPLidar
from robot_hat import utils
import threading
import time
import sys
import select
import termios
import tty

PORT = "/dev/ttyUSB0"

lidar = None
scanning = False
scan_thread = None
stop_scan_flag = False

LOG_EVERY = 5   # 이 숫자마다 한 번씩만 로그 출력 (1 = 매 바퀴)


def read_battery():
    try:
        v = utils.get_battery_voltage()
        return f"{v:.2f} V"
    except Exception as e:
        return f"읽기 실패 ({e})"


def out(text):
    """raw 모드에서도 줄이 안 밀리도록 \r\n을 붙여 출력"""
    sys.stdout.write("\r" + text + "\r\n")
    sys.stdout.flush()


def scan_loop():
    global stop_scan_flag
    try:
        for i, scan in enumerate(lidar.iter_scans()):
            if stop_scan_flag:
                break
            if i % LOG_EVERY != 0:
                continue
            dists = [d for (_, _, d) in scan if d > 0]
            if dists:
                nearest = min(dists)
                nearest_ang = [a for (_, a, d) in scan if d == nearest][0]
                out(f"[스캔 {i:4d}] 측정점 {len(scan):3d}개 | "
                    f"{nearest_ang:5.1f}도 | 최근접 {nearest:6.0f}mm")
            else:
                out(f"[스캔 {i:4d}] 측정점 {len(scan):3d}개 | 유효 거리 없음")
    except Exception as e:
        out(f"스캔 중단: {e}")


def lidar_on():
    global scanning, scan_thread, stop_scan_flag
    if scanning:
        out(">> 이미 켜져 있습니다.")
        return
    out("=" * 50)
    out(f"[Q] 라이다 켜기   배터리: {read_battery()}")
    out("=" * 50)
    lidar.start_motor()
    time.sleep(1.0)
    stop_scan_flag = False
    scan_thread = threading.Thread(target=scan_loop, daemon=True)
    scan_thread.start()
    scanning = True


def lidar_off():
    global scanning, stop_scan_flag
    if not scanning:
        out(">> 이미 꺼져 있습니다.")
        return
    stop_scan_flag = True
    time.sleep(0.3)
    lidar.stop()
    lidar.stop_motor()
    scanning = False
    out("=" * 50)
    out(f"[W] 모터 정지   남은 배터리: {read_battery()}")
    out("=" * 50)


def get_key_nonblocking():
    """키가 눌려 있으면 그 문자를, 없으면 None을 즉시 반환 (기다리지 않음)"""
    # select로 0초 대기: 입력 버퍼에 뭔가 있을 때만 읽음
    if select.select([sys.stdin], [], [], 0)[0]:
        return sys.stdin.read(1)
    return None


def main():
    global lidar
    lidar = RPLidar(PORT)
    lidar.stop()
    lidar.stop_motor()

    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)

    try:
        # raw 모드를 여기서 딱 한 번 켜고, 프로그램 내내 유지 (껐다 켜기 안 함)
        tty.setcbreak(fd)

        out("=" * 50)
        out(" 라이다 제어 (Enter 없이 키 하나로)")
        out("  q : 켜기(로그)   w : 끄기(배터리)   e : 종료")
        out("=" * 50)
        out("현재 상태: 꺼짐")

        while True:
            key = get_key_nonblocking()
            if key is not None:
                key = key.lower()
                if key == "q":
                    lidar_on()
                elif key == "w":
                    lidar_off()
                elif key == "e":
                    out("종료합니다.")
                    break
            time.sleep(0.05)   # CPU 낭비 방지 (키 없을 때 잠깐 쉼)

    except KeyboardInterrupt:
        out("중단")
    finally:
        stop_scan_flag_local = True
        # 터미널을 원래 상태로 복구 (이걸 안 하면 종료 후 터미널이 이상해짐)
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        try:
            lidar.stop()
            lidar.stop_motor()
            lidar.disconnect()
        except Exception:
            pass
        print("라이다 정지 및 연결 해제 완료")


if __name__ == "__main__":
    main()
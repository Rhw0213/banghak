# lidar_rear_test_simple.py
# 라이다 단독 실행 - 후방 섹터 감지 단순 테스트
#
# 출력 항목 2개만:
#   1. 인식거리(cm) - 감지 안되면 '인식없음'
#   2. 포인트숫자 - 0으로 떨어지는 지점이 사각지대 시작점
#
# 종료: Ctrl+C

import time
from rplidar import RPLidar

LIDAR_PORT = '/dev/ttyUSB0'   # 실제 포트에 맞게 수정
SCAN_MIN_LEN = 60
REAR_SECTOR_DEG = 25          # 후방 기준 ±25도 섹터
PRINT_INTERVAL_SEC = 0.3


def normalize_angle(angle):
    angle = angle % 360
    if angle > 180:
        angle -= 360
    return angle


def get_rear_points(scan):
    """후방 섹터(±REAR_SECTOR_DEG) 내 유효 거리(mm) 리스트 반환"""
    dists = []
    for quality, angle, distance in scan:
        if distance <= 0:
            continue
        norm = normalize_angle(angle)
        if abs(norm) >= (180 - REAR_SECTOR_DEG):
            dists.append(distance)
    return dists


def main():
    print(f"라이다 연결: {LIDAR_PORT}")
    lidar = RPLidar(LIDAR_PORT)

    print("측정 시작 (Ctrl+C 종료)\n")
    print(f"{'인식거리(cm)':>12} | {'포인트숫자':>8}")
    print("-" * 30)

    last_print = 0

    try:
        for scan in lidar.iter_scans(min_len=SCAN_MIN_LEN):
            now = time.time()
            if now - last_print < PRINT_INTERVAL_SEC:
                continue
            last_print = now

            dists = get_rear_points(scan)

            if not dists:
                print(f"{'인식없음':>12} | {0:>8}")
                continue

            min_cm = min(dists) / 10.0
            print(f"{min_cm:12.1f} | {len(dists):>8}")

    except KeyboardInterrupt:
        print("\n측정 종료")
    finally:
        lidar.stop()
        lidar.stop_motor()
        lidar.disconnect()
        print("라이다 연결 해제 완료")


if __name__ == "__main__":
    main()

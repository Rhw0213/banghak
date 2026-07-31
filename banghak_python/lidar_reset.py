# lidar_reset.py
# RPLidar "쓰레기통 비우기" 스크립트
# Descriptor length mismatch 등 통신 오류 발생 시 이것부터 단독 실행해보세요.
#
# 사용법:
#   python3 lidar_reset.py
#   (포트가 다르면) python3 lidar_reset.py /dev/ttyUSB1

import sys
import time
from rplidar import RPLidar

DEFAULT_PORT = '/dev/ttyUSB0'


def reset_lidar(port):
    print(f"[1/5] 라이다 연결 시도: {port}")
    lidar = None
    try:
        lidar = RPLidar(port)

        print("[2/5] 모터 정지 명령 전송")
        try:
            lidar.stop()
        except Exception as e:
            print(f"   stop() 실패 (무시하고 계속): {e}")

        try:
            lidar.stop_motor()
        except Exception as e:
            print(f"   stop_motor() 실패 (무시하고 계속): {e}")

        print("[3/5] 0.5초 대기 (모터/통신 안정화)")
        time.sleep(0.5)

        print("[4/5] 시리얼 수신 버퍼 비우기 (clean_input)")
        try:
            lidar.clean_input()
        except Exception as e:
            print(f"   clean_input() 실패: {e}")

        print("[5/5] 통신 확인 (get_info)")
        info = lidar.get_info()
        print(f"   라이다 정보: {info}")

        health = lidar.get_health()
        print(f"   라이다 상태: {health}")

        print("\n✅ 리셋 완료. 라이다가 정상 응답합니다.")
        return True

    except Exception as e:
        print(f"\n❌ 리셋 실패: {e}")
        print("   -> 전원 재인가, USB 포트 변경, 다른 프로세스 점유 여부(lsof)를 확인해보세요.")
        return False

    finally:
        if lidar is not None:
            try:
                lidar.disconnect()
                print("라이다 연결 종료 완료")
            except Exception:
                pass


if __name__ == "__main__":
    port = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_PORT
    ok = reset_lidar(port)
    sys.exit(0 if ok else 1)
# lidar_raw_test.py
# rplidar 라이브러리를 거치지 않고, pyserial로 직접 GET_INFO 명령을 보내
# 원시(raw) 응답 바이트를 확인하는 진단 스크립트.
#
# 사용법:
#   python3 lidar_raw_test.py
#   python3 lidar_raw_test.py /dev/ttyUSB0 115200
#   python3 lidar_raw_test.py /dev/ttyUSB0 256000

import sys
import time
import serial

DEFAULT_PORT = '/dev/ttyUSB0'
DEFAULT_BAUD = 115200

SYNC_BYTE = b'\xA5'
GET_INFO_CMD = b'\xA5\x50'

def try_baud(port, baud):
    print(f"\n===== {port} @ {baud} baud 테스트 =====")
    try:
        ser = serial.Serial(port, baudrate=baud, timeout=1)
    except Exception as e:
        print(f"포트 열기 실패: {e}")
        return

    try:
        # DTR을 이용한 모터 제어 라인 정리 (rplidar 라이브러리와 동일하게)
        ser.dtr = False
        time.sleep(0.1)

        # 남은 버퍼 비우기
        ser.reset_input_buffer()
        ser.reset_output_buffer()

        print(f"GET_INFO 명령 전송: {GET_INFO_CMD.hex()}")
        ser.write(GET_INFO_CMD)
        time.sleep(0.2)

        waiting = ser.in_waiting
        print(f"응답 대기 중 수신된 바이트 수: {waiting}")

        raw = ser.read(64)  # 넉넉히 읽어봄
        print(f"수신된 원시 바이트 ({len(raw)}개): {raw.hex(' ') if raw else '(없음)'}")

        if len(raw) == 0:
            print(">> 결과: 응답 없음. 라이다가 명령에 전혀 반응하지 않음")
            print("   -> 전원/케이블/포트 방향(TX-RX) 문제 가능성이 높습니다.")
        elif raw[:1] != SYNC_BYTE:
            print(f">> 결과: 첫 바이트가 예상(0xA5)과 다름 -> 응답이 아니라 노이즈/깨진 데이터")
            print("   -> 다른 baudrate 이거나, 전원 불안정으로 신호가 깨졌을 가능성")
        else:
            print(">> 결과: 정상적인 SYNC 바이트(0xA5) 확인됨. 통신 자체는 되고 있음")
            if len(raw) >= 7:
                dsize = raw[2] | (raw[3] << 8) | (raw[4] << 16) | ((raw[5] & 0x3F) << 24)
                print(f"   디스크립터 길이 필드: {dsize} (get_info 정상이면 20이어야 함)")

    except Exception as e:
        print(f"통신 중 오류: {e}")
    finally:
        ser.close()


if __name__ == "__main__":
    port = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_PORT
    if len(sys.argv) > 2:
        try_baud(port, int(sys.argv[2]))
    else:
        # baudrate 지정 없으면 둘 다 테스트
        try_baud(port, 115200)
        time.sleep(0.5)
        try_baud(port, 256000)
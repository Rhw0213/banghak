"""
ultrasonic_test.py
목적: 초음파 거리 센서가 정상적으로 값을 읽어오는지 확인
"""

from picarx import Picarx
import time

px = Picarx()

try:
    while True:
        # 초음파 센서로 전방 거리 측정 (단위: cm)
        distance = px.ultrasonic.read()
        print(f"거리: {distance} cm")

        time.sleep(0.3)  # 0.3초마다 측정 (너무 빠르면 센서 노이즈 심해짐)

except KeyboardInterrupt:
    # Ctrl+C로 종료 시 깔끔하게 메시지 출력하고 끝냄
    print("측정 종료")

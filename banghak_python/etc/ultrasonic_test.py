"""
ultrasonic_test.py
목적: 초음파 거리 센서가 정상적으로 값을 읽어오는지 확인
"""
from picarx import Picarx
import time

px = Picarx()

try:
    while(True):
        distance = px.ultrasonic.read()
        print(f"거리:{distance}cm")
        time.sleep(2)
except KeyboardInterrupt:
    print("측정 종료")
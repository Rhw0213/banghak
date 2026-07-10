"""
camera_test.py
목적: PiCar-X 카메라를 켜고 웹 브라우저로 실시간 영상을 확인
"""
from vilib import Vilib
import time

# 카메라 시작 (vflip/hflip: 화면이 뒤집혀 보이면 True/False로 조정)
Vilib.camera_start(vflip=False, hflip=False)

# local=False  : SSH 환경이라 모니터 창(cv2 imshow)은 띄우지 않음
# web=True     : 웹 브라우저로 스트리밍 (http://라즈베리파이IP:9000/mjpg)
Vilib.display(local=False, web=True)

print("카메라 시작됨. 웹 브라우저에서 아래 주소로 확인하세요:")
print("http://<라즈베리파이_IP>:9000/mjpg")
print("종료하려면 Ctrl+C")

try:
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    print("종료합니다.")
finally:
    Vilib.camera_close()    # 60초간 표시

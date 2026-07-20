# mock_servo.py
# 역할: 실제 로봇팔 하드웨어 없이도 테스트할 수 있게, Servo/reset_mcu를 흉내내는 가짜 클래스
# 실제 신호는 안 보내고, 어떤 포트가 몇 도로 바뀌는지 print로만 알려줌

class MockServo:
    def __init__(self, port):
        self.port = port

    def angle(self, value):
        print(f"[MOCK] {self.port} -> {value:.1f}도")


def mock_reset_mcu():
    print("[MOCK] reset_mcu() 호출됨 (실제 리셋 없음)")

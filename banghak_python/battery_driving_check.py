"""
battery_driving_check.py
역할: 주행 중 남은 배터리를 표시하는 '부품' 모듈

메인 프로그램에서 이렇게 불러다 씁니다:
    from driving_battery_check import BatteryMonitor

    bat = BatteryMonitor(interval=2.0)   # 2초마다 갱신
    while True:
        # ... 주행 로직 ...
        bat.show()        # 갱신 주기가 됐을 때만 배터리 출력 (아니면 그냥 넘어감)
        time.sleep(0.05)

* 배터리를 매 루프 읽으면 부담되므로, interval(초)마다 한 번만 읽음.
"""
import time

# 배터리 읽기: deprecated 경고를 피하려고 device 쪽에서 직접 가져옴
try:
    from robot_hat import get_battery_voltage
except ImportError:
    from robot_hat.utils import get_battery_voltage


def Get_Battery_Low():
    """이 전압(V) 이하이면 '낮음' 경고를 표시할 기준값"""
    return 6.7


class BatteryMonitor:
    """
    주행 중 배터리 전압을 주기적으로 읽어서 표시하는 부품.
    메인 루프에서 show()를 반복 호출하면, 정해진 간격마다만 실제로 읽고 출력한다.
    """

    def __init__(self, interval=2.0):
        self.interval = interval      # 배터리를 읽고 표시할 간격(초)
        self.last_time = 0            # 마지막으로 읽은 시각
        self.last_v = None            # 마지막으로 읽은 전압
        self.min_v = 99.0             # 지금까지 본 최저 전압

    def read(self):
        """지금 배터리 전압(V)을 읽어서 반환. 실패 시 None."""
        try:
            v = get_battery_voltage()
            if 0 < v < self.min_v:
                self.min_v = v
            self.last_v = v
            return v
        except Exception:
            return None

    def show(self):
        """
        interval이 지났을 때만 배터리를 읽고 한 줄 출력.
        아직 때가 안 됐으면 아무것도 안 하고 넘어감(주행에 방해 안 됨).
        반환값: 방금 표시했으면 전압(V), 아니면 None
        """
        now = time.time()
        if now - self.last_time < self.interval:
            return None      # 아직 갱신할 때가 아님

        self.last_time = now
        v = self.read()
        if v is None:
            print("배터리: 읽기 실패")
            return None

        warn = "  <-- 낮음! (충전 필요)" if v <= Get_Battery_Low() else ""
        print(f"##[배터리] {v:.2f}V  (최저 {self.min_v:.2f}V){warn}##")
        return 

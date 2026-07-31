"""
battery_check.py
목적: 배터리 전압 확인 및 충전 상태 판단
"""

from robot_hat import utils

voltage = utils.get_battery_voltage()
print(f"배터리 전압: {voltage:.2f} V")

if voltage > 7.6:
    print("상태: 충분함")
elif voltage >= 7.15:
    print("상태: 보통")
else:
    print("상태: 낮음 - 충전 필요")

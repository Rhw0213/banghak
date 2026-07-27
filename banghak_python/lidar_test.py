from rplidar import RPLidar

PORT = '/dev/ttyUSB0'
lidar = RPLidar(PORT)

# 라이다 정보 출력
info = lidar.get_info()
print("장비 정보:", info)

health = lidar.get_health()
print("상태:", health)

try:
    # 스캔 데이터 받기
    for i, scan in enumerate(lidar.iter_scans()):
        print(f"--- 스캔 {i}: 측정점 {len(scan)}개 ---")
        # scan은 (quality, angle, distance) 튜플의 리스트
        for quality, angle, distance in scan[:5]:   # 앞 5개만 출력
            print(f"  각도 {angle:.1f}도, 거리 {distance:.0f}mm")

        if i >= 3:   # 3바퀴만 보고 멈춤
            break

except KeyboardInterrupt:
    print("중단")
finally:
    lidar.stop()
    lidar.stop_motor()
    lidar.disconnect()

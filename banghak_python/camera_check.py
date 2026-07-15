"""
camera_check.py
목적: 카메라가 실제로 무엇을 보고 있는지, THRESHOLD를 얼마로 잡아야 하는지 확인

사용법:
    1. 차를 흰 선 위에 올려놓는다 (실제 주행할 자세 그대로)
    2. sudo python3 camera_check.py
    3. 출력된 밝기 통계를 보고, 저장된 이미지들을 확인

결과 파일 (/home/banghak/ 에 저장):
    check_raw.jpg      - 카메라 원본
    check_th140.jpg    - 임계값 140으로 이진화
    check_th160.jpg
    check_th180.jpg    - 현재 코드 기본값
    check_th200.jpg
"""
from picamera2 import Picamera2
from picarx import Picarx
import cv2
import numpy as np
import time
import os

WIDTH, HEIGHT = 320, 240
CAM_PAN = 0
CAM_TILT = -40          # line_trace.py와 같은 값으로 맞출 것
ROI_TOP = 150
ROI_BOTTOM = 225

OUT_DIR = os.path.expanduser("~")

px = Picarx()
px.set_cam_pan_angle(CAM_PAN)
px.set_cam_tilt_angle(CAM_TILT)

picam2 = Picamera2()
picam2.configure(picam2.create_preview_configuration(
    main={"size": (WIDTH, HEIGHT), "format": "RGB888"}))
picam2.start()
time.sleep(2.0)   # 노출 안정화 (충분히 기다림)

frame = picam2.capture_array()
picam2.stop()

gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
roi = gray[ROI_TOP:ROI_BOTTOM, :]

print("=" * 50)
print("전체 화면 밝기")
print(f"  최소 {gray.min():3d}   평균 {gray.mean():6.1f}   최대 {gray.max():3d}")
print()
print(f"관심영역(ROI, y={ROI_TOP}~{ROI_BOTTOM}) 밝기")
print(f"  최소 {roi.min():3d}   평균 {roi.mean():6.1f}   최대 {roi.max():3d}")
print()

# 밝기 분위수: 흰 선이 화면의 몇 %를 차지하는지에 따라 임계값을 정한다
for p in [50, 80, 90, 95, 99]:
    print(f"  상위 {100-p:2d}% 밝기 기준값: {np.percentile(roi, p):.0f}")
print()

if roi.max() < 180:
    print("!! ROI 최대 밝기가 180 미만입니다.")
    print("   -> THRESHOLD=180 이면 아무것도 안 잡힙니다. 값을 낮추세요.")
elif roi.mean() > 150:
    print("!! ROI 평균이 너무 밝습니다. 바닥까지 하얗게 잡힐 수 있습니다.")
    print("   -> THRESHOLD를 올리거나 카메라 노출을 줄이세요.")
else:
    print("밝기 분포는 정상 범위로 보입니다.")
print("=" * 50)

# ROI 표시한 원본 저장
marked = frame.copy()
cv2.rectangle(marked, (0, ROI_TOP), (WIDTH - 1, ROI_BOTTOM), (0, 255, 255), 1)
cv2.imwrite(os.path.join(OUT_DIR, "check_raw.jpg"), marked)

# 여러 임계값으로 이진화해서 저장 -> 어느 값이 맞는지 눈으로 비교
for th in [140, 160, 180, 200]:
    _, binary = cv2.threshold(gray, th, 255, cv2.THRESH_BINARY)
    out = cv2.cvtColor(binary, cv2.COLOR_GRAY2BGR)
    cv2.rectangle(out, (0, ROI_TOP), (WIDTH - 1, ROI_BOTTOM), (0, 255, 255), 1)

    # 이 임계값에서 ROI 안의 흰 덩어리가 몇 개, 얼마나 큰지
    contours, _ = cv2.findContours(binary[ROI_TOP:ROI_BOTTOM, :],
                                   cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    areas = sorted([int(cv2.contourArea(c)) for c in contours], reverse=True)
    print(f"THRESHOLD={th:3d} -> ROI 안 흰 덩어리 {len(contours)}개, 면적 상위: {areas[:4]}")

    cv2.imwrite(os.path.join(OUT_DIR, f"check_th{th}.jpg"), out)

print(f"\n이미지 저장 완료: {OUT_DIR}/check_raw.jpg, check_th*.jpg")
print("VS Code 탐색기에서 열어보세요.")

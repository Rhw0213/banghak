import time
from picarx import Picarx

# PiCar-X 객체 초기화
px = Picarx()

# 글로벌 상태 변수 (인터락용)
is_driving = False  # 차량이 이동 중인가?
is_arm_moving = False  # 로봇 팔이 작동 중인가?

# 소프트 스타트 및 감속 설정
current_speed = 0
UPDATE_INTERVAL = 0.05  # 50ms마다 속도 갱신
ACCEL_STEP = 5          # 한 번에 변화할 속도 폭

def set_motor_speed_soft(target_speed):
    """
    소프트 스타트 및 소프트 스톱을 적용하여 모터 속도를 제어하는 함수
    """
    global current_speed
    
    # 목표 속도까지 서서히 도달
    while current_speed != target_speed:
        if current_speed < target_speed:
            current_speed += ACCEL_STEP
            if current_speed > target_speed:
                current_speed = target_speed
        elif current_speed > target_speed:
            current_speed -= ACCEL_STEP
            if current_speed < target_speed:
                current_speed = target_speed
        
        # PiCar-X 모터에 속도 적용
        px.forward(current_speed)
        time.sleep(UPDATE_INTERVAL)

def move_vehicle(target_speed):
    """
    인터락이 적용된 차량 이동 함수
    """
    global is_driving, is_arm_moving
    
    # [인터락] 로봇 팔이 움직이고 있다면 차량 이동 명령을 무시함
    if is_arm_moving:
        print("[경고] 로봇 팔이 작동 중이므로 차량을 움직일 수 없습니다. (인터락)")
        return

    # 차량 이동 시작 상태 표시
    if target_speed > 0:
        is_driving = True
        print(f"[차량] 이동 시작 (목표 속도: {target_speed})")
        set_motor_speed_soft(target_speed)
    else:
        # 멈출 때도 부드럽게 감속
        set_motor_speed_soft(0)
        is_driving = False
        print("[차량] 정지 완료")

def control_arm(servo_angle):
    """
    인터락이 적용된 로봇 팔(서보 모터) 제어 함수
    """
    global is_driving, is_arm_moving
    
    # [인터락] 차량이 이동 중이라면 로봇 팔 조작 명령을 무시함
    if is_driving:
        print("[경고] 차량이 이동 중이므로 로봇 팔을 움직일 수 없습니다. (인터락)")
        return
        
    # 로봇 팔 작동 시작
    is_arm_moving = True
    print(f"[로봇 팔] 동작 수행 중... 각도: {servo_angle}")
    
    # 예시: PiCar-X의 카메라 서보나 임의의 서보 핀 제어 (핀 번호는 환경에 맞게 수정)
    # px.set_servo_angle(1, servo_angle) 
    time.sleep(1.0) # 팔이 움직이는 시간 동안 대기
    
    is_arm_moving = False
    print("[로봇 팔] 동작 완료")

# --- 테스트 시나리오 동작 확인 ---
if __name__ == "__main__":
    try:
        print("1. 차량 전진 명령 (소프트 스타트 작동)")
        move_vehicle(50)  # 0에서 50까지 부드럽게 가속
        
        time.sleep(1)
        
        print("\n2. 차량이 달리는 도중 로봇 팔 작동 시도 (인터락 작동)")
        control_arm(90)   # 차량이 움직이고 있으므로 거부됨
        
        time.sleep(1)
        
        print("\n3. 차량 정지 (소프트 스톱 작동)")
        move_vehicle(0)   # 50에서 0으로 부드럽게 감속 후 완전히 멈춤
        
        time.sleep(1)
        
        print("\n4. 차량이 멈춘 후 로봇 팔 작동 시도 (인터락 해제 상태)")
        control_arm(90)   # 안전하게 작동됨
        
    except KeyboardInterrupt:
        px.forward(0)

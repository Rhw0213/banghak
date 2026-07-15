# smooth_servo.py
from robot_hat import Servo, reset_mcu
import time


class SmoothJoint:
    """현재 각도를 기억하면서 속도 제어가 가능한 서보 관절"""

    def __init__(self, port, init_angle=0.0, min_angle=-90, max_angle=90, move_on_init=True):
        self.servo = Servo(port)
        self.min_angle = min_angle
        self.max_angle = max_angle
        self.angle = init_angle
        if move_on_init:
            self.servo.angle(init_angle)   # 초기 위치로 고정

    def move_to(self, target, speed=30.0, step=1.0):
        """
        target: 목표 각도
        speed: 초당 몇 도로 움직일지 (도/초). 낮을수록 느림
        step: 한 번에 움직이는 각도 (작을수록 부드럽지만 통신량 증가)
        """
        target = max(self.min_angle, min(self.max_angle, target))
        delay = step / speed   # 스텝당 대기 시간

        direction = 1.0 if target > self.angle else -1.0

        while abs(target - self.angle) > step:
            self.angle += step * direction
            self.servo.angle(self.angle)
            time.sleep(delay)

        self.angle = target          # 남은 오차 마무리
        self.servo.angle(self.angle)
        return self.angle


def move_all(joints, targets, duration=2.0, steps=50):
    """
    여러 관절을 동시에 움직임 (같이 출발해서 같이 도착)
    joints: [j1, j2, j3, j4]
    targets: [목표각도 4개]
    duration: 전체 이동에 걸릴 시간(초)
    """
    starts = [j.angle for j in joints]
    delay = duration / steps

    for i in range(steps + 1):
        t = i / steps                        # 진행률 0.0 -> 1.0
        for j, s, e in zip(joints, starts, targets):
            a = s + (e - s) * t              # 선형 보간
            a = max(j.min_angle, min(j.max_angle, a))
            j.angle = a
            j.servo.angle(a)
        time.sleep(delay)


# ================== 사용 예시 ==================
if __name__ == "__main__":

    reset_mcu()
    time.sleep(1)
    m1 = SmoothJoint("P0", init_angle=0)    # 베이스
    m2 = SmoothJoint("P1", init_angle=0)    # 베이스
    m3 = SmoothJoint("P2", init_angle=0)    # 베이스

    j1 = SmoothJoint("P4", init_angle=0)    # 베이스
    j2 = SmoothJoint("P5", init_angle=0)    # 어깨
    j3 = SmoothJoint("P6", init_angle=0)   # 팔꿈치
    j4 = SmoothJoint("P7", init_angle=0)    # 그리퍼
    joints = [j1, j2, j3, j4]
    time.sleep(1)

    #초기화 
    move_all(joints, [0, 0, 0, 0], duration=1.5)   # 안전하게 홈으로

    time.sleep(1)

    #for i in range(10): 
    try:
    	# 1) 관절 하나만 천천히
        print("베이스 천천히 30도로")
        #m1.move_to(0)      
        #m2.move_to(0)      
        #m3.move_to(0)      
    	
    	#집고 
        #j4.move_to(-25, speed =11)

        time.sleep(0.5)
        
        #들어올린다 
        #j2.move_to(-50, speed = 11)    
        
        #j1.move_to(0)
        #j3.move_to(0)    
        
        time.sleep(0.5)
        
        #print("베이스 빠르게 복귀")
        #j1.move_to(0, speed=60)       # 초당 60도 -> 0.5초
        
        ## 2) 4개 관절 동시에 부드럽게
        #print("전체 관절 동시 이동")
        #move_all(joints, [20, 30, 60, 10], duration=2.0)
        #time.sleep(0.5)
        
        #move_all(joints, [0, 0, 45, 0], duration=2.0)   # 홈 복귀

    except KeyboardInterrupt:
        print("중단")
    #finally:
	#move_all(joints, [0, 0, 0, -25], duration=1.5)   # 안전하게 홈으로

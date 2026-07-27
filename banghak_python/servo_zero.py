from picarx import Picarx, utils
import time

px = Picarx()

#30도
# px.set_cam_tilt_angle(-40)
px.set_dir_servo_angle(0)
# px.set_cam_pan_angle(0)
time.sleep(2)

#90도  
#그랩   :그랩벌림 / 25 그랩물림  .

#엘보   :0도 ~ -30
#어깨   : 
#베이스 :0도 ~  
# px.set_cam_pan_angle(90)
# time.sleep(2)

# utils.reset_mcu() 
# time.sleep(0.2)

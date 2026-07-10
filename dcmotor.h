#ifndef DCMOTOR_H 
#define DCMOTOR_H 


void mcu_hard_reset(void)
void motor_init(void)
void motor_set_speed(int speed)
void motor_set_direction(int forward)
void motor_stop(void)
int lidar_read_distance_median(int sample)

#endif

#ifndef DCMOTOR_H 
#define DCMOTOR_H 

#include <wiringPi.h>
#include <wiringPiI2C.h>

#define MCU_ADDR 	0x14
#define MOTOR1_PWM_REG  0X2D
#define MOTOR2_PWM_REG  0X2C

#define TIMER_PERIOD_REG 0x47
#define TIMER_PERSCALER_REG 0x43
#define PERIOD_VALUE 4095 
#define PRECLAER_VALUE 10 
#define DIR_PIN_1 23 //BCM NUMBER
#define DIR_PIN_2 24
#define MCU_RST_PIN 5

void mcu_hard_reset(void)
void motor_init(void)
void motor_set_speed(int speed)
void motor_set_direction(int forward)
void motor_stop(void)
int lidar_read_distance_median(int sample)
#endif

#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <stdio.h>
#include <stdlib.h>
#include "lidar.h" 
#include "servo.h"
#include "common.h"
#include "keyboard.h"
#include "dcmotor.h"

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

void init();
void run();

int speed = 50;

int main(void)
{

	init();
	run();

	motor_stop();

	return 0;
}

void init()
{
	wiringPiSetupGpio();
	lidar_init();
	motor_init();
	keyboard_init();
	fd = wiringPiI2CSetup(MCU_ADDR);
	servo_init();

	servo_set_angle(0);
}

void run()
{
	int isActive = 0;	
	float sumAngle = 0;

	while(keyboard_run())
	{
		int distance = lidar_read_distance_median(12);
		int beforeDistance = 0;

		if (distance < 0)
		{
			distance = beforeDistance;
		}
		
		if (isActive == 0)
		{
			servo_set_angle(0);
			motor_set_speed(speed);
			motor_set_direction(0);
			isActive = 1;
		}

		printf("distance : %d\r\n", distance);

		if (distance <= 30)
		{ 
			sumAngle = 35.0f;  
		}
		else if (distance <= 35) 
		{ 
			sumAngle = 30.0f; 
		}
		else if (distance <= 40) 
		{ 
			sumAngle = 25.0f; 
		}
		else if (distance <= 45) 
		{ 
			sumAngle = 20.0f; 
		}
		else if (distance <= 50) 
		{ 
			sumAngle = 15.0f; 
		}
		else if (distance <= 55) 
		{ 
			sumAngle = 10.0f; 
		}
		else if (distance <= 60) 
		{ 
			sumAngle = 5.0f; 
		}
		else if (distance <= 65) 
		{ 
			sumAngle = .0f; 
		}
		else  
		{ 
			sumAngle = 0.0f; 
		}

		speed = 90 - (int)(sumAngle * 2.0f);

		printf("speed : %d\r\n", speed);
		if (sumAngle >= 40) 
		{
			sumAngle = 40; 
		}

		if (sumAngle != 0) 
		{
			//motor_set_speed(0);
			servo_set_angle(sumAngle);
			//delay(10);
			isActive = 0;
		}
		motor_set_speed(speed);

		beforeDistance = distance;

		//delay(5); 
	}	
}

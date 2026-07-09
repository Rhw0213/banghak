#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <stdio.h>
#include <stdlib.h>
#include <ncurses.h>
#include "lidar.h" 
#include "servo.h"
#include "common.h"

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

//static int motor_fd;
static int count = 0; 

void keyboard_init();

void mcu_hard_reset(void);
void motor_init(void);
void motor_set_speed(int speed);
void motor_set_direction(int forward);
void motor_stop(void);
void input();
int lidar_read_distance_median(int sample);

int main(void)
{

	wiringPiSetupGpio();

	//delay(1000);

	lidar_init();

	motor_init();
	keyboard_init();

	fd = wiringPiI2CSetup(MCU_ADDR);

	servo_init();
	printf("%d\n",fd);
	//servo_set_angle(5);
	servo_set_angle(0);

	//mcu_hard_reset();

	int distance = 0; 

	int isActive = 0;	

	int key = 0;

	float angle = 0;
	float sumAngle = 0;

	while(key != 27)
	{

		key = getch();

		switch(key)
		{
			case 32:
				motor_stop();
				endwin();
				return 1;
		}

		distance = lidar_read_distance_median(8);
		
		if (isActive == 0)
		{
			servo_set_angle(0);
			motor_set_speed(50);
			motor_set_direction(0);
			isActive = 1;
		}

		printf("distance : %d\n", distance);

		if (distance < 30)
		{
			angle = 5.0f;
			sumAngle = 35.0f;
		} 
		else if (distance < 35)
		{	
			angle = 5;
			sumAngle = 30.0f;
		}
		else if (distance < 40)
		{	
			angle = 5;
			sumAngle = 25.0f;
		}
		else if (distance < 45)
		{	
			angle = 5;
			sumAngle = 20.0f;
		}
		else if (distance < 50)
		{	
			angle = 5;
			sumAngle = 15.0f;
		}
		else if (distance < 55)
		{	
			angle = 5;
			sumAngle = 10.0f;
		}
		else if (distance < 60)
		{	
			angle = 5;
			sumAngle = 5.0f;
		}
		else
		{
			angle = 0;
			sumAngle = 0.0f;
		}


		if (sumAngle >= 35) 
		{
			sumAngle = 35; 
			angle = 5;
		}

		//if (sumAngle <= -35) sumAngle = 0; 

		if (sumAngle != 0) 
		{
			motor_set_speed(0);
			servo_set_angle(sumAngle);
			delay(50);
			isActive = 0;
			motor_set_speed(50);
		}


		delay(10); 
	}	

	delay(2000);
	motor_stop();
	endwin();
	return 0;
}

void keyboard_init() 
{
	initscr();
	cbreak();
	noecho();
	keypad(stdscr, TRUE);
	nodelay(stdscr, TRUE);
}

//int write_reg16_reversed(int fd, int reg, int value)
//{
//	int swapped_value = ((value >> 8) & 0x00FF) | ((value << 8) & 0xFF00);
//	return wiringPiI2CWriteReg16(fd, reg, swapped_value);
//} 

void mcu_hard_reset(void)
{
	pinMode(MCU_RST_PIN, OUTPUT); 
	digitalWrite(MCU_RST_PIN, LOW);
	delay(10);
	digitalWrite(MCU_RST_PIN, HIGH);
	delay(10);
} 

void motor_init(void)
{
	//fd = wiringPiSetupGpio();
	//mcu_hard_reset();

	pinMode(DIR_PIN_1,OUTPUT);
	pinMode(DIR_PIN_2,OUTPUT);

	//fd = wiringPiI2CSetup(MCU_ADDR);

	if (fd < 0)
	{
		fprintf(stderr, "it can't connect to I2c\n");
		exit(1);
	}

	write_reg16_reversed(TIMER_PERIOD_REG, PERIOD_VALUE);
	write_reg16_reversed(TIMER_PERSCALER_REG, PRECLAER_VALUE - 1);

}

void motor_set_speed(int speed)
{
  	if (speed < 0) speed = 0;
	if (speed > 100) speed = 100;
	
	for(int i = 0; i < 10; i++)
	{
		int signal_1 = write_reg16_reversed(MOTOR1_PWM_REG, (PERIOD_VALUE * speed) / 100);
		int signal_2 = write_reg16_reversed(MOTOR2_PWM_REG, (PERIOD_VALUE * speed) / 100);
	
		if (signal_1 == 0 && signal_2 == 0) break;

		delay(2);
	}
}	

void motor_set_direction(int forward)
{
	digitalWrite(DIR_PIN_1, forward ? HIGH : LOW);
	digitalWrite(DIR_PIN_2, forward ? LOW : HIGH);
}

void motor_stop(void)
{
	motor_set_speed(0);
}

void input()
{
	keyboard_init();

	int key = 0;

	while(key != 27)
	{
		key = getch();

		switch(key)
		{
			case 119:
				printf("up\n");
				motor_init();
				motor_set_speed(50);
				motor_set_direction(0);
				break;
			case 115:
				printf("down\n");
				motor_init();
				motor_set_speed(50);
				motor_set_direction(1);
				break;
			case 32:
				motor_stop();
				endwin();
				return;

				//printf("stop\n");
				//motor_init();
				//motor_set_speed(0);
				//motor_set_direction(0);
				//break;
		}

		delay(5);
	}
}

int lidar_read_distance_median(int sample)
{
	int values[sample];
	int valid_count = 0;

	for (int i = 0; i < sample; i++) 
	{
        	int d = lidar_read_distance();

        	if (d > 0) {
        	    values[valid_count++] = d;
        	}
        	delay(10);   // 측정 사이 간격 (초음파 잔향 방지)
    	}

    	// 2) 유효 측정이 하나도 없으면 -1
    	if (valid_count == 0) return -1;

    	// 3) 정렬 후 가운데 값 반환
    	qsort(values, valid_count, sizeof(int), compare_int);
    	return values[valid_count / 2];
}

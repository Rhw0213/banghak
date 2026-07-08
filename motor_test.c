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
#define PERIOD_VALUE 849
#define PRECLAER_VALUE 848
#define DIR_PIN_1 23 //BCM NUMBER
#define DIR_PIN_2 24
#define MCU_RST_PIN 5

static int motor_fd;
static int count = 0; 

void keyboard_init();

//int write_reg16_reversed(int fd, int reg, int value);

void mcu_hard_reset(void);
void motor_init(void);
void motor_set_speed(int speed);
void motor_set_direction(int forward);
void motor_stop(void);
void input();

int main(void)
{

	wiringPiSetupGpio();
	//lidar_init();

	int servo_fd = wiringPiI2CSetup(MCU_ADDR);

	servo_init(servo_fd);

	servo_set_angle(-45, servo_fd);


	//mcu_hard_reset();

	//input();

	//int distance = 0; 

	//while(1)
	//{
	//	distance = lidar_read_distance();
	//	delay(100); 
	//}	

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
	wiringPiSetupGpio();
	mcu_hard_reset();

	pinMode(DIR_PIN_1,OUTPUT);
	pinMode(DIR_PIN_2,OUTPUT);

	motor_fd = wiringPiI2CSetup(MCU_ADDR);

	if (motor_fd < 0)
	{
		fprintf(stderr, "it can't connect to I2c\n");
		exit(1);
	}

	write_reg16_reversed(motor_fd, TIMER_PERIOD_REG, PERIOD_VALUE);
	write_reg16_reversed(motor_fd, TIMER_PERSCALER_REG, PRECLAER_VALUE - 1);

}

void motor_set_speed(int speed)
{
  	if (speed < 0) speed = 0;
	if (speed > 100) speed = 100;
	
	for(int i = 0; i < 10; i++)
	{
		int signal_1 = write_reg16_reversed(motor_fd, MOTOR1_PWM_REG, (PERIOD_VALUE * speed) / 100);
		int signal_2 = write_reg16_reversed(motor_fd, MOTOR2_PWM_REG, (PERIOD_VALUE * speed) / 100);
	
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
				printf("stop\n");
				motor_init();
				motor_set_speed(0);
				motor_set_direction(0);
				break;
		}

		delay(5);
	}
}


#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <stdio.h>
#include "servo.h"
#include "common.h"

#define MCU_ADDR 0x14
#define SERVO_CHANNEL 2 
#define SERVO_PWM_REG (0x20 + SERVO_CHANNEL)
#define SERVO_ARR_REG (0x44 + SERVO_CHANNEL/4)
#define SERVO_PSC_REG (0x40 + SERVO_CHANNEL/4)
#define SERVO_PERIOD 4095
#define SERVO_PRESCALER 351


void servo_init(int fd)
{
	write_reg16_reversed(fd, SERVO_ARR_REG, SERVO_PERIOD);
	write_reg16_reversed(fd, SERVO_PSC_REG, SERVO_PRESCALER);
}

void servo_set_angle(float angle, int fd)
{

	if (angle < -90) angle = -90;
	if (angle > 90) angle = 90;
	float pulse_us = 500 + (angle + 90) * (2500 - 500) / 180.0;
	int value = (int)((pulse_us / 20000.0) * SERVO_PERIOD);
	write_reg16_reversed(fd, SERVO_PWM_REG, value);
}

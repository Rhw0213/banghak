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
#define SERVO_OFFSET 	5.0

extern int fd;

void servo_init()
{
	printf("%d", fd);
	write_reg16_reversed(SERVO_ARR_REG, SERVO_PERIOD);
	write_reg16_reversed(SERVO_PSC_REG, SERVO_PRESCALER);
}

void servo_set_angle(float angle)
{
	angle += SERVO_OFFSET;
	if (angle < -90) angle = -90;
	if (angle > 90) angle = 90;
	float pulse_us = 500 + (angle + 90) * (2500 - 500) / 180.0;
	int value = (int)((pulse_us / 20000.0) * SERVO_PERIOD);
	write_reg16_reversed(SERVO_PWM_REG, value);
}

void servo_set_angle1(float angle) {
    angle += SERVO_OFFSET;
    if (angle < -90) angle = -90;
    if (angle > 90) angle = 90;
    float pulse_us = 500 + (angle + 90) * (2500 - 500) / 180.0;
    int value = (int)((pulse_us / 20000.0) * SERVO_PERIOD);
    int ret = write_reg16_reversed(SERVO_PWM_REG, value);
    printf("value=%d reg=0x%02X ret=%d\n", value, SERVO_PWM_REG, ret);
}

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

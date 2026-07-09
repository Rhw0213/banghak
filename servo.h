#ifndef SERVO_H
#define SERVO_H

#define MCU_ADDR 0x14

void servo_init();
void servo_set_angle(float angle);
void servo_set_angle1(float angle);

#endif

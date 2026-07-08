#ifndef SERVO_H
#define SERVO_H

#define MCU_ADDR 0x14

void servo_init(int fd);
void servo_set_angle(float angle, int fd);

#endif

#include "common.h"
#include <wiringPi.h>
#include <wiringPiI2C.h>

int fd;

int write_reg16_reversed(int reg, int value)
{
        int swapped_value = ((value >> 8) & 0x00FF) | ((value << 8) & 0xFF00);
        return wiringPiI2CWriteReg16(fd, reg, swapped_value);
}

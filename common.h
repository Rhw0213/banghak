#ifndef COMMON_H
#define COMMON_H
#include <stdlib.h>

int write_reg16_reversed(int reg, int value);
static int compare_int(const void *a, const void *b)
{
	return (*(int*)a - *(int*)b);
} 
extern int fd;

#endif

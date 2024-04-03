#include <stdio.h>
#include "stm32f0xx.h"

#define GYRO_ADDR 0x69
#define WHO_AM_I 0x0F
#define CTRL_REG1 0x20
#define OUT_X 0xA8
#define OUT_Y 0xAA
#define OUT_Z 0xAC

void i2c_init(void);
void gyro_xyz_enable(void);
int16_t get_gyro_x(void);
int16_t get_gyro_y(void);
int16_t get_gyro_z(void);
#include <stdio.h>
#include "stm32f0xx.h"

#define GYRO_ADDR 0x69
#define WHO_AM_I 0x0F
#define CTRL_REG1 0x20
#define OUT_X_LOW 0x28
#define OUT_X_HIGH 0x29
#define OUT_Y_LOW 0x2A
#define OUT_Y_HIGH 0x2B
#define OUT_Z_LOW 0x2C
#define OUT_Z_HIGH 0x2D

void i2c_init(void);
void i2c_WriteReg(uint16_t deviceAdd, uint8_t regAdd, uint8_t data);
int8_t i2c_ReadReg(uint16_t deviceAdd);
void i2c_SetRegAdd(uint16_t deviceAdd, uint8_t regAdd);
int16_t get_gyro_x(void);
int16_t get_gyro_y(void);
int16_t get_gyro_z(void);
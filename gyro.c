#include "gyro.h"

uint8_t NUM_BYTES = 2;

void i2c_init(void) {
		// initialize clocks
		RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
		RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
		RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;
		// initialize digital outputs PB14 and PC0
		GPIOB->MODER |= (1 << 28);
		GPIOB->OTYPER &= ~(1 << 14);
		GPIOC->MODER |= (1 << 0);
		GPIOC->OTYPER &= ~(1 << 0);
		// initialize I2C2
		// SDA: GPIOB 11 -- AF1
		// SCL: GPIOB 13 -- AF5
		GPIOB->MODER |= (1 << 23) | (1 << 27);
		GPIOB->OTYPER |= (1 << 11) | (1 << 13);
		GPIOB->AFR[1] |= (1 << 12) | (1 << 20) | (1 << 22);
		/* Configure I2C2 timing
		 * Prescaler = 1
		 * SCLDEL = 0x4
		 * SDADEL = 0x2
		 * SCLH = 0x0F
		 * SCLL = 0x13
		 */
		I2C2->TIMINGR |= 0x10420F13;
		// enable I2C2
		I2C2->CR1 |= (1 << 0);
		// set PC0 and PB14 high
		GPIOB->ODR |= (1 << 14);
		GPIOC->ODR |= (1 << 0);
}

void i2c_SetRegAdd(uint16_t deviceAdd, uint8_t regAdd) {
		I2C2->CR2 = 0;  // clear register
		// Use SADD[7:1] bit field in CR2 register to set slave address to addr
		I2C2->CR2 |= (deviceAdd << 1);
		// Use NBYTES[7:0] bit field to set number of data bytes to be transmitted to
		// 1
		I2C2->CR2 |= (0x1 << 16);
		// Set RD_WRN to WRITE operation - 0 indicates WRITE
		I2C2->CR2 &= ~(1 << 10);
		// Set START bit to begin the address frame
		I2C2->CR2 |= I2C_CR2_START;
		// While TXIS or NACKF flags not set wait
		while (!(I2C2->ISR & (I2C_ISR_TXIS | I2C_ISR_NACKF))) {
		}  // getting stuck here on second call to this function!
		// Once TXIS flag set continue
		// Check if NACK set
		if (I2C2->ISR & I2C_ISR_NACKF) {
			// GPIOC->ODR |= GPIO_ODR_6; // RED - I2C not working!
		}
		// Write data into the TXDR
		I2C2->TXDR = regAdd;
		// Wait until TC flag set - transfer complete
		while (!(I2C2->ISR & I2C_ISR_TC)) {
		}
}

void i2c_WriteReg(uint16_t deviceAdd, uint8_t regAdd, uint8_t data) {
		I2C2->CR2 = 0;  // clear register
		// Use SADD[7:1] bit field in CR2 register to set slave address to addr
		I2C2->CR2 |= (deviceAdd << 1);
		// Use NBYTES[7:0] bit field to set number of data bytes to be transmitted to
		// 1
		I2C2->CR2 |= (0x2 << 16);
		// Set RD_WRN to WRITE operation - 0 indicates WRITE
		I2C2->CR2 &= ~(1 << 10);
		// Set START bit to begin the address frame
		I2C2->CR2 |= I2C_CR2_START;
		// While TXIS or NACKF flags not set wait
		while (!(I2C2->ISR & (I2C_ISR_TXIS | I2C_ISR_NACKF))) {
		}  // getting stuck here on second call to this function!
		// Once TXIS flag set continue
		// Check if NACK set
		if (I2C2->ISR & I2C_ISR_NACKF) {
			// GPIOC->ODR |= GPIO_ODR_6; // RED - I2C not working!
		}
		// Set reg address
		I2C2->TXDR = regAdd;
		while (!(I2C2->ISR & I2C_ISR_TXIS)) {
		}
		// Write data into the TXDR
		I2C2->TXDR = data;
		// Wait until TC flag set - transfer complete
		while (!(I2C2->ISR & I2C_ISR_TC)) {
		}
}

int8_t i2c_ReadReg(uint16_t deviceAdd) {
		I2C2->CR2 = 0;  // clear register
		int8_t data = 0;
		// SADD[7:1] bit field in CR2 register to set slave address to L3GD20
		I2C2->CR2 |= (deviceAdd << 1);
		//  NBYTES[7:0] bit field to set number of data bytes to be transmitted to 1
		I2C2->CR2 |= (0x1 << 16);
		// RD_WRN to READ operation - 1 indicates READ
		I2C2->CR2 |= (1 << 10);
		// START bit to begin the address frame
		I2C2->CR2 |= I2C_CR2_START;
		// While RXNE or NACKF flags not set wait
		while (!(I2C2->ISR & (I2C_ISR_RXNE | I2C_ISR_NACKF))) {
		}
		// Once RXNE flag set continue
		// Check if NACK set
		if (I2C2->ISR & I2C_ISR_NACKF) {
			GPIOC->ODR |= GPIO_ODR_8;  // ORANGE - I2C not working!
		}
		// Wait for TC flag set
		while (!(I2C2->ISR & I2C_ISR_TC)) {
		}
		// Read contents of RXDR register and return data - remember it is 1 byte at a
		// time
		data = I2C2->RXDR;
		return data;
}

int16_t get_gyro_x(void) {
    i2c_SetRegAdd(GYRO_ADDR, OUT_X_LOW);
    int8_t xlow = i2c_ReadReg(GYRO_ADDR);
    i2c_SetRegAdd(GYRO_ADDR, OUT_X_HIGH);
    int8_t xhigh = i2c_ReadReg(GYRO_ADDR);
    int16_t xd = ((int16_t)xhigh << 8) | (uint8_t)xlow;
		return xd;
}

int16_t get_gyro_y(void) {
    i2c_SetRegAdd(GYRO_ADDR, OUT_Y_LOW);
    int8_t ylow = i2c_ReadReg(GYRO_ADDR);
    i2c_SetRegAdd(GYRO_ADDR, OUT_Y_HIGH);
    int8_t yhigh = i2c_ReadReg(GYRO_ADDR);
    int16_t yd = ((int16_t)yhigh << 8) | (uint8_t)ylow;
		return yd;
}

int16_t get_gyro_z(void) {
    i2c_SetRegAdd(GYRO_ADDR, OUT_Z_LOW);
    int8_t zlow = i2c_ReadReg(GYRO_ADDR);
    i2c_SetRegAdd(GYRO_ADDR, OUT_Z_HIGH);
    int8_t zhigh = i2c_ReadReg(GYRO_ADDR);
    int16_t zd = ((int16_t)zhigh << 8) | (uint8_t)zlow;
		return zd;
}
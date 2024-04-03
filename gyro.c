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

void gyro_xyz_enable(void) {
		I2C2->CR2 &= ~(0xFF << 1) & ~(0xFF << 16); // clear address and number of bytes
		I2C2->CR2 |= (NUM_BYTES << 16) | (GYRO_ADDR << 1); // set number of bytes in transaction and slave address (shifted 1 for 7-bit addresses)
		I2C2->CR2 &= ~(1 << 10); // R/W bit set to 0 (write)
		I2C2->CR2 |= (1 << 13); // start
		while((I2C2->ISR & (1 << 1)) == 0) { // wait for TXIS to be set
			if(I2C2->ISR & (1 << 4)) { // check for NACK
			}
		}
		I2C2->TXDR = CTRL_REG1;
		while((I2C2->ISR & (1 << 1)) == 0) { // wait for TXIS to be set
			if(I2C2->ISR & (1 << 4)) { // check for NACK
			}
		}
		I2C2->TXDR = 0x0F;
		while(!(I2C2->ISR & (1 << 6))) {} // wait for transaction to complete
		I2C2->CR2 |= (1 << 14); // stop
}

int16_t get_gyro_x(void) {
		uint16_t data1, data2;
		int16_t fullData;
		I2C2->CR2 &= ~(0xFF << 1) & ~(0xFF << 16); // clear address and number of bytes
		I2C2->CR2 |= (1 << 16) | (GYRO_ADDR << 1); // set number of bytes in transaction and slave address (shifted 1 for 7-bit addresses)
		I2C2->CR2 &= ~(1 << 10); // R/W bit set to 0 (write)
		I2C2->CR2 |= (1 << 13); // start
		while((I2C2->ISR & (1 << 1)) == 0) { // wait for TXIS to be set
			if(I2C2->ISR & (1 << 4)) { // check for NACK
			}
		}
		I2C2->TXDR = OUT_X;
		while(!(I2C2->ISR & (1 << 6))) {} // wait for transaction to complete
		I2C2->CR2 &= ~(0xFF << 1) & ~(0xFF << 16); // clear address and number of bytes
		I2C2->CR2 |= (NUM_BYTES << 16) | (GYRO_ADDR << 1); // set number of bytes in transaction and slave address (shifted 1 for 7-bit addresses)
		I2C2->CR2 |= (1 << 10); // set R/W bit to 1 (read)
		I2C2->CR2 |= (1 << 13); // start
		while((I2C2->ISR & (1 << 2)) == 0) { // wait for RXNE to be set
			if(I2C2->ISR & (1 << 4)) { // check for NACK
			}
		}
		data1 = I2C2->RXDR; // collect byte 1
		while((I2C2->ISR & (1 << 2)) == 0) { // wait for RXNE to be set
			if(I2C2->ISR & (1 << 4)) { // check for NACK
			}
		}
		data2 = I2C2->RXDR; // collect byte 2
		I2C2->CR2 |= (1 << 14); // stop
		fullData = data1 | (data2 << 8); // concatenate both bytes into one string of 16 signed bits
		return fullData;
}

int16_t get_gyro_y(void) {
		uint16_t data1, data2;
		int16_t fullData;
		I2C2->CR2 &= ~(0xFF << 1) & ~(0xFF << 16); // clear address and number of bytes
		I2C2->CR2 |= (1 << 16) | (GYRO_ADDR << 1); // set number of bytes in transaction and slave address (shifted 1 for 7-bit addresses)
		I2C2->CR2 &= ~(1 << 10); // R/W bit set to 0 (write)
		I2C2->CR2 |= (1 << 13); // start
		while((I2C2->ISR & (1 << 1)) == 0) { // wait for TXIS to be set
			if(I2C2->ISR & (1 << 4)) { // check for NACK
			}
		}
		I2C2->TXDR = OUT_Y;
		while(!(I2C2->ISR & (1 << 6))) {} // wait for transaction to complete
		I2C2->CR2 &= ~(0xFF << 1) & ~(0xFF << 16); // clear address and number of bytes
		I2C2->CR2 |= (NUM_BYTES << 16) | (GYRO_ADDR << 1); // set number of bytes in transaction and slave address (shifted 1 for 7-bit addresses)
		I2C2->CR2 |= (1 << 10); // set R/W bit to 1 (read)
		I2C2->CR2 |= (1 << 13); // start
		while((I2C2->ISR & (1 << 2)) == 0) { // wait for RXNE to be set
			if(I2C2->ISR & (1 << 4)) { // check for NACK
			}
		}
		data1 = I2C2->RXDR; // collect byte 1
		while((I2C2->ISR & (1 << 2)) == 0) { // wait for RXNE to be set
			if(I2C2->ISR & (1 << 4)) { // check for NACK
			}
		}
		data2 = I2C2->RXDR; // collect byte 2
		I2C2->CR2 |= (1 << 14); // stop
		fullData = data1 | (data2 << 8); // concatenate both bytes into one string of 16 signed bits
		return fullData;
}

int16_t get_gyro_z(void) {
		uint16_t data1, data2;
		int16_t fullData;
		I2C2->CR2 &= ~(0xFF << 1) & ~(0xFF << 16); // clear address and number of bytes
		I2C2->CR2 |= (1 << 16) | (GYRO_ADDR << 1); // set number of bytes in transaction and slave address (shifted 1 for 7-bit addresses)
		I2C2->CR2 &= ~(1 << 10); // R/W bit set to 0 (write)
		I2C2->CR2 |= (1 << 13); // start
		while((I2C2->ISR & (1 << 1)) == 0) { // wait for TXIS to be set
			if(I2C2->ISR & (1 << 4)) { // check for NACK
			}
		}
		I2C2->TXDR = OUT_Z;
		while(!(I2C2->ISR & (1 << 6))) {} // wait for transaction to complete
		I2C2->CR2 &= ~(0xFF << 1) & ~(0xFF << 16); // clear address and number of bytes
		I2C2->CR2 |= (NUM_BYTES << 16) | (GYRO_ADDR << 1); // set number of bytes in transaction and slave address (shifted 1 for 7-bit addresses)
		I2C2->CR2 |= (1 << 10); // set R/W bit to 1 (read)
		I2C2->CR2 |= (1 << 13); // start
		while((I2C2->ISR & (1 << 2)) == 0) { // wait for RXNE to be set
			if(I2C2->ISR & (1 << 4)) { // check for NACK
			}
		}
		data1 = I2C2->RXDR; // collect byte 1
		while((I2C2->ISR & (1 << 2)) == 0) { // wait for RXNE to be set
			if(I2C2->ISR & (1 << 4)) { // check for NACK
			}
		}
		data2 = I2C2->RXDR; // collect byte 2
		I2C2->CR2 |= (1 << 14); // stop
		fullData = data1 | (data2 << 8); // concatenate both bytes into one string of 16 signed bits
		return fullData;
}
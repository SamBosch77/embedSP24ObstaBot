/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <stdlib.h>
#include "main.h"
#include "motor.h"
#include "gyro.h"
//#include "ultrasonic.h"
#include "stm32f0xx.h"

/*
USED PINS:
PC6-PC9 - LEDs

PC6 - M1 PWM signal (TIM3)
PA0,PA1 - M1 directional control pins
PC7 - M2 PWM signal (TIM3)
PA5,PA6 - M2 directional control pins

PC1 - ADC_IN7 Rangefinder analog input

PA4 - Servo PWM signal (TIM14 CH1) (duty cycle: 2-left, 6-front, 11-right)
*/

/*
KNOWNS:
- A 3V PWM signal can be used to drive the motors
- 3V output from board to motor driver IN1,IN2, etc pins is okay
- Motors can be driven at 5V successfully
- Motor encoder VCC must be > 3.4V to measure successfully
- Pull ups resistors likely needed for encoders since not soldered on board

UNKNOWNS:
- Encoder sampling frequency defined on chip or in software?
- Ultrasonic ranging procedure
- PWM signal
*/

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

volatile uint32_t debouncer;
volatile uint32_t encoder_count = 0;
int16_t gyro_x = 0;
int16_t gyro_y = 0;
int16_t gyro_z = 0;
int32_t threshold = 2000;
float distance = 0;
volatile int32_t xIntegral = 0;
//volatile uint8_t duty_cycle = 0;
volatile char newData;
volatile uint32_t ndFlag = 0;
uint8_t dirFlag = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Define character transmission function
void transmitChar(char input)
{
	uint32_t txeMask = 1 << 7;
	while ((USART3->ISR & txeMask) == 0)
	{
		// Do nothing while transmit register is not empty
	}
	// Transmit register is now empty, write to register
	USART3->TDR = input;
	
}

char readChar()
{
	char val;
	uint32_t rxneMask = 1 << 5;
	while((USART3->ISR & rxneMask) == 0)
	{
		// Don't do anything while recieve register is empty
	}
	val = USART3->RDR;
	return val;
}

void transmitStr(char *input)
{
	uint32_t i = 0;
	while(input[i] != '\0') 
	{
		transmitChar(input[i]);
		i = i + 1;
	}
	transmitChar('\n');
	transmitChar('\r');
}

void USART3_4_IRQHandler()
{
	newData = (USART3->RDR & 0xFF); // Only take needed bits, not reserved ones
	ndFlag = 1;
}

void uint16ToStr(int input, char *buffer)
{
	// Max string length of 2^16 = 65536 is 5 digits, plus 1 for null terminator
	snprintf(buffer, 6, "%d", input);
}

void LED_init(void) {
    // Initialize PC8 and PC9 for LED's
    RCC->AHBENR |= RCC_AHBENR_GPIOCEN;                                          // Enable peripheral clock to GPIOC
    GPIOC->MODER |= (0x55 << 12); // 01 01 01 01 << 12 red, blue, orange, green general purpose output
    GPIOC->OTYPER &= ~(0xF << 6); // ~(1 1 1 1) << 6 Push-pull output
		GPIOC->OSPEEDR &= ~(0xFF << 12); // ~(11 11 11 11) << 12 low speed
		GPIOC->PUPDR &= ~(0xFF << 12); // ~(11 11 11 11) << 12 no pull-up pull-down;
    GPIOC->ODR &= ~(0xF << 6); // write 0000 to pins 6-9
}

void  button_init(void) {
    // Initialize PA0 for button input
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;                                          // Enable peripheral clock to GPIOA
    GPIOA->MODER &= ~(GPIO_MODER_MODER0_0 | GPIO_MODER_MODER0_1);               // Set PA0 to input
    GPIOC->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEEDR0_0 | GPIO_OSPEEDR_OSPEEDR0_1);     // Set to low speed
    GPIOC->PUPDR |= GPIO_PUPDR_PUPDR0_1;                                        // Set to pull-down
}

/* Called by SysTick Interrupt
 * Performs button debouncing, changes wave type on button rising edge
 * Updates frequency output from ADC value
 */
void HAL_SYSTICK_Callback(void) {
    // Remember that this function is called by the SysTick interrupt
    // You can't call any functions in here that use delay

    debouncer = (debouncer << 1);
    if(GPIOA->IDR & (1 << 0)) {
        debouncer |= 0x1;
    }

    if(debouncer == 0x7FFFFFFF) {
    switch(target_rpm) {
        case 80:
            target_rpm = 50;
            break;
        case 50:
            target_rpm = 81;
            break;
        case 0:
            target_rpm = 80;
            break;
        default:
            target_rpm = 0;
            break;
        }
    }
}

	void turnRight()
	{
		GPIOA->ODR &= ~(0x3 << 0) & ~(0x3 << 5);
		GPIOA->ODR |= (0x1 << 0) | (0x2 << 5);
		pwm_setDutyCycleLeft(24);
		pwm_setDutyCycleRight(20);
	  HAL_Delay(700);
		GPIOA->ODR &= ~(0x3 << 0) & ~(0x3 << 5);
		GPIOA->ODR |= (0x1 << 0) | (0x1 << 5);
		pwm_setDutyCycleLeft(0);
		pwm_setDutyCycleRight(0);
	}
		void turnLeft()
	{
		GPIOA->ODR &= ~(0x3 << 0) & ~(0x3 << 5);
		GPIOA->ODR |= (0x2 << 0) | (0x1 << 5);
		pwm_setDutyCycleLeft(24);
		pwm_setDutyCycleRight(20);
	  HAL_Delay(700);
		GPIOA->ODR &= ~(0x3 << 0) & ~(0x3 << 5);
		GPIOA->ODR |= (0x1 << 0) | (0x1 << 5);
		pwm_setDutyCycleLeft(0);
		pwm_setDutyCycleRight(0);
	}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
	volatile uint8_t dist[3] = {0,0,0};
	
//  button_init();                          // Initialize button
//  motor_init();

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  /* USER CODE BEGIN 2 */

	// Enable needed peripheral clocks
	RCC->AHBENR |= (1 << 19); // Enable GPIOC clock
	RCC->AHBENR |= (1 << 17); // Enable GPIOA clock
	// RCC->AHBENR |= (1 << 18); // Enable GPIOB clock
	RCC->APB2ENR |= (1 << 9); // Enable ADC clock
	RCC->APB1ENR |= (0x1 << 18); // (1) enable USART3 clock for serial debugging
	RCC->APB1ENR |= (1 << 8); // Enable TIM14 clock
	
	// USART Debugging Config
	GPIOC->MODER |= (0xA << 20); // (10 10) Use PC10 TX and PC11 RX in alt func mode
	USART3->BRR = 69; // OVER8 = 0, so 69 -> 115942 baud rate -> 0.64% error
	GPIOC->AFR[1] |= (0x11 << 8); // (0001 0001) select USART3_TX for PC10, USART3_RX for PC11
	// Enable USART (do this last)
	USART3->CR1 |= (0x1 << 2); // Enable reciever
	USART3->CR1 |= (0x1 << 3); // Enable transmitter
	USART3->CR1 |= (0x1 << 5); // Enable USART RXNE interrupts
	USART3->CR1 |= (0x1); // Set UE to enable USART

	NVIC_EnableIRQ(USART3_4_IRQn); // 29
	NVIC_SetPriority(USART3_4_IRQn, 1); // 29
	
	// LED Config
	GPIOC->MODER |= (0x5 << 16); // Set PC8, PC9 to 01 - general purpose output
	GPIOC->OTYPER &= ~(0x3 << 8); // Set output type to push-pull (0 0)
	GPIOC->OSPEEDR &= ~(0xF << 16); // Set output speed to low (00 00)
	GPIOC->PUPDR &= ~(0xF << 16); // Set to no pull-up/pull-down (00 00)
	
	// Rangefinder Config
	// Use PC1 as analog input ADC_IN10
	GPIOC->MODER |= (0x3 << 2); // Set PC1 mode to  11 - analog input mode
	GPIOC->PUPDR &= ~(0x3 << 2); // Set PC1 PUPDR to 00 - no pull-up/pull-down
	// Configure ADC options:
	// CFGR1 RES: 10 - 8 bits
	// CFGR1 CONT: 1 - continuous conversion mode
	// CFGR1 EXTEN: 00 - hardware trigger detection disabled
	// CHSELR CHSEL11: 1 - select channel 11 for conversion
	ADC1->CFGR1 |= (0x2 << 3); // Set bit resolution to 10 - 8 bits
	ADC1->CFGR1 |= (0x1 << 13); // Set single/continuous conversion mode to 1 - continuous conversion;
	ADC1->CFGR1 &= ~(0x3 << 10); // Set external trigger/polarity selection to 00 - hardware trigger detection disabled
	// Select ADC channel for conversion
	ADC1->CHSELR |= (0x1 << 11); // Select channel ADC_IN11
	// Calibrate ADC
	ADC1->CR |= (0x1 << 31); // Set ADCAL
	while((ADC1->CR & (0x1 << 31)) != 0)
	{
		// ADCAL is 1, Do nothing while waiting for calibration
	}
	volatile uint16_t calibFactor = ADC1->DR;

	// Enable ADC
	ADC1->CR |= (0x1 << 0); // Set ADEN
	
	while((ADC1->ISR & (0x1 << 0)) == 0)
	{
		// ADRDY is 0, wait for ready
	}
	// Start ADC
	ADC1->CR |= (0x1 << 2); // Set ADSTART
	
	
	// Init motors
	pwm_init();
	
	// initialize I2C bus with gyro and enable x,y,z axes
	i2c_init();
	i2c_WriteReg(GYRO_ADDR,CTRL_REG1,0xF);

	//uart_init();

	debouncer = 0;
	duty_cycle = 25;
	pwm_setDutyCycleLeft(duty_cycle);
	pwm_setDutyCycleRight(duty_cycle);
	
	// Servo setup
	GPIOA->MODER |= (2 << 8); // PA4 alternate function mode
	GPIOA->AFR[0] |= (4 << 16); // Select PA4 alternate function 4 (0100) - TIM14 CH1
	TIM14->ARR = 2000;
	TIM14->PSC = 79;
	TIM14->CCMR1 &= ~(1 << 0) & ~(1 << 1); // Set CCS1 and CCS2 to output
	TIM14->CCMR1 |= (1 << 5) | (1 << 6); // set channel 1 to PWM mode 1
	TIM14->CCMR1 &= ~(1 << 4); // clearing low order bit to select PWM mode 1
	TIM14->CCMR1 |= (1 << 3); // enable output compare preload for channel 1 and 2
	TIM14->CCER |= (1 << 0); // set output enable bits for channel 1 and 2
	TIM14->CR1 |= (1 << 0); // enable clock
			// Set servo duty cycle
		
	
	// Initialize useful variables
	volatile uint16_t adcOutput = 0;
	volatile uint16_t prevADC = 0;
	
	
	// Start at 50 percent duty cycle
    
  /* USER CODE END 2 */

  /* Infinite loop */
	while (1) {
		TIM14->CCR1 = ((uint32_t)6*TIM14->ARR)/100;
		// Wait for end of conversion
		while((ADC1->ISR & (0x1 << 2)) == 0)
		{
			// EOC is 0, wait for conversion to end
		}

		dist[1] = ADC1->DR; // Read DR (should reset EOC)
		char uint16Buffer[6];
		uint16ToStr(adcOutput, uint16Buffer);
		transmitStr(uint16Buffer);
		
		
		if (dist[1] >= 3){
			pwm_setDutyCycleLeft(0);
			pwm_setDutyCycleRight(0);
			HAL_Delay(100);
					// Set servo duty cycle
			dist[1]= ADC1->DR;
			TIM14->CCR1 = ((uint32_t)11*TIM14->ARR)/100;
			HAL_Delay(500);
			dist[2] = ADC1->DR;
			TIM14->CCR1 = ((uint32_t)2*TIM14->ARR)/100;
			HAL_Delay(500);
				dist[0] = ADC1->DR;
			TIM14->CCR1 = ((uint32_t)6*TIM14->ARR)/100;
			
			if(dist[0]>2)
			{
				turnRight();
				dirFlag = 0;
			}
		
			else if(dist[2]>2)
			{
				turnLeft();
				dirFlag = 1;
			}
			else {
				if(dirFlag == 0) {
					turnRight();
				}
				else if(dirFlag == 1) {
					turnLeft();
				}
			}

			continue;
		} else
		{
			pwm_setDutyCycleLeft(24);
			pwm_setDutyCycleRight(20);
		}
		
		gyro_x = get_gyro_x();
		gyro_y = get_gyro_y();
		xIntegral += gyro_x;
		
		
		//distance = getDistance() / 1000;

//    GPIOC->ODR &= ~(GPIO_ODR_7 | GPIO_ODR_6 | GPIO_ODR_8 |
//                    GPIO_ODR_9);  // Reset the ODR bits for LEDs

//    if (gyro_y > threshold) 
//		{
//      GPIOC->ODR |= GPIO_ODR_6;  // Red LED for positive Y
//    } 
//		else if (gyro_y < -threshold) 
//		{
//      GPIOC->ODR |= GPIO_ODR_7;  // Blue LED for negative Y
//    }

//    if (gyro_x > threshold) 
//		{
//      pwm_setDutyCycleLeft(duty_cycle+10);  // Green LED for positive X
//			pwm_setDutyCycleRight(duty_cycle+10);
//    } 
//		else if (gyro_x < -threshold) 
//		{
//      pwm_setDutyCycleLeft(duty_cycle-10);  // Orange LED for negative X
//			pwm_setDutyCycleRight(duty_cycle-10);
//    }
//		else {
//			pwm_setDutyCycleLeft(duty_cycle);
//			pwm_setDutyCycleRight(duty_cycle);
//		}
		


    HAL_Delay(100);
  }	
		

		
		// Test IN1, IN2, IN3, IN4 combos
		// Clear all LEDs
		// GPIOC->ODR &= ~(0xF << 6); // write 0000 to pins 6-9
		
		
		
		// Set RED HI, BLU LO (M1 - Forward) for 1 second
		// GPIOC->ODR |= (1 << 6); // set red
		// GPIOC->ODR &= ~(1 << 7); // clear blue
		
		//pwm_setDutyCycle(duty_cycle); // TIM14 pin
		
		// Set ORANGE HI, GREEN LO (M2 - Forward)
		// GPIOC->ODR |= (1 << 8); // set orange
		// GPIOC->ODR &= ~(1 << 9); // clear green
		
//		HAL_Delay(1000); // Wait a second
//		
//		
//		// Stop the motors and wait a second
//		GPIOC->ODR &= ~(0xF << 6); // write 0000 to pins 6-9
//		HAL_Delay(1000);
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

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
PA4 - M1 PWM signal (TIM14)
PA5 - M1 directional control pins
PA6 - M2 PWM signal
 - M2 directional control pins
 - Enc1 timer
 - Enc1 sampling pin
 - Enc2 timer
 - Enc2 sampling pin
 - Ultrasonic 
PB15 - gyro MOSI (SPI2)
PB14 - gyro MISO (SPI2)
PB13 - gyro SCLK (SPI2)
PC0  - gyro mode select/CS -- drive low for SPI communication enable
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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
	
	// LED Config
	//LED_init();
	
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
	
	// Start at 50 percent duty cycle
    
  /* USER CODE END 2 */

  /* Infinite loop */
	while (1) {

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

    if (gyro_x > threshold) 
		{
      pwm_setDutyCycleLeft(duty_cycle+10);  // Green LED for positive X
			pwm_setDutyCycleRight(duty_cycle+10);
    } 
		else if (gyro_x < -threshold) 
		{
      pwm_setDutyCycleLeft(duty_cycle-10);  // Orange LED for negative X
			pwm_setDutyCycleRight(duty_cycle-10);
    }
		else {
			pwm_setDutyCycleLeft(duty_cycle);
			pwm_setDutyCycleRight(duty_cycle);
		}

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

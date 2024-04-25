/* -------------------------------------------------------------------------------------------------------------
 *  Motor Control and Initialization Functions
 * -------------------------------------------------------------------------------------------------------------
 */
#include "motor.h"

volatile int16_t error_integral = 0;    // Integrated error signal
volatile uint8_t duty_cycle = 0;    	// Output PWM duty cycle
volatile int16_t target_rpm = 0;    	// Desired speed target
volatile int16_t motor_speed = 0;   	// Measured motor speed
volatile int8_t adc_value = 0;      	// ADC measured motor current
volatile int16_t error = 0;         	// Speed error signal
volatile uint8_t Kp = 1;            	// Proportional gain
volatile uint8_t Ki = 1;            	// Integral gain

// Sets up the entire motor drive system
void motor_init(void) {
    pwm_init();
    encoder_init();
    ADC_init();
}

// Sets up the PWM and direction signals to drive the H-Bridge
void pwm_init(void) {
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

    // Set up (PA5, PA6) and (PA0, PA1) as GPIO output pins for motor direction control
    GPIOA->MODER &= 0xFFFF33FF; // clear PA5, PA7 bits,
    GPIOA->MODER |= (1 << 10) | (1 << 14);
		GPIOA->MODER &= 0xFFFFFFF0; // clear PA0, PA1
		GPIOA->MODER |= (1 << 0) | (1 << 2);
    //Initialize one direction pin to high, the other low
    GPIOA->ODR |= (1 << 5);
    GPIOA->ODR &= ~(1 << 7);
		GPIOA->ODR |= (1 << 0);
		GPIOA->ODR &= ~(1 << 1);

		RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
		RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
		GPIOC->MODER |= (1 << 15); // set blue LED (PC7) to alt. function
		GPIOC->MODER |= (1 << 13); // set red LED (PC6) to alt. function
		GPIOC->AFR[0] &= ~(0xFF000000);
		TIM3->ARR = 100; // 100
		TIM3->PSC |= 0x63; // 99
		TIM3->CCMR1 &= ~(1 << 0) & ~(1 << 1) & ~(1 << 8) & ~(1 << 9); // Set CCS1 and CCS2 to output
		TIM3->CCMR1 |= (1 << 5) | (1 << 6); // set channel 1 to PWM mode 1
		TIM3->CCMR1 &= ~(1 << 4);
		TIM3->CCMR1 &= ~(1 << 12); // set channel 2 to PWM mode 1
		TIM3->CCMR1 |= (1 << 13) | (1 << 14);
		TIM3->CCMR1 |= (1 << 3) | (1 << 11); // enable output compare preload for channel 1 and 2
		TIM3->CCER |= (1 << 0) | (1 << 4); // set output enable bits for channel 1 and 2
		TIM3->CCR1 = 0; // 20% of ARR for CH1 (red - 6)
		TIM3->CCR2 = 0; // 20% of ARR for CH2 (blue - 7)
		TIM3->CR1 |= (1 << 0); // enable clock
}

// Set the duty cycle of the PWM, accepts (0-100)
void pwm_setDutyCycleLeft(uint8_t duty) {
    if(duty <= 100) {
        TIM3->CCR1 = duty;//((uint32_t)duty*TIM15->ARR)/100;  // Use linear transform to produce CCR1 value
        // (CCR1 == "pulse" parameter in PWM struct used by peripheral library)
    }
}

void pwm_setDutyCycleRight(uint8_t duty) {
    if(duty <= 100) {
        TIM3->CCR2 = duty;//((uint32_t)duty*TIM15->ARR)/100;  // Use linear transform to produce CCR2 value
        // (CCR2 == "pulse" parameter in PWM struct used by peripheral library)
    }
}

// Sets up encoder interface to read motor speed
void encoder_init(void) {
    
    // Set up encoder input pins (TIMER 3 CH1 and CH2)
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;

    GPIOB->MODER &= ~(GPIO_MODER_MODER4_0 | GPIO_MODER_MODER5_0);
    GPIOB->MODER |= (GPIO_MODER_MODER4_1 | GPIO_MODER_MODER5_1);
    GPIOB->AFR[0] |= ( (1 << 16) | (1 << 20));

    // Set up encoder interface (TIM3 encoder input mode)
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
    TIM3->CCMR1 = 0;
    TIM3->CCER = 0;
    TIM3->SMCR = 0;
    TIM3->CR1 = 0;

    TIM3->CCMR1 |= (TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_0);   // TI1FP1 and TI2FP2 signals connected to CH1 and CH2
    TIM3->SMCR |= (TIM_SMCR_SMS_1 | TIM_SMCR_SMS_0);        // Capture encoder on both rising and falling edges
    TIM3->ARR = 0xFFFF;                                     // Set ARR to top of timer (longest possible period)
    TIM3->CNT = 0x7FFF;                                     // Bias at midpoint to allow for negative rotation
    // (Could also cast unsigned register to signed number to get negative numbers if it rotates backwards past zero
    //  just another option, the mid-bias is a bit simpler to understand though.)
    TIM3->CR1 |= TIM_CR1_CEN;                               // Enable timer

    // Configure a second timer (TIM6) to fire an ISR on update event
    // Used to periodically check and update speed variable
    RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;
    
    // Select PSC and ARR values that give an appropriate interrupt rate
    TIM6->PSC = 11;
    TIM6->ARR = 30000;
    
    TIM6->DIER |= TIM_DIER_UIE;             // Enable update event interrupt
    TIM6->CR1 |= TIM_CR1_CEN;               // Enable Timer

    NVIC_EnableIRQ(TIM6_DAC_IRQn);          // Enable interrupt in NVIC
    NVIC_SetPriority(TIM6_DAC_IRQn,2);
}

// Encoder interrupt to calculate motor speed, also manages PI controller
void TIM6_DAC_IRQHandler(void) {
    /* Calculate the motor speed in raw encoder counts
     * Note the motor speed is signed! Motor can be run in reverse.
     * Speed is measured by how far the counter moved from center point
     */
    motor_speed = (TIM3->CNT - 0x7FFF);
    TIM3->CNT = 0x7FFF; // Reset back to center point
    
    // Call the PI update function
    PI_update();

    TIM6->SR &= ~TIM_SR_UIF;        // Acknowledge the interrupt
}

void ADC_init(void) {

    // Configure PA1 for ADC input (used for current monitoring)
    GPIOA->MODER |= (GPIO_MODER_MODER1_0 | GPIO_MODER_MODER1_1);

    // Configure ADC to 8-bit continuous-run mode, (asynchronous clock mode)
    RCC->APB2ENR |= RCC_APB2ENR_ADCEN;

    ADC1->CFGR1 = 0;                        // Default resolution is 12-bit (RES[1:0] = 00 --> 12-bit)
    ADC1->CFGR1 |= ADC_CFGR1_CONT;          // Set to continuous mode
    ADC1->CHSELR |= ADC_CHSELR_CHSEL1;      // Enable channel 1

    ADC1->CR = 0;
    ADC1->CR |= ADC_CR_ADCAL;               // Perform self calibration
    while(ADC1->CR & ADC_CR_ADCAL);         // Delay until calibration is complete

    ADC1->CR |= ADC_CR_ADEN;                // Enable ADC
    while(!(ADC1->ISR & ADC_ISR_ADRDY));    // Wait until ADC ready
    ADC1->CR |= ADC_CR_ADSTART;             // Signal conversion start
}

void PI_update(void) {
    
    /* Run PI control loop
     *
     * Make sure to use the indicated variable names. This allows STMStudio to monitor
     * the condition of the system!
     *
     * target_rpm -> target motor speed in RPM
     * motor_speed -> raw motor speed in encoder counts
     * error -> error signal (difference between measured speed and target)
     * error_integral -> integrated error signal
     * Kp -> Proportional Gain
     * Ki -> Integral Gain
     * output -> raw output signal from PI controller
     * duty_cycle -> used to report the duty cycle of the system 
     * adc_value -> raw ADC counts to report current
     *
     */
    
    /// TODO: calculate error signal and write to "error" variable
		// T_dac = (12 / 8e6) * 30000 (TIM6 ARR and PSC)
		// (encCounts counts) * (1 rev / 3200 counts) * (1 / T_dac s) * (60 s / m) = 0.4166... * encCounts RPM ~= 0.5 -> encoder counts / 2
		int16_t current_rpm = motor_speed / 2;
	
		error = target_rpm - current_rpm;
    
    /* Hint: Remember that your calculated motor speed may not be directly in RPM!
     *       You will need to convert the target or encoder speeds to the same units.
     *       I recommend converting to whatever units result in larger values, gives
     *       more resolution.
     */
    
    
    /// TODO: Calculate integral portion of PI controller, write to "error_integral" variable
		error_integral = error_integral + Ki * error;
    
    /// TODO: Clamp the value of the integral to a limited positive range
		if (error_integral > 3200){
			error_integral = 3200;
		}
		
    
    /* Hint: The value clamp is needed to prevent excessive "windup" in the integral.
     *       You'll read more about this for the post-lab. The exact value is arbitrary
     *       but affects the PI tuning.
     *       Recommend that you clamp between 0 and 3200 (what is used in the lab solution)
     */
    
    /// TODO: Calculate proportional portion, add integral and write to "output" variable
    
    int16_t output = Kp * error + error_integral; // Change this!
    
    /* Because the calculated values for the PI controller are significantly larger than 
     * the allowable range for duty cycle, you'll need to divide the result down into 
     * an appropriate range. (Maximum integral clamp / X = 100% duty cycle)
     * 
     * Hint: If you chose 3200 for the integral clamp you should divide by 32 (right shift by 5 bits), 
     *       this will give you an output of 100 at maximum integral "windup".
     *
     * This division also turns the above calculations into pseudo fixed-point. This is because
     * the lowest 5 bits act as if they were below the decimal point until the division where they
     * were truncated off to result in an integer value. 
     *
     * Technically most of this is arbitrary, in a real system you would want to use a fixed-point
     * math library. The main difference that these values make is the difference in the gain values
     * required for tuning.
     */

     /// TODO: Divide the output into the proper range for output adjustment
		 output = output >> 5;
     
     /// TODO: Clamp the output value between 0 and 100 
		if(output > 100){
			 output = 100;
		} else if(output < 0){
			output = 0;
		}
    
    pwm_setDutyCycleLeft(output);
		pwm_setDutyCycleRight(output);
    duty_cycle = output;            // For debug viewing

    // Read the ADC value for current monitoring, actual conversion into meaningful units 
    // will be performed by STMStudio
    if(ADC1->ISR & ADC_ISR_EOC) {   // If the ADC has new data for us
        adc_value = ADC1->DR;       // Read the motor current for debug viewing
    }
}

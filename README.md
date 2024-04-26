# embedSP24ObstaBot
Group:
- Samuel Bosch
- Krishna Ashish Chinnari
- Nicholas Schneider
- Mohammed Ayman Habib

Milestone 1 Check-in:
- Be ambitious with your goals, but make them realistic in the sense that you can guarantee a minimum viable product that accomplishes most of that goal by the deadline
- Create a hierarchy of tasks that contribute most to you your minimum viable product and work on those first. Add additional/special functionality later.
- Don't delegate tasks between members of the group; find times to work together as a team on developing the project robot

Milestone 2 Check-in:
- Successful integration of gyroscope measurements from I2C to change PWM duty cycle to motors. Tilting up makes it go slightly faster, tilting down makes it go slightly slower.
- Have both motors working and independently tunable from two channels of TIM3.
- Unable to figure out ultrasonic sensor so far due to lack of datasheet and internet information. Tried to use the supposed UART capability, but there is no documentation at all for it. I2C is possible but according to the internet, it does not do well with other sensors on the same I2C bus. Only other option is using GPIO triggering, but each read takes multiple milliseconds, which makes scheduling other measurements and commands difficult. Might look into other sensors for obstacle detection.
- Also on the ultrasonic sensor issue, we need to talk to Ashton to see where the line is for Arduino usage. We had the idea of potentially using an Arduino ONLY for capturing the data from the ultrasonic and sending it to the STM32 over UART. Not sure how this would work with the "no Arduino" rule since the Arduino->STM communication would still need to be done at the register level on the STM. 

Milestone 3 Check-in:

-As we had issue in reading values from ultrasound sensors, based on the suggestions given by Ashton before finals, we tried debugging it, nothing worked so we replaced the ultrasound sensor with an IR range finder (GP2Y0A51SK0F).
-The range finder is a analog sensor; we could use the ADC input mode pins of the controller.

-We discussed the issue of reading the sensor value, but not the case with the potentiometer or direct bypassing the ADC IN pin with 3V and GND.
-As per discussion, probably due to less impedance, we need to try adding impedance or by using an Opamp.

Milestone 4 Check-in:
- Project name: ObstaBot
- Purpose: 2-wheeled obstacle avoidance robot based on rangefinder distance sensing. Possible use cases could be cleaning, hazardous environment exploration/patrolling, swarm robotics platform.
- Functionality: Autonomous driving, obstacle detection, directional scanning, gyroscope-based motor speed augmentation
- Demo: https://drive.google.com/drive/folders/1PRIEzp456W-ngQTMmI2s2VneioT8lKJb

Project Setup:
1. You will need a 2-wheel chassis that uses caster balls or another stabilizing mechanism, DC motors that can be mounted to the chassis, a HW-095 dual motor driver board (or equivalent board based on LN298N, with regulated 5V output as a pin option), an STM32F072 Discovery Kit board, a servo motor, an IR or Ultrasonic rangefinder (IR GP2Y0A51SK0F used in this project), resistors (to form pull-ups for I2C communication), a LiPo battery of 5+ volts (7.5 V 2S LiPo used), DuPont jumper wires, a switch (to disconnect battery power), and a computer with STMCubeMX and ARM Keil μVision installed.
2. Follow the block diagram to construct and wire the robot according to the appropriate pins.
3. Create a new project in STMCube for the microcontroller used in the STM32F072 discovery board and set the project to use only included libraries. Once generated, replace main.c and put gyro.c, and motor.c in Core/Src, and put the respective .h files into Core/Inc
4. If your program fails to compile, try changing the build target to use default compiler version 6. Once your program compiles successfully, flash your STM32F072 with the program code.
5. A software flowchart is available to demonstrate the basic logic of how the robot operates in its default state.
6. If you want to modify functionality of the robot beyond what is provided by default, here is a list of useful functions:
- pwminit(): Sets up the motors to be used with PWM
- i2cinit(): Initializes I2C for communication with the onboard gyroscope
- pwm_setDutyCycleLeft(): sets the duty cycle for the left motor between 0 and 100. Direction control is achieved by setting appropriate GPIO bits according to the motor driver's IN1...IN4 pins
- pwm_setDutyCycleRight(): same as above but for right motor
- turnLeft(): tells the robot to turn-in-place left by 90 degrees. Angle can be adjusted by modifying the delay values within the function
- turnRight(): same as above, but right
- get_gyro_x(): performs an I2C transaction to read the angular rate about the x axis of the onboard gyroscope. Same functionality for get_gyro_y and get_gyro_z()
- (Servo control): servo control is achieved through modification of the value written to the TIM14 CCR1 register. 2% PWM corresponds to looking right, 6% corresponds to straight ahead, and 11% corresponds to looking left.
- (Rangefinder reading): rangefinder reading is performed by reading the value of ADC1->DR after the EOC flag (third bit in ADC1->ISR) is set. Due to as of yet unresolved bugs, the values seem to be pulled down to a smaller magnitude, outputting values between 0 and 4 rather than the intended 0-255.
- (Optional UART): UART-based character and string transmission functions are available for serial value debugging assuming you have a USB-UART chip and a serial interface terminal like PuTTY installed on your computer.

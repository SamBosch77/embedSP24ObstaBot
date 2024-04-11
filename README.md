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

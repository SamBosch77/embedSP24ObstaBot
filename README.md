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
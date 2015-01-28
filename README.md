# L6474
Stepper motor driver library for Arduino Uno using X-NUCLEO-IHM01A1 shield (based on L6474 component)

1.  Introduction
----------------
This L6474 library is compatible with the Arduino Uno combined with one or several X-NUCLEO-IHM01A1 shields. It allows the Uno board to control up to three stepper motors provided having one X-NUCLEO-IHM01A1 shield by motor. 

2.  Installing the library into the Arduino IDE
------------------------------------------------

To install the library into the Arduino IDE, you need first to zip the folder "\L6474\libraries\L6474" to L6474.zip.
Then from the Arduino IDE:
  - Go to "Sketch > Import Library". At the top of the drop down list, select the option "Add Library". Browse to the location where you put the L6474.zip file and select it.
  - Return to the "Sketch > Import Library", you should now find the L6474 library at the bottom of the drop-down menu. It is ready to use in new Sketches.
  
  
  3.  Library content
---------------------
  
The library contents 4 files:
- l6474.cpp: core functions of the L6474 class
- l6474.h: declaration of the L6474 class and the associated definitions
- l6474_target_config.h: predefines values for the L6474 registers and the shields parameters (speed profile)
- keywords.txt: definitions of the L6474 class keywords for the syntax highlighting with the Arduino IDE.

4.  Example Sketches
--------------------

The library is provided with some example sketches:
- L6474SketchFor1MotorShield.ino: examples of driving one stepper motor in configuration with 1 X-NUCLEO-IHM01A1 shield
- L6474SketchFor2MotorShields.ino: examples of driving two stepper motors in  in configuration with 2 X-NUCLEO-IHM01A1 shields
- L6474SketchFor3MotorsShields.ino: examples of driving three stepper motors in configuration with 3 X-NUCLEO-IHM01A1 shields
- L6474SketchRegisterHandling.ino: examples of L6474 control functions  (access to the registers) with 1 X-NUCLEO-IHM01A1 shield

To use the L6474SketchFor3MotorsShields sketch, you will also have to modify the file:  "L6474.h" by uncommenting the following line:
//#define _USE_TIMER_0_FOR_L6474

Indeed, the flag _USE_TIMER_0_FOR_L6474 is required for three shields configuration. (More details in chapter 6).


5.  Library features
--------------------
The L6474 library has the following features: 
- L6474 registers read, write 
- Uno and X-NUCLEO-IHM01A shields configuration (GPIOs, PWM s, IRQs...)
- Speed profile configuration 
- Motion commands 
- FLAG interrupts handling (alarms reporting)
- Microstepping handling
- Daisy chaining to handle up to three X-NUCLEO-IHM01A shields

By starting the library, the user specifies the number of X-NUCLEO-IHM01A shields which are connected to the Arduino board.  Once set, the number of X-NUCLEO-IHM01A shields must not be changed.

Depending of the shields number, the library will:
- setup the required GPIOs to handle the motor directions and  the FLAG interrupt
- initialize the PWMs  that will act as the step clock generator
- initialize the speed profile (acceleration, deceleration, min and max speed) of each shield by using the parameters of the file "l6474_target_config.h"
- starts the SPI library to communicate with the L6474 chips
- Release the reset of each of the L6474 chips
- Disable the power bridge and clear the status flags of the L6474 chips
- load the registers of each of the L6474 with the predefined values from "l6474_target_config.h"

Once the initialization is done, the user can modifies the L6474 registers and speed profile configurations as desired. Most of the functions of the library take a shield Id (from 0 to 2) as input parameter. It gives the user the possibility to specify which of the three shields configuration he wants to modify. 

The user can also write a callback function and attach it to the Flag interrupt handler depending of the actions he wants to perform when an alarms is reported (read the flags, clear and read the flags...)

Then, he can request the move of one or several motors (till using the same principle of shield Id).  This request can be:
- to move for a given number of steps in a specified direction
- to go to a specific position 
- to run till reception of a new instruction

On reception of this request, the library will enable the PWM which is used as the step clock of the corresponding L6474.
At each pulse period, the motor will perform one step and an ISR (Interrupt Service Routine) is triggered on the microcontroller side.
This ISR is used by the library to count the number of performed steps and to update the speed. Indeed, the motor starts moving by using the minimum speed parameter.  At each step, the speed is increased using the acceleration parameter.
If the target position is far enough, the motor will perform a trapezoidal move:
- accelerating phase using the shield acceleration parameter
- steady phase where the motor turns at max speed
- decelerating phase using the shield deceleration parameter
- stop at the targeted position
Else, if the target position does not allow reaching the max speed, the motor will perform a triangular move:
- accelerating phase using the shield acceleration  parameter
- decelerating phase using the shield deceleration parameter
- stop at the targeted position

A moving command can be stopped at any moment:
- either by a soft stop which progressively decreases the speed using the deceleration parameter.  Once the minimum speed is reached, the motor is stopped.
- or by an hard stop command which immediately stops the motor.
In both cases, once the motor is stopped, the power bridge is automatically disabled.

To avoid sending a new command to a shield before the completion of the previous one, the library offers a WaitWhileActive() command which locks the program execution till the motor ends moving.

The library also offers the possibility to change the step mode (from full step till 1/16 microstep mode) for a given shield.  When the step mode is change, the current position (ABS_POSITION register) is automatically reset but it is up to the user to update the speed profile (max and min speed, acceleration deceleration).

6.  Library required resources
------------------------------

The L6474 library requires the use of the SPI library. That's why the header file "SPI.h" must be included at the beginning of each sketch.
The step clock of each of the shield board is generated by a PWM. The frequency of this PWM can be updated at each pulse depending on the speed profile of the shield. That's why the library required one timer and its corresponding ISR by shield.
For the handling of the flag interrupt, the FW library uses the external interrupt 0: only one external interrupt is used as all flag pin are connected together.
One GPIO by shield is also required for the handling of the direction. 
And at last, one common GPIO is used for the reset of all the shields.

Resources        | Digital Pin  |  Features               |  Shield
Interrupt 0      |     2        |  Flag interrupt         |  All
GPIO             |     8        |  L6474 Reset            | All
SS               |     10       |  SPI Slave select       | All
MOSI             |     11       | SPI Master Out Slave IN | All
MISO             |     12       | SPI Master IN Slave Out | All
SCK              |     13       | SPI serial clock        | All
Timer 1 and ISR1 |     9        | PWM 1                   | 0
GPIO             |     7        | Direction 1             | 0
Timer 2 and ISR2 |     3        | PWM 2                   | 1
GPIO             |     4        | Direction 2             | 1
Timer 0 and ISR0 |     6        | PWM 3                   | 2
GPIO             |     5        | Direction 3             | 2


Of course, if shield 2 or/and 1 are unplugged, the corresponding resources are free.

The use of timer 0 and ISR0 implies some constraints on the way to write sketches for 3 X-NUCLEO-IHM01A1 shields configuration.
In this case, the standard Arduino delay functions which are based on timer 0 (delay(),millis(),...) become unusable. To overcome this problem, the library proposes its own delay functions (WaitUs() and WaitMs()). Nevertheless, the use of the standard Arduino functions should be preferred for one or two X-NUCLEO-IHM01A1 shields configuration.
Moreover, the usage of ISR0 brings some dependencies problem with the standard Arduino library. To limit them, the definition of the ISR0 in the library is performed under a specific compilation flag: _USE_TIMER_0_FOR_L6474. This flag has to be enabled in file: l6474.h" for three shields configuration.
At last, the way to write the sketch is also impacted. It will be necessary to define a main() function additionally to the usual setup() and loop() functions. For more details, see the example sketch" L6474SketchFor3MotorsShields.ino".

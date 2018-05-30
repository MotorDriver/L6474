# L6474
Stepper motor driver library for Arduino Uno using X-NUCLEO-IHM01A1 shield (based on L6474 component)

1.  Introduction
----------------
This L6474 library is compatible with the Arduino Uno combined with one or several X-NUCLEO-IHM01A1 shields. It allows the Uno board to control up to three stepper motors provided having one X-NUCLEO-IHM01A1 shield per motor. 

2.  Installing the library into the Arduino IDE
------------------------------------------------

- Download https://github.com/MotorDriver/L6474/archive/master.zip
- (In the Arduino IDE) Sketch > Include Library > Add .ZIP Library... > select the downloaded file > Open

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

Once the initialization is done, the user can modify the L6474 registers and speed profile configurations as desired. Most of the functions of the library take a shield Id (from 0 to 2) as input parameter. It gives the user the possibility to specify which of the three shields configuration he wants to modify. 

The user can also write a callback function and attach it to the Flag interrupt handler depending on the actions he wants to perform when an alarms is reported (read the flags, clear and read the flags...)

Then, he can request the move of one or several motors (still using the same principle of shield Id).  This request can be:
- to move for a given number of steps in a specified direction
- to go to a specific position 
- to run until reception of a new instruction

On reception of this request, the library will enable the PWM which is used as the step clock of the corresponding L6474.
At each pulse period, the motor will perform one step and an ISR (Interrupt Service Routine) is triggered on the microcontroller side.
This ISR is used by the library to count the number of performed steps and to update the speed. Indeed, the motor starts moving by using the minimum speed parameter.  At each step, the speed is increased using the acceleration parameter.
If the target position is far enough, the motor will perform a trapezoidal move:
- accelerating phase using the shield acceleration parameter
- steady phase where the motor turns at max speed
- decelerating phase using the shield deceleration parameter
- stop at the targeted position
Otherwise, if the target position does not allow reaching the max speed, the motor will perform a triangular move:
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
The step clock of each of the shield board is generated by a PWM. The frequency of this PWM can be updated at each pulse depending on the speed profile of the shield. That's why the library required one timer and its corresponding ISR per shield.
For the handling of the flag interrupt, the FW library uses the external interrupt 0: only one external interrupt is used as all flag pin are connected together.
One GPIO by shield is also required for the handling of the direction. 
And at last, one common GPIO is used for the reset of all the shields.
<br>
Resources        | Digital Pin  |  Features               |  Shield<br>
Interrupt 0      |     2        |  Flag interrupt         |  All<br>
GPIO             |     8        |  L6474 Reset            | All<br>
SS               |     10       |  SPI Slave select       | All<br>
MOSI             |     11       | SPI Master Out Slave IN | All<br>
MISO             |     12       | SPI Master IN Slave Out | All<br>
SCK              |     13       | SPI serial clock        | All<br>
Timer 1 and ISR1 |     9        | PWM 1                   | 0<br>
GPIO             |     7        | Direction 1             | 0<br>
Timer 2 and ISR2 |     3        | PWM 2                   | 1<br>
GPIO             |     4        | Direction 2             | 1<br>
Timer 0 and ISR0 |     6        | PWM 3                   | 2<br>
GPIO             |     5        | Direction 3             | 2<br>
<br>

Of course, if shield 2 or/and 1 are unplugged, the corresponding resources are free.

The use of timer 0 and ISR0 implies some constraints on the way to write sketches for 3 X-NUCLEO-IHM01A1 shields configuration.
In this case, the standard Arduino delay functions which are based on timer 0 (delay(),millis(),...) become unusable. To overcome this problem, the library proposes its own delay functions (WaitUs() and WaitMs()). Nevertheless, the use of the standard Arduino functions should be preferred for one or two X-NUCLEO-IHM01A1 shields configuration.
Moreover, the usage of ISR0 brings some dependencies problem with the standard Arduino library. To limit them, the definition of the ISR0 in the library is performed under a specific compilation flag: _USE_TIMER_0_FOR_L6474. This flag has to be enabled in file: l6474.h" for three shields configuration.
At last, the way to write the sketch is also impacted. It will be necessary to define a main() function additionally to the usual setup() and loop() functions. For more details, see the example sketch" L6474SketchFor3MotorsShields.ino".

7.  Boards wiring for multi-motors configurations
------------------------------------------------

If you want to drive several motors with the library, you will first need to have 2 or 3 X-NUCLEO-IHM01A1 boards.
The driving of the different motors is done thanks to daisy chaining (see next paragraph). To handle this daising chaining, the boards have two crossover pins that are combined with some shunt (0K) resistors to propagate the SDO (or MOSI) of one board to the SDI (or MISO) of the following one.

By default, the shunt resistors board are set for 1 board configuration (R1, R4, R7, R12 resistors mounted).

For 2 boards configuration, 
- the first board you have to move these resistors to have R1, R4, R7, R10 resistors mounted
- the second must have R2, R5, R8, R12 resistors mounted

For 3 boards configuration, 
- the first must have R1, R4, R7, R10 resistors mounted
- the second must have R2, R5, R8, R11 resistors mounted
- the third must have R3, R6, R9, R12 resistors mounted

To have more details you can have a look in paragraph 2.2 of ST document " UM1857: Stepper motor driver expansion board based on L6474 " that you can find here: http://www.st.com/st-web-ui/static/active/en/resource/technical/document/user_manual/DM00156746.pdf

Once the shunt resistors are correctly  mounted, you only have to plug on board on top of the other to have a 2 or 3 boards configuration.

Resistors R25 and R24 could also be moved to use another CS (chip select) line or Clock line. But in the case of this library, you can keep the default configuration.

8.  Additionnal information regarding daisy chaining
----------------------------------------------------

The daisy chaining is used to send command via the SPI from the Uno to several X-NUCLEO-IHM01A1 boards.
The purpose of these commands can be to enable/disable the bridges, set the parameters of the L6474 (to dynamically adjust the torques for examples), to know the position, to get the alarm status (over current detection, over temperature,...) of the L6474.

The principles of the daisy chaining are the following:
- all boards share the same clock and chip select.
- the SDO of the Uno is linked to the SDI of the first shield
- then each SDO of one shield is linked to the SDI of the following shield (thanks to the two additional crossover pins)
- the SDO of the last shieled is linked to the SDI of the UNO
- the data are transmitted byte by byte.
- When the CS is high, the SPI works as a delay line: at each clock enabling , a byte is read from the SDI and pushed to the SDO
- When the CS is released, the last byte received by the L6474 is latched and decoded. The answer is prepared to be sent out vai SDO at the next clock enabling.

And if you want the full details, you can have a look of the application note (still from ST): "AN4290: L647x, L648x and powerSTEP01 family communication protocol" http://www.st.com/st-web-ui/static/active/en/resource/technical/document/application_note/DM00082126.pdf
and specifically at figure 9 where there is a time diagram with the clock and several devices!

From the point of view of a library user, the daisy chaining is hidden, and its use is quite simple as you only have to specify the index (from 0 to 2) of the targetted board to use it!

For example, if you want to get the status of the first board you have to write:<br>
uint16_t statusRegister = myL6474.CmdGetStatus(0); <br>
to get the status of the second shield:<br>
uint16_t statusRegister = myL6474.CmdGetStatus(1); <br>
to get the status of the third shield:<br>
uint16_t statusRegister = myL6474.CmdGetStatus(2); <br>

To set the torque regulation current  of the first shield board to 625 mA (caution only multiple of 31.25 are supported):<br><br>
myL6474.CmdSetParam(0, L6474_TVAL, 625); <br>

To set the torque regulation current  of the second shield board to 1000 mA<br><br>
myL6474.CmdSetParam(2, L6474_TVAL, 1000); <br>

9.  Drive one motor by uart commands
------------------------------------

If you want to drive one motor by sending commands from a PC to the uno board by Uart, you can take the sketch "L6474SketchFor1MotorShieldDrivenByUart".

To use it:
-	You have to install the L6474 library 
-	The Uno must have a X-Nucleo-Ihm01A1 on top of it. A stepper motor has to be connected to the bridges of the expansion board and its external power supply must be enabled.
-	The terminal port must be configured to 9600 baud, new line handled with option “CR+LF” and I recommend to enable local echo.
-	Then, just download the sketch to the Arduino Uno with the IDE.   
If your setup is correct, as this step you should see the token :”>>:” at the serial terminal.
It means that the Uno firmware is expecting some instructions.
Then you have to enter one of the following command line:<br>
C1                -> to get the acceleration<br>
C2                -> to get the current speed<br>
C3                -> to Get the deceleration<br>
C4                -> to Get the shield state<br>
C5                -> to Get the shield the FW version<br>
C6                -> to Get the Mark<br>
C7                -> to Get the max speed state<br>
C8                -> to Get the min speed shield state<br>
C9                -> to Get the current position<br>
C10               -> to go home  the shield state<br>
C11               -> to go to mark the position<br>
C12 <arg1>        -> to go to  <arg1> position<br>
C13               -> to make a hard stop<br>
C14 <arg1> <arg2> -> to move of <arg2> steps in forward direction if <arg1> is 0,<br>
                     in backward direction if <arg1> is 1<br>
C15               -> to reset all shields<br>
C16 <arg1>        -> to run in forward direction if <arg1> is 0,<br>
                     in backward direction if <arg1> is 1<br>
C17 <arg1>        -> to set the acceleration to <arg1><br>
C18 <arg1>        -> to set the deceleration to <arg1><br>
C19               -> to set current position to be the home position<br>
C20               -> to set current position to be the mark position<br>
C21 <arg1>        -> to set max speed to <arg1><br>
C22 <arg1>        -> to set min speed to <arg1><br>
C23               -> to make a soft stop<br>
C24               -> to wait while the motor is active<br>
C25               -> to send a disable command (bridges off)<br>
C26               -> to send a enable command (bridges on)<br>
C27 <arg1>        -> to send a get param command with  <arg1> as register address<br>
C28               -> to send a get status command (read and clear status register)<br>
C29               -> to send a nop command <br>
C30 <arg1> <arg2> -> to send a set param command with  <arg1> as register address,<br>
                     <arg2> as value <br>
C31               -> to read status register (no clear is done)<br>
C32               -> to reset the shield<br>
C33               -> to release the reset<br>
C34 <arg1>        -> to set the step mode to full step if <arg1> is 1<br>
                                          to half step if <arg1> is 2<br>
                                          to 1/4 step if <arg1> is 4<br>
                                          to 1/8 step if <arg1> is 8<br>
                                          to 1/16 step if <arg1> is 16<br>
C35 <arg1>        -> to set direction: forward direction if <arg1> is 0,<br>
                     backward direction if <arg1> is 1<br>


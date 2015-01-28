/* This sketch is an example use of the L6474 library with two shields  X-NUCLEO-IHM01A1*/
/* It drives two stepper motors */

#include <l6474.h>
#include <SPI.h>

L6474 myL6474;  

void setup()
{
  int32_t pos;
  uint32_t myMaxSpeed;
  uint32_t myMinSpeed;
  uint16_t myAcceleration;
  uint16_t myDeceleration;

  /* Start the L6474 library to use 2 shields x-NUCLEO-IHM01A1 */
  /* The L6474s registers are set with the predefined values */
  /* from file l6474_target_config.h*/
  myL6474.Begin(2);
  
  /* Attach the function MyFlagInterruptHandler (defined below) to the flag interrupt */
  myL6474.AttachFlagInterrupt(MyFlagInterruptHandler);

  /* Set Current position to be the home position for shield 0 */
  myL6474.SetHome(0);

  /* Set Current position to be the home position for shield 1 */
  myL6474.SetHome(1);

  /* Request shield 0 to Goto position 3200 */
  myL6474.GoTo(0,3200);  
   
  /* Wait for shield 0 ends moving */  
  myL6474.WaitWhileActive(0);

  /* Get current position of shield 0*/
  pos = myL6474.GetPosition(0);
  
  /* If the read position of shield 0 is 3200 */
  /* Request shield 1 to go to the same position */
  if (pos == 3200)
  {
    myL6474.GoTo(1,pos); 
    /* Wait for  shield 1 ends moving */  
    myL6474.WaitWhileActive(1);
  }
  
  /* Wait for 2 seconds */
  delay(2000);  
  
  /* Set current position of shield 1 to be its mark position*/
  myL6474.SetMark(1); 
  
 /* Request shield 1 to Goto position -3200 */
  myL6474.GoTo(1,-3200);  
   
  /* Wait for shield 1 ends moving */  
  myL6474.WaitWhileActive(1);

  /* Get current position of shield 1*/
  pos = myL6474.GetPosition(1);
  
  /* If the read position of shield 1 is -3200 */
  /* Request shield 1 to go to the same position */
  if (pos == -3200)
  {
    myL6474.GoTo(0,pos); 
    /* Wait for  shield 1 ends moving */  
    myL6474.WaitWhileActive(0);
  }

  /* Wait for 2 seconds */
  delay(2000);  
  
  /* Set current position of shield 1 to be its mark position*/
  myL6474.SetMark(0); 
 
  /* Request both shields to go home */
  myL6474.GoHome(0);  
  myL6474.GoHome(1);  

  /* Wait for both shields end moving */  
  myL6474.WaitWhileActive(0);
  myL6474.WaitWhileActive(1);

  /* Wait for 2 seconds */
  delay(2000);  

 /* Request both shields to go to their mark position */
  myL6474.GoMark(0);  
  myL6474.GoMark(1);  

  /* Wait for both shields end moving */  
  myL6474.WaitWhileActive(0);
  myL6474.WaitWhileActive(1);

  /* Wait for 2 seconds */
  delay(2000);  

  /* Request shield 0 to run in FORWARD direction */
  myL6474.Run(0, FORWARD);
  
  /* Get shield 0 max speed */
  myMaxSpeed = myL6474.GetMaxSpeed(0);
  
  /* Wait for shield 0 reaches its max speed */
  while (myL6474.GetCurrentSpeed(0) != myMaxSpeed);

  /* Set max speed of shield 1 to be half of the one of shield 0 */
  myL6474.SetMaxSpeed(1, myMaxSpeed / 2);

  /* Request shield 1 to run in backward direction*/
  myL6474.Run(1, FORWARD);
  
  /* Wait for shield 1 reaches its max speed */
  while (myL6474.GetCurrentSpeed(1) != (myMaxSpeed / 2));
  
  /* Wait for 5 seconds */
  delay(5000);
  
  /* Request shield 0 to make a soft stop */
  myL6474.SoftStop(0);
  
  /* Wait for shield 0 ends moving */  
  myL6474.WaitWhileActive(0);

  /* Request shield 1 to make a hard stop */
  myL6474.HardStop(1);

 /* Wait for shield 1 ends moving */  
  myL6474.WaitWhileActive(1);

  /* Wait for 2 seconds */
  delay(2000);  

  /* Request both shields to go home */
  myL6474.GoHome(0);  
  myL6474.GoHome(1);  

  /* Wait for both shields end moving */  
  myL6474.WaitWhileActive(0);
  myL6474.WaitWhileActive(1);

  /* Wait for 2 seconds */
  delay(2000);  

  /* Get acceleration, deceleration and MinSpeed of shield 0*/
  myAcceleration = myL6474.GetAcceleration(0);
  myDeceleration = myL6474.GetDeceleration(0);
  myMinSpeed = myL6474.GetMinSpeed(0);
  
  /* Select ful step mode for shield 1 */
  myL6474.SelectStepMode(1,L6474_STEP_SEL_1);

  /* Set speed and acceleration of shield 1 to be consistent with full step mode */
  myL6474.SetMaxSpeed(1, myMaxSpeed / 16);
  myL6474.SetMinSpeed(1, myMinSpeed / 16);
  myL6474.SetAcceleration(1, myAcceleration / 16);
  myL6474.SetDeceleration(1, myDeceleration / 16);
  
  /* Request shield 0 to go to position 400 */
  myL6474.GoTo(0, 6400);  

  /* Request shield 1 to go to the equivalent reverse position */
  /* in full step mode -6400 /16 = -400 */
  myL6474.GoTo(1, -400 );  
  
  /* Wait for both shields end moving */  
  myL6474.WaitWhileActive(0);
  myL6474.WaitWhileActive(1);

  /* Wait for 2 seconds */
  delay(2000);  

  /* Set the new position of both shield to be their new mark position */  
  myL6474.SetMark(0);
  myL6474.SetMark(1);
  
  /* Wait for 5 seconds before entering the loop */
  delay(5000);
}

void loop()
{
  /* Shield 0 is using 1/16 microstepping mode */
  /* Shield 1 is using full step mode */
  myL6474.GoMark(0);  
  myL6474.GoMark(1);  
  myL6474.WaitWhileActive(0);
  myL6474.WaitWhileActive(1);

  myL6474.GoHome(0);  
  myL6474.GoHome(1);  
  myL6474.WaitWhileActive(0);
  myL6474.WaitWhileActive(1);
}

void MyFlagInterruptHandler(void)
{
  /* Get status of device 0 */
  /* this will clear the flags */
  uint16_t statusRegister = myL6474.CmdGetStatus(0);

  /* Get status of device 1 */
  /* this will clear the flags */
  statusRegister = myL6474.CmdGetStatus(1);
}





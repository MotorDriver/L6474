/* This sketch is an example use of the L6474 library with three shields  X-NUCLEO-IHM01A1*/
/* It drives three stepper motors */
/* To use 3 motors board X-NUCLEO-IHM01A1, you need to use timer0 for L6474 library */
/* while this timer is generally used in Arduino standard library for */
/* delay, millis,... functions. So, you cannot use these functions    */
/* with this configuration */
/* Moreover, this configuration required to  to use the handler: ISR(TIMER0_OVF_vect)  */
/* which is already used via default Arduino function */
/* That's why we need to redefine here the main() and myinit() functions */
/*To enable the use of timer0 by L6474 library you need to define */
/* the follwing line  //#define _USE_TIMER_0_FOR_L6474 in l6474.h */

#include <l6474.h>
#include <SPI.h>

#ifndef _USE_TIMER_0_FOR_L6474
#error MISSING_FLAG__USE_TIMER_0_FOR_L6474__FOR_3_SHIELDS_CONFIGURATION
/* you need to define _USE_TIMER_0_FOR_L6474 in l6474.h */
/* if you want to use 3 L6474 shields (see header file) */
#endif

L6474 myL6474;

void myinit()
{
  // this needs to be called before setup() or some functions won't
  // work there
  sei();
}

int main( void )
{
  myinit();
 setup();
  while ( true ) { loop(); }
}

void setup()
{
  int32_t pos;
  uint32_t myMaxSpeed;
  uint32_t myMinSpeed;
  uint16_t myAcceleration;
  uint16_t myDeceleration;

  /* Start the L6474 library to use 3 shields X-NUCLEO-IHM01A1*/
  /* The L6474 registers are set with the predefined values */
  /* from file l6474_target_config.h*/
  myL6474.Begin(3);

  /* Attach the function MyFlagInterruptHandler (defined below) to the flag interrupt */
  myL6474.AttachFlagInterrupt(MyFlagInterruptHandler);

  /* Set Current position to be the home position for shield 0 */
  myL6474.SetHome(0);

  /* Set Current position to be the home position for shield 1 */
  myL6474.SetHome(1);

  /* Set Current position to be the home position for shield 2 */
  myL6474.SetHome(2);
 
  /* Request shield 0 to Goto position 3200 */ 
  myL6474.GoTo(0,3200);  

  /* Wait for  shield 0 ends moving */  
  myL6474.WaitWhileActive(0);

  /* Get current position of shield 0*/
  pos = myL6474.GetPosition(0);
    
  /* If the read position of shield 0 is 3200 */
  /* Request shield 1 to go to the same position */
  if (pos == 3200)
  {
    /* Set current position of shield 0 to be its mark position*/
    myL6474.SetMark(0); 
    
    /* Request shield 1 to Go to the same position  */ 
    myL6474.GoTo(1,pos); 
    
    /* Wait for  shield 1 ends moving */  
    myL6474.WaitWhileActive(1);
  }
  
  /* Get current position of shield 0*/
  pos = myL6474.GetPosition(1);
  
  /* If the read position of shield 1 is 3200 */
  /* Request shield 2 to go to the same position */
  if (pos == 3200)
  {
    /* Request shield 2 to Go to the same position  */ 
    myL6474.GoTo(2,pos); 

    /* Wait for shield 2 ends moving */  
    myL6474.WaitWhileActive(2);
  }
  
  /* Get current position of shield 0*/
  pos = myL6474.GetPosition(2);

  /* Wait for 1s */
  L6474::WaitMs(1000);
  
  if (pos == 3200)
  {
    /* Request all shields to go home */
    myL6474.GoHome(0); 
    myL6474.GoHome(1); 
    myL6474.GoHome(2); 

    /* Wait for all shield ends moving */ 
    myL6474.WaitWhileActive(0);
    myL6474.WaitWhileActive(1);
    myL6474.WaitWhileActive(2);
  }
  
    /* Wait for 1s */
  L6474::WaitMs(1000);
  
  /* Request shield 0 to Goto position -3200 */ 
  myL6474.GoTo(0,-3200);  

  /* Wait for shield 0 ends moving */  
  myL6474.WaitWhileActive(0);

  /* Get current position of shield 0*/
  pos = myL6474.GetPosition(0);
    
  /* If the read position of shield 0 is -3200 */
  /* Request shield 1 to go to the same position */
  if (pos == -3200)
  {
    /* Request shield 1 to go to the same position  */ 
    myL6474.GoTo(1,pos); 
    
    /* Wait for shield 1 ends moving */  
    myL6474.WaitWhileActive(1);
  }
  
  /* Get current position of shield 0*/
  pos = myL6474.GetPosition(1);
  
  /* If the read position of shield 1 is -3200 */
  /* Request shield 2 to go to the same position */
  if (pos == -3200)
  {
    /* Request shield 2 to Go to the same position  */ 
    myL6474.GoTo(2,pos); 

    /* Wait for shield 2 ends moving */  
    myL6474.WaitWhileActive(2);
  }
  
  /* Get current position of shield 0*/
  pos = myL6474.GetPosition(2);

  /* Wait for 1s */
  L6474::WaitMs(1000);

  if (pos == -3200)
  {
    /* Set current position of shield 2 to be its mark position*/
    myL6474.SetMark(2); 

    /* Request all shields to go home */
    myL6474.GoHome(0); 
    myL6474.GoHome(1); 
    myL6474.GoHome(2); 

    /* Wait for all shield ends moving */ 
    myL6474.WaitWhileActive(0);
    myL6474.WaitWhileActive(1);
    myL6474.WaitWhileActive(2);
  }

  /* Wait for 1s */
  L6474::WaitMs(1000);
  
  /* Request shield 0 and shield 2 to go their mark position */
  myL6474.GoMark(0); 
  myL6474.GoMark(2); 
  
  /* Wait for shield 0 and 2 ends moving */ 
  myL6474.WaitWhileActive(0);
  myL6474.WaitWhileActive(2);
  
  /* Wait for 1s */
  L6474::WaitMs(1000);
  
   /* Request shield 0 to run in FORWARD direction */
  myL6474.Run(0,FORWARD); 
 
  /* Get shield 0 max speed */
  myMaxSpeed = myL6474.GetMaxSpeed(0);
  
  /* Wait for shield 0 reaches its max speed */
  while (myL6474.GetCurrentSpeed(0) != myMaxSpeed);

  /* Set max speed of shield 1 to be the one of shield 0 -320 step/s */
  myMaxSpeed -= 320;
  myL6474.SetMaxSpeed(1, myMaxSpeed);

  /* Request shield 0 to run in FORWARD direction */
  myL6474.Run(1,FORWARD); 
 
  /* Wait for shield 1 reaches its max speed */
  while (myL6474.GetCurrentSpeed(1) != myMaxSpeed);

   /* Set max speed of shield 2 to be the one of shield 1 -320 step/s */
  myMaxSpeed -= 320;
  myL6474.SetMaxSpeed(2, myMaxSpeed);

  /* Request shield 2 to run in FORWARD direction */
  myL6474.Run(2,FORWARD); 

  /* Wait for shield 1 reaches its max speed */
  while (myL6474.GetCurrentSpeed(2) != myMaxSpeed);

  /* Wait for 3s */
  L6474::WaitMs(3000);
  
  /* Request shield 1 to make a soft stop */
  myL6474.SoftStop(1);
  
  /* Wait for both shields end moving */
  myL6474.WaitWhileActive(1);  
  
  /* Request shield 0 and 2 to make a hard stop */
  myL6474.HardStop(0);
  myL6474.HardStop(2);
  
  /* Wait for both shields end moving */
  myL6474.WaitWhileActive(0);  
  myL6474.WaitWhileActive(2);    
  
  /* Request all shields shields to go home */
  myL6474.GoHome(0);  
  myL6474.GoHome(1);  
  myL6474.GoHome(2);  
  
  /* Wait for all shields end moving */  
  myL6474.WaitWhileActive(0);
  myL6474.WaitWhileActive(1);
  myL6474.WaitWhileActive(2);

  /* Get acceleration, deceleration, Maxspeed and MinSpeed of shield 0*/
  myAcceleration = myL6474.GetAcceleration(0);
  myDeceleration = myL6474.GetDeceleration(0);
  myMaxSpeed = myL6474.GetMaxSpeed(0);
  myMinSpeed = myL6474.GetMinSpeed(0);
  
  /* Select 1/8 microstepping mode for shield 1 */
  myL6474.SelectStepMode(1, L6474_STEP_SEL_1_8);

  /* Set speed and acceleration of shield 1 to be consistent with 1/8 microstepping mode */
  myL6474.SetMaxSpeed(1, myMaxSpeed / 2 );
  myL6474.SetMinSpeed(1, myMinSpeed / 2 );
  myL6474.SetAcceleration(1, myAcceleration / 2 );
  myL6474.SetDeceleration(1, myDeceleration / 2 );
  
  /* Select ful step mode  for shield 2 */
  myL6474.SelectStepMode(2, L6474_STEP_SEL_1);

  /* Set speed and acceleration of shield 1 to be consistent with full step mode */
  myL6474.SetMaxSpeed(2, myMaxSpeed / 16);
  myL6474.SetMinSpeed(2, myMinSpeed / 16);
  myL6474.SetAcceleration(2, myAcceleration / 16);
  myL6474.SetDeceleration(2, myDeceleration / 16);

}

void loop()
{

  /* Shield 0 is using 1/16 microstepping mode */ 
  /* Shield 1 is using 1/8 microstepping mode */
  /* Shield 2 is using full step mode */
  myL6474.GoTo(0, -3200); 
  myL6474.GoTo(1, 1600); 
  myL6474.GoTo(2, -200);   
  myL6474.WaitWhileActive(0);
  myL6474.WaitWhileActive(1);
  myL6474.WaitWhileActive(2);

  myL6474.GoTo(0, 3200); 
  myL6474.GoTo(1, -1600); 
  myL6474.GoTo(2, 200); 
  myL6474.WaitWhileActive(0);
  myL6474.WaitWhileActive(1);
  myL6474.WaitWhileActive(2);
}



void MyFlagInterruptHandler(void)
{
  /* Get status of device 0 */
  /* this will clear the flags */
  uint16_t statusRegister = myL6474.CmdGetStatus(0);

  /* Get status of device 1 */
  /* this will clear the flags */
  statusRegister = myL6474.CmdGetStatus(1);
  
  /* Get status of device 2 */
  /* this will clear the flags */
  statusRegister = myL6474.CmdGetStatus(2);
}

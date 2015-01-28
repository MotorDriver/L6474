/* This sketch is an example use of the L6474 library with one shield  X-NUCLEO-IHM01A1*/
/* It drives one stepper motor */

#include <l6474.h>
#include <SPI.h>

L6474 myL6474;  
  
void setup()
{
  int32_t pos;
  uint16_t mySpeed;

//----- Init
  /* Start the library to use 1 shield */
  /* The L6474 registers are set with the predefined values */
  /* from file l6474_target_config.h*/
  myL6474.Begin(1);

  /* Attach the function MyFlagInterruptHandler (defined below) to the flag interrupt */
  myL6474.AttachFlagInterrupt(MyFlagInterruptHandler);

//----- Move of 16000 steps in the FW direction

  /* Move shield 0 of 16000 steps in the FORWARD direction*/
  myL6474.Move(0, FORWARD, 16000);
  
  /* Wait for the motor of shield 0 ends moving */
  myL6474.WaitWhileActive(0);

  /* Wait for 2 seconds */
  delay(2000);  

//----- Move of 16000 steps in the BW direction

  /* Move shield 0 of 16000 steps in the BACKWARD direction*/
  myL6474.Move(0, BACKWARD, 16000);

  /* Wait for the motor of shield 0 ends moving */
  myL6474.WaitWhileActive(0);

   /* Set the current position of shield 0 to be the Home position */
  myL6474.SetHome(0);
  
  /* Wait for 2 seconds */
  delay(2000);

//----- Go to position -6400

  /* Request shield 0 to go to position -6400 */
  myL6474.GoTo(0,-6400);  
  
  /* Wait for the motor ends moving */
  myL6474.WaitWhileActive(0);

  /* Get current position of shield 0*/
  pos = myL6474.GetPosition(0);

   /* Set the current position of shield 0 to be the Mark position */
  myL6474.SetMark(0);

/* Wait for 2 seconds */
  delay(2000);
  
//----- Go Home

  /* Request shield 0 to go to Home */
  myL6474.GoHome(0);  
  myL6474.WaitWhileActive(0);

  /* Get current position of shield 0 */
  pos = myL6474.GetPosition(0);
  
  /* Wait for 2 seconds */
  delay(2000);

//----- Go to position 6400

  /* Request shield 0 to go to position 6400 */
  myL6474.GoTo(0,6400);  
  
  /* Wait for the motor of shield 0 ends moving */
  myL6474.WaitWhileActive(0);

  /* Get current position of shield 0*/
  pos = myL6474.GetPosition(0);

  /* Wait for 2 seconds */
  delay(2000);
  
//----- Go Mark which was set previously after go to -6400

  /* Request shield 0 to go to Mark position */
  myL6474.GoMark(0);  
  
  /* Wait for the motor of shield 0 ends moving */
  myL6474.WaitWhileActive(0);

  /* Get current position of shield 0 */
  pos = myL6474.GetPosition(0);

  /* Wait for 2 seconds */
  delay(2000);

//----- Run the motor BACKWARD

  /* Request shield 0 to run BACKWARD */
   myL6474.Run(0,BACKWARD);       
   delay(5000);

   /* Get current speed of shield 0 */
   mySpeed = myL6474.GetCurrentSpeed(0);

//----- Increase the speed while running

  /* Increase speed of shield 0 to 2400 step/s */
  myL6474.SetMaxSpeed(0,2400);
  delay(5000);

   /* Get current speed of shield 0 */
   mySpeed = myL6474.GetCurrentSpeed(0);

//----- Decrease the speed while running

  /* Decrease speed of shield 0 to 1200 step/s */
  myL6474.SetMaxSpeed(0,1200);
  delay(5000);

  /* Get current speed */
  mySpeed = myL6474.GetCurrentSpeed(0);

//----- Increase acceleration while running

  /* Increase acceleration of shield 0 to 480 step/s^2 */
  myL6474.SetAcceleration(0,480);
  delay(5000);

  /* Increase speed of shield 0 to 2400 step/s */
 myL6474.SetMaxSpeed(0,2400);
 delay(5000);

 /* Get current speed of shield 0 */
 mySpeed = myL6474.GetCurrentSpeed(0);

//----- Increase deceleration while running

  /* Increase deceleration of shield 0 to 480 step/s^2 */
  myL6474.SetDeceleration(0,480);
  delay(5000);

  /* Decrease speed of shield 0 to 1200 step/s */
  myL6474.SetMaxSpeed(0,1200);
  delay(5000);

  /* Get current speed */
  mySpeed = myL6474.GetCurrentSpeed(0);

//----- Soft stopped required while running

  /* Request soft stop of shield 0 */
  myL6474.SoftStop(0);

  /* Wait for the motor of shield 0 ends moving */  
  myL6474.WaitWhileActive(0);

  /* Wait for 2 seconds */
  delay(2000);

//----- Run stopped by hardstop

  /* Request shield 0 to run in FORWARD direction */
  myL6474.Run(0,FORWARD);       
  delay(5000);
  
  /* Request shield 0 to immediatly stop */
  myL6474.HardStop(0);
  myL6474.WaitWhileActive(0);

  /* Wait for 2 seconds */
  delay(2000);
  
//----- GOTO stopped by softstop

 /* Request shield 0 to go to position 20000  */
  myL6474.GoTo(0,20000);  
  delay(5000);

  /* Request shield 0 to perform a soft stop */
  myL6474.SoftStop(0);
  myL6474.WaitWhileActive(0);

  /* Wait for 2 seconds */
  delay(2000);

//----- Read inexistent register to test MyFlagInterruptHandler

  /* Try to read an inexistent register */
  /* the flag interrupt should be raised */
  /* and the MyFlagInterruptHandler function called */
  myL6474.CmdGetParam(0,(L6474_Registers_t)0x1F);
  delay(500);

//----- Change step mode to full step mode

  /* Select full step mode for shield 0 */
  myL6474.SelectStepMode(0,L6474_STEP_SEL_1);

  /* Set speed and acceleration to be consistent with full step mode */
  myL6474.SetMaxSpeed(0,100);
  myL6474.SetMinSpeed(0,50);
  myL6474.SetAcceleration(0,10);
  myL6474.SetDeceleration(0,10);

  /* Request shield 0 to go position 200 */
  myL6474.GoTo(0,200);  

  /* Wait for the motor of shield 0 ends moving */
  myL6474.WaitWhileActive(0);

  /* Get current position */
  pos = myL6474.GetPosition(0);

  /* Wait for 2 seconds */
  delay(2000);
  
//----- Restore 1/16 microstepping mode

  /* Reset shield 0 to 1/16 microstepping mode */
  myL6474.SelectStepMode(0,L6474_STEP_SEL_1_16);    

  /* Update speed, acceleration, deceleration for 1/16 microstepping mode*/
  myL6474.SetMaxSpeed(0,1600);
  myL6474.SetMinSpeed(0,800);
  myL6474.SetAcceleration(0,160);
  myL6474.SetDeceleration(0,160);

}

void loop()
{
  /* Request shield 0 to go position -6400 */
  myL6474.GoTo(0,-6400);

  /* Wait for the motor of shield 0 ends moving */
  myL6474.WaitWhileActive(0);

  /* Request shield 0 to go position 6400 */
  myL6474.GoTo(0,6400);

  /* Wait for the motor of shield 0 ends moving */
  myL6474.WaitWhileActive(0);  
}

void MyFlagInterruptHandler(void)
{
  /* Get the value of the status register via the L6474 command GET_STATUS */
  uint16_t statusRegister = myL6474.CmdGetStatus(0);

  /* Check HIZ flag: if set, power brigdes are disabled */
  if ((statusRegister & L6474_STATUS_HIZ) == L6474_STATUS_HIZ)
  {
    // HIZ state
  }

  /* Check direction bit */
  if ((statusRegister & L6474_STATUS_DIR) == L6474_STATUS_DIR)
  {
    // Forward direction is set
  }  
  else
  {
    // Backward direction is set
  }  

  /* Check NOTPERF_CMD flag: if set, the command received by SPI can't be performed */
  /* This often occures when a command is sent to the L6474 */
  /* while it is in HIZ state */
  if ((statusRegister & L6474_STATUS_NOTPERF_CMD) == L6474_STATUS_NOTPERF_CMD)
  {
       // Command received by SPI can't be performed
  }  

  /* Check WRONG_CMD flag: if set, the command does not exist */
  if ((statusRegister & L6474_STATUS_WRONG_CMD) == L6474_STATUS_WRONG_CMD)
  {
     //command received by SPI does not exist 
  }  

  /* Check UVLO flag: if not set, there is an undervoltage lock-out */
  if ((statusRegister & L6474_STATUS_UVLO) == 0)
  {
     //undervoltage lock-out 
  }  

  /* Check TH_WRN flag: if not set, the thermal warning threshold is reached */
  if ((statusRegister & L6474_STATUS_TH_WRN) == 0)
  {
    //thermal warning threshold is reached
  }    

  /* Check TH_SHD flag: if not set, the thermal shut down threshold is reached */
  if ((statusRegister & L6474_STATUS_TH_SD) == 0)
  {
    //thermal shut down threshold is reached * 
  }    

  /* Check OCD  flag: if not set, there is an overcurrent detection */
  if ((statusRegister & L6474_STATUS_OCD) == 0)
  {
    //overcurrent detection 
  }      
}

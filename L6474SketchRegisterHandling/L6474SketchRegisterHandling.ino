/* This sketch is an example use of the L6474 library to read and modify the L6474 registers */
/* It can be used with one shield X-NUCLEO-IHM01A1 and drives one stepper motor */
/* It sets ABS_POS, EL_POS and MARK registers to 1 */
/* It replaces the predefined values of some registers by the ones of shield 1 */ 
/* if they are different of those of shield 0 */
/* Once done, in the loop function, the motor is requested to alternate between 2 positions */

#include <l6474.h>
#include <SPI.h>

L6474 myL6474;

void setup()
{
  uint32_t registerValue;
  
  /* Start the L6474 library to use 1 shield */
  /* The L6474 registers are set with the predefined values */
  /* from file L6474_target_config.h*/
  myL6474.Begin(1);

  /* Read ABS_POS register of shield 0*/  
  registerValue = myL6474.CmdGetParam(0, L6474_ABS_POS);

  if (registerValue != 1)
  {
    /* Set  ABS_POS register of shield 0 to 1 */
    myL6474.CmdSetParam(0, L6474_ABS_POS, 1);
  }
  
  /* Read EL_POS register of shield 0*/  
  registerValue = myL6474.CmdGetParam(0, L6474_EL_POS);

  if (registerValue != 0)
  {
    /* Set  EL_POS register of shield 0 to 1 */
    myL6474.CmdSetParam(0, L6474_EL_POS, 1);
  }

  /* Read MARK register of shield 0*/  
  registerValue = myL6474.CmdGetParam(0, L6474_MARK);

  if (registerValue != 1)
  {
    /* Set  MARK register of shield 0 to 0 */
    myL6474.CmdSetParam(0, L6474_MARK, 1);
  }

  /* Read T_FAST register of shield 0*/  
  registerValue = myL6474.CmdGetParam(0,L6474_T_FAST);  

  if (registerValue != ((uint8_t)L6474_CONF_PARAM_TOFF_FAST_SHIELD_1 |
                        (uint8_t)L6474_CONF_PARAM_FAST_STEP_SHIELD_1))
  {
    /* Set  T_FAST register of shield 0 to predefined value of shield 1 */
    myL6474.CmdSetParam(0, L6474_T_FAST, (uint8_t)L6474_CONF_PARAM_TOFF_FAST_SHIELD_1 |
                                         (uint8_t)L6474_CONF_PARAM_FAST_STEP_SHIELD_1);
  }


  /* Read OCD_TH register of shield 0*/  
  registerValue = myL6474.CmdGetParam(0, L6474_OCD_TH);  

  if (registerValue != L6474_CONF_PARAM_OCD_TH_SHIELD_1)
  {
   /* Set  OCD_TH register of shield 0 to predefined value of shield 1 */
    myL6474.CmdSetParam(0, L6474_OCD_TH, L6474_CONF_PARAM_OCD_TH_SHIELD_1);
  }

  /* Read STEP_MODE register of shield 0*/  
  registerValue = myL6474.CmdGetParam(0,L6474_STEP_MODE);    

  if (registerValue != ((uint8_t)L6474_CONF_PARAM_STEP_SEL_SHIELD_1 |
                        (uint8_t)L6474_CONF_PARAM_SYNC_SEL_SHIELD_1))
  {
   /* Set  STEP_MODE register of shield 0 to predefined value of shield 1*/
    myL6474.CmdSetParam(0, L6474_STEP_MODE, (uint8_t)L6474_CONF_PARAM_STEP_SEL_SHIELD_1 |
                                            (uint8_t)L6474_CONF_PARAM_SYNC_SEL_SHIELD_1);
  }

  /* Read ALARM_EN register of shield 0*/  
  registerValue = myL6474.CmdGetParam(0, L6474_ALARM_EN);    

  if (registerValue != L6474_CONF_PARAM_ALARM_EN_SHIELD_1)
  {
   /* Set ALARM_EN register of shield 0 to predefined value of shield 1 */
    myL6474.CmdSetParam(0, L6474_ALARM_EN, L6474_CONF_PARAM_ALARM_EN_SHIELD_1);
  }

  /* Read CONFIG register of shield 0*/  
  registerValue = myL6474.CmdGetParam(0, L6474_CONFIG);    

  if (registerValue != ((uint16_t)L6474_CONF_PARAM_CLOCK_SETTING_SHIELD_1 |
                        (uint16_t)L6474_CONF_PARAM_TQ_REG_SHIELD_1 |
                        (uint16_t)L6474_CONF_PARAM_OC_SD_SHIELD_1 |
                        (uint16_t)L6474_CONF_PARAM_SR_SHIELD_1 |
                        (uint16_t)L6474_CONF_PARAM_TOFF_SHIELD_1))
  {
     /* Set CONFIG register of shield 0 to predefined value of shield 1 */
     myL6474.CmdSetParam(0, L6474_CONFIG, (uint16_t)L6474_CONF_PARAM_CLOCK_SETTING_SHIELD_1 |
                                          (uint16_t)L6474_CONF_PARAM_TQ_REG_SHIELD_1 |
                                          (uint16_t)L6474_CONF_PARAM_OC_SD_SHIELD_1 |
                                          (uint16_t)L6474_CONF_PARAM_SR_SHIELD_1 |
                                          (uint16_t)L6474_CONF_PARAM_TOFF_SHIELD_1);
  }
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

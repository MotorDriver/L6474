/******************************************************//**
 * @file    l6474.cpp 
 * @version V1.0
 * @date    March 3, 2014
 * @brief   L6474 library for arduino 
 *
 * This file is free software; you can redistribute it and/or modify
 * it under the terms of either the GNU General Public License version 2
 * or the GNU Lesser General Public License version 2.1, both as
 * published by the Free Software Foundation.
 **********************************************************/ 

#include "l6474.h"
#include <SPI.h>

#ifdef _DEBUG_L6474
/// Log buffer
char l6474StrOut[DEBUG_BUFFER_SIZE];
#endif

const uint16_t L6474::prescalerArrayTimer0_1[PRESCALER_ARRAY_TIMER0_1_SIZE] = { 0, 1, 8, 64, 256, 1024};
const uint16_t L6474::prescalerArrayTimer2[PRESCALER_ARRAY_TIMER2_SIZE] = {0, 1, 8, 32, 64, 128, 256, 1024};
volatile void (*L6474::flagInterruptCallback)(void);
volatile uint8_t L6474::numberOfShields;
uint8_t L6474::spiTxBursts[L6474_CMD_ARG_MAX_NB_BYTES][MAX_NUMBER_OF_SHIELDS];
uint8_t L6474::spiRxBursts[L6474_CMD_ARG_MAX_NB_BYTES][MAX_NUMBER_OF_SHIELDS];
volatile bool L6474::spiPreemtionByIsr = false;
volatile bool L6474::isrFlag = false;
volatile class L6474* L6474::instancePtr = NULL;
  
/******************************************************//**
 * @brief  Constructor
 * @param  None
 * @retval None
 **********************************************************/ 
L6474::L6474()
{
  uint8_t i;
  for (i = 0; i < MAX_NUMBER_OF_SHIELDS; i++)
  {
    shieldPrm[i].motionState = INACTIVE;
    shieldPrm[i].commandExecuted = NO_CMD;
    shieldPrm[i].stepsToTake = MAX_STEPS;
  }
    instancePtr = this;
}

/******************************************************//**
 * @brief  Attaches a user callback to the flag Interrupt
 * The call back will be then called each time the status 
 * flag pin will be pulled down due to the occurrence of 
 * a programmed alarms ( OCD, thermal pre-warning or 
 * shutdown, UVLO, wrong command, non-performable command)
 * @param[in] callback Name of the callback to attach 
 * to the Flag Interrupt
 * @retval None
 **********************************************************/
void L6474::AttachFlagInterrupt(void (*callback)(void))
{
  flagInterruptCallback = (volatile void (*)())callback;
}

/******************************************************//**
 * @brief Starts the L6474 library
 * @param[in] nbShields Number of L6474 shields to use (from 1 to 3)
 * @retval None
 **********************************************************/
void L6474::Begin(uint8_t nbShields)
{
  numberOfShields = nbShields;
  
  // start the SPI library:
  SPI.begin();
  SPI.setBitOrder(MSBFIRST);
  SPI.setDataMode(SPI_MODE3);
  SPI.setClockDivider(SPI_CLOCK_DIV4);
  
  // flag pin
  pinMode(L6474_FLAG_Pin, INPUT_PULLUP);
  attachInterrupt(0, FlagInterruptHandler, FALLING);
  
  //reset pin
  pinMode(L6474_Reset_Pin, OUTPUT);
  
  switch (nbShields)
  {
    case 3:
      pinMode(L6474_DIR_3_Pin, OUTPUT);
      pinMode(L6474_PWM_3_Pin, OUTPUT);
      PwmInit(2);
    case 2:
      pinMode(L6474_DIR_2_Pin, OUTPUT);
      pinMode(L6474_PWM_2_Pin, OUTPUT);
      PwmInit(1);
    case 1:
      pinMode(L6474_DIR_1_Pin, OUTPUT);
      pinMode(L6474_PWM_1_Pin, OUTPUT);
      PwmInit(0);
    default:
      ;
  }
  
  /* Standby-reset deactivation */
  ReleaseReset();
  
  /* Set all registers and context variables to the predefined values from l6474_target_config.h */
  SetShieldParamsToPredefinedValues();
  
  /* Disable L6474 powerstage */
  for (uint32_t i = 0; i < nbShields; i++)
  {
    CmdDisable(i);
    /* Get Status to clear flags after start up */
    CmdGetStatus(i);
  }
}

/******************************************************//**
 * @brief Returns the acceleration of the specified shield
 * @param[in] shieldId (from 0 to 2)
 * @retval Acceleration in pps^2
 **********************************************************/
uint16_t L6474::GetAcceleration(uint8_t shieldId)
{                                                  
  return (shieldPrm[shieldId].acceleration);
}            

/******************************************************//**
 * @brief Returns the current speed of the specified shield
 * @param[in] shieldId (from 0 to 2)
 * @retval Speed in pps
 **********************************************************/
uint16_t L6474::GetCurrentSpeed(uint8_t shieldId)
{
  return shieldPrm[shieldId].speed;
}

/******************************************************//**
 * @brief Returns the deceleration of the specified shield
 * @param[in] shieldId (from 0 to 2)
 * @retval Deceleration in pps^2
 **********************************************************/
uint16_t L6474::GetDeceleration(uint8_t shieldId)
{                                                  
  return (shieldPrm[shieldId].deceleration);
}          

/******************************************************//**
 * @brief Returns the FW version of the library
 * @param None
 * @retval L6474_FW_VERSION
 **********************************************************/
uint8_t L6474::GetFwVersion(void)
{
  return (L6474_FW_VERSION);
}

/******************************************************//**
 * @brief  Returns the mark position  of the specified shield
 * @param[in] shieldId (from 0 to 2)
 * @retval Mark register value converted in a 32b signed integer 
 **********************************************************/
int32_t L6474::GetMark(uint8_t shieldId)
{
  return ConvertPosition(CmdGetParam(shieldId,L6474_MARK));
}

/******************************************************//**
 * @brief  Returns the max speed of the specified shield
 * @param[in] shieldId (from 0 to 2)
 * @retval maxSpeed in pps
 **********************************************************/
uint16_t L6474::GetMaxSpeed(uint8_t shieldId)
{                                                  
  return (shieldPrm[shieldId].maxSpeed);
}                                                     

/******************************************************//**
 * @brief  Returns the min speed of the specified shield
 * @param[in] shieldId (from 0 to 2)
 * @retval minSpeed in pps
 **********************************************************/
uint16_t L6474::GetMinSpeed(uint8_t shieldId)
{                                                  
  return (shieldPrm[shieldId].minSpeed);
}                                                     

/******************************************************//**
 * @brief  Returns the ABS_POSITION of the specified shield
 * @param[in] shieldId (from 0 to 2)
 * @retval ABS_POSITION register value converted in a 32b signed integer
 **********************************************************/
int32_t L6474::GetPosition(uint8_t shieldId)
{
  return ConvertPosition(CmdGetParam(shieldId,L6474_ABS_POS));
}

/******************************************************//**
 * @brief Returns the shield state
 * @param[in] shieldId (from 0 to 2)
 * @retval State (ACCELERATING, DECELERATING, STEADY or INACTIVE)
 **********************************************************/
shieldState_t L6474::GetShieldState(uint8_t shieldId)
{
  return shieldPrm[shieldId].motionState;
}

/******************************************************//**
 * @brief  Requests the motor to move to the home position (ABS_POSITION = 0)
 * @param[in] shieldId (from 0 to 2)
 * @retval None
 **********************************************************/
void L6474::GoHome(uint8_t shieldId)
{
  GoTo(shieldId, 0);
} 
  
/******************************************************//**
 * @brief  Requests the motor to move to the mark position 
 * @param[in] shieldId (from 0 to 2)
 * @retval None
 **********************************************************/
void L6474::GoMark(uint8_t shieldId)
{
	uint32_t mark;

	mark = ConvertPosition(CmdGetParam(shieldId,L6474_MARK));
	GoTo(shieldId,mark);  
}

/******************************************************//**
 * @brief  Requests the motor to move to the specified position 
 * @param[in] shieldId (from 0 to 2)
 * @param[in] targetPosition absolute position in steps
 * @retval None
 **********************************************************/
void L6474::GoTo(uint8_t shieldId, int32_t targetPosition)
{
  dir_t direction;
  int32_t steps;
  
  /* Eventually deactivate motor */
  if (shieldPrm[shieldId].motionState != INACTIVE) 
  {
    HardStop(shieldId);
  }

  /* Get current position */
  shieldPrm[shieldId].currentPosition = ConvertPosition(CmdGetParam(shieldId,L6474_ABS_POS));
  
  /* Compute the number of steps to perform */
  steps = targetPosition - shieldPrm[shieldId].currentPosition;
  
  if (steps >= 0) 
  {
    shieldPrm[shieldId].stepsToTake = steps;
    direction = FORWARD;
    
  } 
  else 
  {
    shieldPrm[shieldId].stepsToTake = -steps;
    direction = BACKWARD;
  }
  
  if (steps != 0) 
  {
    
    shieldPrm[shieldId].commandExecuted = MOVE_CMD;
        
    /* Direction setup */
    SetDirection(shieldId,direction);

    ComputeSpeedProfile(shieldId, shieldPrm[shieldId].stepsToTake);
    
    /* Motor activation */
    StartMovement(shieldId);
  }  
}

/******************************************************//**
 * @brief  Immediatly stops the motor and disable the power bridge
 * @param[in] shieldId (from 0 to 2)
 * @retval None
 **********************************************************/
void L6474::HardStop(uint8_t shieldId) 
{
  /* Disable corresponding PWM */
  PwmStop(shieldId);

  /* Disable power stage */
  CmdDisable(shieldId);

  /* Set inactive state */
  shieldPrm[shieldId].motionState = INACTIVE;
  shieldPrm[shieldId].commandExecuted = NO_CMD;
  shieldPrm[shieldId].stepsToTake = MAX_STEPS;  

#ifdef _DEBUG_L6474
 Serial.println("Inactive\n");
#endif     
}

/******************************************************//**
 * @brief  Moves the motor of the specified number of steps
 * @param[in] shieldId (from 0 to 2)
 * @param[in] direction FORWARD or BACKWARD
 * @param[in] stepCount Number of steps to perform
 * @retval None
 **********************************************************/
void L6474::Move(uint8_t shieldId, dir_t direction, uint32_t stepCount)
{
  /* Eventually deactivate motor */
  if (shieldPrm[shieldId].motionState != INACTIVE) 
  {
    HardStop(shieldId);
  }
  
  if (stepCount != 0) 
  {
    shieldPrm[shieldId].stepsToTake = stepCount;
    
    shieldPrm[shieldId].commandExecuted = MOVE_CMD;
    
    shieldPrm[shieldId].currentPosition = ConvertPosition(CmdGetParam(shieldId,L6474_ABS_POS));
    
    /* Direction setup */
    SetDirection(shieldId,direction);

    ComputeSpeedProfile(shieldId, stepCount);
    
    /* Motor activation */
    StartMovement(shieldId);
  }  
}

/******************************************************//**
 * @brief Resets all L6474 shields
 * @param None
 * @retval None
 **********************************************************/
void L6474::ResetAllShields(void)
{
 	uint8_t loop;
 	
 	for (loop = 0; loop < numberOfShields; loop++)
 	{
   	/* Stop movement and disable power stage*/
  	HardStop(loop);
  }
	Reset();
	WaitUs(20); // Reset pin must be forced low for at least 10us
	ReleaseReset();
}

/******************************************************//**
 * @brief  Runs the motor. It will accelerate from the min 
 * speed up to the max speed by using the shield acceleration.
 * @param[in] shieldId (from 0 to 2)
 * @param[in] direction FORWARD or BACKWARD
 * @retval None
 **********************************************************/
void L6474::Run(uint8_t shieldId, dir_t direction)
{
  /* Eventually deactivate motor */
  if (shieldPrm[shieldId].motionState != INACTIVE) 
  {
    HardStop(shieldId);
  }
  
	/* Direction setup */
	SetDirection(shieldId,direction);

	shieldPrm[shieldId].commandExecuted = RUN_CMD;

	/* Motor activation */
	StartMovement(shieldId); 
}

/******************************************************//**
 * @brief  Changes the acceleration of the specified shield
 * @param[in] shieldId (from 0 to 2)
 * @param[in] newAcc New acceleration to apply in pps^2
 * @retval true if the command is successfully executed, else false
 * @note The command is not performed is the shield is executing 
 * a MOVE or GOTO command (but it can be used during a RUN command)
 **********************************************************/
bool L6474::SetAcceleration(uint8_t shieldId,uint16_t newAcc)
{                                                  
  bool cmdExecuted = false;
  if ((newAcc != 0)&&
      ((shieldPrm[shieldId].motionState == INACTIVE)||
       (shieldPrm[shieldId].commandExecuted == RUN_CMD)))
  {
    shieldPrm[shieldId].acceleration = newAcc;
    cmdExecuted = true;
  }    
  return cmdExecuted;
}            

/******************************************************//**
 * @brief  Changes the deceleration of the specified shield
 * @param[in] shieldId (from 0 to 2)
 * @param[in] newDec New deceleration to apply in pps^2
 * @retval true if the command is successfully executed, else false
 * @note The command is not performed is the shield is executing 
 * a MOVE or GOTO command (but it can be used during a RUN command)
 **********************************************************/
bool L6474::SetDeceleration(uint8_t shieldId, uint16_t newDec)
{                                                  
  bool cmdExecuted = false;
  if ((newDec != 0)&& 
      ((shieldPrm[shieldId].motionState == INACTIVE)||
       (shieldPrm[shieldId].commandExecuted == RUN_CMD)))
  {
    shieldPrm[shieldId].deceleration = newDec;
    cmdExecuted = true;
  }      
  return cmdExecuted;
}        

/******************************************************//**
 * @brief  Set current position to be the Home position (ABS pos set to 0)
 * @param[in] shieldId (from 0 to 2)
 * @retval None
 **********************************************************/
void L6474::SetHome(uint8_t shieldId)
{
  CmdSetParam(shieldId, L6474_ABS_POS, 0);
}
 
/******************************************************//**
 * @brief  Sets current position to be the Mark position 
 * @param[in] shieldId (from 0 to 2)
 * @retval None
 **********************************************************/
void L6474::SetMark(uint8_t shieldId)
{
  uint32_t mark = CmdGetParam(shieldId,L6474_ABS_POS);
  CmdSetParam(shieldId,L6474_MARK, mark);
}

/******************************************************//**
 * @brief  Changes the max speed of the specified shield
 * @param[in] shieldId (from 0 to 2)
 * @param[in] newMaxSpeed New max speed  to apply in pps
 * @retval true if the command is successfully executed, else false
 * @note The command is not performed is the shield is executing 
 * a MOVE or GOTO command (but it can be used during a RUN command).
 **********************************************************/
bool L6474::SetMaxSpeed(uint8_t shieldId, uint16_t newMaxSpeed)
{                                                  
  bool cmdExecuted = false;
  if ((newMaxSpeed > L6474_MIN_PWM_FREQ)&&
      (newMaxSpeed <= L6474_MAX_PWM_FREQ) &&
      (shieldPrm[shieldId].minSpeed <= newMaxSpeed) &&
      ((shieldPrm[shieldId].motionState == INACTIVE)||
       (shieldPrm[shieldId].commandExecuted == RUN_CMD)))
  {
    shieldPrm[shieldId].maxSpeed = newMaxSpeed;
    cmdExecuted = true;
  }
  return cmdExecuted;
}                                                     

/******************************************************//**
 * @brief  Changes the min speed of the specified shield
 * @param[in] shieldId (from 0 to 2)
 * @param[in] newMinSpeed New min speed  to apply in pps
 * @retval true if the command is successfully executed, else false
 * @note The command is not performed is the shield is executing 
 * a MOVE or GOTO command (but it can be used during a RUN command).
 **********************************************************/
bool L6474::SetMinSpeed(uint8_t shieldId, uint16_t newMinSpeed)
{                                                  
  bool cmdExecuted = false;
  if ((newMinSpeed >= L6474_MIN_PWM_FREQ)&&
      (newMinSpeed < L6474_MAX_PWM_FREQ) &&
      (newMinSpeed <= shieldPrm[shieldId].maxSpeed) && 
      ((shieldPrm[shieldId].motionState == INACTIVE)||
       (shieldPrm[shieldId].commandExecuted == RUN_CMD)))
  {
    shieldPrm[shieldId].minSpeed = newMinSpeed;
    cmdExecuted = true;
  }  
  return cmdExecuted;
}                 

/******************************************************//**
 * @brief  Stops the motor by using the shield deceleration
 * @param[in] shieldId (from 0 to 2)
 * @retval true if the command is successfully executed, else false
 * @note The command is not performed is the shield is in INACTIVE state.
 **********************************************************/
bool L6474::SoftStop(uint8_t shieldId)
{	
  bool cmdExecuted = false;
  if (shieldPrm[shieldId].motionState != INACTIVE)
  {
    shieldPrm[shieldId].commandExecuted = SOFT_STOP_CMD;
    cmdExecuted = true;
  }
  return (cmdExecuted);
}

/******************************************************//**
 * @brief  Locks until the shield state becomes Inactive
 * @param[in] shieldId (from 0 to 2)
 * @retval None
 **********************************************************/
void L6474::WaitWhileActive(uint8_t shieldId)
 {
	/* Wait while motor is running */
	while (GetShieldState(shieldId) != INACTIVE);
}

/******************************************************//**
 * @brief  Issue the Disable command to the L6474 of the specified shield
 * @param[in] shieldId (from 0 to 2)
 * @retval None
 **********************************************************/
void L6474::CmdDisable(uint8_t shieldId)
{
  SendCommand(shieldId, L6474_DISABLE);
}

/******************************************************//**
 * @brief  Issues the Enable command to the L6474 of the specified shield
 * @param[in] shieldId (from 0 to 2)
 * @retval None
 **********************************************************/
void L6474::CmdEnable(uint8_t shieldId)
{
  SendCommand(shieldId, L6474_ENABLE);
}

/******************************************************//**
 * @brief  Issues the GetParam command to the L6474 of the specified shield
 * @param[in] shieldId (from 0 to 2)
 * @param[in] param Register adress (L6474_ABS_POS, L6474_MARK,...)
 * @retval Register value
 **********************************************************/
uint32_t L6474::CmdGetParam(uint8_t shieldId, L6474_Registers_t param)
{
  uint32_t i;
  uint32_t spiRxData;
  uint8_t maxArgumentNbBytes = 0;
  uint8_t spiIndex = numberOfShields - shieldId - 1;
  bool itDisable = false;  
  
  do
  {
    spiPreemtionByIsr = false;
    if (itDisable)
    {
      /* re-enable interrupts if disable in previous iteration */
      interrupts();
      itDisable = false;
    }
  
    for (i = 0; i < numberOfShields; i++)
    {
      spiTxBursts[0][i] = L6474_NOP;
      spiTxBursts[1][i] = L6474_NOP;
      spiTxBursts[2][i] = L6474_NOP;
      spiTxBursts[3][i] = L6474_NOP;
      spiRxBursts[1][i] = 0;
      spiRxBursts[2][i] = 0;
      spiRxBursts[3][i] = 0;    
    }
    switch (param)
    {
      case L6474_ABS_POS: ;
      case L6474_MARK:
        spiTxBursts[0][spiIndex] = ((uint8_t)L6474_GET_PARAM )| (param);
        maxArgumentNbBytes = 3;
        break;
      case L6474_EL_POS: ;
      case L6474_CONFIG: ;
      case L6474_STATUS:
        spiTxBursts[1][spiIndex] = ((uint8_t)L6474_GET_PARAM )| (param);
        maxArgumentNbBytes = 2;
        break;
      default:
        spiTxBursts[2][spiIndex] = ((uint8_t)L6474_GET_PARAM )| (param);
        maxArgumentNbBytes = 1;
    }
    
    /* Disable interruption before checking */
    /* pre-emption by ISR and SPI transfers*/
    noInterrupts();
    itDisable = true;
  } while (spiPreemtionByIsr); // check pre-emption by ISR
    
  for (i = L6474_CMD_ARG_MAX_NB_BYTES-1-maxArgumentNbBytes;
       i < L6474_CMD_ARG_MAX_NB_BYTES;
       i++)
  {
     WriteBytes(&spiTxBursts[i][0],
                          &spiRxBursts[i][0]);
  }
  
  spiRxData = ((uint32_t)spiRxBursts[1][spiIndex] << 16)|
              (spiRxBursts[2][spiIndex] << 8) |
              (spiRxBursts[3][spiIndex]);
  
  /* re-enable interrupts after SPI transfers*/
  interrupts();
    
  return (spiRxData);
}

/******************************************************//**
 * @brief  Issues the GetStatus command to the L6474 of the specified shield
 * @param[in] shieldId (from 0 to 2)
 * @retval Status Register value
 * @note Once the GetStatus command is performed, the flags of the status register
 * are reset. This is not the case when the status register is read with the
 * GetParam command (via the functions ReadStatusRegister or CmdGetParam).
 **********************************************************/
uint16_t L6474::CmdGetStatus(uint8_t shieldId)
{
  uint32_t i;
  uint16_t status;
  uint8_t spiIndex = numberOfShields - shieldId - 1;
  bool itDisable = false;  
  
  do
  {
    spiPreemtionByIsr = false;
    if (itDisable)
    {
      /* re-enable interrupts if disable in previous iteration */
      interrupts();
      itDisable = false;
    }

    for (i = 0; i < numberOfShields; i++)
    {
       spiTxBursts[0][i] = L6474_NOP;
       spiTxBursts[1][i] = L6474_NOP;
       spiTxBursts[2][i] = L6474_NOP;
       spiRxBursts[1][i] = 0;
       spiRxBursts[2][i] = 0;
    }
    spiTxBursts[0][spiIndex] = L6474_GET_STATUS;

    /* Disable interruption before checking */
    /* pre-emption by ISR and SPI transfers*/
    noInterrupts();
    itDisable = true;
  } while (spiPreemtionByIsr); // check pre-emption by ISR

  for (i = 0; i < L6474_CMD_ARG_NB_BYTES_GET_STATUS + L6474_RSP_NB_BYTES_GET_STATUS; i++)
  {
     WriteBytes(&spiTxBursts[i][0], &spiRxBursts[i][0]);
  }
  status = (spiRxBursts[1][spiIndex] << 8) | (spiRxBursts[2][spiIndex]);
  
  /* re-enable interrupts after SPI transfers*/
  interrupts();
  
  return (status);
}

/******************************************************//**
 * @brief  Issues the Nop command to the L6474 of the specified shield
 * @param[in] shieldId (from 0 to 2)
 * @retval None
 **********************************************************/
void L6474::CmdNop(uint8_t shieldId)
{
  SendCommand(shieldId, L6474_NOP);
}

/******************************************************//**
 * @brief  Issues the SetParam command to the L6474 of the specified shield
 * @param[in] shieldId (from 0 to 2)
 * @param[in] param Register adress (L6474_ABS_POS, L6474_MARK,...)
 * @param[in] value Value to set in the register
 * @retval None
 **********************************************************/
void L6474::CmdSetParam(uint8_t shieldId,
                        L6474_Registers_t param,
                        uint32_t value)
{
  uint32_t i;
  uint8_t maxArgumentNbBytes = 0;
  uint8_t spiIndex = numberOfShields - shieldId - 1;
  bool itDisable = false;  
  do
  {
    spiPreemtionByIsr = false;
    if (itDisable)
    {
      /* re-enable interrupts if disable in previous iteration */
      interrupts();
      itDisable = false;
    }
    for (i = 0; i < numberOfShields; i++)
    {
      spiTxBursts[0][i] = L6474_NOP;
      spiTxBursts[1][i] = L6474_NOP;
      spiTxBursts[2][i] = L6474_NOP;
      spiTxBursts[3][i] = L6474_NOP;
    }
    switch (param)
  {
    case L6474_ABS_POS: ;
    case L6474_MARK:
        spiTxBursts[0][spiIndex] = param;
        spiTxBursts[1][spiIndex] = (uint8_t)(value >> 16);
        spiTxBursts[2][spiIndex] = (uint8_t)(value >> 8);
        maxArgumentNbBytes = 3;
        break;
    case L6474_EL_POS: ;
    case L6474_CONFIG:
        spiTxBursts[1][spiIndex] = param;
        spiTxBursts[2][spiIndex] = (uint8_t)(value >> 8);
        maxArgumentNbBytes = 2;
        break;
    default:
        spiTxBursts[2][spiIndex] = param;
        maxArgumentNbBytes = 1;
    }
    spiTxBursts[3][spiIndex] = (uint8_t)(value);
    
    /* Disable interruption before checking */
    /* pre-emption by ISR and SPI transfers*/
    noInterrupts();
    itDisable = true;
  } while (spiPreemtionByIsr); // check pre-emption by ISR
 
  /* SPI transfer */
  for (i = L6474_CMD_ARG_MAX_NB_BYTES-1-maxArgumentNbBytes;
       i < L6474_CMD_ARG_MAX_NB_BYTES;
       i++)
  {
     WriteBytes(&spiTxBursts[i][0],&spiRxBursts[i][0]);
  }
  /* re-enable interrupts after SPI transfers*/
  interrupts();
}

/******************************************************//**
 * @brief  Reads the Status Register value
 * @param[in] shieldId (from 0 to 2)
 * @retval Status register valued
 * @note The status register flags are not cleared 
 * at the difference with CmdGetStatus()
 **********************************************************/
uint16_t L6474::ReadStatusRegister(uint8_t shieldId)
{
  return (CmdGetParam(shieldId,L6474_STATUS));
}

/******************************************************//**
 * @brief  Releases the L6474 reset (pin set to High) of all shields
 * @param  None
 * @retval None
 **********************************************************/
void L6474::ReleaseReset(void)
{ 
  digitalWrite(L6474_Reset_Pin, HIGH);
}

/******************************************************//**
 * @brief  Resets the L6474 (reset pin set to low) of all shields
 * @param  None
 * @retval None
 **********************************************************/
void L6474::Reset(void)
{
  digitalWrite(L6474_Reset_Pin, LOW);
}

/******************************************************//**
 * @brief  Set the stepping mode 
 * @param[in] shieldId (from 0 to 2)
 * @param[in] stepMod from full step to 1/16 microstep as specified in enum L6474_STEP_SEL_t
 * @retval None
 **********************************************************/
void L6474::SelectStepMode(uint8_t shieldId, L6474_STEP_SEL_t stepMod)
{
  uint8_t stepModeRegister;
  
  /* Eventually deactivate motor */
  if (shieldPrm[shieldId].motionState != INACTIVE) 
  {
    HardStop(shieldId);
  }
  
  /* Read Step mode register and clear STEP_SEL field */
  stepModeRegister = (uint8_t)(0xF8 & CmdGetParam(shieldId,L6474_STEP_MODE)) ;
  
  /* Apply new step mode */
  CmdSetParam(shieldId, L6474_STEP_MODE, stepModeRegister | (uint8_t)stepMod);

  /* Reset abs pos register */
  SetHome(shieldId);
}

/******************************************************//**
 * @brief  Specifies the direction 
 * @param[in] shieldId (from 0 to 2)
 * @param[in] dir FORWARD or BACKWARD
 * @note The direction change is only applied if the shield 
 * is in INACTIVE state
 * @retval None
 **********************************************************/
void L6474::SetDirection(uint8_t shieldId, dir_t dir)
{
  if (shieldPrm[shieldId].motionState == INACTIVE)
  {
    shieldPrm[shieldId].direction = dir;
    
    switch (shieldId)
    {
      case 2:
        digitalWrite(L6474_DIR_3_Pin, dir);
        break;
      case 1:
        digitalWrite(L6474_DIR_2_Pin, dir);
        break;
      case 0:
        digitalWrite(L6474_DIR_1_Pin, dir);
        break;
      default:
        ;
    }
  }
}

/******************************************************//**
 * @brief  Waits for the specify delay in milliseconds
 * @param[in] msDelay delay in milliseconds
 * @retval None
 * @note Should only be used for 3 shields configuration.
 * Else, prefer the standard Arduino function delay().
 **********************************************************/
void L6474::WaitMs(uint16_t msDelay)
{
  uint16_t i;
  for (i = 0; i < msDelay ; i++)
  {
    WaitUs(1000);
  }
}

/******************************************************//**
 * @brief  Waits for the specify delay in microseconds
 * @param[in] usDelay delay in microseconds
 * @retval None
 * @note Should be only used for 3 shields configuration.
 * Else, prefer the standard Arduino function delayMicroseconds().
 * Besides, this function is a copy of delayMicroseconds inside
 * the L6474 library to avoid dependencies conflicts 
 * (a redefinition of ISR(TIMER0_OVF_vect)). 
 **********************************************************/
void L6474::WaitUs(uint16_t usDelay)
{
	// calling avrlib's delay_us() function with low values (e.g. 1 or
	// 2 microseconds) gives delays longer than desired.
	//delay_us(us);
#if F_CPU >= 20000000L
	// for the 20 MHz clock on rare Arduino boards

	// for a one-microsecond delay, simply wait 2 cycle and return. The overhead
	// of the function call yields a delay of exactly a one microsecond.
	__asm__ __volatile__ (
		"nop" "\n\t"
		"nop"); //just waiting 2 cycle
	if (--usDelay == 0)
		return;

	// the following loop takes a 1/5 of a microsecond (4 cycles)
	// per iteration, so execute it five times for each microsecond of
	// delay requested.
	usDelay = (usDelay<<2) + usDelay; // x5 us

	// account for the time taken in the preceeding commands.
	usDelay -= 2;

#elif F_CPU >= 16000000L
	// for the 16 MHz clock on most Arduino boards

	// for a one-microsecond delay, simply return.  the overhead
	// of the function call yields a delay of approximately 1 1/8 us.
	if (--usDelay == 0)
		return;

	// the following loop takes a quarter of a microsecond (4 cycles)
	// per iteration, so execute it four times for each microsecond of
	// delay requested.
	usDelay <<= 2;

	// account for the time taken in the preceeding commands.
	usDelay -= 2;
#else
	// for the 8 MHz internal clock on the ATmega168

	// for a one- or two-microsecond delay, simply return.  the overhead of
	// the function calls takes more than two microseconds.  can't just
	// subtract two, since us is unsigned; we'd overflow.
	if (--usDelay == 0)
		return;
	if (--usDelay == 0)
		return;

	// the following loop takes half of a microsecond (4 cycles)
	// per iteration, so execute it twice for each microsecond of
	// delay requested.
	usDelay <<= 1;
    
	// partially compensate for the time taken by the preceeding commands.
	// we can't subtract any more than this or we'd overflow w/ small delays.
	usDelay--;
#endif

	// busy wait
	__asm__ __volatile__ (
		"1: sbiw %0,1" "\n\t" // 2 cycles
		"brne 1b" : "=w" (usDelay) : "0" (usDelay) // 2 cycles
	);
}  
                  
/******************************************************//**
 * @brief  Gets the pointer to the L6474 instance
 * @param  None
 * @retval Pointer to the instance of L6474
  **********************************************************/
class L6474* L6474::GetInstancePtr(void)
{
  return (class L6474*)instancePtr;
}

/******************************************************//**
 * @brief  Handles the shield state machine at each ste
 * @param[in] shieldId (from 0 to 2)
 * @retval None
 * @note Must only be called by the timer ISR
 **********************************************************/
void L6474::StepClockHandler(uint8_t shieldId)
{
  /* Set isr flag */
  isrFlag = true;
  
  /* Incrementation of the relative position */
  shieldPrm[shieldId].relativePos++;
  
  /* Periodically check that estimated position is correct */
  if ((shieldPrm[shieldId].commandExecuted != RUN_CMD) && 
      ((shieldPrm[shieldId].relativePos % 10) == 00))
  {
    uint32_t AbsPos= ConvertPosition(CmdGetParam(shieldId,L6474_ABS_POS));
  
    /* Correct estimated position if needed */
    if (AbsPos != 0)
    {  
      if ((shieldPrm[shieldId].direction == FORWARD) && 
          (AbsPos != shieldPrm[shieldId].currentPosition + shieldPrm[shieldId].relativePos))
      {
#ifdef _DEBUG_L6474
        snprintf(l6474StrOut, DEBUG_BUFFER_SIZE, "F EstPos:%ld RealPos: %ld\n",shieldPrm[shieldId].relativePos,(AbsPos - shieldPrm[shieldId].currentPosition));
        Serial.println(l6474StrOut);  
#endif                
        shieldPrm[shieldId].relativePos = AbsPos - shieldPrm[shieldId].currentPosition;
        
      }
      else if ((shieldPrm[shieldId].direction == BACKWARD) && 
               (AbsPos != shieldPrm[shieldId].currentPosition - shieldPrm[shieldId].relativePos))
      {
#ifdef _DEBUG_L6474
        snprintf(l6474StrOut, DEBUG_BUFFER_SIZE, "B EstPos:%ld RealPos: %ld\n",shieldPrm[shieldId].relativePos,(AbsPos - shieldPrm[shieldId].currentPosition));
        Serial.println(l6474StrOut);  
#endif        
        shieldPrm[shieldId].relativePos = shieldPrm[shieldId].currentPosition - AbsPos;
      }
    }
  }
 
  switch (shieldPrm[shieldId].motionState) 
  {
    case ACCELERATING: 
    {
        if ((shieldPrm[shieldId].commandExecuted == SOFT_STOP_CMD)||
            ((shieldPrm[shieldId].commandExecuted != RUN_CMD)&&  
             (shieldPrm[shieldId].relativePos == shieldPrm[shieldId].startDecPos)))
        {
          shieldPrm[shieldId].motionState = DECELERATING;
          shieldPrm[shieldId].accu = 0;
#ifdef _DEBUG_L6474
          snprintf(l6474StrOut, DEBUG_BUFFER_SIZE, "Acc->Dec: speed: %u relativepos: %ld \n",shieldPrm[shieldId].speed,shieldPrm[shieldId].relativePos);
          Serial.println(l6474StrOut);  
#endif    
        }
        else if ((shieldPrm[shieldId].speed >= shieldPrm[shieldId].maxSpeed)||
                 ((shieldPrm[shieldId].commandExecuted != RUN_CMD)&&
                  (shieldPrm[shieldId].relativePos == shieldPrm[shieldId].endAccPos)))
        {
          shieldPrm[shieldId].motionState = STEADY;
#ifdef _DEBUG_L6474
        snprintf(l6474StrOut, DEBUG_BUFFER_SIZE, "Acc->Steady: speed: %u relativepos: %ld \n",shieldPrm[shieldId].speed,shieldPrm[shieldId].relativePos);
        Serial.println(l6474StrOut);  
#endif    
        }
        else
        {
          bool speedUpdated = false;
          /* Go on accelerating */
          shieldPrm[shieldId].accu += ((uint32_t)shieldPrm[shieldId].acceleration << 16) / shieldPrm[shieldId].speed;
          while (shieldPrm[shieldId].accu >= (0X10000L))
          {
            shieldPrm[shieldId].accu -= (0X10000L);
            shieldPrm[shieldId].speed +=1;
            speedUpdated = true;
          }
          
          if (speedUpdated)
          {
            ApplySpeed(shieldId, shieldPrm[shieldId].speed);
          }
        }
        break;
    }
    case STEADY: 
    {
      if  ((shieldPrm[shieldId].commandExecuted == SOFT_STOP_CMD)||
           ((shieldPrm[shieldId].commandExecuted != RUN_CMD)&&
            (shieldPrm[shieldId].relativePos >= (shieldPrm[shieldId].startDecPos))) ||
           ((shieldPrm[shieldId].commandExecuted == RUN_CMD)&&
            (shieldPrm[shieldId].speed > shieldPrm[shieldId].maxSpeed)))
      {
        shieldPrm[shieldId].motionState = DECELERATING;
        shieldPrm[shieldId].accu = 0;
      }
      else if ((shieldPrm[shieldId].commandExecuted == RUN_CMD)&&
               (shieldPrm[shieldId].speed < shieldPrm[shieldId].maxSpeed))
      {
        shieldPrm[shieldId].motionState = ACCELERATING;
        shieldPrm[shieldId].accu = 0;
      }
      break;
    }
    case DECELERATING: 
    {
      if (((shieldPrm[shieldId].commandExecuted == SOFT_STOP_CMD)&&(shieldPrm[shieldId].speed <=  shieldPrm[shieldId].minSpeed))||
          ((shieldPrm[shieldId].commandExecuted != RUN_CMD)&&
           (shieldPrm[shieldId].relativePos >= shieldPrm[shieldId].stepsToTake)))
      {
        /* Motion process complete */
        HardStop(shieldId);
#ifdef _DEBUG_L6474
        snprintf(l6474StrOut, DEBUG_BUFFER_SIZE, "Dec->Stop: speed: %u relativepos: %ld \n",shieldPrm[shieldId].speed,shieldPrm[shieldId].relativePos );
        Serial.println(l6474StrOut);  
#endif   
      }
      else if ((shieldPrm[shieldId].commandExecuted == RUN_CMD)&&
               (shieldPrm[shieldId].speed <= shieldPrm[shieldId].maxSpeed))
      {
        shieldPrm[shieldId].motionState = STEADY;
#ifdef _DEBUG_L6474
        snprintf(l6474StrOut, DEBUG_BUFFER_SIZE, "Dec->Steady: speed: %u relativepos: %ld \n",shieldPrm[shieldId].speed,shieldPrm[shieldId].relativePos);
        Serial.println(l6474StrOut);  
#endif            
      }
      else
      {
        /* Go on decelerating */
        if (shieldPrm[shieldId].speed > shieldPrm[shieldId].minSpeed)
        {
          bool speedUpdated = false;
          shieldPrm[shieldId].accu += ((uint32_t)shieldPrm[shieldId].deceleration << 16) / shieldPrm[shieldId].speed;
          while (shieldPrm[shieldId].accu >= (0X10000L))
          {
            shieldPrm[shieldId].accu -= (0X10000L);
            shieldPrm[shieldId].speed -=1;
            speedUpdated = true;
          }
          if (speedUpdated)
          {
            ApplySpeed(shieldId, shieldPrm[shieldId].speed);
          }
        }
      }
      break;
    }
    default: 
    {
      break;
    }
  }  
  /* Set isr flag */
  isrFlag = false;
}

/******************************************************//**
 * @brief  Updates the current speed of the shield
 * @param[in] shieldId (from 0 to 2)
 * @param[in] newSpeed in pps
 * @retval None
 **********************************************************/
void L6474::ApplySpeed(uint8_t shieldId, uint16_t newSpeed)
{
  if (newSpeed < L6474_MIN_PWM_FREQ)
  {
    newSpeed = L6474_MIN_PWM_FREQ;  
  }
  if (newSpeed > L6474_MAX_PWM_FREQ)
  {
    newSpeed = L6474_MAX_PWM_FREQ;
  }
  
  shieldPrm[shieldId].speed = newSpeed;

  switch (shieldId)
  {
    case  0:
      Pwm1SetFreq(newSpeed);
      break;
    case 1:
      Pwm2SetFreq(newSpeed);
      break;
    case 2:
      Pwm3SetFreq(newSpeed);
      break;
    default:
      break; //ignore error
  }
}

/******************************************************//**
 * @brief  Computes the speed profile according to the number of steps to move
 * @param[in] shieldId (from 0 to 2)
 * @param[in] nbSteps number of steps to perform
 * @retval None
 * @note Using the acceleration and deceleration of the shield,
 * this function determines the duration in steps of the acceleration,
 * steady and deceleration phases.
 * If the total number of steps to perform is big enough, a trapezoidal move
 * is performed (i.e. there is a steady phase where the motor runs at the maximum
 * speed.
 * Else, a triangular move is performed (no steady phase: the maximum speed is never
 * reached.
 **********************************************************/
void L6474::ComputeSpeedProfile(uint8_t shieldId, uint32_t nbSteps)
{
  uint32_t reqAccSteps; 
	uint32_t reqDecSteps;
   
  /* compute the number of steps to get the targeted speed */
  reqAccSteps = (shieldPrm[shieldId].maxSpeed - shieldPrm[shieldId].minSpeed);
  reqAccSteps *= (shieldPrm[shieldId].maxSpeed + shieldPrm[shieldId].minSpeed);
  reqDecSteps = reqAccSteps;
  reqAccSteps /= (uint32_t)shieldPrm[shieldId].acceleration;
  reqAccSteps /= 2;

  /* compute the number of steps to stop */
  reqDecSteps /= (uint32_t)shieldPrm[shieldId].deceleration;
  reqDecSteps /= 2;

	if(( reqAccSteps + reqDecSteps ) > nbSteps)
	{	
    /* Triangular move  */
    /* reqDecSteps = (Pos * Dec) /(Dec+Acc) */
   
    reqDecSteps =  ((uint32_t) shieldPrm[shieldId].deceleration * nbSteps) / (shieldPrm[shieldId].acceleration + shieldPrm[shieldId].deceleration);
    if (reqDecSteps > 1)
    {
      reqAccSteps = reqDecSteps - 1;
      if(reqAccSteps == 0)
      {
        reqAccSteps = 1;
      }      
    }
    else
    {
      reqAccSteps = 0;
    }
    shieldPrm[shieldId].endAccPos = reqAccSteps;
    shieldPrm[shieldId].startDecPos = reqDecSteps;
	}
	else
	{	 
    /* Trapezoidal move */
    /* accelerating phase to endAccPos */
    /* steady phase from  endAccPos to startDecPos */
    /* decelerating from startDecPos to stepsToTake*/
    shieldPrm[shieldId].endAccPos = reqAccSteps;
    shieldPrm[shieldId].startDecPos = nbSteps - reqDecSteps - 1;
	}
}

/******************************************************//**
 * @brief  Converts the ABS_POSITION register value to a 32b signed integer
 * @param[in] abs_position_reg value of the ABS_POSITION register
 * @retval operation_result 32b signed integer corresponding to the absolute position 
 **********************************************************/
int32_t L6474::ConvertPosition(uint32_t abs_position_reg)
{
	int32_t operation_result;

  if (abs_position_reg & L6474_ABS_POS_SIGN_BIT_MASK) 
  {
		/* Negative register value */
		abs_position_reg = ~abs_position_reg;
		abs_position_reg += 1;

		operation_result = (int32_t) (abs_position_reg & L6474_ABS_POS_VALUE_MASK);
		operation_result = -operation_result;
  } 
  else 
  {
		operation_result = (int32_t) abs_position_reg;
	}
	return operation_result;
}

/******************************************************//**
 * @brief  Handlers of the flag interrupt which calls the user callback (if defined)
 * @param None
 * @retval None
 **********************************************************/
void L6474::FlagInterruptHandler(void)
{
  if (flagInterruptCallback != NULL)
  {
    /* Set isr flag */
    isrFlag = true;
    
    flagInterruptCallback();
    
    /* Reset isr flag */
    isrFlag = false;   
  }
}

/******************************************************//**
 * @brief  Sends a command without arguments to the L6474 via the SPI
 * @param[in] shieldId (from 0 to 2)
 * @param[in] param Command to send 
 * @retval None
 **********************************************************/
void L6474::SendCommand(uint8_t shieldId, uint8_t param)
{
  uint8_t spiIndex = numberOfShields - shieldId - 1;
  bool itDisable = false;  
  
  do
  {
    spiPreemtionByIsr = false;
    if (itDisable)
    {
      /* re-enable interrupts if disable in previous iteration */
      interrupts();
      itDisable = false;
    }
  
    for (uint32_t i = 0; i < numberOfShields; i++)
    {
      spiTxBursts[3][i] = L6474_NOP;     
    }
    spiTxBursts[3][spiIndex] = param;
    
    /* Disable interruption before checking */
    /* pre-emption by ISR and SPI transfers*/
    noInterrupts();
    itDisable = true;
  } while (spiPreemtionByIsr); // check pre-emption by ISR

  WriteBytes(&spiTxBursts[3][0], &spiRxBursts[3][0]); 
  
  /* re-enable interrupts after SPI transfers*/
  interrupts();
}

/******************************************************//**
 * @brief  Sets the registers of the L6474 to their predefined values 
 * from l6474_target_config.h
 * @param[in] shieldId (from 0 to 2)
 * @retval None
 **********************************************************/
void L6474::SetRegisterToPredefinedValues(uint8_t shieldId)
{
  CmdSetParam(shieldId,
                    L6474_ABS_POS,
                    0);
  CmdSetParam(shieldId,
                    L6474_EL_POS,
                    0);
  CmdSetParam(shieldId,
                    L6474_MARK,
                    0);
  switch (shieldId)
  {
    case 0:
      CmdSetParam(shieldId,
                        L6474_TVAL,
                        Tval_Current_to_Par(L6474_CONF_PARAM_TVAL_SHIELD_0));
      CmdSetParam(shieldId,
                        L6474_T_FAST,
                        (uint8_t)L6474_CONF_PARAM_TOFF_FAST_SHIELD_0 |
                        (uint8_t)L6474_CONF_PARAM_FAST_STEP_SHIELD_0);
      CmdSetParam(shieldId,
                        L6474_TON_MIN,
                        Tmin_Time_to_Par(L6474_CONF_PARAM_TON_MIN_SHIELD_0));
      CmdSetParam(shieldId,
                        L6474_TOFF_MIN,
                        Tmin_Time_to_Par(L6474_CONF_PARAM_TOFF_MIN_SHIELD_0));
      CmdSetParam(shieldId,
                        L6474_OCD_TH,
                        L6474_CONF_PARAM_OCD_TH_SHIELD_0);
      CmdSetParam(shieldId,
                        L6474_STEP_MODE,
                        (uint8_t)L6474_CONF_PARAM_STEP_SEL_SHIELD_0 |
                        (uint8_t)L6474_CONF_PARAM_SYNC_SEL_SHIELD_0);
      CmdSetParam(shieldId,
                        L6474_ALARM_EN,
                        L6474_CONF_PARAM_ALARM_EN_SHIELD_0);
      CmdSetParam(shieldId,
                        L6474_CONFIG,
                        (uint16_t)L6474_CONF_PARAM_CLOCK_SETTING_SHIELD_0 |
                        (uint16_t)L6474_CONF_PARAM_TQ_REG_SHIELD_0 |
                        (uint16_t)L6474_CONF_PARAM_OC_SD_SHIELD_0 |
                        (uint16_t)L6474_CONF_PARAM_SR_SHIELD_0 |
                        (uint16_t)L6474_CONF_PARAM_TOFF_SHIELD_0);
      break;
    case 1:
      CmdSetParam(shieldId,
                        L6474_TVAL,
                        Tval_Current_to_Par(L6474_CONF_PARAM_TVAL_SHIELD_1));
      CmdSetParam(shieldId,
                        L6474_T_FAST,
                        (uint8_t)L6474_CONF_PARAM_TOFF_FAST_SHIELD_1 |
                        (uint8_t)L6474_CONF_PARAM_FAST_STEP_SHIELD_1);
      CmdSetParam(shieldId,
                        L6474_TON_MIN,
                        Tmin_Time_to_Par(L6474_CONF_PARAM_TON_MIN_SHIELD_1));
      CmdSetParam(shieldId,
                        L6474_TOFF_MIN,
                        Tmin_Time_to_Par(L6474_CONF_PARAM_TOFF_MIN_SHIELD_1));
      CmdSetParam(shieldId,
                        L6474_OCD_TH,
                        L6474_CONF_PARAM_OCD_TH_SHIELD_1);
      CmdSetParam(shieldId,
                        L6474_STEP_MODE,
                        (uint8_t)L6474_CONF_PARAM_STEP_SEL_SHIELD_1 |
                        (uint8_t)L6474_CONF_PARAM_SYNC_SEL_SHIELD_1);
      CmdSetParam(shieldId,
                        L6474_ALARM_EN,
                        L6474_CONF_PARAM_ALARM_EN_SHIELD_1);
      CmdSetParam(shieldId,
                        L6474_CONFIG,
                        (uint16_t)L6474_CONF_PARAM_CLOCK_SETTING_SHIELD_1 |
                        (uint16_t)L6474_CONF_PARAM_TQ_REG_SHIELD_1 |
                        (uint16_t)L6474_CONF_PARAM_OC_SD_SHIELD_1 |
                        (uint16_t)L6474_CONF_PARAM_SR_SHIELD_1 |
                        (uint16_t)L6474_CONF_PARAM_TOFF_SHIELD_1);
      break;
    case 2:
      CmdSetParam(shieldId,
                        L6474_TVAL,
                        Tval_Current_to_Par(L6474_CONF_PARAM_TVAL_SHIELD_2));
      CmdSetParam(shieldId,
                        L6474_T_FAST,
                        (uint8_t)L6474_CONF_PARAM_TOFF_FAST_SHIELD_2 |
                        (uint8_t)L6474_CONF_PARAM_FAST_STEP_SHIELD_2);
      CmdSetParam(shieldId,
                        L6474_TON_MIN,
                        Tmin_Time_to_Par(L6474_CONF_PARAM_TON_MIN_SHIELD_2));
      CmdSetParam(shieldId,
                        L6474_TOFF_MIN,
                        Tmin_Time_to_Par(L6474_CONF_PARAM_TOFF_MIN_SHIELD_2));
      CmdSetParam(shieldId,
                        L6474_OCD_TH,
                        L6474_CONF_PARAM_OCD_TH_SHIELD_2);
      CmdSetParam(shieldId,
                        L6474_STEP_MODE,
                        (uint8_t)L6474_CONF_PARAM_STEP_SEL_SHIELD_2 |
                        (uint8_t)L6474_CONF_PARAM_SYNC_SEL_SHIELD_2);
      CmdSetParam(shieldId,
                        L6474_ALARM_EN,
                        L6474_CONF_PARAM_ALARM_EN_SHIELD_2);
      CmdSetParam(shieldId,
                        L6474_CONFIG,
                        (uint16_t)L6474_CONF_PARAM_CLOCK_SETTING_SHIELD_2 |
                        (uint16_t)L6474_CONF_PARAM_TQ_REG_SHIELD_2 |
                        (uint16_t)L6474_CONF_PARAM_OC_SD_SHIELD_2 |
                        (uint16_t)L6474_CONF_PARAM_SR_SHIELD_2 |
                        (uint16_t)L6474_CONF_PARAM_TOFF_SHIELD_2);
      break;
    default: ;
  }
}

/******************************************************//**
 * @brief  Sets the registers of the L6474 to their predefined values 
 * from l6474_target_config.h
 * @param[in] shieldId (from 0 to 2)
 * @retval None
 **********************************************************/
void L6474::WriteBytes(uint8_t *pByteToTransmit, uint8_t *pReceivedByte)
{
  digitalWrite(SS, LOW);
  for (uint32_t i = 0; i < numberOfShields; i++)
  {
    *pReceivedByte = SPI.transfer(*pByteToTransmit);
    pByteToTransmit++;
    pReceivedByte++;
  }
  digitalWrite(SS, HIGH);
  if (isrFlag)
  {
    spiPreemtionByIsr = true;
  }
}

/******************************************************//**
 * @brief  Initialises the PWM uses by the specified shield
 * @param[in] shieldId (from 0 to 2)
 * @retval None
 * @note Shield 0 uses PW1 based on timer 1 
 * Shield 1 uses PWM 2 based on timer 2
 * Shield 2 uses PWM3 based timer 0
 **********************************************************/
void L6474::PwmInit(uint8_t shieldId)
{
  switch (shieldId)
  {
    case 0:
      /* PWM1 uses timer 1 */
      /* Initialise timer by setting waveform generation mode  
      to PWM phase and Frequency correct: mode = 8 
      (WGM10 = 0, WGM11 = 0, WGM12 = 0, WGM13 = 1) */
    
      /* Stop timer1 by clearing CSS bits and set WGM13 and WGM12 */
      TCCR1B = 0x10;  
       
      /* Set WGM10 and WGM11 */
      TCCR1A =  0x00;  
      
      /*  Disable Timer1 interrupt */
      TIMSK1 = 0;
    
      break;
    case  1:
      /* PWM2 uses timer 2 */
      /* Initialise timer by setting waveform generation mode  
      to PWM phase correct: mode = 5
      (WGM0 = 1, WGM21 = 0, WGM22 = 1) */
      
      /* Stop timer2 by clearing CSS bits and set WGM22 */
      TCCR2B = 0x08;  
       
      /* Set WGM20 and WGM21 */
      TCCR2A =  0x01;  
      
      /*  Disable Timer2 interrupt */
      TIMSK2 = 0; 
      
      break;


    case 2:
      /* PWM3 uses timer 0 */
      /* !!!!! Caution: Calling this configuration will break */
      /* all default Arduino's timing functions as delay(),millis()... */
      
      /* Initialise timer by setting waveform generation mode  
      to PWM phase correct: mode = 5
      (WGM0 = 1, WGM21 = 0, WGM22 = 1) */
      
      /* Stop timer0 by clearing CSS bits and set WGM22 */
      TCCR0B = 0x08;  
       
      /* Set WGM00 and WGM01 */
      TCCR0A =  0x01;  
      
      /*  Disable Timer0 interrupt */
      TIMSK0 = 0;
      break;
    default:
      break;//ignore error
  }
}

/******************************************************//**
 * @brief  Sets the frequency of PWM1 used by shield 0
 * @param[in] newFreq in Hz
 * @retval None
 * @note The frequency is directly the current speed of the shield
 **********************************************************/
void L6474::Pwm1SetFreq(uint16_t newFreq)
{
  uint8_t index = 0;
  uint32_t top;
  uint16_t TargetedPrescaler;
 
  TargetedPrescaler = (int16_t)(F_CPU / (2 * (uint32_t) newFreq * UINT16_MAX));

  do
  {
    while ((index < PRESCALER_ARRAY_TIMER0_1_SIZE - 1)&&
           (TargetedPrescaler > prescalerArrayTimer0_1[++index]));
    TargetedPrescaler = prescalerArrayTimer0_1[index];
    top = F_CPU/(2* (uint32_t)newFreq * TargetedPrescaler);
   } while ((index < PRESCALER_ARRAY_TIMER0_1_SIZE - 1) && (top > UINT16_MAX));

  /* Disable Timer1 Interrupt */
  cbi(TIMSK1,TOIE1);
  
  ICR1 = (uint16_t)top;
  OCR1A = ((uint16_t)top) >> 1; // Set a 50 % duty cycle
  
  /* Enable compare match channel A output */
  sbi(TCCR1A, COM1A1);
  
    /* Reenable Timer1 Interrupt */
  sbi(TIMSK1,TOIE1);
  
  /* Set the Prescaler without erasing WGM12 and WGM13 bit*/
  /* And so, start the timer */
  TCCR1B = (TCCR1B & 0x18) | index;
}

/******************************************************//**
 * @brief  Sets the frequency of PWM2 used by shield 1
 * @param[in] newFreq in Hz
 * @retval None
 * @note The frequency is directly the current speed of the shield
 **********************************************************/
void L6474::Pwm2SetFreq(uint16_t newFreq)
{
  uint8_t index = 0;
  uint16_t top;
  uint16_t TargetedPrescaler;
 
  TargetedPrescaler = (uint16_t)(F_CPU / (2 * (uint32_t)newFreq * UINT8_MAX));

  /* Compute timer2 top */
  do
  {
    while ((index < PRESCALER_ARRAY_TIMER2_SIZE - 1) && (TargetedPrescaler > prescalerArrayTimer2[++index]));
    TargetedPrescaler = prescalerArrayTimer2[index];
    top = (uint16_t)(F_CPU/(2* (uint32_t)newFreq * TargetedPrescaler));
   } while ((index < PRESCALER_ARRAY_TIMER2_SIZE - 1) && (top > UINT8_MAX));
  
  /* Disable Timer2 Interrupt */
  cbi(TIMSK2,TOIE2);
  
  OCR2A = (uint8_t)top;
  OCR2B = (uint8_t)top >> 1; // Set a 50 % duty cycle
  
  /* Enable compare match channel B output */
  sbi(TCCR2A, COM2B1);

  /* Reenable Timer2 Interrupt */
  sbi(TIMSK2,TOIE2);
  
  /* Set the Prescaler without erasing WGM22 bit*/
  /* And so, start the timer */
  TCCR2B = (TCCR2B & 0x8) | index;
}

/******************************************************//**
 * @brief  Sets the frequency of PWM3 used by shield 2
 * @param[in] newFreq in Hz
 * @retval None
 * @note The frequency is directly the current speed of the shield
 **********************************************************/
void L6474::Pwm3SetFreq(uint16_t newFreq)
{
  uint8_t index = 0;
  uint16_t top;
  uint16_t TargetedPrescaler;
  
 
  TargetedPrescaler = (uint16_t)(F_CPU / (4 * (uint32_t)newFreq * UINT8_MAX));

  /* Compute timer0 top */
  do
  {
    while ((index < PRESCALER_ARRAY_TIMER0_1_SIZE - 1) && 
           (TargetedPrescaler > prescalerArrayTimer0_1[++index]));
    TargetedPrescaler = prescalerArrayTimer0_1[index];

    top = (uint16_t)(F_CPU/(4* (uint32_t)newFreq * TargetedPrescaler));
    
   } while ((index < PRESCALER_ARRAY_TIMER0_1_SIZE - 1) && (top > UINT8_MAX));
  
  /* Disable Timer0 Interrupt */
  cbi(TIMSK0,TOIE0);
  
  OCR0A = (uint8_t)top  ;
  OCR0B = (uint8_t)(top) >> 1 ; // Set a 50 % duty cycle
  
  /* Enable compare match channel A output */
  sbi(TCCR0A, COM0A0);
    
  /* Reenable Timer0 Interrupt */
  sbi(TIMSK0,TOIE0);

  /* Set the Prescaler without erasing WGM02 bit*/
  /* And so, start the timer */
  TCCR0B = (TCCR0B & 0x8) | index;
}

/******************************************************//**
 * @brief  Stops the PWM uses by the specified shield
 * @param[in] shieldId (from 0 to 2)
 * @retval None
 **********************************************************/
void L6474::PwmStop(uint8_t shieldId)
{
  switch (shieldId)
  {
    case 0:
      /* PWM1 uses timer 1 */
    
      /* Stop timer1 by clearing CSS bits (keep  WGM13 and WGM12) */
      TCCR1B = 0x10;  
             
      /*  Disable Timer1 interrupt */
      TIMSK1 = 0;
    
      break;
    case  1:
      /* PWM2 uses timer 2 */
     
      /* Stop timer2 by clearing CSS bits (keep  WGM22) */
      TCCR2B = 0x08;  
       
      /*  Disable Timer2 interrupt */
      TIMSK2 = 0; 
      
      break;
    case 2:
      /* PWM3 uses timer 0 */
      /* !!!!! Caution: Calling this configuration will break */
      /* all default Arduino's timing functions as delay(),millis()... */
           
      /* Stop timer0 by clearing CSS bits (keep  WGM22) */
      TCCR0B = 0x08;  
       
      
      /*  Disable Timer0 interrupt */
      TIMSK0 = 0;
      
      break;
    default:
      break;//ignore error
  }
}

/******************************************************//**
 * @brief  Sets the parameters of the shield to predefined values
 * from l6474_target_config.h
 * @param None
 * @retval None
 **********************************************************/
void L6474::SetShieldParamsToPredefinedValues(void)
{
  shieldPrm[0].acceleration = L6474_CONF_PARAM_ACC_SHIELD_0;
  shieldPrm[0].deceleration = L6474_CONF_PARAM_DEC_SHIELD_0;
  shieldPrm[0].maxSpeed = L6474_CONF_PARAM_MAX_SPEED_SHIELD_0;
  shieldPrm[0].minSpeed = L6474_CONF_PARAM_MIN_SPEED_SHIELD_0;
  
  shieldPrm[1].acceleration = L6474_CONF_PARAM_ACC_SHIELD_1;
  shieldPrm[1].deceleration = L6474_CONF_PARAM_DEC_SHIELD_1;
  shieldPrm[1].maxSpeed = L6474_CONF_PARAM_MAX_SPEED_SHIELD_1;
  shieldPrm[1].minSpeed = L6474_CONF_PARAM_MIN_SPEED_SHIELD_1;
  
  shieldPrm[2].acceleration = L6474_CONF_PARAM_ACC_SHIELD_2;
  shieldPrm[2].deceleration = L6474_CONF_PARAM_DEC_SHIELD_2;
  shieldPrm[2].maxSpeed = L6474_CONF_PARAM_MAX_SPEED_SHIELD_2;
  shieldPrm[2].minSpeed = L6474_CONF_PARAM_MIN_SPEED_SHIELD_2;
  
  for (uint8_t i = 0; i < numberOfShields; i++)
  {
    SetRegisterToPredefinedValues(i);
  }   
}

/******************************************************//**
 * @brief Initialises the bridge parameters to start the movement
 * and enable the power bridge
 * @param[in] shieldId (from 0 to 2)
 * @retval None
 **********************************************************/
void L6474::StartMovement(uint8_t shieldId)  
{
  /* Enable L6474 powerstage */
  CmdEnable(shieldId);

  if (shieldPrm[shieldId].endAccPos != 0)
  {
    shieldPrm[shieldId].motionState = ACCELERATING;;
  }
  else
  {
    shieldPrm[shieldId].motionState = DECELERATING;    
  }

  shieldPrm[shieldId].accu = 0;
  shieldPrm[shieldId].relativePos = 0;
  ApplySpeed(shieldId, shieldPrm[shieldId].minSpeed);
#ifdef _DEBUG_L6474
  snprintf(l6474StrOut, DEBUG_BUFFER_SIZE, "Stop->Acc: speed: %u relPos: %ld\n", shieldPrm[shieldId].minSpeed, shieldPrm[shieldId].relativePos) ;
  Serial.println(l6474StrOut);
#endif        
}

/******************************************************//**
 * @brief Converts mA in compatible values for TVAL register 
 * @param[in] Tval
 * @retval TVAL values
 **********************************************************/
inline uint8_t L6474::Tval_Current_to_Par(double Tval)
{
  return ((uint8_t)(((Tval - 31.25)/31.25)+0.5));
}

/******************************************************//**
 * @brief Convert time in us in compatible values 
 * for TON_MIN register
 * @param[in] Tmin
 * @retval TON_MIN values
 **********************************************************/
inline uint8_t L6474::Tmin_Time_to_Par(double Tmin)
{
  return ((uint8_t)(((Tmin - 0.5)*2)+0.5));
}

#ifdef _USE_TIMER_0_FOR_L6474
/******************************************************//**
 * @brief Timer0 interrupt handler used by PW3 for shield 2
 * and enable the power bridge
 * @param None
 * @retval None
 **********************************************************/
ISR(TIMER0_OVF_vect) 
{
  static bool isr0Toggle = false;
  class L6474* instancePtr = L6474::GetInstancePtr();
  if (isr0Toggle)
  {
    if (instancePtr != NULL)
    {  
      if (instancePtr->GetShieldState(2) != INACTIVE)
      {
        instancePtr->StepClockHandler(2);
      }
    }
    isr0Toggle = false;
  }
  else
  {
    isr0Toggle = true;
  }
}
#endif

/******************************************************//**
 * @brief Timer1 interrupt handler used by PW1 for shield 0
 * and enable the power bridge
 * @param None
 * @retval None
 **********************************************************/
ISR(TIMER1_OVF_vect) 
{
  class L6474* instancePtr = L6474::GetInstancePtr();
  if (instancePtr != NULL)
  {  
    if (instancePtr->GetShieldState(0) != INACTIVE)
    {
      instancePtr->StepClockHandler(0);
    }
  }
}

/******************************************************//**
 * @brief Timer2 interrupt handler used by PW2 for shield 1
 * and enable the power bridge
 * @param  None
 * @retval None
 **********************************************************/
 ISR(TIMER2_OVF_vect) 
{
  class L6474* instancePtr = L6474::GetInstancePtr();
  if (instancePtr != NULL)
  {  
    if (instancePtr->GetShieldState(1) != INACTIVE)
    {
      instancePtr->StepClockHandler(1);
    }
  }
}

/******************************************************//**
 * @brief  Debug   function to get the amount of free ram
 * @param  None
 * @retval number of bytes of free ram
 **********************************************************/
#ifdef _DEBUG_L6474
uint16_t GetFreeRam (void)
{
  extern uint16_t __heap_start, *__brkval;
  uint16_t v;
  return (uint16_t) &v - (__brkval == 0 ? (uint16_t) &__heap_start : (uint16_t) __brkval);
}
#endif



                                    


/**
  ******************************************************************************
  * @file    stm32_SMBUS_stack.c
  * @author  MCD Application Team
  * @version V2.0.1
  * @date    28-June-2017
  * @brief   This file provides a set of functions needed to manage the SMBUS STACK.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2016 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32_SMBUS_stack.h"
#include "stm32_PMBUS_stack.h"

/** @addtogroup STM32_SMBUS_STACK
  * @{
  */

/** @defgroup STM32_SMBUS_STACK_Defines
  * @{
  */
#define SMBUS_INSTANCES_COUNT   ((uint8_t)2)  /*!< how many stacks can we run in parallel, initialized */

/**
  * @}
  */

/** @defgroup STM32_SMBUS_STACK_Constants SMBus stack data constants
  * @{
  */
const SMBUS_ZoneStateTypeDef ZERO_ZONE = {0, 0, 0, 0};
/*!<
    Zone structure init value - no zone
 */

st_command_t ALERT_RESPONSE = {0, READ, 0, 1};
/*!<
    dedicated command definition for the alert response protocol:
 */

st_command_t HOST_NOTIFY_PROTOCOL = { 0, WRITE, 3, 0};
/*!<
    dedicated command definition for the host notify protocol:
 */

st_command_t PMBUS_COMMANDS_ARP[] =
  {
    { SMBUS_ARP_CC_PREPARE, WRITE, 1, 0 },
    { SMBUS_ARP_CC_RESET, WRITE, 1, 0 },
    { SMBUS_ARP_CC_GET_ID, BLOCK_READ, 1, 18 },
#ifdef  HOST1
    { SMBUS_ARP_CC_ASSIGN, BLOCK_WRITE, 18, 0 },
#else
    { SMBUS_ARP_CC_ASSIGN, BLOCK_WRITE, 1, 0 }, /* really it is 19 bytes written, but we need this command to be treated in a special way */
#endif
  };
/*!<
    dedicated command definition table for the address resolution protocol:
 */

/**
  * @}
  */

/** @defgroup STM32_SMBUS_STACK_Variables
  * @{
  */

/**
  A list in which the SMBUS stack instances are registered. The purpose is to
  identify correct working context for each action. The rule is that no more than
  one instance per physical interface is possible
 */
SMBUS_StackHandleTypeDef* SMBUSInstancesList[SMBUS_INSTANCES_COUNT];

/**
  * @}
  */

/** @defgroup STM32_SMBUS_STACK_Interfaces
  * @{
  */

/**
  * @brief  SMBUS stack identification based on HW resource.
  * @param  hsmbus : Pointer to a SMBUS_HandleTypeDef structure that contains
  *                the configuration information for the specified SMBUS.
  * @retval Pointer to relevant stack instance.
  */
SMBUS_StackHandleTypeDef* STACK_SMBUS_ResolveContext( SMBUS_HandleTypeDef *hsmbus )
{
  SMBUS_StackHandleTypeDef* pSelected = NULL;
  uint32_t index = 0;

  /*
    Basically match the physical device with stack instance
   */
  do
  {
    /*
      The loop is searching for the first stack context instance initialized to
      use the actual HAL Driver handle
     */
    pSelected = SMBUSInstancesList[index];
    if  ( pSelected->Device != hsmbus )
    {
      pSelected = NULL;
    }
    index++;
  }
  while ((index < SMBUS_INSTANCES_COUNT ) && ( pSelected == NULL ));

  /*
    Error - no identified stack instance
   */
  return pSelected;
}

/**
  * @brief  Master Tx Transfer completed callbacks.
  * @param  hsmbus : Pointer to a SMBUS_HandleTypeDef structure that contains
  *                the configuration information for the specified SMBUS.
  * @retval None
  */
void HAL_SMBUS_MasterTxCpltCallback(SMBUS_HandleTypeDef *hsmbus)
{
  SMBUS_StackHandleTypeDef*    pStackContext;
  uint16_t              size;

  /*
    Resolve which stack serves the port that initiated callback
   */
  pStackContext = STACK_SMBUS_ResolveContext( hsmbus );

  /* Transmission phase is completed */
  pStackContext->StateMachine &= ~SMBUS_SMS_TRANSMIT;

  /*
    NOTE - optionally clear the buffer here
  */

#ifdef PMBUS13
  /* If this is a READ_ZONE, then it gets special treatment.
    Only simple status read is supported in this example */
  if ( pStackContext->SlaveAddress == SMBUS_ADDR_ZONE_READ )
  {
    /* Requesting next response */
    STACK_PMBUS_MasterZoneReadStatusCont(pStackContext);
    return;
  }
#endif

  /*
   Is there data to receive after transfer?
   check if the command includes reading phase and if application that invoked
   the command cared for receiving data from the slave
  */
  if ( pStackContext->CurrentCommand != 0U )
  {
    if (
      ( ( pStackContext->CurrentCommand->cmnd_query & BLOCK ) == BLOCK ) && \
      (
        ( ( pStackContext->CurrentCommand->cmnd_query & PROCESS_CALL ) == PROCESS_CALL ) || \
        ( ( pStackContext->OpMode & READ ) == READ )
      )
    )
    {
      /*
        the amount of data to be read is yet to be sent by the slave - reading the byte count now:
      */
      pStackContext->StateMachine |= SMBUS_SMS_RECEIVE | SMBUS_SMS_PROCESSING;
      HAL_SMBUS_Master_Sequential_Receive_IT( hsmbus, pStackContext->SlaveAddress, &(pStackContext->Buffer[1]), 1, SMBUS_NEXT_FRAME );
    }
    else if ((pStackContext->CurrentCommand->cmnd_master_Rx_size > 0 ) && (( pStackContext->OpMode & WRITE ) == 0 ))
    {
      /*
        the amount of data to be read is known and determined by the command code
      */
      pStackContext->StateMachine |= SMBUS_SMS_RECEIVE;
      size = pStackContext->CurrentCommand->cmnd_master_Rx_size;
      if ( pStackContext->StateMachine & SMBUS_SMS_PEC_ACTIVE )
      {
        size += 1;
      }
      HAL_SMBUS_Master_Sequential_Receive_IT( hsmbus, pStackContext->SlaveAddress, &(pStackContext->Buffer[1]), size, SMBUS_LAST_FRAME | ( pStackContext->StateMachine & SMBUS_SMS_PEC_ACTIVE ));
    }
  }
  else
  {
    /*
      There was an alert during command that we could not treat before
     */
    STACK_SMBUS_ReadyIfNoAlert( pStackContext );
  }
  HAL_SMBUS_EnableAlert_IT(hsmbus);
}

/**
  * @brief  Master Rx Transfer completed callbacks.
  * @param  hsmbus : Pointer to a SMBUS_HandleTypeDef structure that contains
  *                the configuration information for the specified SMBUS.
  * @retval None
  */
void HAL_SMBUS_MasterRxCpltCallback(SMBUS_HandleTypeDef *hsmbus)
{
  SMBUS_StackHandleTypeDef* pStackContext;
  uint8_t            size;

  /*
    Resolve which stack serves the port that initiated callback
   */
  pStackContext = STACK_SMBUS_ResolveContext( hsmbus );

#ifdef PMBUS13
  /* If this is a READ_ZONE, then it gets special treatment.
    Only simple status read is supported in this example */
  if ( pStackContext->SlaveAddress == SMBUS_ADDR_ZONE_READ )
  {
    /* Third byte - page number?
    if (pStackContext->Byte_count == 2)
    {
      pStackContext->Device->XferSize++;
    }*/
    /* Fail-safe stopping the Zone Read on empty answer */
    if ( pStackContext->Buffer[0] != 0xFF )
    {
      pStackContext->Device->PreviousState = HAL_SMBUS_STATE_BUSY_TX;
      STACK_PMBUS_MasterZoneReadStatusCont(pStackContext);
    }
    else
    {
      /*
        There could be an alert during command that we could not treat before
      */
      STACK_SMBUS_ReadyIfNoAlert( pStackContext );
    }
    return;
  }
#endif

  if ( pStackContext->StateMachine & SMBUS_SMS_RECEIVE )
  {
    /*
      Reception completed
     */
    pStackContext->StateMachine &= ~SMBUS_SMS_RECEIVE;

    /*
      A case of block transfer follow-up follows
     */
    if ( ( pStackContext->CurrentCommand->cmnd_query & BLOCK ) && ( pStackContext->StateMachine & SMBUS_SMS_PROCESSING ) )
    {
      /*
        the amount of data to be read was sent by the slave - it is on position 1 of the IO buffer
        is limited to STACK_NBYTE_SIZE
      */
      if ( pStackContext->Buffer[1] > STACK_NBYTE_SIZE )
      {
        pStackContext->Buffer[1] = STACK_NBYTE_SIZE;
      }

      if (  pStackContext->Buffer[1] == 0 )
      {
        /*
        special case - slave indicates it has no further data to send
        We generate stop and close the frame
        */
        STACK_SMBUS_ReadyIfNoAlert( pStackContext );
      }
      else
      {
        /*
          This will conclude the block processing, we clear the flag
         */
        pStackContext->StateMachine &= ~SMBUS_SMS_PROCESSING;

        /*
          Reception next
          Usually there is something left to be read, we continue on position 2 of the IO buffer
         */
        pStackContext->StateMachine |= SMBUS_SMS_RECEIVE;
        size = pStackContext->Buffer[1];
        /* Applying upper limit on read size */
        if ( size > pStackContext->CurrentCommand->cmnd_master_Rx_size )
        {
          size = pStackContext->CurrentCommand->cmnd_master_Rx_size;
        }
        HAL_SMBUS_Master_Sequential_Receive_IT( hsmbus, pStackContext->SlaveAddress, &(pStackContext->Buffer[2]), size, SMBUS_LAST_FRAME  | ( pStackContext->StateMachine & SMBUS_SMS_PEC_ACTIVE ));
      }
    }
    else
    {
      /*
        There could be an alert during command that we could not treat before
      */
      STACK_SMBUS_ReadyIfNoAlert( pStackContext );
    }
  }
  else
  {
    pStackContext->StateMachine |= SMBUS_SMS_ERROR;
  }
  /* Program should not reach this statement */
  HAL_SMBUS_EnableAlert_IT(hsmbus);
}

/**
  * @brief  SMBUS alert signalling (device only).
  * @param  pStackContext : Pointer to a SMBUS_StackHandleTypeDef structure that contains
  *                the configuration information for the specified SMBUS.
  * @retval None
  */
void STACK_SMBUS_SendAlert( SMBUS_StackHandleTypeDef* pStackContext )
{
  /*
    check mode consistency - device only sends
   */
  if ( ( pStackContext->Device->Instance->CR1 & SMBUS_PERIPHERAL_MODE_HOST ) == 0)
  {
    pStackContext->Device->Instance->CR1 |= I2C_CR1_ALERT;
  }
}

/**
  * @brief  SMBUS error callbacks.
  * @param  hsmbus : Pointer to a SMBUS_HandleTypeDef structure that contains
  *                the configuration information for the specified SMBUS.
  * @retval None
  */
void HAL_SMBUS_ErrorCallback(SMBUS_HandleTypeDef *hsmbus)
{
  SMBUS_StackHandleTypeDef* pStackContext;
  uint32_t  error = hsmbus->ErrorCode;

  pStackContext = STACK_SMBUS_ResolveContext( hsmbus );

  if ( error & HAL_SMBUS_ERROR_ALERT)
  {
    /*
      alert signal detected, update state
     */
    pStackContext->StateMachine |= SMBUS_SMS_ALERT_PENDING;

    if ( (pStackContext->StateMachine & SMBUS_SMS_ACTIVE_MASK) != 0 )
    {
    	if((pStackContext->StateMachine & SMBUS_SMS_READY) != SMBUS_SMS_READY)
    	{
    		pStackContext->StateMachine |= SMBUS_SMS_READY;
    	}

    	/*
        The stack is not busy - we can react immediately
    	*/
    	pStackContext->CurrentCommand = &ALERT_RESPONSE;

    	STACK_SMBUS_HostRead( pStackContext, hsmbus->pBuffPtr , SMBUS_ADDR_ARA);
    }

    /*
     the alert has been treated, clear the flag
     */
    error &= ~HAL_SMBUS_ERROR_ALERT;
  }
  else if ( error & (HAL_SMBUS_ERROR_BERR | HAL_SMBUS_ERROR_BUSTIMEOUT | HAL_SMBUS_ERROR_HALTIMEOUT ))
  {
    /*
      Critical error plagued the command - reset the stack
     */
    if ( STACK_SMBUS_IsBusy(pStackContext))
    {
      __HAL_SMBUS_DISABLE( hsmbus );
      while ( hsmbus->Instance->CR1 & I2C_CR1_PE )
      {}
      __HAL_SMBUS_ENABLE( hsmbus );
      if ( pStackContext->Device->State != HAL_SMBUS_STATE_READY )
      {
        HAL_SMBUS_DeInit( pStackContext->Device );
        HAL_SMBUS_Init( pStackContext->Device );
      }
    }
    pStackContext->StateMachine |= SMBUS_SMS_READY;
    pStackContext->CurrentCommand = NULL;
    pStackContext->StateMachine &= ~SMBUS_SMS_ACTIVE_MASK;

  }
  else if ( error & HAL_SMBUS_ERROR_OVR)
  {
    /*
      A case of quick command read probably, setting the flag
     */
    pStackContext->StateMachine |= SMBUS_SMS_QUICK_CMD_R | SMBUS_SMS_READY;
    HAL_SMBUS_Master_Abort_IT( hsmbus, pStackContext->SlaveAddress );
    hsmbus->ErrorCode = HAL_SMBUS_ERROR_NONE;
  }
  else if ( error & HAL_SMBUS_ERROR_ARLO)
  {
#ifdef PMBUS13
    /* Zone Read arbitration lost - retry */
    if (( pStackContext->SlaveAddress == SMBUS_ADDR_ZONE_READ ) && ((pStackContext->StateMachine & SMBUS_SMS_ZONE_READ) != 0 ))
    {
      pStackContext->StateMachine &= ~SMBUS_SMS_ZONE_READ;
      __SMBUS_ZONE_ENABLE(hsmbus);
    }
    else
    {
#endif /* PMBUS13 */
      /*
        Arbitration lost, giving up
      */
      CLEAR_BIT(hsmbus->Instance->CR1, I2C_CR1_ACK);

      if ( hsmbus->Instance->SR1 & I2C_SR1_ADDR )
      {
        __HAL_SMBUS_CLEAR_ADDRFLAG(hsmbus);
      }

      /*
      Clearing the rest of the transmission, including the HW buffer of peripheral
      */
      hsmbus->Instance->SR1 |= I2C_SR1_TXE;
      hsmbus->XferCount = 0;
      hsmbus->XferSize = 0;
      hsmbus->XferOptions = 0;

      /*
        Putting the stack back to original state.
      */
      pStackContext->StateMachine &= ~SMBUS_SMS_TRANSMIT | SMBUS_SMS_RESPONSE_READY | SMBUS_SMS_PROCESSING;
      pStackContext->StateMachine |= SMBUS_SMS_IGNORED | SMBUS_SMS_READY;
#ifdef PMBUS13
    }
#endif  /* PMBUS13 */
  }
  else if ( error & HAL_SMBUS_ERROR_ACKF )
  {
    pStackContext->Device->PreviousState = hsmbus->State;
    pStackContext->Device->State = HAL_SMBUS_STATE_READY;

    /* Process Unlocked */
    __HAL_UNLOCK(pStackContext->Device);

    /* Cease the transmit/receive effort */
    pStackContext->StateMachine &= ~(SMBUS_SMS_TRANSMIT | SMBUS_SMS_RECEIVE);
    STACK_SMBUS_ReadyIfNoAlert(pStackContext);
    error = 0;
  }

  /*
    keep any other error marked in the state machine
   */
  pStackContext->StateMachine |= error << 16;

  /*
    and clear the record of error in the hal driver
   */
  hsmbus->ErrorCode = HAL_SMBUS_ERROR_NONE;
}

/**
  * @brief  function determines if the address could identify this device.
  * @param  pStackContext : Pointer to a SMBUS_StackHandleTypeDef structure that contains
  *                the configuration information for the specified SMBUS.
  * @param  AddrMatchCode: Address Match Code
  * @retval ErrorStatus ERROR if address is not recognized as own.
  */
__weak ErrorStatus STACK_SMBUS_AddrAccpt( SMBUS_StackHandleTypeDef* pStackContext, uint16_t AddrMatchCode)
{
  ErrorStatus retvalue = ERROR;

  /*
   A device could have fixed address, support the ARP or answer to a general call.
   */
#ifdef ARP      /* Address resolution protocol follows */

  /* ARP address match - general call*/
  if (AddrMatchCode == SMBUS_ADDR_DEFAULT )
  {
    /* set the flag that the current call is ARP */
    pStackContext->StateMachine |= SMBUS_SMS_ARP_AM;
    retvalue = SUCCESS;
  }
#endif /*ARP supported*/

#ifdef PMBUS13

  /* Zone command address */
  if ( (AddrMatchCode == SMBUS_ADDR_ZONE_READ ) || (AddrMatchCode == SMBUS_ADDR_ZONE_WRITE ))
  {

   if ( pStackContext->StateMachine & SMBUS_SMS_ZONE_READ )
   {
     retvalue = ERROR;
   }
   else
   {
    retvalue = SUCCESS;
   }
  }
#endif /* PMBUS13 */

  /* Own address match */
  if (AddrMatchCode == pStackContext->OwnAddress )
  {
    retvalue = SUCCESS;
  }

  /* NOTE : This function Should not be modified, when the callback is needed,
           the STACK_SMBUS_AddrAccpt could be implemented in the user file
  */
  return retvalue;
}

/**
  * @brief  Function optionally extending the ExecuteCommand to be ARP capable
  * @param  pStackContext : Pointer to a SMBUS_StackHandleTypeDef structure that contains
  *                the context information for the specified SMBUS stack.
  * @retval HAL_StatusTypeDef response code. Equal STACK_OK if success.
  */
__weak HAL_StatusTypeDef STACK_SMBUS_ExecuteCommandARP( SMBUS_StackHandleTypeDef* pStackContext )
{
  HAL_StatusTypeDef result = STACK_OK;
#ifdef ARP      /* Address resolution protocol follows */
  uint32_t index;
  /*
    Host should implement it's reaction to Host notify protocol and ARP Host notify.
    Regular devices implement all the commands they support.
	Instead of modifying directly this function, use the weak property to replace it.
   */

  if ( pStackContext->CurrentCommand == &(PMBUS_COMMANDS_ARP[0]))
  {
    /*
      1. reset the AR flag
     */
    pStackContext->StateMachine &= ~( SMBUS_SMS_ARP_AR );
    /*
      2. regenerate random UDID part - not implemented in this example
     */
  }
  else
    if ( pStackContext->CurrentCommand == &(PMBUS_COMMANDS_ARP[1]) )
    {
      /*
       1. reset the ARP flags
      */
#ifdef  DEV_PSA /* persistent address prevents AV reset*/
      pStackContext->StateMachine &= ~( SMBUS_SMS_ARP_AR );
#else /* DEV_PSA */
      pStackContext->StateMachine &= ~( SMBUS_SMS_ARP_AV | SMBUS_SMS_ARP_AR );
#endif /* DEV_PSA */
      /*
        2. regenerate random UDID part
       */
    }
    else
      if ( pStackContext->CurrentCommand ==  &(PMBUS_COMMANDS_ARP[2]))
      {
        /*
          1. Copy the UDID to the buffer (preparing answer )
         */
        for ( index = 2; index < 18; index++ )
        {
          pStackContext->Buffer[index] = pStackContext->ARP_UDID[index-2];
        }

        pStackContext->Buffer[1] = 17;
        pStackContext->Buffer[18] = pStackContext->OwnAddress;
      }
      else
        if ( pStackContext->CurrentCommand == &(PMBUS_COMMANDS_ARP[3]))
        {
          if ( (pStackContext->StateMachine & SMBUS_SMS_IGNORED ) == 0)
          {
            /*
             This is a new address assigned by the bus host
              this slave has to remember it
             */
            pStackContext->StateMachine |= SMBUS_SMS_ARP_AV | SMBUS_SMS_ARP_AR;
            pStackContext->OwnAddress = pStackContext->Buffer[18];
            pStackContext->Device->Instance->OAR1 = (I2C_OAR1_OA1EN | pStackContext->Buffer[18]);
          }
        }
        else
        {
          /* no command selected - could be that the stack NACKed previous write and now is the read phase */
          result = STACK_ERROR;
        }
#endif /*ARP*/

  /*
    Returning zero means no problem with execution, if reply is expected, then it is correctly placed in the IO buffer
   */
  return result;
}

#ifdef ARP

/**
  * @brief a subroutine of the LocateCommand called in case or ARP
  * @param  pStackContext : Pointer to a SMBUS_StackHandleTypeDef structure that contains
  *                the configuration information for the specified SMBUS.
  * @param  commandCode : the command code byte value read from input buffer
  * @retval None
  */
void STACK_SMBUS_LocateCommandARP( SMBUS_StackHandleTypeDef* pStackContext, uint8_t commandCode )
{
  switch ( commandCode )
  {
#ifdef  DEV_DIS /* discoverable device only */
    case SMBUS_ARP_CC_PREPARE:
      /* Prepare for ARP command incoming */
      pStackContext->CurrentCommand = &(PMBUS_COMMANDS_ARP[0]);
      break;
    case SMBUS_ARP_CC_RESET:
      /* ARP reset command incoming */
      pStackContext->CurrentCommand = &(PMBUS_COMMANDS_ARP[1]);
      break;
    case SMBUS_ARP_CC_GET_ID:
      /* ID reading command - we only answer if address is not resolved yet */
      if ( ( pStackContext->StateMachine & SMBUS_SMS_ARP_AR ) == 0 )
      {
        pStackContext->CurrentCommand = &(PMBUS_COMMANDS_ARP[2]);
      }
      else
      {
        /* do the best to avoid replying on that command */
        pStackContext->StateMachine |= SMBUS_SMS_IGNORED;
      }
      break;
    case SMBUS_ARP_CC_ASSIGN:
      /* Address assignment from host */
      pStackContext->CurrentCommand = &(PMBUS_COMMANDS_ARP[3]);
      break;
#endif /* DEV_DIS */
    default:
      if ( pStackContext->StateMachine & SMBUS_SMS_ARP_AV )
      {
        /* directed command for this device only, address must be valid then */
        if ( commandCode ==  ( pStackContext->OwnAddress | 0x01 ) )
        {
          /* GetID again */
          pStackContext->CurrentCommand = &(PMBUS_COMMANDS_ARP[2]);
        }
#ifdef DEV_DIS
        else if ( commandCode == pStackContext->OwnAddress )
        {
          /* reset command */
          pStackContext->CurrentCommand = &(PMBUS_COMMANDS_ARP[1]);
        }
#endif /* DEV_DIS */
      }
      break;
  }
  return;
}
#endif /*ARP */

/**
  * @brief  locates a received command in the supported command table
  * @param  pStackContext : Pointer to a SMBUS_StackHandleTypeDef structure that contains
  *                the configuration information for the specified SMBUS.
  * @retval None
  */
__weak void STACK_SMBUS_LocateCommand( SMBUS_StackHandleTypeDef* pStackContext )
{
  uint8_t       commandCode = pStackContext->Buffer[0];
#ifdef      DENSE_CMD_TBL
  uint32_t      current, low, top;
#endif

#ifdef  ARP
  if ( pStackContext->StateMachine & SMBUS_SMS_ARP_AM )
  {
    /* on default device address we do not seek in default command table */
    STACK_SMBUS_LocateCommandARP( pStackContext, commandCode );

  }
  else
#endif /* ARP treatment */
  {
    /*
       Code searching for command based on command code
     */
#ifdef      DENSE_CMD_TBL

    /*
      initializing the command code search - the table must have all commands sorted, but there may be gaps
     */
    low = 0;
    top = pStackContext->CMD_tableSize - 1;
    pStackContext->CurrentCommand = NULL;

    while ( top >= low )
    {
      /*
        Pick interval half
       */
      current = ( low + top ) >> 1 ;
      if (pStackContext->CMD_table[current].cmnd_code == commandCode)
      {
        /*
          we have found our command
         */
        pStackContext->CurrentCommand = &(pStackContext->CMD_table[current]);
        return;
      }
      else if (pStackContext->CMD_table[current].cmnd_code < commandCode)
      {
        /*
          Look at upper half
         */
        low = current + 1;
      }
      else
      {
        top = current - 1;
      }
    }
#else
    /*
    Simple command table - command code equals the table index
    */
    pStackContext->CurrentCommand = &(pStackContext->CMD_table[commandCode]);

    /* a sanity check */
    if ( pStackContext->CurrentCommand->cmnd_code != commandCode )
    {
      pStackContext->CurrentCommand = NULL;
    }
#endif
  }
}

/**
  * @brief  This callback informs the host side application SW that an Alert has been
  *         treated and there is received alert address in the context.
  * @param  pStackContext : Pointer to a SMBUS_StackHandleTypeDef structure that contains
  *                the configuration information for the specified SMBUS.
  * @retval None
  */
__weak void STACK_SMBUS_AlertClbk( SMBUS_StackHandleTypeDef* pStackContext )
{
  /*
   Do not modify this function directly, replace it with own version in your code
   */
  return;
}

/**
  * @brief  check if alert is pending and treat it, otherwise reset state machine
  * @param  pStackContext : Pointer to a SMBUS_StackHandleTypeDef structure that contains
  *                the configuration information for the specified SMBUS.
  * @retval None
  */
void STACK_SMBUS_ReadyIfNoAlert( SMBUS_StackHandleTypeDef* pStackContext )
{

  if ((pStackContext->StateMachine & SMBUS_SMS_ALERT_PENDING) == SMBUS_SMS_ALERT_PENDING )
  {
    /*
      During a command processing, a device signalled an alert
     */
    if (pStackContext->CurrentCommand != &ALERT_RESPONSE)
    {
      /*
       Last command was not an alert response - we send alert response
       */
      pStackContext->CurrentCommand = &ALERT_RESPONSE;
      pStackContext->StateMachine |= SMBUS_SMS_READY;
      STACK_SMBUS_HostRead( pStackContext, &(pStackContext->OwnAddress) , SMBUS_ADDR_ARA);
    }
    else
    {
      /*
       Means we already sent alert response and the address has been read
       */
      pStackContext->StateMachine |= SMBUS_SMS_READY | SMBUS_SMS_ALERT_ADDRESS;

      /*
       the callback...
       */
      STACK_SMBUS_AlertClbk( pStackContext );
    }
  }
  else
  {
    /*
    No alert, Master is ready to poke another device
    */
    pStackContext->StateMachine |= SMBUS_SMS_READY;
    pStackContext->CurrentCommand = NULL;
  }
}

/**
  * @brief  SMBUS initialization routine.
  * @param  pStackContext : Pointer to a SMBUS_StackHandleTypeDef structure that contains
  *                the configuration information for the specified SMBUS.
  * @retval SMBus stack return code
  */
HAL_StatusTypeDef STACK_SMBUS_Init( SMBUS_StackHandleTypeDef* pStackContext )
{
  uint32_t              index = 0;
  HAL_StatusTypeDef     result = STACK_OK;

  while ( SMBUSInstancesList[index] != NULL )
  {
    index++;
  }

  pStackContext->SlaveAddress = 0;

  SMBUSInstancesList[index] = pStackContext;

  /*
   No mode, nothing going on
   */
  pStackContext->StateMachine |= SMBUS_SMS_READY;
  pStackContext->OpMode = 0;

#ifdef PMBUS13
  /* initializing the Zone settings */
  pStackContext->TheZone = ZERO_ZONE;
#endif /* PMBUS13 */

#ifdef ALERT
  /*
    If master is initialized
   */
  if ( pStackContext->Device->Instance->CR1 & SMBUS_PERIPHERAL_MODE_HOST )
  {
    /*
     Alert is by default enabled for a host
    */
    result = HAL_SMBUS_EnableAlert_IT( pStackContext->Device );
  }
#endif /* ALERT */

  /*
    the device listens for common commands, the host for notify protocol
   */
  if ( result == STACK_OK )
  {
    return HAL_OK;
  }
  else
  {
    return result;
  }
}

/**
  * @brief  SMBUS IO buffer access function. Returns a valid address if buffer can be written by application.
  *        Returns NULL if the stack or driver is busy using the buffer. Application should not store the pointer
  *        and always retrieve the buffer this way to check availability.
  * @param  pStackContext : Pointer to a SMBUS_StackHandleTypeDef structure that contains
  *                the configuration information for the specified SMBUS.
  * @retval I/O buffer
  */
uint8_t* STACK_SMBUS_GetBuffer( SMBUS_StackHandleTypeDef* pStackContext )
{
  uint8_t* pResult;
  /*
    return NULL if the stack/driver is busy
   */
  if (
    ( STACK_SMBUS_IsBusy(pStackContext) ) ||
    (
      ( pStackContext->Device->State != HAL_SMBUS_STATE_READY ) &&
      ( pStackContext->Device->State != HAL_SMBUS_STATE_LISTEN )
    )
  )
  {
    pResult = NULL;
  }
  else
  {
    /*
      starts at second position, first is reserved for the command code
     */
    pResult = &(pStackContext->Buffer[1]);
  }
  return pResult;
}

/**
  * @brief  SMBUS master command transmit.
  * @param  pStackContext : Pointer to a SMBUS_StackHandleTypeDef structure that contains
  *                the context information for the specified SMBUS stack.
  * @param  pCommand : description of the command to be transmitted, NULL for quick command
  * @param  address : slave address to be used in the transmission
  * @param  direction : either READ or WRITE, some commands have the choice with the same ordinal.
  *                     Do not use BLOCK - it is tied to command definition and implicit.
  *                     Leave zero for process call.
  * @retval SMBus stack return code
  */
HAL_StatusTypeDef STACK_SMBUS_HostCommand(SMBUS_StackHandleTypeDef *pStackContext, st_command_t* pCommand, uint16_t address, uint32_t direction)
{
  HAL_StatusTypeDef     result = STACK_BUSY;
  uint32_t              xFerOptions = SMBUS_FIRST_FRAME;
  uint8_t               *com_buffer;
  uint16_t              size;

  /*
    First check status of the SMBUS - no transaction ongoing
  */
  if ( ( (pStackContext->StateMachine) & SMBUS_SMS_ACTIVE_MASK ) == 0 )
  {
    if ( pCommand == NULL )
    {
      /*
      quick command case
      */
      size = 0;
      xFerOptions = SMBUS_FIRST_AND_LAST_FRAME;

      /* set buffer to NULL to remove autoend and manage STOP by SW */
      com_buffer = NULL;
    }
    else
    {
      /* set buffer to current context */
      com_buffer = pStackContext->Buffer;

      /*
          Remembering the address and command code for case of further processing of non-trivial command
        */
      pStackContext->SlaveAddress = address;
      pStackContext->CurrentCommand = pCommand;

      /*
      First byte, the command code is transmitted
      */
      com_buffer[0] = pCommand->cmnd_code;

      if ( ( pCommand->cmnd_query & BLOCK ) && ( direction != READ ) )
      {
        /*
            Block write and process call with data size prepared in the buffer.
            Data size is limited to STACK_NBYTE_SIZE
          */
        if (com_buffer[1] > STACK_NBYTE_SIZE )
        {
          com_buffer[1] = STACK_NBYTE_SIZE;
        }
        size = 2 + com_buffer[1];            /* 1 cmd code + 1 count + data size */
      }
      else
      {
        if ( direction == READ )
        {
          /*
            transmitting only the command code, then begins the read phase
          */
          size = 1;
        }
        else
        {
          /*
            fixed size write
          */
          size = pCommand->cmnd_master_Tx_size;
        }
      }

      if ( pCommand->cmnd_query == BLK_PRC_CALL)
      {
        /*
          Process call starts as BLOCK_WRITE mode
        */
        pStackContext->OpMode = BLOCK_WRITE;
        xFerOptions = SMBUS_FIRST_FRAME;
      }
      else
      {
        /*
          Check direction sanity and mark direction (mainly for READ_OR_WRITE kind of command)
        */
        if (
          ( pCommand->cmnd_query & READ_OR_WRITE ) &&
          ( ( direction != WRITE ) && ( direction != READ ) )
        )
        {
          result = STACK_ERROR;
        }
        else
        {
          pStackContext->OpMode = direction;

          /*
          In case of Write this is a last frame too
          */
          if (( direction == WRITE ) || ( direction == BLOCK_WRITE ))
          {
            /*
              Size of transmission may include the PEC byte
              */
            xFerOptions = SMBUS_FIRST_AND_LAST_FRAME | ( pStackContext->StateMachine & SMBUS_SMS_PEC_ACTIVE );
            if (pStackContext->StateMachine & SMBUS_SMS_PEC_ACTIVE )
            {
              size += 1; /* PEC_SIZE */
            }
          }
        }
      }
    }

    if ( result != STACK_ERROR )
    {

      /*
      Initiating a transmission - even for a Read the command code is sent first
      */
      pStackContext->StateMachine |= SMBUS_SMS_TRANSMIT;
      pStackContext->StateMachine &= ~SMBUS_SMS_READY;

      /*
        Sending the data and logging the result
      */
      result = HAL_SMBUS_Master_Sequential_Transmit_IT( pStackContext->Device, address, com_buffer, size, xFerOptions );
      if (result != HAL_OK )
      {
        pStackContext->StateMachine |= SMBUS_SMS_ERROR;
      }
    }
  }
  return result;
}

/**
  * @brief  SMBUS notify host command transmit.
  * @param  pStackContext : Pointer to a SMBUS_StackHandleTypeDef structure that contains
  *                the context information for the specified SMBUS stack.
  * @retval HAL_StatusTypeDef SMBus stack return code
  */
HAL_StatusTypeDef STACK_SMBUS_NotifyHost(SMBUS_StackHandleTypeDef *pStackContext)
{
  HAL_StatusTypeDef     result = STACK_BUSY;
  uint32_t              xFerOptions = SMBUS_FIRST_AND_LAST_FRAME;

  /*
     First check status of the SMBUS - no transaction ongoing
   */
  if (( (pStackContext->StateMachine) & SMBUS_SMS_ACTIVE_MASK ) == 0 )
  {
    /*
      Remembering the address and command code for case of further processing of non-trivial command
    */
    pStackContext->SlaveAddress = SMBUS_ADDR_HOST;
    pStackContext->CurrentCommand = &HOST_NOTIFY_PROTOCOL;

    /*
      First byte, the return address is transmitted
    */
    if ( ( pStackContext->OwnAddress == 0 ) || (( pStackContext->StateMachine & SMBUS_SMS_ARP_AR) == 0 ) )
    {
      /* no address resolved yet, initiating the ARP */
      pStackContext->Buffer[0] = SMBUS_ADDR_DEFAULT;
    }
    else
    {
      /* own address known and recognized */
      pStackContext->Buffer[0] = pStackContext->OwnAddress;
    }

    pStackContext->OpMode = WRITE;

    /*
    Initiating a transmission - even for a Read the command code is sent first
    */
    pStackContext->StateMachine |= SMBUS_SMS_TRANSMIT;
    pStackContext->StateMachine &= ~SMBUS_SMS_READY;

    /*
      Sending the data and logging the result
    */
    result = HAL_SMBUS_Master_Sequential_Transmit_IT( pStackContext->Device, SMBUS_ADDR_HOST, pStackContext->Buffer, 3, xFerOptions );
    if (result != HAL_OK )
    {
      pStackContext->StateMachine |= SMBUS_SMS_ERROR;
    }
  }
  return result;
}

/**
  * @brief  SMBUS master command receive - only usable for quick command (read) and receive byte.
  * @param  pStackContext : Pointer to a SMBUS_StackHandleTypeDef structure that contains
  *                the context information for the specified SMBUS stack.
  * @param  pData  : pointer to the variable where response should be stored
  * @param  address : slave address to be used in the transmission
  * @retval HAL_StatusTypeDef SMBus stack return code
  */
HAL_StatusTypeDef STACK_SMBUS_HostRead(SMBUS_StackHandleTypeDef *pStackContext, uint8_t* pData, uint16_t address)
{
  HAL_StatusTypeDef     result = STACK_BUSY;
  uint8_t               size;
  uint32_t              xFerOptions;

  /*
    First check state of the SMBUS
   */
  if ( ( pStackContext->StateMachine & SMBUS_SMS_READY ) == SMBUS_SMS_READY )
  {
    /*
      State transition from Ready to Reception
    */
    pStackContext->StateMachine &= ~SMBUS_SMS_READY;
    pStackContext->StateMachine |= SMBUS_SMS_RECEIVE;

    /*
      May be a receive byte ( then we need pointer to store reply ) or quick command read
    */
    if ( pData == NULL )
    {
      size = 0;
      xFerOptions = SMBUS_FIRST_AND_LAST_FRAME;
    }
    else
    {
      size = 1;
      if (pStackContext->StateMachine & SMBUS_SMS_PEC_ACTIVE )
      {
        size += PEC_SIZE;
      }
      xFerOptions = SMBUS_FIRST_AND_LAST_FRAME | ( pStackContext->StateMachine & SMBUS_SMS_PEC_ACTIVE );
    }

    /*
      Ordering the HAL to do single frame read operation, checking the result
    */
    result = HAL_SMBUS_Master_Sequential_Receive_IT( pStackContext->Device, address, pData, size, xFerOptions );
    if (result != HAL_OK )
    {
      pStackContext->StateMachine |= SMBUS_SMS_ERROR;
    }
  }

  return result;
}

/**
  * @brief  Callback function notifying slave about command incoming.
  * @param  pStackContext : Pointer to a SMBUS_StackHandleTypeDef structure that contains
  *                the context information for the specified SMBUS stack.
  * @retval HAL_StatusTypeDef response code. STACK_OK if success, any other value means problem
  */
__weak HAL_StatusTypeDef STACK_SMBUS_ExecuteCommand( SMBUS_StackHandleTypeDef* pStackContext )
{
  /*
    Host should implement here it's reaction to Host notify protocol and ARP Host notify.
    Regular devices implement all the commands they support.
   */

  /*
    Returning zero means no problem with execution, if reply is expected, then it is correctly placed in the IO buffer
   */
  return STACK_OK;
}

/**
  * @brief  Callback function notifying application about command extension.
  * @param  pStackContext : Pointer to a SMBUS_StackHandleTypeDef structure that contains
  *                the context information for the specified SMBUS stack.
  * @retval HAL_StatusTypeDef response code. STACK_OK if success, any other value means problem
  */
__weak HAL_StatusTypeDef STACK_SMBUS_ExtendCommand( SMBUS_StackHandleTypeDef* pStackContext )
{
  /*
    Host should implement here it's reaction to Host notify protocol and ARP Host notify.
    Regular devices implement all the commands they support.
   */

  /*
    Returning zero means no problem with execution, if reply is expected, then it is correctly placed in the IO buffer
   */
  return STACK_OK;
}

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

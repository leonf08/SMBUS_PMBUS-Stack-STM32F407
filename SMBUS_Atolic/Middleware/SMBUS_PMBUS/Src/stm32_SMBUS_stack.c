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
}

/** @brief  Slave Tx Transfer completed callbacks.
  * @param  hsmbus: Pointer to a SMBUS_HandleTypeDef structure that contains
  *                the configuration information for the specified SMBUS.
  * @retval None
  */
void HAL_SMBUS_SlaveTxCpltCallback(SMBUS_HandleTypeDef *hsmbus)
{
  SMBUS_StackHandleTypeDef* pStackContext;

  /*
    Resolve which stack serves the port that initiated callback
   */
  pStackContext = STACK_SMBUS_ResolveContext( hsmbus );

  /*
    Completed transmission means this was not a quick command read but actual read.
    We need to record this by clearing the flag.
   */
  pStackContext->StateMachine &= ~SMBUS_SMS_TRANSMIT;

#ifdef  PMBUS13
  /*
    A case of Zone read - this slave devide has transmitted its data, will ignore further reads
   */
  if (pStackContext->SlaveAddress == SMBUS_ADDR_ZONE_READ)
  {
    __SMBUS_ZONE_DISABLE(hsmbus);
    pStackContext->StateMachine |= SMBUS_SMS_ZONE_READ;    
  }
#endif  
}

/**
  * @brief  Slave Rx Transfer completed callbacks.
  * @param  hsmbus : Pointer to a SMBUS_HandleTypeDef structure that contains
  *                the configuration information for the specified SMBUS.
  * @retval None
  */
void HAL_SMBUS_SlaveRxCpltCallback(SMBUS_HandleTypeDef *hsmbus)
{
  SMBUS_StackHandleTypeDef* pStackContext;
  uint32_t       size;
  uint32_t       xFerOptions = SMBUS_NEXT_FRAME;

  /*
    Resolve which stack serves the port that initiated callback
   */
  pStackContext = STACK_SMBUS_ResolveContext( hsmbus );

  if ( pStackContext->StateMachine & SMBUS_SMS_IGNORED )
  {
    __HAL_SMBUS_GENERATE_NACK( hsmbus );
    /* TCR may still stretch the SCL */
    if ( hsmbus->Instance->ISR & I2C_ISR_TCR )
    {
      hsmbus->Instance->CR2 |= (((uint32_t) 1 << 16 ) & I2C_CR2_NBYTES);
    }
  }
  else
  {
    /*
      Case of no command identified yet
    */
    if ( pStackContext->CurrentCommand == NULL )
    {
      pStackContext->StateMachine &= ~SMBUS_SMS_RECEIVE;

      /*
        Reception completed, now we locate what command was received
      */
#ifdef PMBUS13
      /*
        Testing for an exception in the specification - the Zone Read has no
        traditional command code
      */
      if (pStackContext->SlaveAddress == SMBUS_ADDR_ZONE_READ )
      {
        /*    
         Example implementation follows, modify if needed.
        */
        if ( pStackContext->TheZone.readZone == pStackContext->TheZone.activeReadZone )
        {              
          /* This slave device will respond to this Zone Read call */
          pStackContext->CurrentCommand = (st_command_t*) &ZONE_READ_COMMAND;
        }
        else 
        {
          /* This slave device will not respond to this Zone Read call */
          pStackContext->StateMachine |= SMBUS_SMS_IGNORED;
          pStackContext->StateMachine |= SMBUS_SMS_ZONE_READ;
        }
      }
      else 
      {
#endif        
        STACK_SMBUS_LocateCommand( pStackContext );
#ifdef PMBUS13
      }
#endif
      
      /*
      Unknown command case - send NACK
      */
      if ( pStackContext->CurrentCommand == NULL )
      {
        pStackContext->StateMachine |= SMBUS_SMS_IGNORED;
        __HAL_SMBUS_GENERATE_NACK( hsmbus );
        /* TCR may still stretch the SCL */
        if ( hsmbus->Instance->ISR & I2C_ISR_TCR )
        {
          hsmbus->Instance->CR2 |= (((uint32_t) 1 << 16 ) & I2C_CR2_NBYTES);
        }
        /*
        NOTE - Marking this case as error is optional, depends on application needs
        */
      }
      else
      {
        /*
          process call or block write case - read number of bytes
        */
        if (
          ( pStackContext->CurrentCommand->cmnd_query & BLOCK_WRITE ) == BLOCK_WRITE ||
          ( pStackContext->CurrentCommand->cmnd_query == BLK_PRC_CALL )
        )
        {
          /*
          Making the book-keeping and initiating the transaction.
          */
          pStackContext->Byte_count++;
          pStackContext->StateMachine |= SMBUS_SMS_RECEIVE | SMBUS_SMS_PROCESSING;
          HAL_SMBUS_Slave_Receive_IT( hsmbus, &(pStackContext->Buffer[1]), 1, xFerOptions );
        }
        else
        {
          /*
            otherwise size of data is fixed
            - TODO - this "read" condition is not perfect, and may fail with READ_OR_WRITE + PEC cases
          */
          if (
            ( pStackContext->CurrentCommand->cmnd_query & READ ) ||
            ( pStackContext->CurrentCommand->cmnd_query & PROCESS_CALL )
          )
          {
            /*
            There will be a read phase to follow, no PEC.
            */
            size = pStackContext->CurrentCommand->cmnd_master_Tx_size;
          }
          else
          {
            /*
            This is going to be the last frame (simple write case)
            */
            size = pStackContext->CurrentCommand->cmnd_master_Tx_size;
            if (pStackContext->StateMachine & SMBUS_SMS_PEC_ACTIVE )
            {
              size += PEC_SIZE;
            }
            xFerOptions = SMBUS_LAST_FRAME_NO_PEC | ( pStackContext->StateMachine & SMBUS_SMS_PEC_ACTIVE );
          }

          /* the case of having 2 byte long command code (or zone read)*/
          if (
              (pStackContext->CurrentCommand->cmnd_code == SMBUS_COMMAND_EXT ) ||
              (pStackContext->SlaveAddress == SMBUS_ADDR_ZONE_READ )
             )
          {
            xFerOptions = 0;
          }
          /*
            anything left to receive?
          */
          if ( size > 1 )
          {
            pStackContext->Byte_count += size - 1;
            pStackContext->StateMachine |= SMBUS_SMS_RECEIVE;
            HAL_SMBUS_Slave_Receive_IT( hsmbus, &(pStackContext->Buffer[1]), size - 1, xFerOptions );
            /*
              command code byte subtracted from the command Tx size - it was received already
            */
          }
        }
      }
    }
    else /* CurrentCommand != NULL */
    {
      /*
        A case of block reading continuation - reading the remaining data
      */
      if (( pStackContext->StateMachine & SMBUS_SMS_PROCESSING) == SMBUS_SMS_PROCESSING)
      {

#ifdef ARP
        /*
          special case of assign address - byte-by-byte reception and comparison
          */
        if ( ( pStackContext->StateMachine & SMBUS_SMS_ARP_AM ) && ( pStackContext->CurrentCommand == &(PMBUS_COMMANDS_ARP[3])) )
        {
          /* Retieving the current position */
          size = pStackContext->Byte_count - 1;
          size++;

          /* If address received, we execute the command implementation */
          if ( size > 18 )
          {
            STACK_SMBUS_ExecuteCommandARP( pStackContext );

            /*
              Command finished, marking the state
            */
            pStackContext->StateMachine &= ~(SMBUS_SMS_RECEIVE | SMBUS_SMS_PROCESSING);
            pStackContext->StateMachine |= SMBUS_SMS_RESPONSE_READY;
            /* Getting PEC byte of the command */
            pStackContext->Byte_count = size + 1;
          }
          /* comparing the UDID with the buffer, NACK if not match */
          else if ((size > 2) && ( pStackContext->Buffer[size-1] != pStackContext->ARP_UDID[size-3] ))
          {
            pStackContext->StateMachine |= SMBUS_SMS_IGNORED;
            pStackContext->StateMachine &= ~(SMBUS_SMS_RECEIVE | SMBUS_SMS_PROCESSING);
            __HAL_SMBUS_GENERATE_NACK( hsmbus );
            /* TCR may still stretch the SCL */
            if ( hsmbus->Instance->ISR & I2C_ISR_TCR )
            {
              hsmbus->Instance->CR2 |= (((uint32_t) 1 << 16 ) & I2C_CR2_NBYTES);
            }
          }
          /* Getting next byte of the command */
          else if (size == 18 )
          {
            pStackContext->Byte_count = size + 2;
            xFerOptions = SMBUS_LAST_FRAME_NO_PEC | ( pStackContext->StateMachine & SMBUS_SMS_PEC_ACTIVE );
            HAL_SMBUS_Slave_Receive_IT( hsmbus, &(pStackContext->Buffer[size]), 1 + PEC_SIZE, xFerOptions );
          }
          else
          {
            pStackContext->Byte_count = size + 1;
            HAL_SMBUS_Slave_Receive_IT( hsmbus, &(pStackContext->Buffer[size]), 1, SMBUS_NEXT_FRAME );
          }

        }
        else
#endif /* ARP */

        {
          /*
            size of remaining data has been alredy read to the buffer on position 1
            is limited to STACK_NBYTE_SIZE
          */
          size = pStackContext->Buffer[1];
          if ( size > STACK_NBYTE_SIZE )
          {
            size = STACK_NBYTE_SIZE;
          }

          /*
            A case of block reading continuation - remaining data ( size read )
          */
          if ( size != 0 )
          {
            /*
              size does not include PEC byte, we receive remaining data with size correction
            */
            if (
              (pStackContext->StateMachine & SMBUS_SMS_PEC_ACTIVE ) &&
              ((pStackContext->CurrentCommand->cmnd_query & ( READ | PROCESS_CALL )) == 0 )
            )
            {
              size += PEC_SIZE;
            }
            pStackContext->Byte_count += size;
            HAL_SMBUS_Slave_Receive_IT( hsmbus, &(pStackContext->Buffer[2]), size, SMBUS_LAST_FRAME_NO_PEC + (pStackContext->StateMachine & SMBUS_SMS_PEC_ACTIVE ) );
          }

          pStackContext->StateMachine &= ~(SMBUS_SMS_RECEIVE | SMBUS_SMS_PROCESSING);
          /*
            reception completed, and the processing flag is cleared to indicate the
            next reception can execute the command code
          */
        }
      }
      else if (pStackContext->CurrentCommand->cmnd_code == SMBUS_COMMAND_EXT )
      {
        pStackContext->StateMachine &= ~SMBUS_SMS_RECEIVE;
        STACK_SMBUS_ExtendCommand( pStackContext );
      }
      else
      {
        pStackContext->StateMachine &= ~SMBUS_SMS_RECEIVE;
        if (pStackContext->SlaveAddress == SMBUS_ADDR_ZONE_READ )
        {
          STACK_PMBUS_ZoneReadCallback( pStackContext, 1 );
        }
      }
    }
  }
}

/**
  * @brief  Address Match callbacks.
  * @param  hsmbus : Pointer to a SMBUS_HandleTypeDef structure that contains
  *                the configuration information for the specified SMBUS.
  * @param  TransferDirection: Master request Transfer Direction (Write/Read)
  * @param  AddrMatchCode: Address Match Code
  * @retval None
  */
void HAL_SMBUS_AddrCallback(SMBUS_HandleTypeDef *hsmbus, uint8_t TransferDirection, uint16_t AddrMatchCode)
{
  SMBUS_StackHandleTypeDef* pStackContext;
  uint16_t           size;
  uint8_t            response;
  uint32_t           result = STACK_OK;

  /*
    Resolve which stack serves the port that initiated callback
   */
  pStackContext = STACK_SMBUS_ResolveContext( hsmbus );
  
  /*
     Clear the history of command records - if flag was not cleared yet, the application probably doesn't need it
   */
  pStackContext->StateMachine &= ~( SMBUS_SMS_QUICK_CMD_W | SMBUS_SMS_QUICK_CMD_R );

  /*
    First normalize address - bit-shift it
   */
  AddrMatchCode = AddrMatchCode << 1;

  /*
    Check for host notify protocol ( we are host and being addressed )
   */
  if (
    ( pStackContext->Device->Instance->CR1 & SMBUS_PERIPHERAL_MODE_HOST ) &&
    ( AddrMatchCode == SMBUS_ADDR_HOST ) &&
    ( TransferDirection == 0 )
  )
  {
    /*
      In this case we know exactly the frame is 3B long, we are taking a short-cut.
     */
    pStackContext->StateMachine &= ~SMBUS_SMS_READY;
    pStackContext->Byte_count = 3;
    HAL_SMBUS_Slave_Receive_IT( hsmbus, pStackContext->Buffer, 3, SMBUS_FIRST_AND_LAST_FRAME);
    pStackContext->CurrentCommand = &HOST_NOTIFY_PROTOCOL;
  }

  /*
    Try match for ARA - alert response is checked, but only in case it is us, who issued the signal ( I2C_CR1_ALERTEN )
   */
  else if ( ( AddrMatchCode == SMBUS_ADDR_ARA ) && (TransferDirection) && ( (pStackContext->Device->Instance->CR1) & I2C_CR1_ALERTEN ) )
  {
    /*
      The ready (listen) state is over, transition to transmission of alert address
     */
    pStackContext->StateMachine &= ~SMBUS_SMS_READY;
    pStackContext->StateMachine |= SMBUS_SMS_TRANSMIT;
    pStackContext->Byte_count = 1;
    HAL_SMBUS_Slave_Transmit_IT( hsmbus, &(pStackContext->OwnAddress), 1, SMBUS_FIRST_AND_LAST_FRAME_NO_PEC | ( pStackContext->StateMachine & SMBUS_SMS_PEC_ACTIVE ));

    /*
      Turn off the alert signal
     */
    pStackContext->Device->Instance->CR1 &= ~I2C_CR1_ALERTEN;
  }
  else
  {
    /*
      Address may not be precisely the same - callback to accept the address
    */
    if ( STACK_SMBUS_AddrAccpt( pStackContext, AddrMatchCode) == 0 )
    {
      
      /* This is the best way not to block the line */      
      __HAL_SMBUS_GENERATE_NACK( hsmbus );
      hsmbus->Instance->ICR |= I2C_ICR_ADDRCF;
      HAL_SMBUS_EnableListen_IT( hsmbus );
      /* sometimes writing 0xFF to TXDR helps too */
    }
    else
    {
      /* Remembering the addres ot the currently processed command */
      pStackContext->SlaveAddress = AddrMatchCode;
 
      /*
        Read part
      */
      if ( TransferDirection != 0 )
      {

        /* Here we test for a condition when the address match is part of an already ignored read/process call command */
        if (
          ( pStackContext->StateMachine & SMBUS_SMS_IGNORED ) && \
          (pStackContext->CurrentCommand->cmnd_query & ( READ | PROCESS_CALL ) )
        )
        {
          /* This is the best way not to block the line */
          hsmbus->Instance->TXDR = 0xFF;
          /* TODO - find a clean way how to abstain from blocking the bus */
   
          if ( hsmbus->Instance->ISR & I2C_ISR_TCR )
          {
            hsmbus->Instance->CR2 |= (((uint32_t) 1 << 16 ) & I2C_CR2_NBYTES);
          }
          __HAL_SMBUS_GENERATE_NACK( hsmbus );
          hsmbus->Instance->ICR |= I2C_ICR_ADDRCF;
          pStackContext->StateMachine &= ~(SMBUS_SMS_IGNORED | SMBUS_SMS_ARP_AM);
          HAL_SMBUS_EnableListen_IT( hsmbus );
          /* sometimes writing 0xFF to TXDR helps too */
        }
        else
        {
          /* else stack makes sure the ignored flag is down */
          pStackContext->StateMachine &= ~SMBUS_SMS_IGNORED;
   
          /*
            If the response is not ready yet, but the read is was preceded by write.
           */
          if (( pStackContext->StateMachine & ( SMBUS_SMS_READY | SMBUS_SMS_RESPONSE_READY ) ) == 0 )
          {
            /*
              this is a case of read command - we must execute it first (callback)
            */
            if ( pStackContext->StateMachine & SMBUS_SMS_ARP_AM )
            {
              /*
                An ARP command ( determined based on address )
              */
              result = STACK_SMBUS_ExecuteCommandARP( pStackContext );
            }
#ifdef PMBUS13            
              else if ( pStackContext->SlaveAddress == SMBUS_ADDR_ZONE_READ )
              {
                //__SMBUS_NOSTRETCH(hsmbus);
                //__SMBUS_ZONE_DISABLE(pStackContext->Device);
                //pStackContext->Device->Instance->OAR2 |= 0x00000700;
                //__SMBUS_ZONE_ENABLE(pStackContext->Device);
                result = STACK_PMBUS_ZoneReadCallback( pStackContext, 0 );
              }
#endif 
            else
            {
              /*
                An regular command
              */
              result = STACK_SMBUS_ExecuteCommand( pStackContext );
            }
   
            if ( result == STACK_OK )
            {
              /*
                successful execution - response should be present in IO buffer
              */
              pStackContext->StateMachine |= SMBUS_SMS_RESPONSE_READY;
            }
          }
   
          if ( result != STACK_OK )
          {
            /*
              Command callback execution failed - sending NACK
            */
            pStackContext->StateMachine |= SMBUS_SMS_IGNORED;
            __HAL_SMBUS_GENERATE_NACK( hsmbus );
            hsmbus->Instance->ICR |= I2C_ICR_ADDRCF;
            HAL_SMBUS_EnableListen_IT( hsmbus );
            /* sometimes writing 0xFF to TXDR helps too */
          }
          else
          {
            if ( pStackContext->StateMachine & SMBUS_SMS_RESPONSE_READY )
            {
              /*
              * means we received and executed command and have data to send back
              */
              pStackContext->StateMachine |= SMBUS_SMS_TRANSMIT;
   
              if  ( ( pStackContext->CurrentCommand->cmnd_query & BLOCK ) == BLOCK )
              {
                /*
                  Variable reply size - size is also prepared
                  by the command in the output buffer (hopefully).
                  is limited to STACK_NBYTE_SIZE
                */
                if ( pStackContext->Buffer[1] > STACK_NBYTE_SIZE )
                {
                  pStackContext->Buffer[1] = STACK_NBYTE_SIZE;
                }
                size = 1 + (pStackContext->Buffer[1]); /* count + actual data */
   
                /*
                  Info about current command will not be needed more.
                  Normally this is reset in listen complete (stop condition) but process call
                  is an exception.
                */
                pStackContext->CurrentCommand = NULL;
              }
              else
              {
                /*
                  a regular read byte or word command
                */
                size = pStackContext->CurrentCommand->cmnd_master_Rx_size;
              }
   
              /*
              Transmission size is automatically increased by the PEC byte
              */
              if (pStackContext->StateMachine & SMBUS_SMS_PEC_ACTIVE )
              {
                size += 1; /* PEC byte */
                pStackContext->Buffer[1] += 1;
              }
   
              /*
                Now we record the buffer position and send the reply
              */
              pStackContext->Byte_count = size;
              HAL_SMBUS_Slave_Transmit_IT( hsmbus, &(pStackContext->Buffer[1]), size, \
                                           SMBUS_LAST_FRAME_NO_PEC | ( pStackContext->StateMachine & SMBUS_SMS_PEC_ACTIVE ) );
            }
            else if ( pStackContext->StateMachine & SMBUS_SMS_READY )
            {
              /*
              * We have no response prepared and no write preceded this read means
              * this is a receive byte case or a quick command (read)
              * It is impossible to determine in advance, there are choices:
              */
              if (!( pStackContext->StateMachine & SMBUS_SMS_RCV_BYTE_OFF ))
              {
                /*
                If ReceiveByte transaction is not completely disabled, we execute it
                */
                size = 1;
                /* PEC byte */
                if (pStackContext->StateMachine & SMBUS_SMS_PEC_ACTIVE )
                {
                  size += 1;
                }
                pStackContext->StateMachine &= ~SMBUS_SMS_RESPONSE_READY;
                pStackContext->StateMachine |= SMBUS_SMS_TRANSMIT;
                pStackContext->Byte_count = size;
                if ( pStackContext->StateMachine & SMBUS_SMS_RCV_BYTE_LMT )
                {
                  /*
                  This is a safe solution that allows the master to transmit STOP and
                  doesn't not block the QuickCommand
                  */
                  response = pStackContext->SRByte | 0x80;
                  HAL_SMBUS_Slave_Transmit_IT( hsmbus, &(response) , size, SMBUS_FIRST_AND_LAST_FRAME_NO_PEC | ( pStackContext->StateMachine & SMBUS_SMS_PEC_ACTIVE ));
                }
                else
                {
                  /*
                  This option allows all possible values of the response byte
                  but is blocking in case master wants a quick command read
                  */
                  HAL_SMBUS_Slave_Transmit_IT( hsmbus, &(pStackContext->SRByte), size, SMBUS_FIRST_AND_LAST_FRAME_NO_PEC | ( pStackContext->StateMachine & SMBUS_SMS_PEC_ACTIVE ));
                }
              }
            }
          }
        }
      }
      else  /* Write direction detected */
      {
        /* New transmission, clearing the 'ignored' flag */
        pStackContext->StateMachine &= ~SMBUS_SMS_IGNORED;
   
        if ( pStackContext->StateMachine & SMBUS_SMS_READY )
        {
          /*
          * A new command arrived - we have to read the first byte to determine it's type
          */
          pStackContext->StateMachine &= ~SMBUS_SMS_READY;
          pStackContext->StateMachine |= SMBUS_SMS_RECEIVE;
          pStackContext->CurrentCommand = NULL;
          pStackContext->Byte_count = 1;
          HAL_SMBUS_Slave_Receive_IT( hsmbus, pStackContext->Buffer, 1, SMBUS_FIRST_FRAME );
        }
      }
    }
  }
}

/**
  * @brief  Listen Complete callbacks.
  * @param  hsmbus : Pointer to a SMBUS_HandleTypeDef structure that contains
  *                the configuration information for the specified SMBUS.
  * @retval None
  */
void HAL_SMBUS_ListenCpltCallback(SMBUS_HandleTypeDef *hsmbus)
{
  SMBUS_StackHandleTypeDef* pStackContext;

  /*
    Resolve which stack serves the port that initiated callback
   */
  pStackContext = STACK_SMBUS_ResolveContext( hsmbus );

  /*
    communication over, let's determine what was it
   */
  if ( pStackContext->CurrentCommand == 0 )
  {
    if ((pStackContext->StateMachine & SMBUS_SMS_RECEIVE) == SMBUS_SMS_RECEIVE )
    {
      /*
        a quick command write detected - address was detected with write but no write occurred
      */
      pStackContext->StateMachine |= SMBUS_SMS_QUICK_CMD_W | SMBUS_SMS_READY;
      STACK_SMBUS_ExecuteCommand( pStackContext );
    }
    else if ((pStackContext->StateMachine & SMBUS_SMS_QUICK_CMD_R) == SMBUS_SMS_QUICK_CMD_R )
    {
      /*
        a quick command read case - flag was set during OVR error treatment
      */
      STACK_SMBUS_ExecuteCommand( pStackContext );
    }
  }
  else if (( pStackContext->StateMachine & SMBUS_SMS_RESPONSE_READY) != SMBUS_SMS_RESPONSE_READY )
  {
    /*
      we received a simple command we need yet to execute
     */
    if ( pStackContext->StateMachine & SMBUS_SMS_ARP_AM )
    {
      STACK_SMBUS_ExecuteCommandARP( pStackContext );
    }
    else
    {
      STACK_SMBUS_ExecuteCommand( pStackContext );
    }
  }

  /*
    As the communication is concluded we want to reset the stack
   */
  pStackContext->StateMachine |= SMBUS_SMS_READY;
  pStackContext->StateMachine &= ~( SMBUS_SMS_RESPONSE_READY | SMBUS_SMS_IGNORED | SMBUS_SMS_ARP_AM | SMBUS_SMS_PROCESSING | SMBUS_SMS_ZONE_READ);
  pStackContext->CurrentCommand = NULL;

  /* sometimes there is a PEC byte left in RXDR due to command type confusion (READ or WRITE command) */
  if ( hsmbus->Instance->ISR & I2C_ISR_RXNE )
  {
    (*hsmbus->pBuffPtr++) = hsmbus->Instance->RXDR;
  }

  /*
    ...and return to listen mode
   */
  HAL_SMBUS_EnableListen_IT( hsmbus );

#ifdef PMBUS13  
  __SMBUS_ZONE_ENABLE(hsmbus);
#endif /* PMBUS13 */
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
  if ( ( pStackContext->Device->Instance->CR1 & SMBUS_PERIPHERAL_MODE_SMBUS_HOST ) == 0)
  {
    pStackContext->Device->Instance->CR1 |= I2C_CR1_ALERTEN;
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

    if ( (pStackContext->StateMachine & SMBUS_SMS_ACTIVE_MASK) == 0 )
    {
      /*
        The stack is not busy - we can react immediately
       */
      pStackContext->CurrentCommand = &ALERT_RESPONSE;
      STACK_SMBUS_HostRead( pStackContext, (uint8_t*)&(pStackContext->OwnAddress) , SMBUS_ADDR_ARA);
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
        HAL_SMBUS_EnableListen_IT( pStackContext->Device );
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
      __HAL_SMBUS_GENERATE_NACK( hsmbus );
      
      /* TCR may still stretch the SCL */
      if ( hsmbus->Instance->ISR & I2C_ISR_TCR )
      {
        hsmbus->Instance->CR2 |= (((uint32_t) 1 << 16 ) & I2C_CR2_NBYTES);
      }
      if ( hsmbus->Instance->ISR & I2C_ISR_ADDR )
      {
        hsmbus->Instance->ICR |= I2C_ICR_ADDRCF;
      }
  
      /*
      Clearing the rest of the transmission, including the HW buffer of peripheral
      */
      hsmbus->Instance->ISR |= I2C_ISR_TXE;
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
  else if ( error & HAL_SMBUS_ERROR_PECERR )
  {
    if ( hsmbus->State == HAL_SMBUS_STATE_RESET )
    {
      HAL_SMBUS_DeInit( pStackContext->Device );
      HAL_SMBUS_Init( pStackContext->Device );
      HAL_SMBUS_EnableListen_IT( pStackContext->Device );
    }
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

    /*
      ...and return to listen mode
    */
    HAL_SMBUS_EnableListen_IT( pStackContext->Device );
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
    return HAL_SMBUS_EnableListen_IT( pStackContext->Device );
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
    /*
    becoming master, not listening any more
    */
    HAL_SMBUS_DisableListen_IT( pStackContext->Device );

    if ( pCommand == NULL )
    {
      /*
      quick command case
      */
      size = 0;
      xFerOptions = SMBUS_FIRST_AND_LAST_FRAME_NO_PEC;
      
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
            xFerOptions = SMBUS_FIRST_AND_LAST_FRAME_NO_PEC | ( pStackContext->StateMachine & SMBUS_SMS_PEC_ACTIVE );
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
  uint32_t              xFerOptions = SMBUS_FIRST_AND_LAST_FRAME_NO_PEC;

  /*
     First check status of the SMBUS - no transaction ongoing
   */
  if (( (pStackContext->StateMachine) & SMBUS_SMS_ACTIVE_MASK ) == 0 )
  {
    /*
    becoming master, not listening any more
    */
    HAL_SMBUS_DisableListen_IT( pStackContext->Device );

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
    becoming master, not listening any more
    */
    HAL_SMBUS_DisableListen_IT( pStackContext->Device );

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
      xFerOptions = SMBUS_FIRST_AND_LAST_FRAME_NO_PEC;
    }
    else
    {
      size = 1;
      if (pStackContext->StateMachine & SMBUS_SMS_PEC_ACTIVE )
      {
        size += PEC_SIZE;
      }
      xFerOptions = SMBUS_FIRST_AND_LAST_FRAME_NO_PEC | ( pStackContext->StateMachine & SMBUS_SMS_PEC_ACTIVE );
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

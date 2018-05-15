/**
  ******************************************************************************
  * @file    stm32f3xx_hal_smbus.c
  * @author  MCD Application Team
  * @brief   SMBUS HAL module driver.
  *          This file provides firmware functions to manage the following
  *          functionalities of the System Management Bus (SMBus) peripheral,
  *          based on I2C principles of operation :
  *           + Initialization and de-initialization functions
  *           + IO operation functions
  *           + Peripheral State and Errors functions
  *
  @verbatim
  ==============================================================================
                        ##### How to use this driver #####
  ==============================================================================
    [..]
    The SMBUS HAL driver can be used as follows:

    (#) Declare a SMBUS_HandleTypeDef handle structure, for example:
        SMBUS_HandleTypeDef  hsmbus;

    (#)Initialize the SMBUS low level resources by implementing the HAL_SMBUS_MspInit() API:
        (##) Enable the SMBUSx interface clock
        (##) SMBUS pins configuration
            (+++) Enable the clock for the SMBUS GPIOs
            (+++) Configure SMBUS pins as alternate function open-drain
        (##) NVIC configuration if you need to use interrupt process
            (+++) Configure the SMBUSx interrupt priority
            (+++) Enable the NVIC SMBUS IRQ Channel

    (#) Configure the Communication Clock Timing, Bus Timeout, Own Address1, Master Addressing mode,
        Dual Addressing mode, Own Address2, Own Address2 Mask, General call, Nostretch mode,
        Peripheral mode and Packet Error Check mode in the hsmbus Init structure.

    (#) Initialize the SMBUS registers by calling the HAL_SMBUS_Init() API:
        (++) These API's configures also the low level Hardware GPIO, CLOCK, CORTEX...etc)
             by calling the customized HAL_SMBUS_MspInit(&hsmbus) API.

    (#) To check if target device is ready for communication, use the function HAL_SMBUS_IsDeviceReady()

    (#) For SMBUS IO operations, only one mode of operations is available within this driver

    *** Interrupt mode IO operation ***
    ===================================
    [..]
      (+) Transmit in master/host SMBUS mode an amount of data in non-blocking mode using HAL_SMBUS_Master_Transmit_IT()
      (++) At transmission end of transfer HAL_SMBUS_MasterTxCpltCallback() is executed and user can
           add his own code by customization of function pointer HAL_SMBUS_MasterTxCpltCallback()
      (+) Receive in master/host SMBUS mode an amount of data in non-blocking mode using HAL_SMBUS_Master_Receive_IT()
      (++) At reception end of transfer HAL_SMBUS_MasterRxCpltCallback() is executed and user can
           add his own code by customization of function pointer HAL_SMBUS_MasterRxCpltCallback()
      (+) Abort a master/host SMBUS process communication with Interrupt using HAL_SMBUS_Master_Abort_IT()
      (++) The associated previous transfer callback is called at the end of abort process
      (++) mean HAL_SMBUS_MasterTxCpltCallback() in case of previous state was master transmit
      (++) mean HAL_SMBUS_MasterRxCpltCallback() in case of previous state was master receive
      (+) Enable/disable the Address listen mode in slave/device or host/slave SMBUS mode
           using HAL_SMBUS_EnableListen_IT() HAL_SMBUS_DisableListen_IT()
      (++) When address slave/device SMBUS match, HAL_SMBUS_AddrCallback() is executed and user can
           add his own code to check the Address Match Code and the transmission direction request by master/host (Write/Read).
      (++) At Listen mode end HAL_SMBUS_ListenCpltCallback() is executed and user can
           add his own code by customization of function pointer HAL_SMBUS_ListenCpltCallback()
      (+) Transmit in slave/device SMBUS mode an amount of data in non-blocking mode using HAL_SMBUS_Slave_Transmit_IT()
      (++) At transmission end of transfer HAL_SMBUS_SlaveTxCpltCallback() is executed and user can
           add his own code by customization of function pointer HAL_SMBUS_SlaveTxCpltCallback()
      (+) Receive in slave/device SMBUS mode an amount of data in non-blocking mode using HAL_SMBUS_Slave_Receive_IT()
      (++) At reception end of transfer HAL_SMBUS_SlaveRxCpltCallback() is executed and user can
           add his own code by customization of function pointer HAL_SMBUS_SlaveRxCpltCallback()
      (+) Enable/Disable the SMBUS alert mode using HAL_SMBUS_EnableAlert_IT() HAL_SMBUS_DisableAlert_IT()
      (++) When SMBUS Alert is generated HAL_SMBUS_ErrorCallback() is executed and user can
           add his own code by customization of function pointer HAL_SMBUS_ErrorCallback()
           to check the Alert Error Code using function HAL_SMBUS_GetError()
      (+) Get HAL state machine or error values using HAL_SMBUS_GetState() or HAL_SMBUS_GetError()
      (+) In case of transfer Error, HAL_SMBUS_ErrorCallback() function is executed and user can
           add his own code by customization of function pointer HAL_SMBUS_ErrorCallback()
           to check the Error Code using function HAL_SMBUS_GetError()

     *** SMBUS HAL driver macros list ***
     ==================================
     [..]
       Below the list of most used macros in SMBUS HAL driver.

      (+) __HAL_SMBUS_ENABLE:      Enable the SMBUS peripheral
      (+) __HAL_SMBUS_DISABLE:     Disable the SMBUS peripheral
      (+) __HAL_SMBUS_GET_FLAG:    Check whether the specified SMBUS flag is set or not
      (+) __HAL_SMBUS_CLEAR_FLAG:  Clear the specified SMBUS pending flag
      (+) __HAL_SMBUS_ENABLE_IT:   Enable the specified SMBUS interrupt
      (+) __HAL_SMBUS_DISABLE_IT:  Disable the specified SMBUS interrupt

     [..]
       (@) You can refer to the SMBUS HAL driver header file for more useful macros


  @endverbatim
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
#include "stm32f4xx_hal.h"

/** @addtogroup STM32F3xx_HAL_Driver
  * @{
  */

/** @defgroup SMBUS SMBUS
  * @brief SMBUS HAL module driver
  * @{
  */

#ifdef HAL_SMBUS_MODULE_ENABLED

/* Private typedef -----------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/** @defgroup SMBUS_Private_Define SMBUS Private Constants
  * @{
  */
#define TIMING_CLEAR_MASK   (0xF0FFFFFFU)      /*!< SMBUS TIMING clear register Mask */
#define HAL_TIMEOUT_ADDR    (10000U)           /*!< 10 s  */
#define HAL_TIMEOUT_BUSY    (25U)              /*!< 25 ms */
#define HAL_TIMEOUT_DIR     (25U)              /*!< 25 ms */
#define HAL_TIMEOUT_RXNE    (25U)              /*!< 25 ms */
#define HAL_TIMEOUT_STOPF   (25U)              /*!< 25 ms */
#define HAL_TIMEOUT_TC      (25U)              /*!< 25 ms */
#define HAL_TIMEOUT_TCR     (25U)              /*!< 25 ms */
#define HAL_TIMEOUT_TXIS    (25U)              /*!< 25 ms */
#define MAX_NBYTE_SIZE      255U
/**
  * @}
  */

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/** @addtogroup SMBUS_Private_Functions SMBUS Private Functions
  * @{
  */

static HAL_StatusTypeDef SMBUS_Enable_IRQ(SMBUS_HandleTypeDef *hsmbus, uint16_t InterruptRequest);
static HAL_StatusTypeDef SMBUS_Disable_IRQ(SMBUS_HandleTypeDef *hsmbus, uint16_t InterruptRequest);

/* Private functions for SMBUS transfer IRQ handler */
static HAL_StatusTypeDef SMBUS_MasterTransmit_TXE(SMBUS_HandleTypeDef *hsmbus);
static HAL_StatusTypeDef SMBUS_MasterTransmit_BTF(SMBUS_HandleTypeDef *hsmbus);
static HAL_StatusTypeDef SMBUS_MasterReceive_RXNE(SMBUS_HandleTypeDef *hsmbus);
static HAL_StatusTypeDef SMBUS_MasterReceive_BTF(SMBUS_HandleTypeDef *hsmbus);
static HAL_StatusTypeDef SMBUS_Master_SB(SMBUS_HandleTypeDef *hsmbus);
static HAL_StatusTypeDef SMBUS_Master_ADDR(SMBUS_HandleTypeDef *hsmbus);

static void SMBUS_ITError(SMBUS_HandleTypeDef *hsmbus);

/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/

/** @defgroup SMBUS_Exported_Functions SMBUS Exported Functions
  * @{
  */

/** @defgroup SMBUS_Exported_Functions_Group1 Initialization and de-initialization functions
 *  @brief    Initialization and Configuration functions
 *
@verbatim
 ===============================================================================
              ##### Initialization and de-initialization functions #####
 ===============================================================================
    [..]  This subsection provides a set of functions allowing to initialize and
          deinitialize the SMBUSx peripheral:

      (+) User must Implement HAL_SMBUS_MspInit() function in which he configures
          all related peripherals resources (CLOCK, GPIO, IT and NVIC ).

      (+) Call the function HAL_SMBUS_Init() to configure the selected device with
          the selected configuration:
        (++) Clock Timing
        (++) Bus Timeout
        (++) Analog Filer mode
        (++) Own Address 1
        (++) Addressing mode (Master, Slave)
        (++) Dual Addressing mode
        (++) Own Address 2
        (++) Own Address 2 Mask
        (++) General call mode
        (++) Nostretch mode
        (++) Packet Error Check mode
        (++) Peripheral mode


      (+) Call the function HAL_SMBUS_DeInit() to restore the default configuration
          of the selected SMBUSx peripheral.

      (+) Enable/Disable Analog/Digital filters with HAL_SMBUS_ConfigAnalogFilter() and
          HAL_SMBUS_ConfigDigitalFilter().

@endverbatim
  * @{
  */

/**
  * @brief  Initialize the SMBUS according to the specified parameters
  *         in the SMBUS_InitTypeDef and initialize the associated handle.
  * @param  hsmbus Pointer to a SMBUS_HandleTypeDef structure that contains
  *                the configuration information for the specified SMBUS.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_SMBUS_Init(SMBUS_HandleTypeDef *hsmbus)
{
	uint32_t freqrange = 0U;
	uint32_t pclk1 = 0U;

  /* Check the SMBUS handle allocation */
  if (hsmbus == NULL)
  {
    return HAL_ERROR;
  }

  /* Check the parameters */
  assert_param(IS_SMBUS_ALL_INSTANCE(hsmbus->Instance));
  assert_param(IS_SMBUS_ANALOG_FILTER(hsmbus->Init.AnalogFilter));
  assert_param(IS_SMBUS_OWN_ADDRESS1(hsmbus->Init.OwnAddress1));
  assert_param(IS_SMBUS_ADDRESSING_MODE(hsmbus->Init.AddressingMode));
  assert_param(IS_SMBUS_DUAL_ADDRESS(hsmbus->Init.DualAddressMode));
  assert_param(IS_SMBUS_OWN_ADDRESS2(hsmbus->Init.OwnAddress2));
  assert_param(IS_SMBUS_OWN_ADDRESS2_MASK(hsmbus->Init.OwnAddress2Masks));
  assert_param(IS_SMBUS_GENERAL_CALL(hsmbus->Init.GeneralCallMode));
  assert_param(IS_SMBUS_NO_STRETCH(hsmbus->Init.NoStretchMode));
  assert_param(IS_SMBUS_PEC(hsmbus->Init.PacketErrorCheckMode));
  assert_param(IS_SMBUS_PERIPHERAL_MODE(hsmbus->Init.PeripheralMode));

  if (hsmbus->State == HAL_SMBUS_STATE_RESET)
  {
    /* Allocate lock resource and initialize it */
    hsmbus->Lock = HAL_UNLOCKED;

    /* Init the low level hardware : GPIO, CLOCK, NVIC */
    HAL_SMBUS_MspInit(hsmbus);
  }

  hsmbus->State = HAL_SMBUS_STATE_BUSY;

  /* Disable the selected SMBUS peripheral */
  __HAL_SMBUS_DISABLE(hsmbus);

  /* Get PCLK1 frequency */
    pclk1 = HAL_RCC_GetPCLK1Freq();

    /* Calculate frequency range */
    freqrange = I2C_FREQRANGE(pclk1);

    /*---------------------------- SMBUS CR2 Configuration ----------------------*/
      /* Configure SMBUS: Frequency range */
      hsmbus->Instance->CR2 = freqrange;

    /*---------------------------- SMBUS TRISE Configuration --------------------*/
    /* Configure : SMBUS Rise Time */
    hsmbus->Instance->TRISE = I2C_RISE_TIME(freqrange, hsmbus->Init.ClockSpeed);

    /*---------------------------- SMBUS CCR Configuration ----------------------*/
    /* Configure : SMBUS Speed */
    hsmbus->Instance->CCR = I2C_SPEED(pclk1, hsmbus->Init.ClockSpeed, hsmbus->Init.DutyCycle);

    /*---------------------------- SMBUS CR1 Configuration ----------------------*/
    /* Configure : SMBUS Generalcall and NoStretch mode */
    hsmbus->Instance->CR1 = (hsmbus->Init.GeneralCallMode | hsmbus->Init.PeripheralMode);

    /*---------------------------- SMBUS OAR1 Configuration ---------------------*/
    /* Configure : SMBUS Own Address1 and addressing mode */
    hsmbus->Instance->OAR1 = (hsmbus->Init.AddressingMode | hsmbus->Init.OwnAddress1);

    /*---------------------------- SMBUS OAR2 Configuration ---------------------*/
    /* Configure : SMBUS Dual mode and Own Address2 */
    hsmbus->Instance->OAR2 = (hsmbus->Init.DualAddressMode | hsmbus->Init.OwnAddress2);

  /* Enable the selected SMBUS peripheral */
  __HAL_SMBUS_ENABLE(hsmbus);

  hsmbus->ErrorCode = HAL_SMBUS_ERROR_NONE;
  hsmbus->PreviousState = HAL_SMBUS_STATE_RESET;
  hsmbus->State = HAL_SMBUS_STATE_READY;

  return HAL_OK;
}

/**
  * @brief  DeInitialize the SMBUS peripheral.
  * @param  hsmbus Pointer to a SMBUS_HandleTypeDef structure that contains
  *                the configuration information for the specified SMBUS.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_SMBUS_DeInit(SMBUS_HandleTypeDef *hsmbus)
{
  /* Check the SMBUS handle allocation */
  if (hsmbus == NULL)
  {
    return HAL_ERROR;
  }

  /* Check the parameters */
  assert_param(IS_SMBUS_ALL_INSTANCE(hsmbus->Instance));

  hsmbus->State = HAL_SMBUS_STATE_BUSY;

  /* Disable the SMBUS Peripheral Clock */
  __HAL_SMBUS_DISABLE(hsmbus);

  /* DeInit the low level hardware: GPIO, CLOCK, NVIC */
  HAL_SMBUS_MspDeInit(hsmbus);

  hsmbus->ErrorCode = HAL_SMBUS_ERROR_NONE;
  hsmbus->PreviousState =  HAL_SMBUS_STATE_NONE;
  hsmbus->State = HAL_SMBUS_STATE_RESET;

  /* Release Lock */
  __HAL_UNLOCK(hsmbus);

  return HAL_OK;
}

/**
  * @brief Initialize the SMBUS MSP.
  * @param  hsmbus Pointer to a SMBUS_HandleTypeDef structure that contains
  *                the configuration information for the specified SMBUS.
  * @retval None
  */
__weak void HAL_SMBUS_MspInit(SMBUS_HandleTypeDef *hsmbus)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hsmbus);

  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_SMBUS_MspInit could be implemented in the user file
   */
}

/**
  * @brief DeInitialize the SMBUS MSP.
  * @param  hsmbus Pointer to a SMBUS_HandleTypeDef structure that contains
  *                the configuration information for the specified SMBUS.
  * @retval None
  */
__weak void HAL_SMBUS_MspDeInit(SMBUS_HandleTypeDef *hsmbus)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hsmbus);

  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_SMBUS_MspDeInit could be implemented in the user file
   */
}

/**
  * @}
  */

/** @defgroup SMBUS_Exported_Functions_Group2 Input and Output operation functions
 *  @brief   Data transfers functions
 *
@verbatim
 ===============================================================================
                      ##### IO operation functions #####
 ===============================================================================
    [..]
    This subsection provides a set of functions allowing to manage the SMBUS data
    transfers.

    (#) Blocking mode function to check if device is ready for usage is :
        (++) HAL_SMBUS_IsDeviceReady()

    (#) There is only one mode of transfer:
       (++) Non-Blocking mode : The communication is performed using Interrupts.
            These functions return the status of the transfer startup.
            The end of the data processing will be indicated through the
            dedicated SMBUS IRQ when using Interrupt mode.

    (#) Non-Blocking mode functions with Interrupt are :
        (++) HAL_SMBUS_Master_Transmit_IT()
        (++) HAL_SMBUS_Master_Receive_IT()
        (++) HAL_SMBUS_Slave_Transmit_IT()
        (++) HAL_SMBUS_Slave_Receive_IT()
        (++) HAL_SMBUS_EnableListen_IT() or alias HAL_SMBUS_EnableListen_IT()
        (++) HAL_SMBUS_DisableListen_IT()
        (++) HAL_SMBUS_EnableAlert_IT()
        (++) HAL_SMBUS_DisableAlert_IT()

    (#) A set of Transfer Complete Callbacks are provided in non-Blocking mode:
        (++) HAL_SMBUS_MasterTxCpltCallback()
        (++) HAL_SMBUS_MasterRxCpltCallback()
        (++) HAL_SMBUS_SlaveTxCpltCallback()
        (++) HAL_SMBUS_SlaveRxCpltCallback()
        (++) HAL_SMBUS_AddrCallback()
        (++) HAL_SMBUS_ListenCpltCallback()
        (++) HAL_SMBUS_ErrorCallback()

@endverbatim
  * @{
  */

/**
  * @brief  Transmit in master/host SMBUS mode an amount of data in non-blocking mode with Interrupt.
  * @param  hsmbus Pointer to a SMBUS_HandleTypeDef structure that contains
  *                the configuration information for the specified SMBUS.
  * @param  DevAddress Target device address: The device 7 bits address value
  *         in datasheet must be shift at right before call interface
  * @param  pData Pointer to data buffer
  * @param  Size Amount of data to be sent
  * @param  XferOptions Options of Transfer, value of @ref SMBUS_XferOptions_definition
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_SMBUS_Master_Transmit_IT(SMBUS_HandleTypeDef *hsmbus, uint16_t DevAddress, uint8_t *pData, uint16_t Size)
{
	volatile uint32_t count = 0;

	if(hsmbus->State == HAL_SMBUS_STATE_READY)
	{
		/* Wait until BUSY flag is reset */
		count = SMBUS_TIMEOUT_BUSY_FLAG * (SystemCoreClock /25U /1000U);
		do
		{
		  if(count-- == 0U)
		  {
			hsmbus->PreviousState = HAL_SMBUS_STATE_NONE;
			hsmbus->State= HAL_SMBUS_STATE_READY;

			/* Process Unlocked */
			__HAL_UNLOCK(hsmbus);

			return HAL_TIMEOUT;
		  }
		}
		while(__HAL_SMBUS_GET_FLAG(hsmbus, SMBUS_FLAG_BUSY) != RESET);

		/* Process Locked */
		__HAL_LOCK(hsmbus);

		/* Check if the I2C is already enabled */
		if((hsmbus->Instance->CR1 & I2C_CR1_PE) != I2C_CR1_PE)
		{
		  /* Enable I2C peripheral */
		  __HAL_I2C_ENABLE(hsmbus);
		}

		/* Disable Pos */
		hsmbus->Instance->CR1 &= ~I2C_CR1_POS;

		hsmbus->State     = HAL_SMBUS_STATE_BUSY_TX;
		//hsmbus->Mode      = HAL_I2C_MODE_MASTER;
		hsmbus->ErrorCode = HAL_I2C_ERROR_NONE;

		/* Prepare transfer parameters */
		hsmbus->pBuffPtr    = pData;
		hsmbus->XferCount   = Size;
		hsmbus->XferOptions = SMBUS_NO_OPTION_FRAME;
		hsmbus->XferSize    = hsmbus->XferCount;
		hsmbus->Devaddress  = DevAddress;

		/* Generate Start */
		SMBUS_GENERATE_START(hsmbus);

		/* Process Unlocked */
		__HAL_UNLOCK(hsmbus);

		/* Note : The SMBUS interrupts must be enabled after unlocking current process
				  to avoid the risk of SMBUS interrupt handle execution before current
				  process unlock */
		/* Enable EVT, BUF and ERR interrupt */

		SMBUS_Enable_IRQ(hsmbus, SMBUS_IT_TX);

		return HAL_OK;
	}
	else
	{
		return HAL_BUSY;
	}
}

/**
  * @brief  Receive in master/host SMBUS mode an amount of data in non-blocking mode with Interrupt.
  * @param  hsmbus Pointer to a SMBUS_HandleTypeDef structure that contains
  *                the configuration information for the specified SMBUS.
  * @param  DevAddress Target device address: The device 7 bits address value
  *         in datasheet must be shift at right before call interface
  * @param  pData Pointer to data buffer
  * @param  Size Amount of data to be sent
  * @param  XferOptions Options of Transfer, value of @ref SMBUS_XferOptions_definition
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_SMBUS_Master_Receive_IT(SMBUS_HandleTypeDef *hsmbus, uint16_t DevAddress, uint8_t *pData, uint16_t Size)
{
	__IO uint32_t count = 0U;

	if(hsmbus->State == HAL_SMBUS_STATE_READY)
	{
	  /* Wait until BUSY flag is reset */
	  count = SMBUS_TIMEOUT_BUSY_FLAG * (SystemCoreClock /25U /1000U);
	  do
	  {
	    if(count-- == 0U)
	    {
	      hsmbus->PreviousState = HAL_SMBUS_STATE_NONE;
	      hsmbus->State= HAL_SMBUS_STATE_READY;

	      /* Process Unlocked */
	      __HAL_UNLOCK(hsmbus);

	      return HAL_TIMEOUT;
	    }
	  }
	  while(__HAL_SMBUS_GET_FLAG(hsmbus, SMBUS_FLAG_BUSY) != RESET);

	  /* Process Locked */
	  __HAL_LOCK(hsmbus);

	  /* Check if the I2C is already enabled */
	  if((hsmbus->Instance->CR1 & I2C_CR1_PE) != I2C_CR1_PE)
	  {
	    /* Enable I2C peripheral */
	    __HAL_I2C_ENABLE(hsmbus);
	  }

	  /* Disable Pos */
	  hsmbus->Instance->CR1 &= ~I2C_CR1_POS;

	  hsmbus->State     = HAL_SMBUS_STATE_BUSY_RX;
	  //hsmbusc->Mode      = HAL_I2C_MODE_MASTER;
	  hsmbus->ErrorCode = HAL_SMBUS_ERROR_NONE;

	  /* Prepare transfer parameters */
	  hsmbus->pBuffPtr    = pData;
	  hsmbus->XferCount   = Size;
	  hsmbus->XferOptions = SMBUS_NO_OPTION_FRAME;
	  hsmbus->XferSize    = hsmbus->XferCount;

	  /* Enable Acknowledge */
	  hsmbus->Instance->CR1 |= I2C_CR1_ACK;

	  /* Generate Start */
	  hsmbus->Instance->CR1 |= I2C_CR1_START;

	  /* Process Unlocked */
	  __HAL_UNLOCK(hsmbus);

	  /* Note : The SMBUS interrupts must be enabled after unlocking current process
              to avoid the risk of SMBUS interrupt handle execution before current
              process unlock */
	  SMBUS_Enable_IRQ(hsmbus, SMBUS_IT_RX);

	  return HAL_OK;
	}
	else
	{
    return HAL_BUSY;
	}
}

/**
  * @brief  Sequential transmit in master mode an amount of data in non-blocking mode with Interrupt
  * @note   This interface allow to manage repeated start condition when a direction change during transfer
  * @param  hsmbus Pointer to a SMBUS_HandleTypeDef structure that contains
  *         the configuration information for the specified SMBUS.
  * @param  DevAddress Target device address The device 7 bits address value
  *         in datasheet must be shifted to the left before calling the interface
  * @param  pData Pointer to data buffer
  * @param  Size Amount of data to be sent
  * @param  XferOptions Options of Transfer, value of @ref SMBUS_XferOptions_definition
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_SMBUS_Master_Sequential_Transmit_IT(SMBUS_HandleTypeDef *hsmbus, uint16_t DevAddress, uint8_t *pData, uint16_t Size, uint32_t XferOptions)
{
  __IO uint32_t Prev_State = 0x00U;
  __IO uint32_t count      = 0x00U;

  /* Check the parameters */
  assert_param(IS_SMBUS_TRANSFER_OPTIONS_REQUEST(XferOptions));

  if(hsmbus->State == HAL_SMBUS_STATE_READY)
  {
    /* Check Busy Flag only if FIRST call of Master interface */
    if((XferOptions == SMBUS_FIRST_AND_LAST_FRAME) || (XferOptions == SMBUS_FIRST_FRAME))
    {
      /* Wait until BUSY flag is reset */
      count = SMBUS_TIMEOUT_BUSY_FLAG * (SystemCoreClock /25U /1000U);
      do
      {
        if(count-- == 0U)
        {
          hsmbus->PreviousState = HAL_SMBUS_STATE_NONE;
          hsmbus->State= HAL_SMBUS_STATE_READY;

          /* Process Unlocked */
          __HAL_UNLOCK(hsmbus);

          return HAL_TIMEOUT;
        }
      }
      while(__HAL_SMBUS_GET_FLAG(hsmbus, SMBUS_FLAG_BUSY) != RESET);
    }

    /* Process Locked */
    __HAL_LOCK(hsmbus);

    /* Check if the SMBUS is already enabled */
    if((hsmbus->Instance->CR1 & I2C_CR1_PE) != I2C_CR1_PE)
    {
      /* Enable I2C peripheral */
      __HAL_SMBUS_ENABLE(hsmbus);
    }

    /* Disable Pos */
    hsmbus->Instance->CR1 &= ~I2C_CR1_POS;

    hsmbus->State     				= HAL_SMBUS_STATE_BUSY_TX;
    hsmbus->Init.PeripheralMode     = SMBUS_PERIPHERAL_MODE_HOST;
    hsmbus->ErrorCode 				= HAL_SMBUS_ERROR_NONE;

    /* Prepare transfer parameters */
    hsmbus->pBuffPtr    = pData;
    hsmbus->XferCount   = Size;
    hsmbus->XferOptions = XferOptions;
    hsmbus->XferSize    = hsmbus->XferCount;
    hsmbus->Devaddress  = DevAddress;

    Prev_State = hsmbus->PreviousState;

    /* Generate Start */
    if((Prev_State == HAL_SMBUS_STATE_BUSY_RX) || (Prev_State == HAL_SMBUS_STATE_NONE))
    {
      /* Generate Start condition if first transfer */
      if((XferOptions == SMBUS_FIRST_AND_LAST_FRAME) || (XferOptions == SMBUS_FIRST_FRAME))
      {
        /* Generate Start */
        SMBUS_GENERATE_START(hsmbus);
      }
      else
      {
        /* Generate ReStart */
        SMBUS_GENERATE_START(hsmbus);
      }
    }

    /* Process Unlocked */
    __HAL_UNLOCK(hsmbus);

    /* Note : The SMBUS interrupts must be enabled after unlocking current process
    to avoid the risk of SMBUS interrupt handle execution before current
    process unlock */

    /* Enable EVT, BUF and ERR interrupt */
    __HAL_SMBUS_ENABLE_IT(hsmbus, SMBUS_IT_TX);

    return HAL_OK;
  }
  else
  {
    return HAL_BUSY;
  }
}

/**
  * @brief  Sequential receive in master mode an amount of data in non-blocking mode with Interrupt
  * @note   This interface allow to manage repeated start condition when a direction change during transfer
  * @param  hsmbus Pointer to a SMBUS_HandleTypeDef structure that contains
  *         the configuration information for the specified SMBUS.
  * @param  DevAddress Target device address The device 7 bits address value
  *         in datasheet must be shifted to the left before calling the interface
  * @param  pData Pointer to data buffer
  * @param  Size Amount of data to be sent
  * @param  XferOptions Options of Transfer, value of @ref SMBUS_XferOptions_definition
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_SMBUS_Master_Sequential_Receive_IT(SMBUS_HandleTypeDef *hsmbus, uint16_t DevAddress, uint8_t *pData, uint16_t Size, uint32_t XferOptions)
{
  __IO uint32_t count = 0U;

  /* Check the parameters */
  assert_param(IS_SMBUS_TRANSFER_OPTIONS_REQUEST(XferOptions));

  if(hsmbus->State == HAL_SMBUS_STATE_READY)
  {
    /* Check Busy Flag only if FIRST call of Master interface */
    if((XferOptions == SMBUS_FIRST_AND_LAST_FRAME) || (XferOptions == SMBUS_FIRST_FRAME))
    {
      /* Wait until BUSY flag is reset */
      count = SMBUS_TIMEOUT_BUSY_FLAG * (SystemCoreClock /25U /1000U);
      do
      {
        if(count-- == 0U)
        {
          hsmbus->PreviousState = HAL_SMBUS_STATE_NONE;
          hsmbus->State= HAL_SMBUS_STATE_READY;

          /* Process Unlocked */
          __HAL_UNLOCK(hsmbus);

          return HAL_TIMEOUT;
        }
      }
      while(__HAL_SMBUS_GET_FLAG(hsmbus, SMBUS_FLAG_BUSY) != RESET);
    }

    /* Process Locked */
    __HAL_LOCK(hsmbus);

    /* Check if the SMBUS is already enabled */
    if((hsmbus->Instance->CR1 & I2C_CR1_PE) != I2C_CR1_PE)
    {
      /* Enable SMBUS peripheral */
      __HAL_SMBUS_ENABLE(hsmbus);
    }

    if((XferOptions == SMBUS_LAST_FRAME) && (Size == 2))
    {
    	/* Enable Pos */
    	hsmbus->Instance->CR1 |= I2C_CR1_POS;
    }
    else
    {
    	/* Disable Pos */
    	hsmbus->Instance->CR1 &= ~I2C_CR1_POS;
    }

    hsmbus->State     				= HAL_SMBUS_STATE_BUSY_RX;
    hsmbus->Init.PeripheralMode		= SMBUS_PERIPHERAL_MODE_HOST;
    hsmbus->ErrorCode 				= HAL_SMBUS_ERROR_NONE;

    /* Prepare transfer parameters */
    hsmbus->pBuffPtr = pData;
    hsmbus->XferCount = Size;
    hsmbus->XferOptions = XferOptions;
    hsmbus->XferSize    = hsmbus->XferCount;
    hsmbus->Devaddress = DevAddress;

    if((hsmbus->PreviousState == HAL_SMBUS_STATE_BUSY_TX) || (hsmbus->PreviousState == HAL_SMBUS_STATE_NONE))
    {
      /* Generate Start condition if first transfer */
      if((XferOptions == SMBUS_FIRST_AND_LAST_FRAME) || (XferOptions == SMBUS_FIRST_FRAME)  || (XferOptions == SMBUS_NO_OPTION_FRAME))
      {
        /* Enable Acknowledge */
        hsmbus->Instance->CR1 |= I2C_CR1_ACK;

        /* Generate Start */
        SMBUS_GENERATE_START(hsmbus);
      }
      else if(hsmbus->PreviousState == HAL_SMBUS_STATE_BUSY_TX)
      {
        /* Enable Acknowledge */
        hsmbus->Instance->CR1 |= I2C_CR1_ACK;

        /* Generate ReStart */
        SMBUS_GENERATE_START(hsmbus);
      }

    /* Process Unlocked */
    __HAL_UNLOCK(hsmbus);

    /* Note : The SMBUS interrupts must be enabled after unlocking current process
    to avoid the risk of SMBUS interrupt handle execution before current
    process unlock */

    /* Enable EVT, BUF and ERR interrupt */
    __HAL_SMBUS_ENABLE_IT(hsmbus, SMBUS_IT_RX);

    }
    else if (hsmbus->PreviousState == HAL_SMBUS_STATE_BUSY_RX)
    {
    	if(hsmbus->XferSize <= 2)
    	{
    	 /* Process Unlocked */
    	 __HAL_UNLOCK(hsmbus);

    	 /* Note : The SMBUS interrupts must be enabled after unlocking current process
    	 to avoid the risk of SMBUS interrupt handle execution before current
    	 process unlock */

    	 /* Enable EVT and ERR interrupt */
    	 __HAL_SMBUS_ENABLE_IT(hsmbus, SMBUS_IT_BTF | SMBUS_IT_ERRI);
    	}
    	else
    	{
    		/* Enable Acknowledge */
    		hsmbus->Instance->CR1 |= I2C_CR1_ACK;

    		/* Process Unlocked */
    		__HAL_UNLOCK(hsmbus);

    		/* Note : The SMBUS interrupts must be enabled after unlocking current process
    		to avoid the risk of SMBUS interrupt handle execution before current
    		process unlock */

    		/* Enable EVT, BUF and ERR interrupt */
    		__HAL_SMBUS_ENABLE_IT(hsmbus, SMBUS_IT_RX);
    	}
    }
    return HAL_OK;
  }
  else
  {
	  return HAL_BUSY;
  }
}

/**
  * @brief  Abort a master/host SMBUS process communication with Interrupt.
  * @note   This abort can be called only if state is ready
  * @param  hsmbus Pointer to a SMBUS_HandleTypeDef structure that contains
  *                the configuration information for the specified SMBUS.
  * @param  DevAddress Target device address: The device 7 bits address value
  *         in datasheet must be shift at right before call interface
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_SMBUS_Master_Abort_IT(SMBUS_HandleTypeDef *hsmbus, uint16_t DevAddress)
{
  if (hsmbus->Init.PeripheralMode == SMBUS_PERIPHERAL_MODE_HOST)
  {
    /* Process Locked */
    __HAL_LOCK(hsmbus);

    /* Keep the same state as previous */
    /* to perform as well the call of the corresponding end of transfer callback */
    if (hsmbus->PreviousState == HAL_SMBUS_STATE_BUSY_TX)
    {
      hsmbus->State = HAL_SMBUS_STATE_BUSY_TX;
    }
    else if (hsmbus->PreviousState == HAL_SMBUS_STATE_BUSY_RX)
    {
      hsmbus->State = HAL_SMBUS_STATE_BUSY_RX;
    }
    else
    {
      /* Wrong usage of abort function */
      /* This function should be used only in case of abort monitored by master device */
      return HAL_ERROR;
    }
    hsmbus->ErrorCode = HAL_SMBUS_ERROR_NONE;

    /*Disable Acknowledge*/
    hsmbus->Instance->CR1 &= ~I2C_CR1_ACK;

    /*Generate Stop*/
    hsmbus->Instance->CR1 |= I2C_CR1_STOP;

    hsmbus->XferCount = 0U;

    SMBUS_Disable_IRQ(hsmbus, SMBUS_IT_TX);

    /* Process Unlocked */
    __HAL_UNLOCK(hsmbus);

    return HAL_OK;
  }
  else
  {
    return HAL_BUSY;
  }
}

/**
  * @brief  Enable the SMBUS alert mode with Interrupt.
  * @param  hsmbus Pointer to a SMBUS_HandleTypeDef structure that contains
  *                the configuration information for the specified SMBUSx peripheral.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_SMBUS_EnableAlert_IT(SMBUS_HandleTypeDef *hsmbus)
{
  /* Enable SMBus alert */
  hsmbus->Instance->CR1 |= I2C_CR1_ALERT;

  /* Clear ALERT flag */
  __HAL_SMBUS_CLEAR_FLAG(hsmbus, SMBUS_FLAG_ALERT);

  /* Enable Alert Interrupt */
  SMBUS_Enable_IRQ(hsmbus, SMBUS_IT_ALERT);

  return HAL_OK;
}
/**
  * @brief  Disable the SMBUS alert mode with Interrupt.
  * @param  hsmbus Pointer to a SMBUS_HandleTypeDef structure that contains
  *                the configuration information for the specified SMBUSx peripheral.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_SMBUS_DisableAlert_IT(SMBUS_HandleTypeDef *hsmbus)
{
  /* Enable SMBus alert */
  hsmbus->Instance->CR1 &= ~I2C_CR1_ALERT;

  /* Disable Alert Interrupt */
  SMBUS_Disable_IRQ(hsmbus, SMBUS_IT_ALERT);

  return HAL_OK;
}

/**
  * @brief  Check if target device is ready for communication.
  * @param  hsmbus Pointer to a SMBUS_HandleTypeDef structure that contains
  *                the configuration information for the specified SMBUS.
  * @param  DevAddress Target device address: The device 7 bits address value
  *         in datasheet must be shift at right before call interface
  * @param  Trials Number of trials
  * @param  Timeout Timeout duration
  * @retval HAL status
  */
/*
HAL_StatusTypeDef HAL_SMBUS_IsDeviceReady(SMBUS_HandleTypeDef *hsmbus, uint16_t DevAddress, uint32_t Trials, uint32_t Timeout)
{
  uint32_t tickstart = 0U, tmp1 = 0U, tmp2 = 0U, tmp3 = 0U, SMBUS_Trials = 1U;

    Get tick
   tickstart = HAL_GetTick();

   if(hsmbus->State == HAL_SMBUS_STATE_READY)
   {
      Wait until BUSY flag is reset
     if(SMBUS_WaitOnFlagUntilTimeout(hsmbus, SMBUS_FLAG_BUSY, SET, HAL_TIMEOUT_BUSY) != HAL_OK)
     {
       return HAL_BUSY;
     }

      Process Locked
     __HAL_LOCK(hsmbus);

      Check if the I2C is already enabled
     if((hsmbus->Instance->CR1 & I2C_CR1_PE) != I2C_CR1_PE)
     {
        Enable I2C peripheral
       __HAL_SMBUS_ENABLE(hsmbus);
     }

      Disable Pos
     hsmbus->Instance->CR1 &= ~I2C_CR1_POS;

     hsmbus->State = HAL_SMBUS_STATE_BUSY;
     hsmbus->ErrorCode = HAL_SMBUS_ERROR_NONE;
     hsmbus->XferOptions = SMBUS_NO_OPTION_FRAME;

    do
    {
       Generate Start
      SMBUS_GENERATE_START(hsmbus);

       Wait until SB flag is set
      if(SMBUS_WaitOnFlagUntilTimeout(hsmbus, SMBUS_FLAG_START, RESET, Timeout) != HAL_OK)
      {
    	  return HAL_TIMEOUT;
      }

       Send slave address
      hsmbus->Instance->DR = SMBUS_ADD_WRITE(DevAddress);

       Wait until ADDR or AF flag are set
       Get tick
      tickstart = HAL_GetTick();

      tmp1 = __HAL_SMBUS_GET_FLAG(hsmbus, SMBUS_FLAG_ADDR);
      tmp2 = __HAL_SMBUS_GET_FLAG(hsmbus, SMBUS_FLAG_AF);
      tmp3 = hsmbus->State;
      while((tmp1 == RESET) && (tmp2 == RESET) && (tmp3 != HAL_I2C_STATE_TIMEOUT))
      {
              if((Timeout == 0U)||((HAL_GetTick() - tickstart ) > Timeout))
              {
                hsmbus->State = HAL_SMBUS_STATE_TIMEOUT;
              }
              tmp1 = __HAL_SMBUS_GET_FLAG(hsmbus, SMBUS_FLAG_ADDR);
              tmp2 = __HAL_SMBUS_GET_FLAG(hsmbus, SMBUS_FLAG_AF);
              tmp3 = hsmbus->State;
            }

            hi2c->State = HAL_I2C_STATE_READY;

             Check if the ADDR flag has been set
            if(__HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_ADDR) == SET)
            {
               Generate Stop
              hi2c->Instance->CR1 |= I2C_CR1_STOP;

               Clear ADDR Flag
              __HAL_I2C_CLEAR_ADDRFLAG(hi2c);

               Wait until BUSY flag is reset
              if(I2C_WaitOnFlagUntilTimeout(hi2c, I2C_FLAG_BUSY, SET, I2C_TIMEOUT_BUSY_FLAG, tickstart) != HAL_OK)
              {
                return HAL_TIMEOUT;
              }

              hi2c->State = HAL_I2C_STATE_READY;

               Process Unlocked
              __HAL_UNLOCK(hi2c);

              return HAL_OK;
            }
            else
            {
               Generate Stop
              hi2c->Instance->CR1 |= I2C_CR1_STOP;

               Clear AF Flag
              __HAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_AF);

               Wait until BUSY flag is reset
              if(I2C_WaitOnFlagUntilTimeout(hi2c, I2C_FLAG_BUSY, SET, I2C_TIMEOUT_BUSY_FLAG, tickstart) != HAL_OK)
              {
                return HAL_TIMEOUT;
              }
            }
          }while(I2C_Trials++ < Trials);

          hi2c->State = HAL_I2C_STATE_READY;

           Process Unlocked
          __HAL_UNLOCK(hi2c);

          return HAL_ERROR;
        }
        else
        {
          return HAL_BUSY;
        }
}
*/
/**
  * @}
  */

/** @defgroup SMBUS_IRQ_Handler_and_Callbacks IRQ Handler and Callbacks
 * @{
 */

/**
  * @brief  Handle SMBUS event interrupt request.
  * @param  hsmbus Pointer to a SMBUS_HandleTypeDef structure that contains
  *                the configuration information for the specified SMBUS.
  * @retval None
  */
void HAL_SMBUS_EV_IRQHandler(SMBUS_HandleTypeDef *hsmbus)
{
	uint32_t sr2itflags   = READ_REG(hsmbus->Instance->SR2);
	uint32_t sr1itflags   = READ_REG(hsmbus->Instance->SR1);
	uint32_t itsources    = READ_REG(hsmbus->Instance->CR2);

	uint32_t CurrentMode  = hsmbus->Init.PeripheralMode;

	/* Master or Memory mode selected */
	if(CurrentMode == SMBUS_PERIPHERAL_MODE_HOST)
	{
		/* SB Set ----------------------------------------------------------------*/
		if(((sr1itflags & SMBUS_FLAG_START) != RESET) && ((itsources & SMBUS_IT_START) != RESET))
		{
			SMBUS_Master_SB(hsmbus);
		}
		/* ADDR Set --------------------------------------------------------------*/
		else if(((sr1itflags & SMBUS_FLAG_ADDR) != RESET) && ((itsources & SMBUS_IT_ADDRI) != RESET))
		{
			SMBUS_Master_ADDR(hsmbus);
		}

		/* SMBUS in mode Transmitter -----------------------------------------------*/
		if((sr2itflags & SMBUS_FLAG_TRA) != RESET)
		{
			/* TXE set and BTF reset -----------------------------------------------*/
			if(((sr1itflags & SMBUS_FLAG_TXE) != RESET) && ((itsources & SMBUS_IT_TXI) != RESET) && ((sr1itflags & SMBUS_FLAG_BTF) == RESET))
			{
				SMBUS_MasterTransmit_TXE(hsmbus);
			}
			/* BTF set -------------------------------------------------------------*/
			else if(((sr1itflags & SMBUS_FLAG_BTF) != RESET) && ((itsources & SMBUS_IT_BTF) != RESET))
			{
				SMBUS_MasterTransmit_BTF(hsmbus);
			}
		}
		/* SMBUS in mode Receiver --------------------------------------------------*/
		else
		{
			/* RXNE set and BTF reset -----------------------------------------------*/
			if(((sr1itflags & SMBUS_FLAG_RXNE) != RESET) && ((itsources & SMBUS_IT_RXI) != RESET) && ((sr1itflags & SMBUS_FLAG_BTF) == RESET))
			{
				SMBUS_MasterReceive_RXNE(hsmbus);
			}
			/* BTF set -------------------------------------------------------------*/
			else if(((sr1itflags & SMBUS_FLAG_BTF) != RESET) && ((itsources & SMBUS_IT_BTF) != RESET))
			{
				SMBUS_MasterReceive_BTF(hsmbus);
			}
		}
	}
	/* Slave mode selected */
	else
	{
		HAL_SMBUS_ErrorCallback(hsmbus);
	}
}

/**
  * @brief  Handle SMBUS error interrupt request.
  * @param  hsmbus Pointer to a SMBUS_HandleTypeDef structure that contains
  *                the configuration information for the specified SMBUS.
  * @retval None
  */
void HAL_SMBUS_ER_IRQHandler(SMBUS_HandleTypeDef *hsmbus)
{
	uint32_t sr1itflags = READ_REG(hsmbus->Instance->SR1);
	uint32_t itsources  = READ_REG(hsmbus->Instance->CR2);

	/* SMBUS Bus error interrupt occurred ----------------------------------------*/
	if(((sr1itflags & SMBUS_FLAG_BERR) != RESET) && ((itsources & SMBUS_IT_ERRI) != RESET))
	{
	  hsmbus->ErrorCode |= HAL_SMBUS_ERROR_BERR;

	  /* Clear BERR flag */
	  __HAL_SMBUS_CLEAR_FLAG(hsmbus, SMBUS_FLAG_BERR);
	}

	/* SMBUS Arbitration Loss error interrupt occurred ---------------------------*/
	if(((sr1itflags & SMBUS_FLAG_ARLO) != RESET) && ((itsources & SMBUS_IT_ERRI) != RESET))
	{
	  hsmbus->ErrorCode |= HAL_SMBUS_ERROR_ARLO;

	  /* Clear ARLO flag */
	  __HAL_SMBUS_CLEAR_FLAG(hsmbus, SMBUS_FLAG_ARLO);
	}

	/* SMBUS Acknowledge failure error interrupt occurred ------------------------*/
	if(((sr1itflags & SMBUS_FLAG_AF) != RESET) && ((itsources & SMBUS_IT_ERRI) != RESET))
	{
	  hsmbus->ErrorCode |= HAL_SMBUS_ERROR_ACKF;

	  /* Generate Stop */
	  SMBUS_GENERATE_STOP(hsmbus);

	  /* Clear AF flag */
	  __HAL_SMBUS_CLEAR_FLAG(hsmbus, SMBUS_FLAG_AF);
	}

	/* I2C Over-Run/Under-Run interrupt occurred -------------------------------*/
	if(((sr1itflags & SMBUS_FLAG_OVR) != RESET) && ((itsources & SMBUS_IT_ERRI) != RESET))
	{
	  hsmbus->ErrorCode |= HAL_SMBUS_ERROR_OVR;
	  /* Clear OVR flag */
	  __HAL_SMBUS_CLEAR_FLAG(hsmbus, SMBUS_FLAG_OVR);
	}

	/* Call the Error Callback in case of Error detected -----------------------*/
	if(hsmbus->ErrorCode != HAL_SMBUS_ERROR_NONE)
	{
	  SMBUS_ITError(hsmbus);
	}
}

/**
  * @brief  Master Tx Transfer completed callback.
  * @param  hsmbus Pointer to a SMBUS_HandleTypeDef structure that contains
  *                the configuration information for the specified SMBUS.
  * @retval None
  */
__weak void HAL_SMBUS_MasterTxCpltCallback(SMBUS_HandleTypeDef *hsmbus)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hsmbus);

  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_SMBUS_MasterTxCpltCallback() could be implemented in the user file
   */
}

/**
  * @brief  Master Rx Transfer completed callback.
  * @param  hsmbus Pointer to a SMBUS_HandleTypeDef structure that contains
  *                the configuration information for the specified SMBUS.
  * @retval None
  */
__weak void HAL_SMBUS_MasterRxCpltCallback(SMBUS_HandleTypeDef *hsmbus)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hsmbus);

  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_SMBUS_MasterRxCpltCallback() could be implemented in the user file
   */
}

/** @brief  Slave Tx Transfer completed callback.
  * @param  hsmbus Pointer to a SMBUS_HandleTypeDef structure that contains
  *                the configuration information for the specified SMBUS.
  * @retval None
  */
__weak void HAL_SMBUS_SlaveTxCpltCallback(SMBUS_HandleTypeDef *hsmbus)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hsmbus);

  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_SMBUS_SlaveTxCpltCallback() could be implemented in the user file
   */
}

/**
  * @brief  Slave Rx Transfer completed callback.
  * @param  hsmbus Pointer to a SMBUS_HandleTypeDef structure that contains
  *                the configuration information for the specified SMBUS.
  * @retval None
  */
__weak void HAL_SMBUS_SlaveRxCpltCallback(SMBUS_HandleTypeDef *hsmbus)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hsmbus);

  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_SMBUS_SlaveRxCpltCallback() could be implemented in the user file
   */
}

/**
  * @brief  Slave Address Match callback.
  * @param  hsmbus Pointer to a SMBUS_HandleTypeDef structure that contains
  *                the configuration information for the specified SMBUS.
  * @param  TransferDirection Master request Transfer Direction (Write/Read)
  * @param  AddrMatchCode Address Match Code
  * @retval None
  */
__weak void HAL_SMBUS_AddrCallback(SMBUS_HandleTypeDef *hsmbus, uint8_t TransferDirection, uint16_t AddrMatchCode)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hsmbus);
  UNUSED(TransferDirection);
  UNUSED(AddrMatchCode);

  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_SMBUS_AddrCallback() could be implemented in the user file
   */
}

/**
  * @brief  Listen Complete callback.
  * @param  hsmbus Pointer to a SMBUS_HandleTypeDef structure that contains
  *                the configuration information for the specified SMBUS.
  * @retval None
  */
__weak void HAL_SMBUS_ListenCpltCallback(SMBUS_HandleTypeDef *hsmbus)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hsmbus);

  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_SMBUS_ListenCpltCallback() could be implemented in the user file
   */
}

/**
  * @brief  SMBUS error callback.
  * @param  hsmbus Pointer to a SMBUS_HandleTypeDef structure that contains
  *                the configuration information for the specified SMBUS.
  * @retval None
  */
__weak void HAL_SMBUS_ErrorCallback(SMBUS_HandleTypeDef *hsmbus)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hsmbus);

  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_SMBUS_ErrorCallback() could be implemented in the user file
   */
}

/**
  * @brief  SMBUS abort callback.
  * @param  hsmbus Pointer to a SMBUS_HandleTypeDef structure that contains
  *                the configuration information for the specified SMBUS.
  * @retval None
  */
__weak void HAL_SMBUS_AbortCpltCallback(SMBUS_HandleTypeDef *hsmbus)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hsmbus);

  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_SMBUS_AbortCpltCallback could be implemented in the user file
   */
}

/**
  * @}
  */

/** @defgroup SMBUS_Exported_Functions_Group3 Peripheral State and Errors functions
 *  @brief   Peripheral State and Errors functions
 *
@verbatim
 ===============================================================================
            ##### Peripheral State and Errors functions #####
 ===============================================================================
    [..]
    This subsection permits to get in run-time the status of the peripheral
    and the data flow.

@endverbatim
  * @{
  */

/**
  * @brief  Return the SMBUS handle state.
  * @param  hsmbus Pointer to a SMBUS_HandleTypeDef structure that contains
  *                the configuration information for the specified SMBUS.
  * @retval HAL state
  */
uint32_t HAL_SMBUS_GetState(SMBUS_HandleTypeDef *hsmbus)
{
  /* Return SMBUS handle state */
  return hsmbus->State;
}

/**
* @brief  Return the SMBUS error code.
  * @param  hsmbus Pointer to a SMBUS_HandleTypeDef structure that contains
  *              the configuration information for the specified SMBUS.
* @retval SMBUS Error Code
*/
uint32_t HAL_SMBUS_GetError(SMBUS_HandleTypeDef *hsmbus)
{
  return hsmbus->ErrorCode;
}

/** @addtogroup SMBUS_Private_Functions SMBUS Private Functions
 *  @brief   Data transfers Private functions
  * @{
  */

/**
 * @brief  Handle SB flag for Master
 * @param  hsmbus pointer to a SMBUS_HandleTypeDef structure that contains
 * 					the configuration information for the specified SMBUS.
 * @retval HAL status
 */
static HAL_StatusTypeDef SMBUS_Master_SB(SMBUS_HandleTypeDef *hsmbus)
{
    if(hsmbus->Init.AddressingMode == SMBUS_ADDRESSINGMODE_7BIT)
    {
      /* Send slave 7 Bits address */
      if(hsmbus->State == HAL_SMBUS_STATE_BUSY_TX)
      {
    	  hsmbus->Instance->DR = SMBUS_ADD_WRITE(hsmbus->Devaddress);
      }
      else
      {
    	  hsmbus->Instance->DR = SMBUS_ADD_READ(hsmbus->Devaddress);
      }
    }
    else
    {
    	return HAL_ERROR;
    }

   return HAL_OK;
}

/**
 * @brief  Handle ADDR flag for Master
 * @param  hsmbus pointer to a SMBUS_HandleTypeDef structure that contains
 * 					the configuration information for the specified SMBUS.
 * @retval HAL status
 */
static HAL_StatusTypeDef SMBUS_Master_ADDR(SMBUS_HandleTypeDef *hsmbus)
{
	/* Declaration of temporary variable to prevent undefined behavior of volatile usage */
	uint32_t CurrentXferOptions = hsmbus->XferOptions;
	uint32_t Prev_State         = hsmbus->PreviousState;

	if(hsmbus->State == HAL_SMBUS_STATE_BUSY_RX)
	{
		if(hsmbus->XferCount == 0U)
		{
			/* Clear ADDR flag */
			__HAL_SMBUS_CLEAR_ADDRFLAG(hsmbus);

			/* Generate Stop */
			hsmbus->Instance->CR1 |= I2C_CR1_STOP;
		}
		else if(hsmbus->XferCount == 1U)
		{
			/* Prepare next transfer or stop current transfer */
			if((CurrentXferOptions != SMBUS_FIRST_AND_LAST_FRAME) && (CurrentXferOptions != SMBUS_LAST_FRAME) \
					&& (Prev_State != HAL_SMBUS_STATE_BUSY_RX))
			{
				if(hsmbus->XferOptions != SMBUS_NEXT_FRAME)
				{
					/* Disable Acknowledge */
					hsmbus->Instance->CR1 &= ~I2C_CR1_ACK;
				}
				else
				{
					/* Enable Acknowledge */
					hsmbus->Instance->CR1 |= I2C_CR1_ACK;
				}

				/* Clear ADDR flag */
				__HAL_SMBUS_CLEAR_ADDRFLAG(hsmbus);
			}
			else
			{
				/* Disable Acknowledge */
				hsmbus->Instance->CR1 &= ~I2C_CR1_ACK;

				/* Clear ADDR flag */
				__HAL_SMBUS_CLEAR_ADDRFLAG(hsmbus);

				/* Generate Stop */
				hsmbus->Instance->CR1 |= I2C_CR1_STOP;
			}
		}
		else if(hsmbus->XferCount == 2U)
		{
			if(hsmbus->XferOptions != SMBUS_NEXT_FRAME)
			{
				/* Disable Acknowledge */
				hsmbus->Instance->CR1 &= ~I2C_CR1_ACK;

				/* Enable Pos */
				hsmbus->Instance->CR1 |= I2C_CR1_POS;
			}
			else
			{
				/* Enable Acknowledge */
				hsmbus->Instance->CR1 |= I2C_CR1_ACK;
			}

			/* Clear ADDR flag */
			__HAL_SMBUS_CLEAR_ADDRFLAG(hsmbus);
		}
		else
		{
			/* Enable Acknowledge */
			hsmbus->Instance->CR1 |= I2C_CR1_ACK;

			/* Clear ADDR flag */
			__HAL_SMBUS_CLEAR_ADDRFLAG(hsmbus);
		}

		/* Reset Event counter  */
		hsmbus->EventCounter = 0U;
	}
	else
	{
		/* Clear ADDR flag */
		__HAL_SMBUS_CLEAR_ADDRFLAG(hsmbus);
	}

	return HAL_OK;
}

/**
  * @brief  Handle TXE flag for Master
  * @param  hsmbus Pointer to a SMBUS_HandleTypeDef structure that contains
  *         the configuration information for SMBUS module
  * @retval HAL status
  */
static HAL_StatusTypeDef SMBUS_MasterTransmit_TXE(SMBUS_HandleTypeDef *hsmbus)
{
  /* Declaration of temporary variables to prevent undefined behavior of volatile usage */
  uint32_t CurrentState       = hsmbus->State;
  uint32_t CurrentXferOptions = hsmbus->XferOptions;

  if((hsmbus->XferSize == 0U) && (CurrentState == HAL_SMBUS_STATE_BUSY_TX))
  {
    /* Call TxCpltCallback() directly if no stop mode is set */
    if((CurrentXferOptions != SMBUS_FIRST_AND_LAST_FRAME) && (CurrentXferOptions != SMBUS_LAST_FRAME) && (CurrentXferOptions != SMBUS_NO_OPTION_FRAME))
    {
    	__HAL_SMBUS_DISABLE_IT(hsmbus, SMBUS_IT_TX);

    	hsmbus->PreviousState = HAL_SMBUS_STATE_BUSY_TX;
    	hsmbus->Init.PeripheralMode = SMBUS_PERIPHERAL_MODE_NONE;
    	hsmbus->State = HAL_SMBUS_STATE_READY;

    	HAL_SMBUS_MasterTxCpltCallback(hsmbus);
    }
    else /* Generate Stop condition then Call TxCpltCallback() */
    {
    	/* Disable EVT, BUF and ERR interrupt */
    	__HAL_SMBUS_DISABLE_IT(hsmbus, SMBUS_IT_TX);

    	/* Generate Stop */
    	SMBUS_GENERATE_STOP(hsmbus);

    	hsmbus->PreviousState = HAL_SMBUS_STATE_BUSY_TX;
    	hsmbus->State = HAL_SMBUS_STATE_READY;
    }
  }
  else if(CurrentState == HAL_SMBUS_STATE_BUSY_TX)
  {
	  if(hsmbus->XferCount == 0)
	  {
		  __HAL_SMBUS_DISABLE_IT(hsmbus, I2C_IT_BUF);
	  }
	  else
	  {
		  /* Write data to DR */
		  hsmbus->Instance->DR = (*hsmbus->pBuffPtr++);
		  hsmbus->XferCount--;
	  }
  }
  return HAL_OK;
}

/**
  * @brief  Handle BTF flag for Master transmitter
  * @param  hsmbus Pointer to a SMBUS_HandleTypeDef structure that contains
  *         the configuration information for SMBUS module
  * @retval HAL status
  */

static HAL_StatusTypeDef SMBUS_MasterTransmit_BTF(SMBUS_HandleTypeDef *hsmbus)
{
	/* Declaration of temporary variables to prevent undefined behavior of volatile usage */
	uint32_t CurrentXferOptions = hsmbus->XferOptions;

	if(hsmbus->State == HAL_SMBUS_STATE_BUSY_TX)
	{
	  if(hsmbus->XferCount != 0U)
	  {
	    /* Write data to DR */
	    hsmbus->Instance->DR = (*hsmbus->pBuffPtr++);
	    hsmbus->XferCount--;
	  }
	  else
	  {
	    /* Call TxCpltCallback() directly if no stop mode is set */
	    if((CurrentXferOptions != SMBUS_FIRST_AND_LAST_FRAME) && (CurrentXferOptions != SMBUS_LAST_FRAME) && (CurrentXferOptions != SMBUS_NO_OPTION_FRAME))
	    {
	      __HAL_SMBUS_DISABLE_IT(hsmbus, SMBUS_IT_TX);

	      hsmbus->PreviousState = HAL_SMBUS_STATE_BUSY_TX;
	      hsmbus->Init.PeripheralMode = SMBUS_PERIPHERAL_MODE_NONE;
	      hsmbus->State = HAL_SMBUS_STATE_READY;

	      HAL_SMBUS_MasterTxCpltCallback(hsmbus);
	    }
	    else /* Generate Stop condition then Call TxCpltCallback() */
	    {
	      /* Disable EVT, BUF and ERR interrupt */
	      __HAL_SMBUS_DISABLE_IT(hsmbus, SMBUS_IT_TX);

	      /* Generate Stop */
	      SMBUS_GENERATE_STOP(hsmbus);

	      hsmbus->PreviousState = HAL_SMBUS_STATE_NONE;
	      hsmbus->State = HAL_SMBUS_STATE_READY;

	      hsmbus->Init.PeripheralMode = SMBUS_PERIPHERAL_MODE_NONE;

	      HAL_SMBUS_MasterTxCpltCallback(hsmbus);
	    }
	  }
	}
	return HAL_OK;
}

/**
  * @brief  Handle RXNE flag for Master
  * @param  hsmbus Pointer to a SMBUS_HandleTypeDef structure that contains
  *         the configuration information for SMBUS module
  * @retval HAL status
  */
static HAL_StatusTypeDef SMBUS_MasterReceive_RXNE(SMBUS_HandleTypeDef *hsmbus)
{
	uint32_t CurrentXferOptions = hsmbus->XferOptions;

	if(hsmbus->State == HAL_SMBUS_STATE_BUSY_RX)
	{
		uint32_t tmp = 0U;

		tmp = hsmbus->XferCount;
		if(tmp > 3U)
		{
		  /* Read data from DR */
		  (*hsmbus->pBuffPtr++) = hsmbus->Instance->DR;
		  hsmbus->XferCount--;

		  if(hsmbus->XferCount == 3)
		  {
			/* Disable BUF interrupt, this help to treat correctly the last 4 bytes
			on BTF subroutine */
			/* Disable BUF interrupt */
			__HAL_SMBUS_DISABLE_IT(hsmbus, I2C_IT_BUF);
		  }
		}
		else if((tmp == 1U) || (tmp == 0U))
		{
	    	if(CurrentXferOptions == SMBUS_NEXT_FRAME)
	    	{
	    		/*Disable Acknowledge*/
	    		hsmbus->Instance->CR1 &= ~I2C_CR1_ACK;

	    		__HAL_SMBUS_DISABLE_IT(hsmbus, SMBUS_IT_RX);
	    		/*Read data from DR*/
	    		(*hsmbus->pBuffPtr++) = hsmbus->Instance->DR;
	    		hsmbus->XferCount--;

	    		hsmbus->PreviousState = HAL_SMBUS_STATE_BUSY_RX;
	    		hsmbus->State = HAL_SMBUS_STATE_READY;

	    		HAL_SMBUS_MasterRxCpltCallback(hsmbus);
	    	}
	    	else
	    	{
	    		/*Disable Acknowledge*/
	    		hsmbus->Instance->CR1 &= ~I2C_CR1_ACK;

	    		/*Disable EVT, BUF and ERR interrupt*/
	    		__HAL_SMBUS_DISABLE_IT(hsmbus, SMBUS_IT_RX);

	    		/*Read data from DR*/
	    		(*hsmbus->pBuffPtr++) = hsmbus->Instance->DR;
	    		hsmbus->XferCount--;

	    		hsmbus->State = HAL_SMBUS_STATE_READY;
	    		hsmbus->PreviousState = HAL_SMBUS_STATE_NONE;

	    		hsmbus->Init.PeripheralMode = SMBUS_PERIPHERAL_MODE_NONE;
	    		HAL_SMBUS_MasterRxCpltCallback(hsmbus);
	    	}
		}
	}
	return HAL_OK;
}

/**
  * @brief  Handle BTF flag for Master receiver
  * @param  hsmbus Pointer to a SMBUS_HandleTypeDef structure that contains
  *         the configuration information for SMBUS module
  * @retval HAL status
  */
static HAL_StatusTypeDef SMBUS_MasterReceive_BTF(SMBUS_HandleTypeDef *hsmbus)
{
  /* Declaration of temporary variables to prevent undefined behavior of volatile usage */
  uint32_t CurrentXferOptions = hsmbus->XferOptions;

  if(hsmbus->XferCount == 4U)
  {
    /* Disable BUF interrupt, this help to treat correctly the last 2 bytes
       on BTF subroutine if there is a reception delay between N-1 and N byte */
    __HAL_SMBUS_DISABLE_IT(hsmbus, I2C_IT_BUF);

    /* Read data from DR */
    (*hsmbus->pBuffPtr++) = hsmbus->Instance->DR;
    hsmbus->XferCount--;
  }
  else if(hsmbus->XferCount == 3U)
  {
    /* Disable BUF interrupt, this help to treat correctly the last 2 bytes
       on BTF subroutine if there is a reception delay between N-1 and N byte */
    __HAL_SMBUS_DISABLE_IT(hsmbus, I2C_IT_BUF);

    /* Disable Acknowledge */
    hsmbus->Instance->CR1 &= ~I2C_CR1_ACK;

    /* Read data from DR */
    (*hsmbus->pBuffPtr++) = hsmbus->Instance->DR;
    hsmbus->XferCount--;
  }
  else if(hsmbus->XferCount == 2U)
  {
    /* Prepare next transfer or stop current transfer */
    if((CurrentXferOptions == SMBUS_NEXT_FRAME) || (CurrentXferOptions == SMBUS_FIRST_FRAME))
    {
      /* Disable Acknowledge */
      hsmbus->Instance->CR1 &= ~I2C_CR1_ACK;

      /* Generate ReStart */
      SMBUS_GENERATE_START(hsmbus);
    }
    else
    {
      /* Generate Stop */
      SMBUS_GENERATE_STOP(hsmbus);
    }

    /* Read data from DR */
    (*hsmbus->pBuffPtr++) = hsmbus->Instance->DR;
    hsmbus->XferCount--;

    /* Read data from DR */
    (*hsmbus->pBuffPtr++) = hsmbus->Instance->DR;
    hsmbus->XferCount--;

    /* Disable EVT and ERR interrupt */
    __HAL_SMBUS_DISABLE_IT(hsmbus, SMBUS_IT_BTF | SMBUS_IT_ERRI);

    hsmbus->State = HAL_SMBUS_STATE_READY;
    hsmbus->PreviousState = HAL_SMBUS_STATE_NONE;

    hsmbus->Init.PeripheralMode = SMBUS_PERIPHERAL_MODE_NONE;

    HAL_SMBUS_MasterRxCpltCallback(hsmbus);
  }
  else
  {
    /* Read data from DR */
    (*hsmbus->pBuffPtr++) = hsmbus->Instance->DR;
    hsmbus->XferCount--;
  }
  return HAL_OK;
}


/**
  * @brief  Manage the enabling of Interrupts.
  * @param  hsmbus Pointer to a SMBUS_HandleTypeDef structure that contains
  *                the configuration information for the specified SMBUS.
  * @param  InterruptRequest Value of @ref SMBUS_Interrupt_configuration_definition.
  * @retval HAL status
  */
static HAL_StatusTypeDef SMBUS_Enable_IRQ(SMBUS_HandleTypeDef *hsmbus, uint16_t InterruptRequest)
{
  uint32_t tmpisr = 0U;

  if ((InterruptRequest & SMBUS_IT_ALERT) == SMBUS_IT_ALERT)
  {
    /* Enable ERR interrupt */
    tmpisr |= SMBUS_IT_ERRI;
  }

  if ((InterruptRequest & SMBUS_IT_ADDR) == SMBUS_IT_ADDR)
  {
    /* Enable ADDR, STOP interrupt */
    tmpisr |= SMBUS_IT_ADDRI | SMBUS_IT_STOPI | SMBUS_IT_NACKI | SMBUS_IT_ERRI;
  }

  if ((InterruptRequest & SMBUS_IT_TX) == SMBUS_IT_TX)
  {
    /* Enable ERR, TC, STOP, NACK, RXI interrupt */
    tmpisr |= SMBUS_IT_ERRI | SMBUS_IT_STOPI | SMBUS_IT_NACKI | SMBUS_IT_TXI;
  }

  if ((InterruptRequest & SMBUS_IT_RX) == SMBUS_IT_RX)
  {
    /* Enable ERR, TC, STOP, NACK, TXI interrupt */
    tmpisr |= SMBUS_IT_ERRI | SMBUS_IT_STOPI | SMBUS_IT_NACKI | SMBUS_IT_RXI;
  }

  /* Enable interrupts only at the end */
  /* to avoid the risk of SMBUS interrupt handle execution before */
  /* all interrupts requested done */
  __HAL_SMBUS_ENABLE_IT(hsmbus, tmpisr);

  return HAL_OK;
}
/**
  * @brief  Manage the disabling of Interrupts.
  * @param  hsmbus Pointer to a SMBUS_HandleTypeDef structure that contains
  *                the configuration information for the specified SMBUS.
  * @param  InterruptRequest Value of @ref SMBUS_Interrupt_configuration_definition.
  * @retval HAL status
  */
static HAL_StatusTypeDef SMBUS_Disable_IRQ(SMBUS_HandleTypeDef *hsmbus, uint16_t InterruptRequest)
{
  uint32_t tmpisr = 0U;

  if (((InterruptRequest & SMBUS_IT_ALERT) == SMBUS_IT_ALERT) && (hsmbus->State == HAL_SMBUS_STATE_READY))
  {
    /* Disable ERR interrupt */
    tmpisr |= SMBUS_IT_ERRI;
  }

  if ((InterruptRequest & SMBUS_IT_TX) == SMBUS_IT_TX)
  {
    /* Disable TC, STOP, NACK, TXI interrupt */
    tmpisr |= SMBUS_IT_TXI;

    if ((SMBUS_GET_ALERT_ENABLED(hsmbus) == RESET)
        && ((hsmbus->State & HAL_SMBUS_STATE_LISTEN) != HAL_SMBUS_STATE_LISTEN))
    {
      /* Disable ERR interrupt */
      tmpisr |= SMBUS_IT_ERRI;
    }

    if ((hsmbus->State & HAL_SMBUS_STATE_LISTEN) != HAL_SMBUS_STATE_LISTEN)
    {
      /* Disable STOPI, NACKI */
      tmpisr |= SMBUS_IT_STOPI | SMBUS_IT_NACKI;
    }
  }

  if ((InterruptRequest & SMBUS_IT_RX) == SMBUS_IT_RX)
  {
    /* Disable TC, STOP, NACK, RXI interrupt */
    tmpisr |=  SMBUS_IT_RXI;

    if ((SMBUS_GET_ALERT_ENABLED(hsmbus) == RESET)
        && ((hsmbus->State & HAL_SMBUS_STATE_LISTEN) != HAL_SMBUS_STATE_LISTEN))
    {
      /* Disable ERR interrupt */
      tmpisr |= SMBUS_IT_ERRI;
    }

    if ((hsmbus->State & HAL_SMBUS_STATE_LISTEN) != HAL_SMBUS_STATE_LISTEN)
    {
      /* Disable STOPI, NACKI */
      tmpisr |= SMBUS_IT_STOPI | SMBUS_IT_NACKI;
    }
  }

  if ((InterruptRequest & SMBUS_IT_ADDR) == SMBUS_IT_ADDR)
  {
    /* Enable ADDR, STOP interrupt */
    tmpisr |= SMBUS_IT_ADDRI | SMBUS_IT_STOPI | SMBUS_IT_NACKI;

    if (SMBUS_GET_ALERT_ENABLED(hsmbus) == RESET)
    {
      /* Disable ERR interrupt */
      tmpisr |= SMBUS_IT_ERRI;
    }
  }

  /* Disable interrupts only at the end */
  /* to avoid a breaking situation like at "t" time */
  /* all disable interrupts request are not done */
  __HAL_SMBUS_DISABLE_IT(hsmbus, tmpisr);

  return HAL_OK;
}

/**
  * @brief  SMBUS interrupts error handler.
  * @param  hsmbus SMBUS handle.
  * @retval None
  */
static void SMBUS_ITError(SMBUS_HandleTypeDef *hsmbus)
{
	  /* Disable Pos bit in I2C CR1 when error occurred in Master/Mem Receive IT Process */
	  hsmbus->Instance->CR1 &= ~I2C_CR1_POS;

	  if(hsmbus->State == HAL_SMBUS_STATE_ABORT)
	  {
	    hsmbus->State = HAL_SMBUS_STATE_READY;
	    hsmbus->ErrorCode = HAL_SMBUS_ERROR_NONE;

	    /* Store Last receive data if any */
	    if(__HAL_SMBUS_GET_FLAG(hsmbus, SMBUS_FLAG_RXNE) == SET)
	    {
	      /* Read data from DR */
	      (*hsmbus->pBuffPtr++) = hsmbus->Instance->DR;
	    }

	    /* Disable SMBUS peripheral to prevent dummy data in buffer */
	    __HAL_SMBUS_DISABLE(hsmbus);

	    /* Call the corresponding callback to inform upper layer of End of Transfer */
	    HAL_SMBUS_AbortCpltCallback(hsmbus);
	  }
	  else
	  {
	    /* Store Last receive data if any */
	    if(__HAL_SMBUS_GET_FLAG(hsmbus, SMBUS_FLAG_RXNE) == SET)
	    {
	      /* Read data from DR */
	      (*hsmbus->pBuffPtr++) = hsmbus->Instance->DR;
	    }

	    /* Call user error callback */
	    HAL_SMBUS_ErrorCallback(hsmbus);
	  }
}

/**
  * @
  */


#endif /* HAL_SMBUS_MODULE_ENABLED */
/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

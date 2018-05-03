/**
  ******************************************************************************
  * @file    stm32_config_stack.h
  * @author  MCD Application Team
  * @version V2.0.0
  * @date    8-August-2016
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


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CONFIG_STACK_H
#define __CONFIG_STACK_H

#ifdef __cplusplus
extern "C"
{
#endif

  /* Includes ------------------------------------------------------------------*/

  /** @addtogroup STM32_SMBUS_STACK     SMBus 2.0 stack implementation
    * @{
    */

  /** @defgroup STM32_SMBUS_STACK_Defines SMBus stack configuration
    * @{
    */

#define DEV_DIS
	/*<! Device is discoverable – reacts to general ARP calls on default address. */
/* #define ARP */
	/*<! Define to enable the automatic address resolution protocol. */
/* #define DEV_PSA */
	/*<! Configure a persistent address. */
/* #define ALERT */
	/*<! Enable Alert HW signal */
#define DENSE_CMD_TABLE
	/*<! Setting indicates that the command code does not equal the table index. */
#define PMBUS12
	/*<! Features introduced in PMBus v1.2 are included. */
#define PMBUS13
	/*<! Features introduced in PMBus v1.3 are included. */
/* #define HOST1 */
	/*<! The target is a bus host. */

  /**
    * @}
    */

  /**
    * @}
    */

#ifdef __cplusplus
}
#endif

#endif /* __CONFIG_STACK_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

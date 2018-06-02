/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
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
#ifndef __MAIN_H__
#define __MAIN_H__

/* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */
#include "stm32f4xx_hal.h"

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the 
  *        HAL drivers code
  */
/* #define USE_FULL_ASSERT    1U */

/* USER CODE BEGIN Private defines */
#define SMBUS_DEVICES_NUMBER				9U

#define LTM_1						0x40
#define LTM_2						0x42
#define LTM_3						0x44
#define LTM_4						0x46
#define LTM_5						0x48
#define LTM_6						0x4A
#define LTM_7						0x4C
#define LTM_8						0x4E
#define LTM_FPGA					0x4F

#define PAGE_0						0x00
#define PAGE_1						0x01
#define TURN_ON						0x80
#define TURN_OFF					0x40

typedef enum {
	LTM_OK					= 0x0000,		// No fault occurred
	NONE_OF_THE_ABOVE			= 0x0001,  		// A fault not listed in bits[7:1] has occurred
	CML					= 0x0002,	 	// A communications, memory or logic fault has occurred
	TEMPERATURE				= 0x0004,  		// A temperature fault or warning has occurred
	IOUT_OC					= 0x0010,  		// An output overcurrent fault has occurred
	VOUT_OV					= 0x0020,  		// An output overvoltage fault has occurred
	OFF					= 0x0040,  		// A channel is off
	BUSY					= 0x0080,  		// A fault was declared because the LTM4675 was unable to respond
	POWER_GOOD				= 0x0800, 		// The POWER_GOOD state is false
	MFR_SPECIFIC				= 0x1000,		// A fault or warning specific to the LTM4675 has occurred
	INPUT					= 0x2000,		// An SVin input voltage fault or warning has occurred
	IOUT					= 0x4000,		// An output current fault or warning has occurred
	VOUT					= 0x8000		// An output voltage fault or warning has occurred
} LTM_StatusTypeDef;

/* USER CODE END Private defines */

#ifdef __cplusplus
 extern "C" {
#endif
void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)
#ifdef __cplusplus
}
#endif



#endif /* __MAIN_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

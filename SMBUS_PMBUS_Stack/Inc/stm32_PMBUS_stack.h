/**
  ******************************************************************************
  * @file    stm32_PMBUS_stack.h
  * @author  MCD Application Team
  * @version V2.0.0
  * @date    8-August-2016
  * @brief   This file provides a set of functions needed to manage the PMBus
  *          on top of the SMBus stack.
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
#ifndef __STACK_PMBUS_H
#define __STACK_PMBUS_H

#ifdef __cplusplus
extern "C"
{
#endif

  /* Includes ------------------------------------------------------------------*/
#include "stm32_SMBUS_stack.h"


  /** @addtogroup STM32_PMBUS_STACK     PMBus 1.3.1 stack implementation
    * @{
    */

#ifdef PMBUS13

  /* Cannot have 1.3 while excluding 1.2 */
#ifndef PMBUS12
#define PMBUS12
#endif /* PMBUS12 */
#define DENSE_CMD_TBL  
#endif /* PMBUS13 */
      
  /** @defgroup STM32_PMBUS_STACK_Defines PMBus stack definitions
    * @{
    */

  /* ----------- PMBUS command codes definition -------- */
#define PMBC_PAGE                       ((uint8_t)0x00)
#define PMBC_OPERATION                  ((uint8_t)0x01)
#define PMBC_ON_OFF_CONFIG              ((uint8_t)0x02)
#define PMBC_CLEAR_FAULTS               ((uint8_t)0x03)
#define PMBC_PHASE                      ((uint8_t)0x04)
#define PMBC_PAGE_PLUS_WRITE            ((uint8_t)0x05)
#define PMBC_PAGE_PLUS_READ             ((uint8_t)0x06)
#define PMBC_ZONE_CONFIG                ((uint8_t)0x07)
#define PMBC_ZONE_ACTIVE                ((uint8_t)0x08)
#define PMBC_WRITE_PROTECT              ((uint8_t)0x10)
#define PMBC_STORE_DEFAULT_ALL          ((uint8_t)0x11)
#define PMBC_RESTORE_DEFAULT_ALL        ((uint8_t)0x12)
#define PMBC_STORE_DEFAULT_CODE         ((uint8_t)0x13)
#define PMBC_RESTORE_DEFAULT_CODE       ((uint8_t)0x14)
#define PMBC_STORE_USER_ALL             ((uint8_t)0x15)
#define PMBC_RESTORE_USER_ALL           ((uint8_t)0x16)
#define PMBC_STORE_USER_CODE            ((uint8_t)0x17)
#define PMBC_RESTORE_USER_CODE          ((uint8_t)0x18)
#define PMBC_CAPABILITY                 ((uint8_t)0x19)
#define PMBC_QUERY                      ((uint8_t)0x1A)
#define PMBC_SMBALERT_MASK              ((uint8_t)0x1B)
#define PMBC_VOUT_MODE                  ((uint8_t)0x20)
#define PMBC_VOUNT_COMMAND              ((uint8_t)0x21)
#define PMBC_VOUT_TRIM                  ((uint8_t)0x22)
#define PMBC_VOUT_CAL_OFFSET            ((uint8_t)0x23)
#define PMBC_VOUT_MAX                   ((uint8_t)0x24)
#define PMBC_VOUT_MARGIN_HIGH           ((uint8_t)0x25)
#define PMBC_VOUT_MARGIN_LOW            ((uint8_t)0x26)
#define PMBC_VOUT_TRANSITION_RATE       ((uint8_t)0x27)
#define PMBC_VOUT_DROOP                 ((uint8_t)0x28)
#define PMBC_VOUT_SCALE_LOOP            ((uint8_t)0x29)
#define PMBC_VOUT_SCALE_MONITOR         ((uint8_t)0x2A)
#define PMBC_VOUT_MIN                   ((uint8_t)0x2B)
#define PMBC_COEFICIENTS                ((uint8_t)0x30)
#define PMBC_POUT_MAX                   ((uint8_t)0x31)
#define PMBC_MAX_DUTY                   ((uint8_t)0x32)
#define PMBC_FREQUENCY_SWITCH           ((uint8_t)0x33)
#define PMBC_POWER_MODE                 ((uint8_t)0x34)
#define PMBC_VIN_ON                     ((uint8_t)0x35)
#define PMBC_VIN_OFF                    ((uint8_t)0x36)
#define PMBC_INTERLEAVE                 ((uint8_t)0x37)
#define PMBC_IOUT_CAL_GAIN              ((uint8_t)0x38)
#define PMBC_IOUT_CAL_OFFSET            ((uint8_t)0x39)
#define PMBC_FAN_CONFIG_1_2             ((uint8_t)0x3A)
#define PMBC_FAN_COMMAND_1              ((uint8_t)0x3B)
#define PMBC_FAN_COMMAND_2              ((uint8_t)0x3C)
#define PMBC_FAN_CONFIG_3_4             ((uint8_t)0x3D)
#define PMBC_FAN_COMMAND_3              ((uint8_t)0x3E)
#define PMBC_FAN_COMMAND_4              ((uint8_t)0x3F)
#define PMBC_VOUT_OV_FAULT_LIMIT        ((uint8_t)0x40)
#define PMBC_VOUT_OV_FAULT_RESPONSE     ((uint8_t)0x41)
#define PMBC_VOUT_OV_WARN_LIMIT         ((uint8_t)0x42)
#define PMBC_VOUT_UV_WARN_LIMIT         ((uint8_t)0x43)
#define PMBC_VOUT_UV_FAULT_LIMIT        ((uint8_t)0x44)
#define PMBC_VOUT_UV_FAULT_RESPONSE     ((uint8_t)0x45)
#define PMBC_IOUT_OC_FAULT_LIMIT        ((uint8_t)0x46)
#define PMBC_IOUT_OC_FAULT_RESPONSE     ((uint8_t)0x47)
#define PMBC_IOUT_OC_LV_FAULT_LIMIT     ((uint8_t)0x48)
#define PMBC_IOUT_OC_LV_FAULT_RESPONSE  ((uint8_t)0x49)
#define PMBC_IOUT_OC_WARN_LIMIT         ((uint8_t)0x4A)
#define PMBC_IOUT_UC_FAULT_LIMIT        ((uint8_t)0x4B)
#define PMBC_IOUT_UC_FAULT_RESPONSE     ((uint8_t)0x4C)
#define PMBC_OT_FAULT_LIMIT             ((uint8_t)0x4F)
#define PMBC_OT_FAULT_RESPONSE          ((uint8_t)0x50)
#define PMBC_OT_WARN_LIMIT              ((uint8_t)0x51)
#define PMBC_UT_WARN_LIMIT              ((uint8_t)0x52)
#define PMBC_UT_FAULT_LIMIT             ((uint8_t)0x53)
#define PMBC_UT_FAULT_RESPONSE          ((uint8_t)0x54)
#define PMBC_VIN_OV_FAULT_LIMIT         ((uint8_t)0x55)
#define PMBC_VIN_OV_FAULT_RESPONSE      ((uint8_t)0x56)
#define PMBC_VIN_OV_WARN_LIMIT          ((uint8_t)0x57)
#define PMBC_VIN_UV_WARN_LIMIT          ((uint8_t)0x58)
#define PMBC_VIN_UV_FAULT_LIMIT         ((uint8_t)0x59)
#define PMBC_VIN_UV_FAULT_RESPONSE      ((uint8_t)0x5A)
#define PMBC_IIN_OC_FAULT_LIMIT         ((uint8_t)0x5B)
#define PMBC_IIN_OC_FAULT_RESPONSE      ((uint8_t)0x5C)
#define PMBC_IIN_OC_WARN_LIMIT          ((uint8_t)0x5D)
#define PMBC_POWER_GOOD_ON              ((uint8_t)0x5E)
#define PMBC_POWER_GOOD_OFF             ((uint8_t)0x5F)
#define PMBC_TON_DELAY                  ((uint8_t)0x60)
#define PMBC_TON_RISE                   ((uint8_t)0x61)
#define PMBC_TON_MAX_FAULT_LIMIT        ((uint8_t)0x62)
#define PMBC_TON_MAX_FAULT_RESPONSE     ((uint8_t)0x63)
#define PMBC_TOFF_DELAY                 ((uint8_t)0x64)
#define PMBC_TOFF_FALL                  ((uint8_t)0x65)
#define PMBC_TOFF_MAX_WARN_LIMIT        ((uint8_t)0x66)
#define PMBC_TOFF_MAX_FAULT_RESPONSE    ((uint8_t)0x67)  /* Removed in v1.1*/
#define PMBC_POUT_OP_FAULT_LIMIT        ((uint8_t)0x68)
#define PMBC_POUT_OP_FAULT_RESPONSE     ((uint8_t)0x69)
#define PMBC_POUT_OP_WARN_LIMIT         ((uint8_t)0x6A)
#define PMBC_PIN_OP_WARN_LIMIT          ((uint8_t)0x6B)
#define PMBC_STATUS_BYTE                ((uint8_t)0x78)
#define PMBC_STATUS_WORD                ((uint8_t)0x79)
#define PMBC_STATUS_VOUT                ((uint8_t)0x7A)
#define PMBC_STATUS_IOUT                ((uint8_t)0x7B)
#define PMBC_STATUS_INPUT               ((uint8_t)0x7C)
#define PMBC_STATUS_TEMPERATURE         ((uint8_t)0x7D)
#define PMBC_STATUS_CML                 ((uint8_t)0x7E)
#define PMBC_STATUS_OTHER               ((uint8_t)0x7F)
#define PMBC_STATUS_MFR_SPECIFIC        ((uint8_t)0x80)
#define PMBC_STATUS_FANS_1_2            ((uint8_t)0x81)
#define PMBC_STATUS_FANS_3_4            ((uint8_t)0x82)
#define PMBC_READ_KWH_IN                ((uint8_t)0x83)
#define PMBC_READ_KWH_OUT               ((uint8_t)0x84)
#define PMBC_READ_KWH_CONFIG            ((uint8_t)0x85)
#define PMBC_READ_EIN                   ((uint8_t)0x86)
#define PMBC_READ_EOUT                  ((uint8_t)0x87)
#define PMBC_READ_VIN                   ((uint8_t)0x88)
#define PMBC_READ_IIN                   ((uint8_t)0x89)
#define PMBC_READ_VCAP                  ((uint8_t)0x8A)
#define PMBC_READ_VOUT                  ((uint8_t)0x8B)
#define PMBC_READ_IOUT                  ((uint8_t)0x8C)
#define PMBC_READ_TEMPERATURE_1         ((uint8_t)0x8D)
#define PMBC_READ_TEMPERATURE_2         ((uint8_t)0x8E)
#define PMBC_READ_TEMPERATURE_3         ((uint8_t)0x8F)
#define PMBC_READ_FAN_SPEED_1           ((uint8_t)0x90)
#define PMBC_READ_FAN_SPEED_2           ((uint8_t)0x91)
#define PMBC_READ_FAN_SPEED_3           ((uint8_t)0x92)
#define PMBC_READ_FAN_SPEED_4           ((uint8_t)0x93)
#define PMBC_READ_DUTY_CYCLE            ((uint8_t)0x94)
#define PMBC_READ_FREQUENCY             ((uint8_t)0x95)
#define PMBC_READ_POUT                  ((uint8_t)0x96)
#define PMBC_READ_PIN                   ((uint8_t)0x97)
#define PMBC_PMBUS_REVISION             ((uint8_t)0x98)
#define PMBC_MFR_ID                     ((uint8_t)0x99)
#define PMBC_MFR_MODEL                  ((uint8_t)0x9A)
#define PMBC_MFR_REVISION               ((uint8_t)0x9B)
#define PMBC_MFR_LOCATION               ((uint8_t)0x9C)
#define PMBC_MFR_DATE                   ((uint8_t)0x9D)
#define PMBC_MFR_SERIAL                 ((uint8_t)0x9E)
#define PMBC_APP_PROFILE_SUPPORT        ((uint8_t)0x9F)
#define PMBC_MFR_VIN_MIN                ((uint8_t)0xA0)
#define PMBC_MFR_VIN_MAX                ((uint8_t)0xA1)
#define PMBC_MFR_IIN_MAX                ((uint8_t)0xA2)
#define PMBC_MFR_PIN_MAX                ((uint8_t)0xA3)
#define PMBC_MFR_VOUT_MIN               ((uint8_t)0xA4)
#define PMBC_MFR_VOUT_MAX               ((uint8_t)0xA5)
#define PMBC_MFR_IOUT_MAX               ((uint8_t)0xA6)
#define PMBC_MFR_POUT_MAX               ((uint8_t)0xA7)
#define PMBC_MFR_TAMBIENT_MAX           ((uint8_t)0xA8)
#define PMBC_MFR_TAMBIENT_MIN           ((uint8_t)0xA9)
#define PMBC_MFR_EFFICIENCY_LL          ((uint8_t)0xAA)
#define PMBC_MFR_EFFICIENCY_HL          ((uint8_t)0xAB)
#define PMBC_MFR_PIN_ACCURACY           ((uint8_t)0xAC)
#define PMBC_IC_DEVICE_ID               ((uint8_t)0xAD)
#define PMBC_IC_DEVICE_REV              ((uint8_t)0xAE)
#define PMBC_USER_DATA_00               ((uint8_t)0xB0)
#define PMBC_USER_DATA_01               ((uint8_t)0xB1)
#define PMBC_USER_DATA_02               ((uint8_t)0xB2)
#define PMBC_USER_DATA_03               ((uint8_t)0xB3)
#define PMBC_USER_DATA_04               ((uint8_t)0xB4)
#define PMBC_USER_DATA_05               ((uint8_t)0xB5)
#define PMBC_USER_DATA_06               ((uint8_t)0xB6)
#define PMBC_USER_DATA_07               ((uint8_t)0xB7)
#define PMBC_USER_DATA_08               ((uint8_t)0xB8)
#define PMBC_USER_DATA_09               ((uint8_t)0xB9)
#define PMBC_USER_DATA_10               ((uint8_t)0xBA)
#define PMBC_USER_DATA_11               ((uint8_t)0xBB)
#define PMBC_USER_DATA_12               ((uint8_t)0xBC)
#define PMBC_USER_DATA_13               ((uint8_t)0xBD)
#define PMBC_USER_DATA_14               ((uint8_t)0xBE)
#define PMBC_USER_DATA_15               ((uint8_t)0xBF)
#define PMBC_MFR_MAX_TEMP_1             ((uint8_t)0xC0)
#define PMBC_MFR_MAX_TEMP_2             ((uint8_t)0xC1)
#define PMBC_MFR_MAX_TEMP_3             ((uint8_t)0xC2)
  /* MFR_SPECIFIC from v1.3.1 on */
#define PMBC_MFR_SPECIFIC_C4            ((uint8_t)0xC4)
#define PMBC_MFR_SPECIFIC_C5            ((uint8_t)0xC5)
#define PMBC_MFR_SPECIFIC_C6            ((uint8_t)0xC6)
#define PMBC_MFR_SPECIFIC_C7            ((uint8_t)0xC7)
#define PMBC_MFR_SPECIFIC_C8            ((uint8_t)0xC8)
#define PMBC_MFR_SPECIFIC_C9            ((uint8_t)0xC9)
#define PMBC_MFR_SPECIFIC_CA            ((uint8_t)0xCA)
#define PMBC_MFR_SPECIFIC_CB            ((uint8_t)0xCB)
#define PMBC_MFR_SPECIFIC_CC            ((uint8_t)0xCC)
#define PMBC_MFR_SPECIFIC_CD            ((uint8_t)0xCD)
#define PMBC_MFR_SPECIFIC_CE            ((uint8_t)0xCE)
#define PMBC_MFR_SPECIFIC_CF            ((uint8_t)0xCF)
#define PMBC_MFR_SPECIFIC_D0            ((uint8_t)0xD0)
#define PMBC_MFR_SPECIFIC_D1            ((uint8_t)0xD1)
#define PMBC_MFR_SPECIFIC_D2            ((uint8_t)0xD2)
#define PMBC_MFR_SPECIFIC_D3            ((uint8_t)0xD3)
#define PMBC_MFR_SPECIFIC_D4            ((uint8_t)0xD4)
#define PMBC_MFR_SPECIFIC_D5            ((uint8_t)0xD5)
#define PMBC_MFR_SPECIFIC_D6            ((uint8_t)0xD6)
#define PMBC_MFR_SPECIFIC_D7            ((uint8_t)0xD7)
#define PMBC_MFR_SPECIFIC_D8            ((uint8_t)0xD8)
#define PMBC_MFR_SPECIFIC_D9            ((uint8_t)0xD9)
#define PMBC_MFR_SPECIFIC_DA            ((uint8_t)0xDA)
#define PMBC_MFR_SPECIFIC_DB            ((uint8_t)0xDB)
#define PMBC_MFR_SPECIFIC_DC            ((uint8_t)0xDC)
#define PMBC_MFR_SPECIFIC_DD            ((uint8_t)0xDD)
#define PMBC_MFR_SPECIFIC_DE            ((uint8_t)0xDE)
#define PMBC_MFR_SPECIFIC_DF            ((uint8_t)0xDF)
#define PMBC_MFR_SPECIFIC_E0            ((uint8_t)0xE0)
#define PMBC_MFR_SPECIFIC_E1            ((uint8_t)0xE1)
#define PMBC_MFR_SPECIFIC_E2            ((uint8_t)0xE2)
#define PMBC_MFR_SPECIFIC_E3            ((uint8_t)0xE3)
#define PMBC_MFR_SPECIFIC_E4            ((uint8_t)0xE4)
#define PMBC_MFR_SPECIFIC_E5            ((uint8_t)0xE5)
#define PMBC_MFR_SPECIFIC_E6            ((uint8_t)0xE6)
#define PMBC_MFR_SPECIFIC_E7            ((uint8_t)0xE7)
#define PMBC_MFR_SPECIFIC_E8            ((uint8_t)0xE8)
#define PMBC_MFR_SPECIFIC_E9            ((uint8_t)0xE9)
#define PMBC_MFR_SPECIFIC_EA            ((uint8_t)0xEA)
#define PMBC_MFR_SPECIFIC_EB            ((uint8_t)0xEB)
#define PMBC_MFR_SPECIFIC_EC            ((uint8_t)0xEC)
#define PMBC_MFR_SPECIFIC_ED            ((uint8_t)0xED)
#define PMBC_MFR_SPECIFIC_EE            ((uint8_t)0xEE)
#define PMBC_MFR_SPECIFIC_EF            ((uint8_t)0xEF)
#define PMBC_MFR_SPECIFIC_F0            ((uint8_t)0xF0)
#define PMBC_MFR_SPECIFIC_F1            ((uint8_t)0xF1)
#define PMBC_MFR_SPECIFIC_F2            ((uint8_t)0xF2)
#define PMBC_MFR_SPECIFIC_F3            ((uint8_t)0xF3)
#define PMBC_MFR_SPECIFIC_F4            ((uint8_t)0xF4)
#define PMBC_MFR_SPECIFIC_F5            ((uint8_t)0xF5)
#define PMBC_MFR_SPECIFIC_F6            ((uint8_t)0xF6)
#define PMBC_MFR_SPECIFIC_F7            ((uint8_t)0xF7)
#define PMBC_MFR_SPECIFIC_F8            ((uint8_t)0xF8)
#define PMBC_MFR_SPECIFIC_F9            ((uint8_t)0xF9)
#define PMBC_MFR_SPECIFIC_FA            ((uint8_t)0xFA)
#define PMBC_MFR_SPECIFIC_FB            ((uint8_t)0xFB)
#define PMBC_MFR_SPECIFIC_FC            ((uint8_t)0xFC)
#define PMBC_MFR_SPECIFIC_FD            ((uint8_t)0xFD)
  /* MFR_SPECIFIC up to v1.3 */
#define PMBC_MFR_SPECIFIC_00            ((uint8_t)0xD0)
#define PMBC_MFR_SPECIFIC_01            ((uint8_t)0xD1)
#define PMBC_MFR_SPECIFIC_02            ((uint8_t)0xD2)
#define PMBC_MFR_SPECIFIC_03            ((uint8_t)0xD3)
#define PMBC_MFR_SPECIFIC_04            ((uint8_t)0xD4)
#define PMBC_MFR_SPECIFIC_05            ((uint8_t)0xD5)
#define PMBC_MFR_SPECIFIC_06            ((uint8_t)0xD6)
#define PMBC_MFR_SPECIFIC_07            ((uint8_t)0xD7)
#define PMBC_MFR_SPECIFIC_08            ((uint8_t)0xD8)
#define PMBC_MFR_SPECIFIC_09            ((uint8_t)0xD9)
#define PMBC_MFR_SPECIFIC_10            ((uint8_t)0xDA)
#define PMBC_MFR_SPECIFIC_11            ((uint8_t)0xDB)
#define PMBC_MFR_SPECIFIC_12            ((uint8_t)0xDC)
#define PMBC_MFR_SPECIFIC_13            ((uint8_t)0xDD)
#define PMBC_MFR_SPECIFIC_14            ((uint8_t)0xDE)
#define PMBC_MFR_SPECIFIC_15            ((uint8_t)0xDF)
#define PMBC_MFR_SPECIFIC_16            ((uint8_t)0xE0)
#define PMBC_MFR_SPECIFIC_17            ((uint8_t)0xE1)
#define PMBC_MFR_SPECIFIC_18            ((uint8_t)0xE2)
#define PMBC_MFR_SPECIFIC_19            ((uint8_t)0xE3)
#define PMBC_MFR_SPECIFIC_20            ((uint8_t)0xE4)
#define PMBC_MFR_SPECIFIC_21            ((uint8_t)0xE5)
#define PMBC_MFR_SPECIFIC_22            ((uint8_t)0xE6)
#define PMBC_MFR_SPECIFIC_23            ((uint8_t)0xE7)
#define PMBC_MFR_SPECIFIC_24            ((uint8_t)0xE8)
#define PMBC_MFR_SPECIFIC_25            ((uint8_t)0xE9)
#define PMBC_MFR_SPECIFIC_26            ((uint8_t)0xEA)
#define PMBC_MFR_SPECIFIC_27            ((uint8_t)0xEB)
#define PMBC_MFR_SPECIFIC_28            ((uint8_t)0xEC)
#define PMBC_MFR_SPECIFIC_29            ((uint8_t)0xED)
#define PMBC_MFR_SPECIFIC_30            ((uint8_t)0xEE)
#define PMBC_MFR_SPECIFIC_31            ((uint8_t)0xEF)
#define PMBC_MFR_SPECIFIC_32            ((uint8_t)0xF0)
#define PMBC_MFR_SPECIFIC_33            ((uint8_t)0xF1)
#define PMBC_MFR_SPECIFIC_34            ((uint8_t)0xF2)
#define PMBC_MFR_SPECIFIC_35            ((uint8_t)0xF3)
#define PMBC_MFR_SPECIFIC_36            ((uint8_t)0xF4)
#define PMBC_MFR_SPECIFIC_37            ((uint8_t)0xF5)
#define PMBC_MFR_SPECIFIC_38            ((uint8_t)0xF6)
#define PMBC_MFR_SPECIFIC_39            ((uint8_t)0xF7)
#define PMBC_MFR_SPECIFIC_40            ((uint8_t)0xF8)
#define PMBC_MFR_SPECIFIC_41            ((uint8_t)0xF9)
#define PMBC_MFR_SPECIFIC_42            ((uint8_t)0xFA)
#define PMBC_MFR_SPECIFIC_43            ((uint8_t)0xFB)
#define PMBC_MFR_SPECIFIC_44            ((uint8_t)0xFC)
#define PMBC_MFR_SPECIFIC_45            ((uint8_t)0xFD)
  /* End of v1.3 section */
#define PMBC_MFR_SPECIFIC_COMMAND_EXT   ((uint8_t)0xFE)
#define PMBC_PMBUS_COMMAND_EXT          ((uint8_t)0xFF)

  /*
   * Note: On top of that there are commands without command code - quick command,
   *       send byte, alert response and receive byte that all need special treatment.
   */

#define PMBUS_CMD_TBL_SIZE              ((uint8_t)8)
  /*!< Size of table of command used in internal test (AN4502) */

#ifdef PMBUS12
#ifdef PMBUS13
#define PMBUS_COMMANDS_TAB_SIZE         ((uint8_t)164)  /* v1.3*/
#else /* PMBUS13 */
#define PMBUS_COMMANDS_TAB_SIZE         ((uint8_t)157)  /* v1.2*/
#endif  /* PMBUS13 */
#else /* PMBUS12 */
#define PMBUS_COMMANDS_TAB_SIZE         ((uint8_t)145)  /* v1.1*/
#endif /* PMBUS12 */
  /*!< Size of table of command as per different PMBus specifications */

#define PMBUS_ZONE_CCC_AR               ((uint8_t)0x80) /* All respond */
#define PMBUS_ZONE_CCC_ST               ((uint8_t)0x40) /* Status response */     
#define PMBUS_ZONE_CCC_DI               ((uint8_t)0x20) /* Data inversion */
#define PMBUS_ZONE_CCC_DS               ((uint8_t)0x10) /* Data byte significance */
  /*!< Command control codes for ZONE_READ */
            
  /**
    * @}
    */

  /** @defgroup STM32_PMBUS_STACK_Constants       PMBus stack data constants
    * @{
    */
  extern st_command_t PMBUS_COMMANDS_TAB[PMBUS_COMMANDS_TAB_SIZE];
  /*!< Table of PMBus commands defined by PMBus specifications */
  extern st_command_t PMBUS_COMMANDS_TEST[PMBUS_CMD_TBL_SIZE];
  /*!< Table of command used in internal test (AN4502) */
  extern st_command_t EXTENDED_READ_BYTE;
  /*!< Command definition of read byte with PMBus command space extension */
  extern st_command_t EXTENDED_READ_WORD;
  /*!< Command definition of read word with PMBus command space extension */
  extern st_command_t EXTENDED_WRITE_BYTE;
  /*!< Command definition of write byte with PMBus command space extension */
  extern st_command_t EXTENDED_WRITE_WORD;
  /*!< Command definition of write word with PMBus command space extension */
  extern st_command_t ZONE_READ_COMMAND;
  /**
    * @}
    */
  
  /** @defgroup STM32_PMBUS_STACK_Functions       PMBus stack functions
    * @{
    */

#define STACK_PMBUS_ExtendExecution(stack) ((stack)->StateMachine &= ~SMBUS_SMS_RESPONSE_READY)
  /*!< Macro used to postpone command execution in case of PMBus extension in palce */

  HAL_StatusTypeDef STACK_PMBUS_HostCommandGroup(SMBUS_StackHandleTypeDef *stackContext, st_command_t* command, uint16_t address, uint8_t last);
  void STACK_SMBUS_LocateCommand( SMBUS_StackHandleTypeDef* stackContext );

#ifdef PMBUS13    
HAL_StatusTypeDef STACK_PMBUS_MasterZoneConfig(SMBUS_StackHandleTypeDef *pStackContext, uint16_t address, SMBUS_ZoneStateTypeDef* zone);
HAL_StatusTypeDef STACK_PMBUS_MasterZoneActive(SMBUS_StackHandleTypeDef *pStackContext, SMBUS_ZoneStateTypeDef* zone);
HAL_StatusTypeDef STACK_PMBUS_MasterReadZoneStatus(SMBUS_StackHandleTypeDef *pStackContext, uint8_t ccode, uint8_t mask);
HAL_StatusTypeDef STACK_PMBUS_MasterZoneWrite(SMBUS_StackHandleTypeDef *pStackContext, st_command_t* pCommand);
HAL_StatusTypeDef STACK_PMBUS_MasterZoneReadStatusCont(SMBUS_StackHandleTypeDef *pStackContext);
HAL_StatusTypeDef STACK_PMBUS_ZoneReadCallback( SMBUS_StackHandleTypeDef* pStackContext, uint8_t number );
#endif

  /**
    * @}
    */

  /**
    * @}
    */

#ifdef __cplusplus
}
#endif

#endif /* __STACK_PMBUS_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

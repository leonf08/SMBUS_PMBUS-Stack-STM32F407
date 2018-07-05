/**
 ******************************************************************************
 * @file    stm32_PMBUS_stack.c
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

/* Includes ------------------------------------------------------------------*/
#include "stm32_PMBUS_stack.h"
#include "stm32_SMBUS_stack.h"

/** @addtogroup STM32_PMBUS_STACK
 * @{
 */

/** @defgroup STM32_PMBUS_STACK_Constants
 * @{
 */

/* ----------- definition of PMBUS commands ---------------- */
st_command_t PMBUS_COMMANDS_TAB[] = {
		{ PMBC_PAGE, READ_OR_WRITE, 2, 1 }, /* code 00 */
		{ PMBC_OPERATION, READ_OR_WRITE, 2, 1 }, /* code 01 */
		{ PMBC_ON_OFF_CONFIG, READ_OR_WRITE, 2, 1 }, /* code 02 */
		{ PMBC_CLEAR_FAULTS, WRITE, 1, 0 }, /* code 03 */
		{ PMBC_PHASE, READ_OR_WRITE, 2, 1 }, /* code 04 */
#ifdef  PMBUS12
		{	PMBC_PAGE_PLUS_WRITE , BLOCK_WRITE, 3, 0}, /* code 05 */
		{	PMBC_PAGE_PLUS_READ , BLK_PRC_CALL, 3, 2}, /* code 06 */
#ifdef  PMBUS13
		{	PMBC_ZONE_CONFIG , READ_OR_WRITE, 3, 2}, /* code 07 */
		{	PMBC_ZONE_ACTIVE , READ_OR_WRITE, 3, 2}, /* code 08 */
#endif
#endif
		{ PMBC_WRITE_PROTECT, READ_OR_WRITE, 2, 1 }, /* code 10 */
		{ PMBC_STORE_DEFAULT_ALL, WRITE, 1, 0 }, /* code 11 */
		{ PMBC_RESTORE_DEFAULT_ALL, WRITE, 1, 0 }, /* code 12 */
		{ PMBC_STORE_DEFAULT_CODE, WRITE, 2, 0 }, /* code 13 */
		{ PMBC_RESTORE_DEFAULT_CODE, WRITE, 2, 0 }, /* code 14 */
		{ PMBC_STORE_USER_ALL, WRITE, 1, 0 }, /* code 15 */
		{ PMBC_RESTORE_USER_ALL, WRITE, 1, 0 }, /* code 16 */
		{ PMBC_STORE_USER_CODE, WRITE, 2, 0 }, /* code 17 */
		{ PMBC_RESTORE_USER_CODE, WRITE, 2, 0 }, /* code 18 */
		{ PMBC_CAPABILITY, READ, 1, 1 }, /* code 19 */
		{ PMBC_QUERY, PROCESS_CALL, 2, 1 }, /* code 1A */
#ifdef  PMBUS12
		{	PMBC_SMBALERT_MASK , READ_OR_WRITE, 3, 2}, /* code 1B */
#endif
		{ PMBC_VOUT_MODE, READ_OR_WRITE, 2, 1 }, /* code 20 */
		{ PMBC_VOUNT_COMMAND, READ_OR_WRITE, 3, 2 }, /* code 21 */
		{ PMBC_VOUT_TRIM, READ_OR_WRITE, 3, 2 }, /* code 22 */
		{ PMBC_VOUT_CAL_OFFSET, READ_OR_WRITE, 3, 2 }, /* code 23 */
		{ PMBC_VOUT_MAX, READ_OR_WRITE, 3, 2 }, /* code 24 */
		{ PMBC_VOUT_MARGIN_HIGH, READ_OR_WRITE, 3, 2 }, /* code 25 */
		{ PMBC_VOUT_MARGIN_LOW, READ_OR_WRITE, 3, 2 }, /* code 26 */
		{ PMBC_VOUT_TRANSITION_RATE, READ_OR_WRITE, 3, 2 }, /* code 27 */
		{ PMBC_VOUT_DROOP, READ_OR_WRITE, 3, 2 }, /* code 28 */
		{ PMBC_VOUT_SCALE_LOOP, READ_OR_WRITE, 3, 2 }, /* code 29 */
		{ PMBC_VOUT_SCALE_MONITOR, READ_OR_WRITE, 3, 2 }, /* code 2A */
#ifdef  PMBUS13
		{	PMBC_VOUT_MIN , READ_OR_WRITE, 3, 2}, /* code 2B */
#endif
		{ PMBC_COEFICIENTS, BLK_PRC_CALL, 3, 5 }, /* code 30 */
		{ PMBC_POUT_MAX, READ_OR_WRITE, 3, 2 }, /* code 31 */
		{ PMBC_MAX_DUTY, READ_OR_WRITE, 3, 2 }, /* code 32 */
		{ PMBC_FREQUENCY_SWITCH, READ_OR_WRITE, 3, 2 }, /* code 33 */
#ifdef  PMBUS13
		{	PMBC_POWER_MODE , READ_OR_WRITE, 2, 1}, /* code 34 */
#endif
		{ PMBC_VIN_ON, READ_OR_WRITE, 3, 2 }, /* code 35 */
		{ PMBC_VIN_OFF, READ_OR_WRITE, 3, 2 }, /* code 36 */
		{ PMBC_INTERLEAVE, READ_OR_WRITE, 3, 2 }, /* code 37 */
		{ PMBC_IOUT_CAL_GAIN, READ_OR_WRITE, 3, 2 }, /* code 38 */
		{ PMBC_IOUT_CAL_OFFSET, READ_OR_WRITE, 3, 2 }, /* code 39 */
		{ PMBC_FAN_CONFIG_1_2, READ_OR_WRITE, 2, 1 }, /* code 3A */
		{ PMBC_FAN_COMMAND_1, READ_OR_WRITE, 3, 2 }, /* code 3B */
		{ PMBC_FAN_COMMAND_2, READ_OR_WRITE, 3, 2 }, /* code 3C */
		{ PMBC_FAN_CONFIG_3_4, READ_OR_WRITE, 2, 1 }, /* code 3D */
		{ PMBC_FAN_COMMAND_3, READ_OR_WRITE, 3, 2 }, /* code 3E */
		{ PMBC_FAN_COMMAND_4, READ_OR_WRITE, 3, 2 }, /* code 3F */
		{ PMBC_VOUT_OV_FAULT_LIMIT, READ_OR_WRITE, 3, 2 }, /* code 40 */
		{ PMBC_VOUT_OV_FAULT_RESPONSE, READ_OR_WRITE, 2, 1 }, /* code 41 */
		{ PMBC_VOUT_OV_WARN_LIMIT, READ_OR_WRITE, 3, 2 }, /* code 42 */
		{ PMBC_VOUT_UV_WARN_LIMIT, READ_OR_WRITE, 3, 2 }, /* code 43 */
		{ PMBC_VOUT_UV_FAULT_LIMIT, READ_OR_WRITE, 3, 2 }, /* code 44 */
		{ PMBC_VOUT_UV_FAULT_RESPONSE, READ_OR_WRITE, 3, 2 }, /* code 45 */
		{ PMBC_IOUT_OC_FAULT_LIMIT, READ_OR_WRITE, 2, 1 }, /* code 46 */
		{ PMBC_IOUT_OC_FAULT_RESPONSE, READ_OR_WRITE, 3, 2 }, /* code 47 */
		{ PMBC_IOUT_OC_LV_FAULT_LIMIT, READ_OR_WRITE, 3, 2 }, /* code 48 */
		{ PMBC_IOUT_OC_LV_FAULT_RESPONSE, READ_OR_WRITE, 2, 1 }, /* code 49 */
		{ PMBC_IOUT_OC_WARN_LIMIT, READ_OR_WRITE, 3, 2 }, /* code 4A */
		{ PMBC_IOUT_UC_FAULT_LIMIT, READ_OR_WRITE, 3, 2 }, /* code 4B */
		{ PMBC_IOUT_UC_FAULT_RESPONSE, READ_OR_WRITE, 2, 1 }, /* code 4C */
		{ PMBC_OT_FAULT_LIMIT, READ_OR_WRITE, 3, 2 }, /* code 4F */
		{ PMBC_OT_FAULT_RESPONSE, READ_OR_WRITE, 2, 1 }, /* code 50 */
		{ PMBC_OT_WARN_LIMIT, READ_OR_WRITE, 3, 2 }, /* code 51 */
		{ PMBC_UT_WARN_LIMIT, READ_OR_WRITE, 3, 2 }, /* code 52 */
		{ PMBC_UT_FAULT_LIMIT, READ_OR_WRITE, 3, 2 }, /* code 53 */
		{ PMBC_UT_FAULT_RESPONSE, READ_OR_WRITE, 2, 1 }, /* code 54 */
		{ PMBC_VIN_OV_FAULT_LIMIT, READ_OR_WRITE, 3, 2 }, /* code 55 */
		{ PMBC_VIN_OV_FAULT_RESPONSE, READ_OR_WRITE, 2, 1 }, /* code 56 */
		{ PMBC_VIN_OV_WARN_LIMIT, READ_OR_WRITE, 3, 2 }, /* code 57 */
		{ PMBC_VIN_UV_WARN_LIMIT, READ_OR_WRITE, 3, 2 }, /* code 58 */
		{ PMBC_VIN_UV_FAULT_LIMIT, READ_OR_WRITE, 3, 2 }, /* code 59 */
		{ PMBC_VIN_UV_FAULT_RESPONSE, READ_OR_WRITE, 2, 1 }, /* code 5A */
		{ PMBC_IIN_OC_FAULT_LIMIT, READ_OR_WRITE, 3, 2 }, /* code 5B */
		{ PMBC_IIN_OC_FAULT_RESPONSE, READ_OR_WRITE, 2, 1 }, /* code 5C */
		{ PMBC_IIN_OC_WARN_LIMIT, READ_OR_WRITE, 3, 2 }, /* code 5D */
		{ PMBC_POWER_GOOD_ON, READ_OR_WRITE, 3, 2 }, /* code 5E */
		{ PMBC_POWER_GOOD_OFF, READ_OR_WRITE, 3, 2 }, /* code 5F */
		{ PMBC_TON_DELAY, READ_OR_WRITE, 3, 2 }, /* code 60 */
		{ PMBC_TON_RISE, READ_OR_WRITE, 3, 2 }, /* code 61 */
		{ PMBC_TON_MAX_FAULT_LIMIT, READ_OR_WRITE, 3, 2 }, /* code 62 */
		{ PMBC_TON_MAX_FAULT_RESPONSE, READ_OR_WRITE, 2, 1 }, /* code 63 */
		{ PMBC_TOFF_DELAY, READ_OR_WRITE, 3, 2 }, /* code 64 */
		{ PMBC_TOFF_FALL, READ_OR_WRITE, 3, 2 }, /* code 65 */
		{ PMBC_TOFF_MAX_WARN_LIMIT, READ_OR_WRITE, 3, 2 }, /* code 66 */
		{ PMBC_POUT_OP_FAULT_LIMIT, READ_OR_WRITE, 3, 2 }, /* code 68 */
		{ PMBC_POUT_OP_FAULT_RESPONSE, READ_OR_WRITE, 2, 1 }, /* code 69 */
		{ PMBC_POUT_OP_WARN_LIMIT, READ_OR_WRITE, 3, 2 }, /* code 6A */
		{ PMBC_PIN_OP_WARN_LIMIT, READ_OR_WRITE, 3, 2 }, /* code 6B */
#ifdef  PMBUS12
		{	PMBC_STATUS_BYTE , READ_OR_WRITE, 2, 1}, /* code 78 */
		{	PMBC_STATUS_WORD , READ_OR_WRITE, 3, 2}, /* code 79 */
#else
		{ PMBC_STATUS_BYTE, READ, 1, 1 }, /* code 78 */
		{ PMBC_STATUS_WORD, READ, 1, 2 }, /* code 79 */
#endif
		{ PMBC_STATUS_VOUT, READ, 1, 1 }, /* code 7A */
		{ PMBC_STATUS_IOUT, READ, 1, 1 }, /* code 7B */
		{ PMBC_STATUS_INPUT, READ, 1, 1 }, /* code 7C */
		{ PMBC_STATUS_TEMPERATURE, READ, 1, 1 }, /* code 7D */
		{ PMBC_STATUS_CML, READ, 1, 1 }, /* code 7E */
		{ PMBC_STATUS_OTHER, READ, 1, 1 }, /* code 7F */
		{ PMBC_STATUS_MFR_SPECIFIC, READ, 1, 1 }, /* code 80 */
		{ PMBC_STATUS_FANS_1_2, READ, 1, 1 }, /* code 81 */
		{ PMBC_STATUS_FANS_3_4, READ, 1, 1 }, /* code 82 */
#ifdef  PMBUS13
		{	PMBC_READ_KWH_IN , READ, 1, 4}, /* code 83 */
		{	PMBC_READ_KWH_OUT , READ, 1, 4}, /* code 84 */
		{	PMBC_READ_KWH_CONFIG , READ_OR_WRITE, 3, 2}, /* code 85 */
#endif
#ifdef  PMBUS12
		{	PMBC_READ_EIN , BLOCK_READ, 1, 6}, /* code 87 */
		{	PMBC_READ_EOUT , BLOCK_READ, 1, 6}, /* code 87 */
#endif
		{ PMBC_READ_VIN, READ, 1, 2 }, /* code 88 */
		{ PMBC_READ_IIN, READ, 1, 2 }, /* code 89 */
		{ PMBC_READ_VCAP, READ, 1, 2 }, /* code 8A */
		{ PMBC_READ_VOUT, READ, 1, 2 }, /* code 8B */
		{ PMBC_READ_IOUT, READ, 1, 2 }, /* code 8C */
		{ PMBC_READ_TEMPERATURE_1, READ, 1, 2 }, /* code 8D */
		{ PMBC_READ_TEMPERATURE_2, READ, 1, 2 }, /* code 8E */
		{ PMBC_READ_TEMPERATURE_3, READ, 1, 2 }, /* code 8F */
		{ PMBC_READ_FAN_SPEED_1, READ, 1, 2 }, /* code 90 */
		{ PMBC_READ_FAN_SPEED_2, READ, 1, 2 }, /* code 91 */
		{ PMBC_READ_FAN_SPEED_3, READ, 1, 2 }, /* code 92 */
		{ PMBC_READ_FAN_SPEED_4, READ, 1, 2 }, /* code 93 */
		{ PMBC_READ_DUTY_CYCLE, READ, 1, 2 }, /* code 94 */
		{ PMBC_READ_FREQUENCY, READ, 1, 2 }, /* code 95 */
		{ PMBC_READ_POUT, READ, 1, 2 }, /* code 96 */
		{ PMBC_READ_PIN, READ, 1, 2 }, /* code 97 */
		{ PMBC_PMBUS_REVISION, READ, 1, 1 }, /* code 98 */
		{ PMBC_MFR_ID, BLK_RD_OR_WR, 1, 1 }, /* code 99 */
		{ PMBC_MFR_MODEL, BLK_RD_OR_WR, 1, 8 }, /* code 9A */
		{ PMBC_MFR_REVISION, BLK_RD_OR_WR, 1, 1 }, /* code 9B */
		{ PMBC_MFR_LOCATION, BLK_RD_OR_WR, 1, 1 }, /* code 9C */
		{ PMBC_MFR_DATE, BLK_RD_OR_WR, 1, 1 }, /* code 9D */
		{ PMBC_MFR_SERIAL, BLK_RD_OR_WR, 1, 1 }, /* code 9E */
#ifdef  PMBUS12
		{	PMBC_APP_PROFILE_SUPPORT , BLOCK_READ, 1, 2}, /* code 9F */
#endif
		{ PMBC_MFR_VIN_MIN, READ, 1, 2 }, /* code A0 */
		{ PMBC_MFR_VIN_MAX, READ, 1, 2 }, /* code A1 */
		{ PMBC_MFR_IIN_MAX, READ, 1, 2 }, /* code A2 */
		{ PMBC_MFR_PIN_MAX, READ, 1, 2 }, /* code A3 */
		{ PMBC_MFR_VOUT_MIN, READ, 1, 2 }, /* code A4 */
		{ PMBC_MFR_VOUT_MAX, READ, 1, 2 }, /* code A5 */
		{ PMBC_MFR_IOUT_MAX, READ, 1, 2 }, /* code A6 */
		{ PMBC_MFR_POUT_MAX, READ, 1, 2 }, /* code A7 */
		{ PMBC_MFR_TAMBIENT_MAX, READ, 1, 2 }, /* code A8 */
		{ PMBC_MFR_TAMBIENT_MIN, READ, 1, 2 }, /* code A9 */
		{ PMBC_MFR_EFFICIENCY_LL, BLK_RD_OR_WR, 15, 14 }, /* code AA */
		{ PMBC_MFR_EFFICIENCY_HL, BLK_RD_OR_WR, 15, 14 }, /* code AB */
#ifdef  PMBUS12
		{	PMBC_MFR_PIN_ACCURACY , READ, 1, 1}, /* code AC */
		{	PMBC_IC_DEVICE_ID , BLOCK_READ, 1, 2}, /* code AD */
		{	PMBC_IC_DEVICE_REV , BLOCK_READ, 1, 2}, /* code AE */
#endif
		{ PMBC_USER_DATA_00, BLK_RD_OR_WR, 1, 1 }, /* code B0 */
		{ PMBC_USER_DATA_01, BLK_RD_OR_WR, 1, 1 }, /* code B1 */
		{ PMBC_USER_DATA_02, BLK_RD_OR_WR, 1, 1 }, /* code B2 */
		{ PMBC_USER_DATA_03, BLK_RD_OR_WR, 1, 1 }, /* code B3 */
		{ PMBC_USER_DATA_04, BLK_RD_OR_WR, 1, 1 }, /* code B4 */
		{ PMBC_USER_DATA_05, BLK_RD_OR_WR, 1, 1 }, /* code B5 */
		{ PMBC_USER_DATA_06, BLK_RD_OR_WR, 1, 1 }, /* code B6 */
		{ PMBC_USER_DATA_07, BLK_RD_OR_WR, 1, 1 }, /* code B7 */
		{ PMBC_USER_DATA_08, BLK_RD_OR_WR, 1, 1 }, /* code B8 */
		{ PMBC_USER_DATA_09, BLK_RD_OR_WR, 1, 1 }, /* code B9 */
		{ PMBC_USER_DATA_10, BLK_RD_OR_WR, 1, 1 }, /* code BA */
		{ PMBC_USER_DATA_11, BLK_RD_OR_WR, 1, 1 }, /* code BB */
		{ PMBC_USER_DATA_12, BLK_RD_OR_WR, 1, 1 }, /* code BC */
		{ PMBC_USER_DATA_13, BLK_RD_OR_WR, 1, 1 }, /* code BD */
		{ PMBC_USER_DATA_14, BLK_RD_OR_WR, 1, 1 }, /* code BE */
		{ PMBC_USER_DATA_15, BLK_RD_OR_WR, 1, 1 }, /* code BF */
#ifdef  PMBUS12
		{	PMBC_MFR_MAX_TEMP_1 , READ_OR_WRITE, 3, 2}, /* code C0 */
		{	PMBC_MFR_MAX_TEMP_2 , READ_OR_WRITE, 3, 2}, /* code C1 */
		{	PMBC_MFR_MAX_TEMP_3 , READ_OR_WRITE, 3, 2}, /* code C2 */
#endif
	};

st_command_t PMBUS_COMMANDS_TEST[] = { { 0, WRITE, 2, 0 }, { 1, BLK_PRC_CALL, 5,
		4 }, { 2, BLOCK_WRITE, 3, 0 }, { 3, READ, 1, 2 },
		{ 4, BLOCK_READ, 1, 4 }, { 5, PROCESS_CALL, 3, 2 }, { 6, READ_OR_WRITE,
				1, 1 }, { 7, BLK_RD_OR_WR, 1, 1 } };

/*
 dedicated command definitions for the extended command support
 */
st_command_t EXTENDED_READ_BYTE = { PMBC_PMBUS_COMMAND_EXT, PROCESS_CALL, 2, 1 };
st_command_t EXTENDED_READ_WORD = { PMBC_PMBUS_COMMAND_EXT, PROCESS_CALL, 2, 2 };
st_command_t EXTENDED_WRITE_BYTE = { PMBC_PMBUS_COMMAND_EXT, WRITE, 3, 0 };
st_command_t EXTENDED_WRITE_WORD = { PMBC_PMBUS_COMMAND_EXT, WRITE, 4, 0 };

/*
 dedicated command definition for the Zone read support
 */
st_command_t ZONE_READ_COMMAND = { 0, PROCESS_CALL, 2, 2 };

/* Private define ------------------------------------------------------------*/
#define NOT_USED(p) ((void)(p))

/**
 * @}
 */

/** @defgroup STM32_PMBUS_STACK_Functions
 * @{
 */

/**
 * @brief  PMBUS master group command transmit, direction is implicitly write.
 * @param  pStackContext : Pointer to a SMBUS_StackHandleTypeDef structure that contains
 *                the configuration information for the specified SMBUS.
 * @param  pCommand : description of the command to be transmitted, NULL for quick command
 * @param  address : device address
 * @param  last : last command in the group - STOP condition is transmitted if this flag is != 0
 * @retval SMBus stack return code
 */
HAL_StatusTypeDef STACK_PMBUS_HostCommandGroup(
		SMBUS_StackHandleTypeDef *pStackContext, st_command_t* pCommand,
		uint16_t address, uint8_t last) {
	HAL_StatusTypeDef result = STACK_ERROR;
	uint16_t size;
	uint32_t xFerOptions = SMBUS_FIRST_FRAME;

	/*
	 First check status of the SMBUS - no transaction ongoing
	 */
	if ((((pStackContext->StateMachine) & SMBUS_SMS_ACTIVE_MASK) == 0)
			&& ((pCommand->cmnd_query & WRITE) == WRITE)) {
		if (pCommand == NULL) {
			/*
			 quick command case
			 */
			size = 0;
		} else {
			/*
			 the command must support write mode, otherwise is unusable for grouping
			 */

			pStackContext->OpMode = pCommand->cmnd_query & BLOCK_WRITE;

			/*
			 Remembering the address and command code for case of further processing of non-trivial command
			 */
			pStackContext->SlaveAddress = address;
			pStackContext->CurrentCommand = pCommand;

			/*
			 First byte, the command code is transmitted
			 */
			pStackContext->Buffer[0] = pCommand->cmnd_code;

			if (pCommand->cmnd_query & BLOCK) {
				/*
				 Block write with data size prepared in the buffer.
				 */
				size = 2 + pStackContext->Buffer[1]; /* 1 cmd code + 1 count + data size */
			} else {
				/*
				 fixed size write
				 */
				size = pCommand->cmnd_master_Tx_size;
			}

			xFerOptions |= (pStackContext->StateMachine & SMBUS_SMS_PEC_ACTIVE);
			if (pStackContext->StateMachine & SMBUS_SMS_PEC_ACTIVE) {
				size += 1;
			}
		}

		/*
		 finishing transmission
		 */
		if (last != 0) {
			xFerOptions |= SMBUS_LAST_FRAME;
		}

		/*
		 Initiating a transmission
		 */
		pStackContext->StateMachine |= SMBUS_SMS_TRANSMIT;
		pStackContext->StateMachine &= ~SMBUS_SMS_READY;

		/*
		 Sending the data and logging the result
		 */
		result = HAL_SMBUS_Master_Sequential_Transmit_IT(pStackContext->Device,
				address, pStackContext->Buffer, size, xFerOptions);
		if (result != HAL_OK) {
			pStackContext->StateMachine |= SMBUS_SMS_ERROR;
		}
	}
	return result;
}

/**
 * @brief  a version of the default SMBUS implementation with support for
 *       PMBus extended command
 * @param  pStackContext : Pointer to a SMBUS_StackHandleTypeDef structure that contains
 *                the configuration information for the specified SMBUS.
 * @retval None
 */
void STACK_SMBUS_LocateCommand(SMBUS_StackHandleTypeDef* pStackContext) {
	uint8_t commandCode = pStackContext->Buffer[0];
#ifdef DENSE_CMD_TBL
	uint32_t current, low, top;
#endif

#ifdef ARP
	if ( pStackContext->StateMachine & SMBUS_SMS_ARP_AM )
	{
		STACK_SMBUS_LocateCommandARP( pStackContext, commandCode );
	}
	else
#endif /* ARP treatment */

	if (commandCode == PMBC_PMBUS_COMMAND_EXT) {
		/*
		 May not be exactly read byte, but the stack cannot know for sure
		 */
		pStackContext->CurrentCommand = &EXTENDED_READ_BYTE;
	} else {
		/*
		 Code searching for command based on command code
		 */
#ifdef DENSE_CMD_TBL

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
			current = ( low + top ) >> 1;
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
		pStackContext->CurrentCommand =
				&(pStackContext->CMD_table[commandCode]);

		/* a sanity check */
		if (pStackContext->CurrentCommand->cmnd_code != commandCode) {
			pStackContext->CurrentCommand = NULL;
		}
#endif /* DENSE_CMD_TBL */
	}
}

#ifdef  PMBUS13 /* Zone commands were introduced in v1.3 */

/**
 * @brief  PMBUS zone write command transmit, direction is implicitly write.
 * @param  pStackContext : Pointer to a SMBUS_StackHandleTypeDef structure that contains
 *                the configuration information for the specified SMBUS.
 * @param  pCommand : description of the command to be transmitted, NULL for quick command
 * @param  zone : the zone context in which the command executes
 * @retval SMBus stack return code
 */
HAL_StatusTypeDef STACK_PMBUS_MasterZoneWrite(SMBUS_StackHandleTypeDef *pStackContext, st_command_t* pCommand)
{
	HAL_StatusTypeDef result = STACK_ERROR;
	uint16_t size;
	uint32_t xFerOptions = SMBUS_FIRST_FRAME;

	/*
	 First check status of the SMBUS - no transaction ongoing, and that the command is write direction
	 */
	if (( ((pStackContext->StateMachine) & SMBUS_SMS_ACTIVE_MASK) == 0 ) && ( ( pCommand->cmnd_query & WRITE ) == WRITE ) )
	{
		if ( pCommand == NULL )
		{
			/*
			 quick command case
			 */
			size = 0;
		}
		else
		{
			/*
			 the command must support write mode, otherwise is unusable for zone
			 */
			pStackContext->OpMode = pCommand->cmnd_query & BLOCK_WRITE;

			/*
			 Remembering the address and command code for case of further processing of non-trivial command
			 */
			pStackContext->SlaveAddress = SMBUS_ADDR_ZONE_WRITE;
			pStackContext->CurrentCommand = pCommand;

			/*
			 First byte, the command code is transmitted
			 */
			pStackContext->Buffer[0] = pCommand->cmnd_code;

			if ( pCommand->cmnd_query & BLOCK )
			{
				/*
				 Block write with data size prepared in the buffer.
				 */
				size = 2 + pStackContext->Buffer[1]; /* 1 cmd code + 1 count + data size */
			}
			else
			{
				/*
				 fixed size write
				 */
				size = pCommand->cmnd_master_Tx_size;
			}

			xFerOptions |= ( pStackContext->StateMachine & SMBUS_SMS_PEC_ACTIVE );
			if (pStackContext->StateMachine & SMBUS_SMS_PEC_ACTIVE )
			{
				size += 1;
			}
		}

		/*
		 finishing transmission
		 */
		xFerOptions |= SMBUS_LAST_FRAME;

		/*
		 Initiating a transmission
		 */
		pStackContext->StateMachine |= SMBUS_SMS_TRANSMIT;
		pStackContext->StateMachine &= ~SMBUS_SMS_READY;

		/*
		 Sending the data and logging the result
		 */
		result = HAL_SMBUS_Master_Sequential_Transmit_IT( pStackContext->Device, SMBUS_ADDR_ZONE_WRITE, pStackContext->Buffer, size, xFerOptions );
		if (result != HAL_OK )
		{
			pStackContext->StateMachine |= SMBUS_SMS_ERROR;
		}
	}
	return result;
}

/**
 * @brief  PMBUS zone config command transmit, direction is implicitly write.
 * @param  pStackContext : Pointer to a SMBUS_StackHandleTypeDef structure that contains
 *                the configuration information for the specified SMBUS.
 * @param  address : address of the device to be configured
 * @param  zone : the zone context to be configured
 * @retval SMBus stack return code
 */
HAL_StatusTypeDef STACK_PMBUS_MasterZoneConfig(SMBUS_StackHandleTypeDef *pStackContext, uint16_t address, SMBUS_ZoneStateTypeDef* zone)
{
	pStackContext->Buffer[1] = zone->writeZone;
	pStackContext->Buffer[2] = zone->readZone;
	return STACK_SMBUS_HostCommand( pStackContext, (st_command_t*)&PMBUS_COMMANDS_TAB[PMBC_ZONE_CONFIG], address, WRITE );
}

/**
 * @brief  PMBUS zone active command transmit, direction is implicitly write.
 * @param  pStackContext : Pointer to a SMBUS_StackHandleTypeDef structure that contains
 *                the configuration information for the specified SMBUS.
 * @param  zone : the zones to be activated
 * @retval SMBus stack return code
 */
HAL_StatusTypeDef STACK_PMBUS_MasterZoneActive(SMBUS_StackHandleTypeDef *pStackContext, SMBUS_ZoneStateTypeDef* zone)
{
	pStackContext->Buffer[1] = zone->activeWriteZone;
	pStackContext->Buffer[2] = zone->activeReadZone;
	return STACK_SMBUS_HostCommand( pStackContext, (st_command_t*)&PMBUS_COMMANDS_TAB[PMBC_ZONE_ACTIVE], SMBUS_ADDR_ZONE_WRITE, WRITE );
}

/**
 * @brief  PMBUS zone read command initiation, simple implementation limited to status retrieve from single page devices.
 * @param  pStackContext : Pointer to a SMBUS_StackHandleTypeDef structure that contains
 *                the configuration information for the specified SMBUS.
 * @param  ccode : command control code
 * @param  mask : status mask
 * @retval SMBus stack return code
 */
HAL_StatusTypeDef STACK_PMBUS_MasterReadZoneStatus(SMBUS_StackHandleTypeDef *pStackContext, uint8_t ccode, uint8_t mask)
{
	HAL_StatusTypeDef result = STACK_ERROR;

	/*
	 First check status of the SMBUS - no transaction ongoing
	 */
	if (((pStackContext->StateMachine) & SMBUS_SMS_ACTIVE_MASK) == 0 )
	{
		/*
		 Remembering the address and command code for case of further processing of non-trivial command
		 */
		pStackContext->SlaveAddress = SMBUS_ADDR_ZONE_READ;
		pStackContext->CurrentCommand = NULL;

		/*
		 First byte, the command code is transmitted
		 */
		pStackContext->Buffer[0] = ccode;
		pStackContext->Buffer[1] = mask;

		/*
		 Initiating a transmission
		 */
		pStackContext->StateMachine |= SMBUS_SMS_TRANSMIT;
		pStackContext->StateMachine &= ~SMBUS_SMS_READY;

		/*
		 Sending the data and logging the result
		 */
		result = HAL_SMBUS_Master_Sequential_Transmit_IT( pStackContext->Device, SMBUS_ADDR_ZONE_READ, pStackContext->Buffer, 2, SMBUS_FIRST_FRAME );
		if (result != HAL_OK )
		{
			pStackContext->StateMachine |= SMBUS_SMS_ERROR;
		}
	}
	return result;
}

/**
 * @brief  Implementation of Zone read continuation - Master side.
 * @param  pStackContext : Pointer to a SMBUS_StackHandleTypeDef structure that contains
 *                the context information for the specified SMBUS stack.
 * @retval HAL_StatusTypeDef response code. STACK_OK if success, any other value means problem
 */
HAL_StatusTypeDef STACK_PMBUS_MasterZoneReadStatusCont(SMBUS_StackHandleTypeDef *pStackContext)
{
	HAL_StatusTypeDef result = STACK_ERROR;

	result = HAL_SMBUS_Master_Sequential_Receive_IT( pStackContext->Device, SMBUS_ADDR_ZONE_READ, pStackContext->Buffer, 2, SMBUS_NO_OPTION_FRAME);
	if (result != HAL_OK )
	{
		pStackContext->StateMachine |= SMBUS_SMS_ERROR;
	}
	return result;
}

/**
 * @brief  Callback function notifying slave about Zone read.
 * @param  pStackContext : Pointer to a SMBUS_StackHandleTypeDef structure that contains
 *                the context information for the specified SMBUS stack.
 * @param  number : Indicates if this command header, or read request
 * @retval HAL_StatusTypeDef response code. STACK_OK if success, any other value means problem
 */
__weak HAL_StatusTypeDef STACK_PMBUS_ZoneReadCallback( SMBUS_StackHandleTypeDef* pStackContext, uint8_t number )
{
	uint8_t ccc, mask;

	if ( number == 1 )
	{
		/* Command just received, here the device should read COMMAND CONTROL CODE and Mask*/
		ccc = pStackContext->Buffer[0];
		mask = pStackContext->Buffer[1];
		NOT_USED(ccc);
		NOT_USED(mask);
	}
	else
	{
		/* Read phase of the Zone read - arbitarry number returned by example*/
		pStackContext->Buffer[1] = 0xa5;
		pStackContext->Buffer[2] = pStackContext->OwnAddress;
	}

	/*
	 Returning zero means no problem with execution, if reply is expected, then it must be placed to the IO buffer
	 */
	return STACK_OK;
}
#endif  /* PMBUS13 */

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

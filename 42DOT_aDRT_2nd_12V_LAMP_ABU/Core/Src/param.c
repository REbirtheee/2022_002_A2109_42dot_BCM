/*
 * param.c
 *
 *  Created on: May 31, 2022
 *      Author: MSI
 */

#include "param.h"

static FLASH_EraseInitTypeDef EraseInitStruct;

static __IO uint32_t gnrl_2;

static __IO uint32_t low_steer;
static __IO uint32_t high_steer;

uint8_t init_flag =0;
/*
void ParamUpdate(struct ParamSequnce *seq, struct PramVaiable *param)
{
	if(seq->timer == 0) {
		InitSaveDataRead(param);
	} else if( seq->timer == 1) {
		GnrlParamSave(param);
	} else if( seq->timer == 2) {
		SteerInterfaceParamSave(param);
	} else if( seq->timer == 3) {
		SaveDataConvert(param);
		SteerInterfaceParamCaliDataConvert(param);
	} else if( seq->timer == 4) {
		if(param->si_flag) {
			ParamCanInfo(0x731, FDCAN_DLC_BYTES_7, param->gnrl_2_diff, &param->si_flag);
		}

	} else if( seq->timer == 5) {
		//ParamCanInfo(0x7f3, FDCAN_DLC_BYTES_8, param->si_diff);
	}
	ParamSequnceTimer(&seq->timer, 0, 5);
}
*/

void GnrlParamSave(struct PramVaiable *param, uint8_t savedata)
{
	if(param->gnrl_2_flag) {
		param->gnrl_2_sum = savedata;
		HAL_FLASH_Unlock();
		FlashErase(FLASH_START_ADDR_CNT, 0, 1);
		FlashSave(FLASH_START_ADDR_CNT, param->gnrl_2_sum, 0);
		gnrl_2 = *((__IO uint32_t*)(FlashRead(FLASH_START_ADDR_CNT, 0)));

		romdata = gnrl_2;
		/*
		param->gnrl_2_sum = gnrl_2 & 0x0000FFFF;
		param->gnrl_2_diff[0] = param->gnrl_2_sum & 0xFF;
		param->gnrl_2_diff[1] = (param->gnrl_2_sum >> 8) & 0xFF;
		*/

		HAL_FLASH_Lock();
		param->gnrl_2_flag = false;
	}
}

void SteerInterfaceParamSave(struct PramVaiable *param)
{
	if (param->si_flag) {
		if (param->si_req_write_flag && param->si_req_active_flag) {
			param->si_write_flag = true;
			param->si_active_flag = true;
		} else if (!param->si_req_write_flag && !param->si_req_active_flag) {
			param->si_write_flag = false;
			param->si_active_flag = false;
		} else if (!param->si_req_write_flag && param->si_req_active_flag) {
			param->si_write_flag = false;
			param->si_active_flag = true;
		} else if (param->si_req_write_flag && !param->si_req_active_flag) {
			param->si_write_flag = true;
			param->si_active_flag = true;
		}
		param->si_flag = false;
	}

	if(param->si_write_flag) {
		param->low_si_sum = (param->si_raw[3] << 24) |(param->si_raw[2] << 16) |(param->si_raw[1] << 8) | param->si_raw[0];
		param->high_si_sum = (param->si_raw[7] << 24) |(param->si_raw[6] << 16) |(param->si_raw[5] << 8) | param->si_raw[4];

		HAL_FLASH_Unlock();
		FlashErase(FLASH_START_ADDR_SCC, 0, 2);
		FlashSave(FLASH_START_ADDR_SCC, param->low_si_sum, 0);
		low_steer = *((__IO uint32_t*)(FlashRead(FLASH_START_ADDR_SCC, 0)));
		param->low_si_sum = low_steer & 0xFFFFFFFF;
		param->si_diff[0] = (uint8_t)param->low_si_sum;
		param->si_diff[1] = (uint8_t)(param->low_si_sum >> 8);
		param->si_diff[2] = (uint8_t)(param->low_si_sum >> 16);
		param->si_diff[3] = (uint8_t)(param->low_si_sum >> 24);

		FlashSave(FLASH_START_ADDR_SCC, param->high_si_sum, 1);
		high_steer = *((__IO uint32_t*)(FlashRead(FLASH_START_ADDR_SCC, 1)));
		param->high_si_sum = high_steer & 0xFFFFFFFF;
		param->si_diff[4] = (uint8_t)param->high_si_sum;
		param->si_diff[5] = (uint8_t)(param->high_si_sum >> 8);
		param->si_diff[6] = (uint8_t)(param->high_si_sum >> 16);
		param->si_diff[7] = (uint8_t)(param->high_si_sum >> 24);

		HAL_FLASH_Lock();
		param->si_write_flag = false;
		param->gnrl_2_flag = true;
	}

}

void InitSaveDataRead(struct PramVaiable *param)
{
	if(!param->read_init) {
		gnrl_2 = *((__IO uint32_t*)(FlashRead(FLASH_START_ADDR_CNT, 0)));

		low_steer = *((__IO uint32_t*)(FlashRead(FLASH_START_ADDR_SCC, 0)));
		high_steer = *((__IO uint32_t*)(FlashRead(FLASH_START_ADDR_SCC, 1)));

		param->gnrl_2_sum = gnrl_2 & 0x0000FFFF;

		param->gnrl_2_diff[0] = (uint8_t)(param->gnrl_2_sum & 0xFF);
		param->gnrl_2_diff[1] = (uint8_t)((param->gnrl_2_sum >> 8) & 0xFF);

		param->gnrl_2_raw[0] = param->gnrl_2_diff[0];
		param->gnrl_2_raw[1] = param->gnrl_2_diff[1];

		param->low_si_sum = low_steer & 0xFFFFFFFF;
		param->high_si_sum = high_steer & 0xFFFFFFFF;

		param->si_diff[0] = (uint8_t)param->low_si_sum;
		param->si_diff[1] = (uint8_t)(param->low_si_sum >> 8);
		param->si_diff[2] = (uint8_t)(param->low_si_sum >> 16);
		param->si_diff[3] = (uint8_t)(param->low_si_sum >> 24);

		param->si_raw[0] = param->si_diff[0];
		param->si_raw[1] = param->si_diff[1];
		param->si_raw[2] = param->si_diff[2];
		param->si_raw[3] = param->si_diff[3];

		param->si_diff[4] = (uint8_t)param->high_si_sum;
		param->si_diff[5] = (uint8_t)(param->high_si_sum >> 8);
		param->si_diff[6] = (uint8_t)(param->high_si_sum >> 16);
		param->si_diff[7] = (uint8_t)(param->high_si_sum >> 24);

		param->si_raw[4] = param->si_diff[4];
		param->si_raw[5] = param->si_diff[5];
		param->si_raw[6] = param->si_diff[6];
		param->si_raw[7] = param->si_diff[7];

		param->read_init = true;
	}
}
void SaveDataConvert(struct PramVaiable *param)
{

	/* GNRL1_Req */
	param->gnrl_2_req_index = param->gnrl_2_diff[0] & 0x0F;
	param->gnrl_2_req_write_flag = (param->gnrl_2_diff[0] & 0x10) >> 4;
	param->gnrl_2_req_active_flag = (param->gnrl_2_diff[0] & 0x20) >> 5;

	param->gnrl_2_req_checksum_enable = (param->gnrl_2_diff[1] & 0x01);
	param->gnrl_2_req_oper_mode = (param->gnrl_2_diff[1] & 0x02) >> 1;
	param->gnrl_2_req_comm_enable = (param->gnrl_2_diff[1] & 0x04) >> 2;


}
void SteerInterfaceParamCaliDataConvert(struct PramVaiable *param)
{
	if( param->si_flag) {
		/* S-Interface */
		param->si_req_index = param->si_diff[0] & 0x0F;
		param->si_req_write_flag = (param->si_diff[0] & 0x10) >> 4;
		param->si_req_active_flag = (param->si_diff[0] & 0x20) >> 5;

		param->si_req_lat_acc_min = ((param->si_diff[2] & 0x03) << 8) | param->si_diff[1];
		param->si_req_lat_acc_max = ((param->si_diff[3] & 0x0F) << 6) | ((param->si_diff[2] & 0xFC) >> 2);

		param->si_req_steer_angle_limit_0 = ((param->si_diff[4] & 0x3F) << 4) | ((param->si_diff[3] & 0xF0) >> 4);
		param->si_req_steer_angle_limit_1 = (param->si_diff[5] << 2) | ((param->si_diff[4] & 0xC0) >> 6);
		param->si_req_steer_angle_rate = param->si_diff[6] & 0x3F;
		param->si_flag = false;
	}

}

uint32_t FlashRead(uint32_t address, uint32_t page)
{
	uint32_t _address = address + page * 32;

	return _address;
}

void FlashErase(uint32_t address, uint32_t page, uint8_t number)
{
	uint32_t PageError = 0;
	uint32_t _page = page * 32;
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_OPTVERR);

	EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
	EraseInitStruct.Page = address + _page;
	EraseInitStruct.NbPages = number;//nb_page;//(FLASH_USER_END_ADDR - FLASH_USER_START_ADDR)/FLASH_PAGE_SIZE + 1;
    EraseInitStruct.Banks = FLASH_BANK_2;//2;//FLASH_BANK_1;


	if(HAL_FLASHEx_Erase(&EraseInitStruct, &PageError) != HAL_OK) {
		ParamErrorHandler();
	}
}

void FlashSave(uint32_t address, uint32_t data, uint32_t page)
{

	uint32_t _page = page * 32;

	__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGSERR);
	if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, address + _page, data) == HAL_OK) {

	} else {
		ParamErrorHandler();
	}
}

void ParamErrorHandler(void)
{

}

void CountLimit(uint32_t *receive, uint32_t max)
{
	if( *receive >= max ) {
		*receive = 0;
	}
}
void ParamSequnceTimer(uint8_t *timer, uint8_t init, uint8_t max) {
	if (++(*timer) == max) {
		(*timer) = init;
	}
}

void ParamCanInfo(uint16_t ID, uint32_t size, uint8_t *send, uint8_t *flag) {

	if( *flag) {
		a_can_tx_header.Identifier = ID;
		a_can_tx_header.IdType = FDCAN_STANDARD_ID;
		a_can_tx_header.TxFrameType = FDCAN_DATA_FRAME;
		a_can_tx_header.FDFormat = FDCAN_FRAME_CLASSIC;
		a_can_tx_header.DataLength = size;
		a_can_tx_header.TxEventFifoControl = FDCAN_STORE_TX_EVENTS;

		HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &a_can_tx_header, send);
		*flag = false;
	}
}

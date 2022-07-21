/*
 * ctrl_routine.c
 *
 *  Created on: Nov 11, 2021
 *      Author: okskt
 */


#include "ctrl_routine.h"


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if( htim->Instance == TIM6 ) {
		if( timer6_cmd_num == 0 ) {
			WiperSWRead(&Input_Status);

		} else if( timer6_cmd_num == 1 ) {
			WiperControl(&Wiper_Status, &Input_Status);

		} else if( timer6_cmd_num == 2 ) {
			AutoWiperControl(&Wiper_Status, &Input_Status);

		} else if( timer6_cmd_num == 3 ) {

		} else if( timer6_cmd_num == 4 ) {
			DimmerLampControl();

		} else if( timer6_cmd_num == 5 ) {
			LampControl(&Lamp_Status);

		} else if( timer6_cmd_num == 6 ) {

		} else if( timer6_cmd_num == 7 ) {
			AnalogDataReceive();

		} else if( timer6_cmd_num == 8 ) {
			WiperDataConv(wiper_tx_data, &wiper_tx_flag);
			CanTxMessage(CAN_ID_WIPERINFO, FDCAN_DLC_BYTES_8, wiper_tx_data, &wiper_tx_flag);

		} else if( timer6_cmd_num == 9 ) {
			CANReStart();
		}

		if( ++timer6_cmd_num == 10 ) {
			timer6_cmd_num = 0;
			if( (ign1_status==0) && (door_open_status==0) && (charge_door_status==0) && (rke_lock_status==0)
					&& (rke_unlock_status==0) && (stop_lamp_on==0) ) {
				if( ++stop_mode_cnt > 1500 ) {
//				if( ++stop_mode_cnt > 500 ) {
					stop_enable = 1;
					stop_mode_cnt = 1495;
//					stop_mode_cnt = 495;
				}
			} else {
				stop_mode_cnt = 0;
				stop_enable = 0;
			}
		}
	}
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
	if( hfdcan->Instance == FDCAN1 ) {
		HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &can1_rx_header, can1_rx_data);


	} else if ( hfdcan->Instance == FDCAN2 ) {
		HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &can2_rx_header, can2_rx_data);
		LocalToABU(&can2_rx_header, can2_rx_data);
		can_not_receive_cnt = 0;
	}
}

void LocalToABU(FDCAN_RxHeaderTypeDef *rxheader, uint8_t *rxdata) {
	switch(rxheader->Identifier) {
	case CAN_ID_DEBUG:
		pBrake_status = rxdata[0];
		vehicle_vel = rxdata[1];
		break;

	case CAN_ID_24VLAMPINFO:
		stop_lamp_on = (rxdata[0]&0x10) >> 4;
		break;

	case CAN_ID_12VLAMPINFO:
		taillamp_on = rxdata[0]&0x01;
		door_open_status = (rxdata[0]&0x40) >> 6;
		charge_door_status = (rxdata[0]&0x80) >> 7;
		rke_unlock_status = rxdata[0]&0x01;
		rke_lock_status = (rxdata[0]&0x02) >> 1;
		break;

	case CAN_ID_DOORINFO:

		break;

	case CAN_ID_BUTTONINFO:
		ign1_status = rxdata[0]&0x01;
		vehicle_vel = rxdata[1] | rxdata[2]<<8;
		pBrake_status = (rxdata[5]&0x38) >> 3;
		auto_mode = (rxdata[6]&0x02) >> 1;
		break;

	case CAN_ID_AKIT_CMD:
		Akit_Command.wiper_int_on = rxdata[3]&0x01;
		Akit_Command.wiper_low_on = (rxdata[3]&0x02) >> 1;
		Akit_Command.wiper_high_on = (rxdata[3]&0x04) >> 2;
		Akit_Command.wiper_off = (rxdata[3]&0x08) >> 3;
		Akit_Command.washer_on = (rxdata[3]&0x10) >> 4;
		break;
	}
}

void CanTxMessage(uint32_t id, uint32_t length, uint8_t *data, uint8_t *flag)
{
	if( *flag ) {
		can2_tx_header.Identifier = id;
		can2_tx_header.TxFrameType = FDCAN_DATA_FRAME;
		can2_tx_header.IdType = FDCAN_STANDARD_ID;
		can2_tx_header.FDFormat = FDCAN_CLASSIC_CAN;
		can2_tx_header.DataLength = length;

		HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &can2_tx_header, data);
	}

	(*flag) = 0;
}

void CANReStart()
{
	if( ++can_not_receive_cnt > 4 ) {
		if( can_not_receive_cnt == 5 ) {
			HAL_FDCAN_Stop(&hfdcan2);
		} else if( can_not_receive_cnt > 5) {
			if( HAL_FDCAN_Start(&hfdcan2) != HAL_OK ) {
				Error_Handler();
			}
			can_not_receive_cnt = 0;
		}
	}
}




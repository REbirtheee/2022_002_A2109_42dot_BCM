/*
 * ctrl_routine.c
 *
 *  Created on: Jan 4, 2022
 *      Author: okskt
 */

#include "ctrl_routine.h"


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if( htim->Instance == TIM6 ) {
		if( timer6_cmd_num == 0 ) {
			LampSWRead(&Input_Status);

		} else if( timer6_cmd_num == 1 ) {
			LampControl(&Lamp_Status, &Input_Status);

		} else if( timer6_cmd_num == 2 ) {
			ButtonControl(&Button_Status, &Input_Status);

		} else if( timer6_cmd_num == 3 ) {
			DoorControl();

		} else if( timer6_cmd_num == 4 ) {
			DistantDoorControl();
			//usdoor_control();
		} else if( timer6_cmd_num == 5 ) {
			RF_DoorControl(); // key door control

		} else if( timer6_cmd_num == 6 ) {
			CurrentDataReceive();
			CMS_Control();
		} else if( timer6_cmd_num == 7 ) {
			boardingpass_AVAS_control();
			doorwaring_AVAS_control();

		} else if( timer6_cmd_num == 8 ) {
			LampDataConv(lamp_tx_data, &lamp_tx_flag);
			CanTxMessage(CAN_ID_12VLAMPINFO, FDCAN_DLC_BYTES_8, lamp_tx_data, &lamp_tx_flag);

		} else if( timer6_cmd_num == 9 ) {
			CANReStart();

			test_tx_flag = 1 ;
			CanTx1Message(CAN_ID_TEST, FDCAN_DLC_BYTES_8, testdata, &test_tx_flag);
		}

		if( ++timer6_cmd_num == 10 ) {
			timer6_cmd_num = 0;
			/*
			 * stop mode enable condition
			 */
			if( (ign1_status==0) && (Input_Status.door_open_status==0) && (Input_Status.charge_door_status==0) && (Input_Status.RKE_lock==0)
					&& (Input_Status.RKE_unlock==0) && (stop_lamp_on==0) ) {
				if( ++stop_mode_cnt > 1500 ) {
					stop_enable = 1;
					stop_mode_cnt = 1495;
					HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, RESET); // US snesor board power off



				}
			} else {
				stop_mode_cnt = 0;
				stop_enable = 0;
				HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, SET); // US snesor board power on

			}

		if ((ign1_status==0) && (ign1_status != ign1_status_old)){
			param.gnrl_2_flag = 1;
			savedata = pBrake_status;
			GnrlParamSave(&param,savedata);

		}

		ign1_status_old = ign1_status;
		}
	}
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{


	if ( hfdcan->Instance == FDCAN2 ) {
		HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &can2_rx_header, can2_rx_data);
		LocalToABU(&can2_rx_header, can2_rx_data);
		can_not_receive_cnt = 0;
	}

	if( hfdcan->Instance == FDCAN1 ) {
			HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &can1_rx_header, can1_rx_data);
			usToABU(&can1_rx_header, can1_rx_data);

		}
}



void LocalToABU(FDCAN_RxHeaderTypeDef *rxheader, uint8_t *rxdata)
{
	switch(rxheader->Identifier) {
	case CAN_ID_DEBUG:
		pBrake_status = rxdata[0];
		vehicle_vel = rxdata[1];
		brake_status = rxdata[2];
		gear_position = rxdata[3];
		testAVAS = rxdata[4];

		if(testAVAS){
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, SET);
		}else
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, RESET);


		break;

	case CAN_ID_24VLAMPINFO:
		hazard_sw_on = rxdata[0]&0x01;
		stop_lamp_on = (rxdata[0]&0x10) >> 4;
		break;

	case CAN_ID_BUTTONINFO:
		ign1_status = rxdata[0]&0x01;
		DRV_door_open = (rxdata[0]&0x08) >> 3;
		DRV_door_close = (rxdata[0]&0x10) >> 4;
		PS_door_open = (rxdata[0]&0x20) >> 5;
		PS_door_close = (rxdata[0]&0x40) >> 6;
		vehicle_vel = rxdata[1] | (rxdata[2]<<8);
		brake_status = rxdata[5]&0x01;
		gear_position = (rxdata[5]&0x06) >> 1;
		check_pBrake_status = (rxdata[5]&0x38) >> 3;
		auto_mode = (rxdata[6]&0x02) >> 1;

		if(check_pBrake_status == 0){
		pBrake_status = romdata;
		}else{
			pBrake_status = check_pBrake_status;
		}

		break;

	case CAN_ID_SEATINFO:
		light_data = rxdata[4] | (rxdata[5]<<8);
		distant_data = rxdata[6] | (rxdata[7]<<8);
		break;

	case CAN_ID_AKIT_CMD:
		akit_command.turn_left_on = rxdata[1]&0x01;
		akit_command.turn_right_on = (rxdata[1]&0x02) >> 1;
		akit_command.tail_on = rxdata[2]&0x01;
		akit_command.head_low_on = (rxdata[2]&0x02) >> 1;
		akit_command.head_high_on = (rxdata[2]&0x04) >> 2;
		akit_command.lamp_off = (rxdata[2]&0x08) >> 3;
		akit_command.door_open = rxdata[0]&0x01;
		akit_command.door_close = (rxdata[0]&0x02) >> 1;
		if( (rxdata[0]&0x03) == 0x03 ) {
			akit_command.door_open = akit_command.door_close = 0;
		}
		break;

		/*
	case CAN_ID_USFRONTINFO:
			Us_front.us_front_head1 = rxdata[0];
			Us_front.us_front_head2 = rxdata[1];
			Us_front.us_front_id = rxdata[2];
			Us_front.us_front_ob_detect = rxdata[3];
			Us_front.us_front_ob_dist = rxdata[4];
			Us_front.us_front_cksm = rxdata[5];
			Us_front.us_front_dist_lpf = rxdata[6];
			Us_front.us_front_dist_cnt = rxdata[7];

			//test_tx_flag = 1;
			//CanTxMessage(CAN_ID_TEST, FDCAN_DLC_BYTES_8, rxdata, &test_tx_flag);
			break;

		case CAN_ID_USREARINFO:
			Us_rear.us_rear_head1 = rxdata[0];
			Us_rear.us_rear_head2 = rxdata[1];
			Us_rear.us_rear_id = rxdata[2];
			Us_rear.us_rear_ob_detect = rxdata[3];
			Us_rear.us_rear_ob_dist = rxdata[4];
			Us_rear.us_rear_cksm = rxdata[5];
			Us_rear.us_rear_dist_lpf = rxdata[6];
			Us_rear.us_rear_dist_cnt = rxdata[7];
			break;
		 	*/

	}
}

void usToABU(FDCAN_RxHeaderTypeDef *rxheader, uint8_t *rxdata)
{
	switch(rxheader->Identifier) {

	case CAN_ID_USFRONTINFO:
		Us_front.us_front_head1 = rxdata[0];
		Us_front.us_front_head2 = rxdata[1];
		Us_front.us_front_id = rxdata[2];
		Us_front.us_front_ob_detect = rxdata[3];
		Us_front.us_front_ob_dist = rxdata[4];
		Us_front.us_front_cksm = rxdata[5];
		Us_front.us_front_dist_lpf = rxdata[6];
		Us_front.us_front_dist_cnt = rxdata[7];

		break;

	case CAN_ID_USREARINFO:
		Us_rear.us_rear_head1 = rxdata[0];
		Us_rear.us_rear_head2 = rxdata[1];
		Us_rear.us_rear_id = rxdata[2];
		Us_rear.us_rear_ob_detect = rxdata[3];
		Us_rear.us_rear_ob_dist = rxdata[4];
		Us_rear.us_rear_cksm = rxdata[5];
		Us_rear.us_rear_dist_lpf = rxdata[6];
		Us_rear.us_rear_dist_cnt = rxdata[7];

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

void CanTx1Message(uint32_t id, uint32_t length, uint8_t *data, uint8_t *flag)
{
	if( *flag ) {
		can1_tx_header.Identifier = id;
		can1_tx_header.TxFrameType = FDCAN_DATA_FRAME;
		can1_tx_header.IdType = FDCAN_STANDARD_ID;
		can1_tx_header.FDFormat = FDCAN_CLASSIC_CAN;
		can1_tx_header.DataLength = length;

		HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &can1_tx_header, data);
	}

	(*flag) = 0;
}


void CANReStart()
{
	if( ++can_not_receive_cnt > 4 ) {
		if( can_not_receive_cnt == 5 ) {
			HAL_FDCAN_Stop(&hfdcan2);
		}
		if( can_not_receive_cnt > 5) {
			if( HAL_FDCAN_Start(&hfdcan2) != HAL_OK ) {
				Error_Handler();
			}
			can_not_receive_cnt = 0;
		}
	}
}





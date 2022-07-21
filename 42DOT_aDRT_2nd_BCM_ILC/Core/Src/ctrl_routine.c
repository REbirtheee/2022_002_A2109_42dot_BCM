/*
 * ctrl_routine.c
 *
 *  Created on: Nov 11, 2021
 *      Author: okskt
 */


#include "ctrl_routine.h"
#include "string.h"


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if( htim->Instance == TIM6 ) {
		if( timer6_cmd_num == 0 ) {
			ButtonRead();		// Read Switch

		} else if( timer6_cmd_num == 1 ) {
			ButtonConrtrol();


		} else if( timer6_cmd_num == 2 ) {
			ButtonDataConv(button_tx_data, &button_tx_flag);
			LocalTxMessage(CAN_ID_ILCINFO, 8, button_tx_data, &button_tx_flag);

		} else if( timer6_cmd_num == 3 ) {
			AkitCmdConv(ak2bc_tx_data, ak2bc_rx_data, &ak2bc_tx_flag);
			LocalTxMessage(CAN_ID_AKIT_CMD, 8, ak2bc_tx_data, &ak2bc_tx_flag);

		} else if( timer6_cmd_num == 4 ) {

			BCMDataConv(bcm_tx_data, &bcm_tx_flag);
			LLCTxMessage(CAN_ID_BCMINFO, 8, bcm_tx_data, &bcm_tx_flag);


		} else if( timer6_cmd_num == 5 ) {
			if( ++cc2ic_tx_cnt > 49 ) {
				if( cluster_can_tx_flag == 1 ) {
					CC2ICDataConv(cc2ic_tx_data, &cc2ic_tx_flag);
					ULCTxMessage(CAN_ID_CC2ICINFO, 8, cc2ic_tx_data, &cc2ic_tx_flag);
				}
				cc2ic_tx_cnt = 0;
			}


		} else if( timer6_cmd_num == 6 ) {
			if( ++sm2ic_tx_cnt > 4 ) {
				if( cluster_can_tx_flag == 1 ) {
					SM2ICDataConv(sm2ic_tx_data, &sm2ic_tx_flag);
					ULCTxMessage(CAN_ID_SM2ICINFO, 8, sm2ic_tx_data, &sm2ic_tx_flag);
				}
				sm2ic_tx_cnt = 0;
			}


		} else if( timer6_cmd_num == 7 ) {
			if( ++dtc_tx_cnt > 4 ) {
				if( cluster_can_tx_flag == 1 ) {
					DTCDataConv(dtc_tx_data, &dtc_tx_flag);
					ULCTxMessage(CAN_ID_DTCINFO, 8, dtc_tx_data, &dtc_tx_flag);
				}
				dtc_tx_cnt = 0;
			}


		} else if( timer6_cmd_num == 8 ) {
			if( ++bc2ic_tx_cnt > 4 ) {
				if( cluster_can_tx_flag == 1 ) {
					BC2ICDataConv(bc2ic_tx_data, &bc2ic_tx_flag);
					ULCTxMessage(CAN_ID_BC2ICINFO, 8, bc2ic_tx_data, &bc2ic_tx_flag);
				}
				bc2ic_tx_cnt = 0;
			}

		} else if( timer6_cmd_num == 9 ) {
//			CANReStart();
			// stop mode wake-up condition
			if( (Button_Status.ign1_status==1) || (door_open_status==1) || (charge_door_status==1) || (rke_lock_staus==1)
					|| (rke_unlock_staus==1) || (stop_lamp_on==1) ) {
				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, SET);
			} else if( (Button_Status.ign1_status==0) && (door_open_status==0) && (charge_door_status==0) && (rke_unlock_staus==0)
					&& (rke_lock_staus==0) && (stop_lamp_on==0) ) {
				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, RESET);
			}

		}

		if( ++timer6_cmd_num == 10 ) {
			timer6_cmd_num = 0;
			// Stop mode enable condition
			if( (Button_Status.ign1_status==0) && (door_open_status==0) && (charge_door_status==0) && (rke_unlock_staus==0)
					&& (rke_lock_staus==0) && (stop_lamp_on==0) ) {
				if( ++stop_mode_cnt > 1500 ) {	// wait until 30s
					stop_enable = 1;
					stop_mode_cnt = 1495;
					cluster_can_tx_flag = 0;
				}
			} else {
				stop_mode_cnt = 0;
				stop_enable = 0;
				cluster_can_tx_flag = 1;
			}


		}
#if 0
		//220628LHS
		if( timer6_cmd_num_100ms == 10 ){
			EpbCanInfo0x47F();
		}  else if( timer6_cmd_num_100ms == 20 ){
			EpbCanInfo0x470();
		} else if( timer6_cmd_num_100ms == 30 ){
			EpbCanInfo0x507();
		} else if( timer6_cmd_num_100ms == 11 ){

		}

		if(++timer6_cmd_num_100ms == 50){
			timer6_cmd_num_100ms = 0;
		}

		if( timer6_cmd_num_1000ms == 10 ){
			EpbCanInfo0x386();
		} else if( timer6_cmd_num_1000ms == 20 ){
			EpbCanInfo0x5B0();
		}

		if(++timer6_cmd_num_1000ms == 501){
			timer6_cmd_num_1000ms = 0;
		}
#endif

	}
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	if( hcan->Instance == CAN1 ) {
		HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &can1_rx_header, can1_rx_data);
		ClusterToILC(&can1_rx_header, can1_rx_data);

	} else if ( hcan->Instance == CAN2 ) {
		HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &can2_rx_header, can2_rx_data);
		LocalToILC(&can2_rx_header, can2_rx_data);
		can_not_receive_cnt = 0;

	} else if( hcan->Instance == CAN3 ) {
		HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &can3_rx_header, can3_rx_data);
		LLCToILC(&can3_rx_header, can3_rx_data);
	}
}

void ClusterToILC(CAN_RxHeaderTypeDef *rxheader, uint8_t *rxdata) {
	switch(rxheader->ExtId) {
	case CAN_ID_AK2BC:
		memcpy(ak2bc_rx_data, rxdata, 8);
		break;
	}
}

void LocalToILC(CAN_RxHeaderTypeDef *rxheader, uint8_t *rxdata) {
	switch(rxheader->StdId) {
	case CAN_ID_LAMPINFO:
		stop_lamp_on = (rxdata[0]&0x10) >> 4;
		head_low_lamp_fail = rxdata[5]&0x01;
		stop_lamp_fail = (rxdata[5]&0x02) >> 1;
		break;

	case CAN_ID_LAMPINFO2:
		tail_lamp_status = rxdata[0]&0x01;
		hazard_lamp_status = (rxdata[0]&0x02) >> 1;
		turn_lamp_left_status = (rxdata[0]&0x04) >> 2;
		turn_lamp_right_status = (rxdata[0]&0x08) >> 3;
		head_lamp_low_status = (rxdata[0]&0x10) >> 4;
		head_lamp_high_status = (rxdata[0]&0x20) >> 5;
		door_open_status = (rxdata[0]&0x40) >> 6;
		charge_door_status = (rxdata[0]&0x80) >> 7;
		rke_unlock_staus = rxdata[1]&0x01;
		rke_lock_staus = (rxdata[1]&0x02) >> 1;
		turn_left_lamp_fail = rxdata[6]&0x01;
		turn_right_lamp_fail = (rxdata[6]&0x02) >> 1;
		Button_Status.accel_decel_sw = rxdata[7]&0x03;
		Button_Status.drvie_mode_sw = (rxdata[7]&0x0C) >> 2;
		Button_Status.logging_sw = (rxdata[7]&0x30) >> 4;
		Button_Status.marker_sw = (rxdata[7]&0xC0) >> 6;
		break;

	case CAN_ID_DOORINFO:
		seatbelt_warning_status = rxdata[0]&0xFF;
		seat_status = rxdata[1];
		break;

	case CAN_ID_CARINFO:
		regenerative_brake_value = rxdata[0];
		if( regenerative_brake_value == 0 ) {
			regenerative_brake_mode = 0;
		} else {
			regenerative_brake_mode = 1;
		}
#ifndef DEBUG_TEST
		soc_value = rxdata[1];
		charge_mode = rxdata[2];
		car_ready_status = rxdata[3];
#endif
		break;
	}
}

void LLCToILC(CAN_RxHeaderTypeDef *rxheader, uint8_t *rxdata) {
	switch(rxheader->StdId) {
	case CAN_ID_LLCINFO:
		auto_mode = rxdata[1] & 0x01;
		ems_status = (rxdata[1]&0x02) >> 1;
		steer_fail_warning = (rxdata[1]&0xF8) >> 3;
		vehicle_vel = rxdata[2] | (rxdata[3]<<8);
		vehicle_dec = rxdata[4] | (rxdata[5]<<8);
		brake_stauts = rxdata[6]&0x01;
		shift_status = (rxdata[6]&0x06) >> 1;
		epb_status = (rxdata[6]&0x38) >> 3;
#ifdef DEBUG_TEST		// for simulator test
		soc_value = rxdata[0];
		charge_mode = rxdata[7]&0x03;
		car_ready_status = (rxdata[7]&0x04) >> 2;
#endif
		break;
	}
}

void ULCTxMessage(uint32_t id, uint8_t length, uint8_t *data, uint8_t *flag)
{
	if( *flag ) {
		can1_tx_header.ExtId = id;
		can1_tx_header.RTR = CAN_RTR_DATA;
		can1_tx_header.IDE = CAN_ID_EXT;
		can1_tx_header.DLC = length;

		can1_tx_mailbox = HAL_CAN_GetTxMailboxesFreeLevel(&hcan1);
		HAL_CAN_AddTxMessage(&hcan1, &can1_tx_header, data, &can1_tx_mailbox);

		if( can1_tx_header.ExtId == CAN_ID_BC2ICINFO ) {
			if( cluster_trip_status_tx != 0) {
				cluster_trip_status_tx = 0;
			}
		}
	}

	(*flag) = 0;
}

void LocalTxMessage(uint32_t id, uint8_t length, uint8_t *data, uint8_t *flag)
{
	if( *flag ) {
		can2_tx_header.StdId = id;
		can2_tx_header.RTR = CAN_RTR_DATA;
		can2_tx_header.IDE = CAN_ID_STD;
		can2_tx_header.DLC = length;

		can2_tx_mailbox = HAL_CAN_GetTxMailboxesFreeLevel(&hcan2);
		HAL_CAN_AddTxMessage(&hcan2, &can2_tx_header, data, &can2_tx_mailbox);
	}

	(*flag) = 0;
}

void LLCTxMessage(uint32_t id, uint8_t length, uint8_t *data, uint8_t *flag)
{
	if( *flag ) {
		can3_tx_header.StdId = id;
		can3_tx_header.RTR = CAN_RTR_DATA;
		can3_tx_header.IDE = CAN_ID_STD;
		can3_tx_header.DLC = length;

		can3_tx_mailbox = HAL_CAN_GetTxMailboxesFreeLevel(&hcan3);
		HAL_CAN_AddTxMessage(&hcan3, &can3_tx_header, data, &can3_tx_mailbox);
	}

	(*flag) = 0;
}

void BC2ICDataConv(uint8_t *txdata, uint8_t *txflag)
{
	if( epb_status == 2 ) {
		pbrake_status = 2;
	} else if( epb_status == 1 ) {
		pbrake_status = 1;
	} else {
		pbrake_status = 0;
	}

	if( shift_status == 1 ) {
		gear_status = 2;
	} else if( shift_status == 2 ) {
		gear_status = 3;
		if( epb_status == 1 ) {
			gear_status = 1;
		}
	} else if( shift_status == 3 ) {
		gear_status = 4;
	} else {
		gear_status = 0;
	}


	txdata[0] = charge_mode | (bcm_fail_status<<2) | (head_lamp_low_status<<4) | (door_open_status<<6);
	txdata[1] = charge_door_status | (regenerative_brake_mode<<2) | (Input_Status.autonomous<<4) | (auto_mode<<5) | (head_lamp_high_status<<6);
	txdata[2] = seatbelt_warning_status;
	txdata[3] = turn_lamp_right_status | (turn_lamp_left_status<<2) | (hazard_lamp_status<<6);
	txdata[4] = gear_status | (pbrake_status<<4);
	txdata[5] = seat_status;
	txdata[6] = brake_stauts | (tail_lamp_status<<2) | (cluster_trip_status_tx<<4) | (car_ready_status<<6);
	txdata[7] = (Button_Status.accel_decel_sw) | (Button_Status.drvie_mode_sw<<2) | (Button_Status.logging_sw<<4) | (Button_Status.marker_sw<<6);
	//txdata[7] = cluster_trip_push_cnt;
	*txflag = 1;
}

void DTCDataConv(uint8_t *txdata, uint8_t *txflag)
{
	if( head_low_lamp_fail || stop_lamp_fail || turn_left_lamp_fail || turn_right_lamp_fail ) {
		bcm_fail_status = 1;
	} else {
		bcm_fail_status = 0;
	}

	Steer_Error_Status.system_interal_fail = (steer_fail_warning&0x01) | ((steer_fail_warning&0x04)>>2);
	Steer_Error_Status.system_degrade = (steer_fail_warning&0x02) >> 1;
	Steer_Error_Status.communication_fail = (steer_fail_warning&0x08) >> 3;


	if( bcm_fail_status == 1 ) {
		txdata[0] = (head_low_lamp_fail) | (turn_right_lamp_fail<<1) | (turn_left_lamp_fail<<2) | (stop_lamp_fail<<3);
		txdata[3] = (Steer_Error_Status.system_interal_fail) | (Steer_Error_Status.system_degrade<<1) | (Steer_Error_Status.communication_fail<<2);

		*txflag = 1;
	}
}

void SM2ICDataConv(uint8_t *txdata, uint8_t *txflag)
{
	uint16_t vehicle_vel_i;
	float vehicle_vel_f;

	if( vehicle_vel < 0 ) {
		vehicle_vel = vehicle_vel * (-1);
	}

	vehicle_vel_f = (float)vehicle_vel;
	vehicle_vel_f = vehicle_vel_f / 10;
	vehicle_vel_f = vehicle_vel_f * 256;
	vehicle_vel_i = (uint16_t)vehicle_vel_f;


	// 220623 LHS

	txdata[0] = vehicle_vel_i;
	txdata[1] = vehicle_vel_i>>8;

	/*
	if(car_ready_status == 1){
		txdata[0] = vehicle_vel_i;
		txdata[1] = vehicle_vel_i>>8;
	}else{
		txdata[0] = 0;
		txdata[1] = 0;
	}
	*/
	*txflag = 1;
}

void CC2ICDataConv(uint8_t *txdata, uint8_t *txflag)
{
	uint8_t soc_value_i;
	float soc_value_f;
	uint8_t regenerative_brake_value_i;
	float regenerative_brake_value_f;

	soc_value_f = (float)soc_value;
	soc_value_f = soc_value_f/0.4;
	soc_value_f = (soc_value_f+1);
	soc_value_i = (uint8_t)soc_value_f;

	regenerative_brake_value_f = (float)regenerative_brake_value;
	regenerative_brake_value_f = regenerative_brake_value_f/0.4;
	regenerative_brake_value_i = (uint8_t)regenerative_brake_value_f;

	txdata[0] = soc_value_i;
	txdata[1] = regenerative_brake_value_i;

	*txflag = 1;
}

void BCMDataConv(uint8_t *txdata, uint8_t *txflag)
{
	if( ++alive_cnt > 15 ) {
		alive_cnt = 0;
	}

	bcm_checksum = (alive_cnt + Input_Status.autonomous + auto_mode + ems_status) & 0xF;

	txdata[0] = bcm_checksum | (alive_cnt<<4);
	txdata[1] = Input_Status.autonomous | (ems_status<<1) | (auto_mode<<2);

	*txflag = 1;
}

void AkitCmdConv(uint8_t *txdata, uint8_t *rxdata, uint8_t *txflag)
{
	memcpy(txdata, rxdata, 8);

	*txflag = 1;
}

void CANReStart()
{
	if( ++can_not_receive_cnt >= 4 ) {
		if( can_not_receive_cnt == 5 ) {
			HAL_CAN_Stop(&hcan2);
		}
		if( can_not_receive_cnt > 5) {
			if( HAL_CAN_Start(&hcan2) != HAL_OK ) {
				Error_Handler();
			}
			can_not_receive_cnt = 0;
		}
	}
}




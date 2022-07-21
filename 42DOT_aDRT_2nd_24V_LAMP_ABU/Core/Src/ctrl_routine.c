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
			//WelcomeControl(); // 220707 aDRT_3 sonng oper -> welcome off
			LampControl(&Lamp_Status, &Input_Status);

		} else if( timer6_cmd_num == 2 ) {
			CurrentDataReceive();

		} else if( timer6_cmd_num == 3 ) {
			VehicleDataConv(vehicle_tx_data, &vehicle_tx_flag, &vehicle_curr_rx_flag);
			CanTxMessage(CAN_ID_VEHICLEINFO, FDCAN_DLC_BYTES_8, vehicle_tx_data, &vehicle_tx_flag);

		} else if( timer6_cmd_num == 4 ) {
			LampDataConv(lamp_tx_data, &lamp_tx_flag);
			CanTxMessage(CAN_ID_24VLAMPINFO, FDCAN_DLC_BYTES_8, lamp_tx_data, &lamp_tx_flag);

		} else if( timer6_cmd_num == 5 ) {
			SeatBeltRead();

		} else if( timer6_cmd_num == 6 ) {

		} else if( timer6_cmd_num == 7 ) {
			DoorWarningControl();
			PBrakeWarningControl();

		} else if( timer6_cmd_num == 8 ) {
			SeatbeltWarningControl();
			WarningAlarmControl();

		} else if( timer6_cmd_num == 9 ) {
			seatbelt_tx_flag = 1;
			SeatbeltDataConv(seatbelt_tx_data, &seatbelt_tx_flag);
			CanTxMessage(CAN_ID_SEATINFO, FDCAN_DLC_BYTES_8, seatbelt_tx_data, &seatbelt_tx_flag);
			CANReStart();
		}

		if( ++timer6_cmd_num == 10 ) {
			timer6_cmd_num = 0;
			if( (ign1_status==0) && (door_open_status==0) && (charge_door_status==0) && (RF_open_lamp==0)
					&& (RF_close_lamp==0) && (Lamp_Status.stop_lamp==0) ) {
				if( ++stop_mode_cnt > 1500 ) {
					stop_enable = 1;
					stop_mode_cnt = 1495;
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
		VehicleToABU(&can1_rx_header, can1_rx_data);

	} else if ( hfdcan->Instance == FDCAN2 ) {
		HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &can2_rx_header, can2_rx_data);
		LocalToABU(&can2_rx_header, can2_rx_data);
		can_not_receive_cnt = 0;
	}
}

void VehicleToABU(FDCAN_RxHeaderTypeDef *rxheader, uint8_t *rxdata)
{
	switch(rxheader->Identifier) {
	case 0x08F200A0:
		//car_ready_status = rxdata[0] & 0x01;
		//220524 LHS
		if(rxdata[0] == 0x01 && ((rxdata[1] == 0x20) || (rxdata[1] == 0x21))){
			car_ready_status = 1;
		}else{
			car_ready_status = 0;
		}

		break;

	case 0x0CFF08EF:
		vehicle_current = (rxdata[4]) | (rxdata[5]<<8);
		vehicle_curr_rx_flag = 1;
		break;

	case 0x1810A6A0:
		soc_value = rxdata[0];
		vehicle_voltage = rxdata[3] | (rxdata[4]<<8);
		break;
#if 0
	case 0x18FFA1F3:
		vehicle_charge_status = rxdata[0];
		break;
#endif
	case 0x18FEA0F0:
		vehicle_charge_status = rxdata[0];
		break;
	}
}

void LocalToABU(FDCAN_RxHeaderTypeDef *rxheader, uint8_t *rxdata) {
	switch(rxheader->Identifier) {
	case CAN_ID_12VLAMPINFO:
		lamp_rx_data[0] = rxdata[0];
		if( (lamp_rx_data[0]&0x01) == 0x01 ) {
			taillamp_on = 1;
		} else if( (lamp_rx_data[0]&0x01) == 0x00 ) {
			taillamp_on = 0;
		}
		if( (lamp_rx_data[0]&0x10) == 0x10 ) {
			headlamp_low_on = 1;
		} else if( (lamp_rx_data[0]&0x10) == 0x00 ){
			headlamp_low_on = 0;
		}
		if( (lamp_rx_data[0]&0x04) == 0x04 ) {
			turnlamp_left_on = 1;
		} else if( (lamp_rx_data[0]&0x04) == 0x00 ){
			turnlamp_left_on = 0;
		}
		if( (lamp_rx_data[0]&0x08) == 0x08 ) {
			turnlamp_right_on = 1;
		} else if( (lamp_rx_data[0]&0x08) == 0x00 ){
			turnlamp_right_on = 0;
		}
		door_open_status = (rxdata[0]&0x40) >> 6;
		charge_door_status = (rxdata[0]&0x80) >> 7;
		RF_open_lamp = rxdata[1]&0x01;
		RF_close_lamp = (rxdata[1]&0x02) >> 1;
		break;

	case CAN_ID_WIPERINFO:
		dimmer_volt = rxdata[3] | (rxdata[4]<<8);
		break;

	case CAN_ID_BUTTONINFO:
		ign1_status = rxdata[0]&0x01;
		ign2_status = (rxdata[0]&0x80)>>7;
		vehicle_vel = rxdata[1] | (rxdata[2]<<8);
		brake_status = rxdata[5] & 0x01;
		pBrake_status = (rxdata[5]&0x38) >> 3;
		ems_status = rxdata[6] & 0x01;
		auto_mode = (rxdata[6]&0x02) >> 1;
		break;

	case CAN_ID_AKIT_CMD:
		akit_command.hazard_cmd = (rxdata[1]&0x10) >> 4;
		break;

	case CAN_ID_DEBUG:
		pBrake_status = rxdata[0];
		vehicle_vel = rxdata[1];
		brake_status = rxdata[2];
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

void VehicleDataConv(uint8_t *txdata, uint8_t *flag, uint8_t *curr_rx_flag)
{
	static int16_t vehicle_current_i16 = 0;

	if( *curr_rx_flag == 1 ) {
		vehicle_current_i16 = vehicle_current;
		vehicle_current_i16 = (vehicle_current_i16 - 5000) * (-1);
		if( vehicle_current_i16 < 0 ) {
			vehicle_current_i16 = 0;
		} else if( vehicle_current_i16 > 100 ) {
			vehicle_current_i16 = 100;
		}
	}

#if 0
	if( charge_door_status == 0 ) {
		charge_mode = 0;
	} else if( charge_door_status == 1 ) {
		if( vehicle_charge_status == 0x10 ) {
			charge_mode = 1;
		} else if( vehicle_charge_status == 0x20 ) {
			charge_mode = 2;
			if( soc_value == 100 ) {
				charge_mode = 3;
			}
		}
	}
#endif
	if( charge_door_status == 0 ) {
		charge_mode = 0;
	} else if( charge_door_status == 1 ) {
		if( ign1_status == 0 ) {
			if( vehicle_charge_status == 0x03 ) {
				charge_mode = 1;
			} else if( vehicle_charge_status == 0x01 ) {
				charge_mode = 2;
				if( soc_value == 100 ) {
					charge_mode = 3;
				}
			}
		} else if( ign1_status == 1 ) {
			charge_mode = 0;
		}
	}

	txdata[0] = vehicle_current_i16;
	txdata[1] = soc_value;
	txdata[2] = charge_mode;
	txdata[3] = car_ready_status;
	txdata[4] = vehicle_voltage;
	txdata[5] = vehicle_voltage >> 8;

	(*flag) = 1;
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



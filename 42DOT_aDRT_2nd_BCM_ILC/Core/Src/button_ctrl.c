/*
 * button_ctrl.c
 *
 *  Created on: Nov 17, 2021
 *      Author: okskt
 */


#include "button_ctrl.h"


void ButtonRead()
{
	Input_Status_Raw.ign1_status = READ_PIN_DI_0;
	if( Button_Status.ign1_status == 1 ) {
		Input_Status_Raw.defogger = READ_PIN_DI_2;
	}
	Input_Status_Raw.autonomous = READ_PIN_DI_3;
	Input_Status_Raw.ps_close = READ_PIN_DI_4;
	Input_Status_Raw.ps_open = READ_PIN_DI_8;
	Input_Status_Raw.drv_close = READ_PIN_DI_6;
	Input_Status_Raw.drv_open = READ_PIN_DI_7;
	Input_Status_Raw.ign2_status = READ_PIN_DI_9;
	Input_Status_Raw.cluster_trip = READ_PIN_DI_5;

	ButtonSWChatt();		// Switch Chattering Function

	Input_Status_Raw_Prev.ign1_status = Input_Status_Raw.ign1_status;
	Input_Status_Raw_Prev.defogger = Input_Status_Raw.defogger;
	Input_Status_Raw_Prev.autonomous = Input_Status_Raw.autonomous;
	Input_Status_Raw_Prev.ps_close = Input_Status_Raw.ps_close;
	Input_Status_Raw_Prev.ps_open = Input_Status_Raw.ps_open;
	Input_Status_Raw_Prev.drv_close = Input_Status_Raw.drv_close;
	Input_Status_Raw_Prev.drv_open = Input_Status_Raw.drv_open;
	Input_Status_Raw_Prev.ign2_status = Input_Status_Raw.ign2_status;
	Input_Status_Raw_Prev.cluster_trip = Input_Status_Raw.cluster_trip;
}

void ButtonSWChatt()
{
	if( Input_Status_Raw_Prev.ign1_status == Input_Status_Raw.ign1_status ) {
		if( ++Chattering.ign1_status_cnt > 2 ) {
			Input_Status.ign1_status = Input_Status_Raw.ign1_status;
			Chattering.ign1_status_cnt = 3;
		}
	} else {
		Chattering.ign1_status_cnt = 0;
	}

	if( Input_Status_Raw_Prev.defogger == Input_Status_Raw.defogger ) {
		if( ++Chattering.defogger_cnt > 2 ) {
			Input_Status.defogger = Input_Status_Raw.defogger;
			if( (Input_Status_Prev.defogger==0) && (Input_Status.defogger==1) ) {
				if( Button_Status.defogger == 0 ) {
					defogger_sw = 1;
				} else if( Button_Status.defogger == 1 ) {
					defogger_sw = 0;
				}
			}
			Input_Status_Prev.defogger = Input_Status.defogger;
			Chattering.defogger_cnt = 3;
		}
	} else {
		Chattering.defogger_cnt = 0;
	}


	if( Input_Status_Raw_Prev.ps_close == Input_Status_Raw.ps_close ) {
		if( ++Chattering.ps_close_cnt > 2 ) {
			Input_Status.ps_close = Input_Status_Raw.ps_close;
			Chattering.ps_close_cnt = 3;
		}
	} else {
		Chattering.ps_close_cnt = 0;
	}

	if( Input_Status_Raw_Prev.ps_open == Input_Status_Raw.ps_open ) {
		if( ++Chattering.ps_open_cnt > 2 ) {
			Input_Status.ps_open = Input_Status_Raw.ps_open;
			Chattering.ps_open_cnt = 3;
		}
	} else {
		Chattering.ps_open_cnt = 0;
	}

	if( Input_Status_Raw_Prev.drv_close == Input_Status_Raw.drv_close ) {
		if( ++Chattering.drv_close_cnt > 2 ) {
			Input_Status.drv_close = Input_Status_Raw.drv_close;
			Chattering.drv_close_cnt = 3;
		}
	} else {
		Chattering.drv_close_cnt = 0;
	}


	if( Input_Status_Raw_Prev.drv_open == Input_Status_Raw.drv_open ) {
		if( ++Chattering.drv_open_cnt > 2 ) {
			Input_Status.drv_open = Input_Status_Raw.drv_open;
			Chattering.drv_open_cnt = 3;
		}
	} else {
		Chattering.drv_open_cnt = 0;
	}

	if( Input_Status_Raw_Prev.ign2_status == Input_Status_Raw.ign2_status ) {
		if( ++Chattering.ign2_status_cnt > 2 ) {
			Input_Status.ign2_status = Input_Status_Raw.ign2_status;
			Chattering.ign2_status_cnt = 3;
		}
	} else {
		Chattering.ign2_status_cnt = 0;
	}

	if( Input_Status_Raw_Prev.autonomous == Input_Status_Raw.autonomous ) {
		if( ++Chattering.autonomous_cnt > 2 ) {
			Input_Status.autonomous = Input_Status_Raw.autonomous;
			Chattering.autonomous_cnt = 3;
		}
	} else {
		Chattering.autonomous_cnt = 0;
	}

	if( Input_Status_Raw_Prev.cluster_trip == Input_Status_Raw.cluster_trip ) {
		if( ++Chattering.cluster_trip_cnt > 2 ) {
			Input_Status.cluster_trip = Input_Status_Raw.cluster_trip;
			Chattering.cluster_trip_cnt = 3;
		}
	} else {
		Chattering.cluster_trip_cnt = 0;
	}
}

void ButtonConrtrol()
{
	//static uint8_t cluster_trip_status_prev = 0;
	//static uint16_t cluster_trip_push_cnt = 0;


	if( Input_Status.ign1_status == 1 )  {
		Button_Status.ign1_status = 1;
	} else if( Input_Status.ign1_status == 0 ) {
		Button_Status.ign1_status = 0;
	}

	if( Input_Status.ign2_status == 1 ) {
		Button_Status.ign2_status = 1;
	} else if( Input_Status.ign2_status == 0 ) {
		Button_Status.ign2_status = 0;
	}

	/*
	 * Defogger Control
	 */
	if( Button_Status.ign1_status == 1 ) {
		if( defogger_sw == 1 ) {
			Button_Status.defogger = 1;
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2 | GPIO_PIN_3, SET);			// defogger sig/lamp on
			if( ++deffoger_cnt > 30000 ) {									// on until 15m
				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2| GPIO_PIN_3, RESET);	// defogger sig/lamp off
				Button_Status.defogger = 0;
				defogger_sw = 0;
				deffoger_cnt = 0;
			}
		} else if( defogger_sw == 0) {
			Button_Status.defogger = 0;
			deffoger_cnt = 0;
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2 | GPIO_PIN_3, RESET);		// defogger sig/lamp off
		}
	} else if( Button_Status.ign1_status == 0 ) {
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2 | GPIO_PIN_3, RESET);			// defogger sig/lamp off
		Button_Status.defogger = 0;
		defogger_sw = 0;
		deffoger_cnt = 0;
	}

	if( Input_Status.drv_open == 1 ) {
		Button_Status.drv_open = 1;
//		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, SET);		//  sw lamp on
	} else if( Input_Status.drv_open == 0 ) {
		Button_Status.drv_open = 0;
//		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, RESET);	//  sw lamp on
	}

	if( Input_Status.drv_close == 1 ) {
		Button_Status.drv_close = 1;
//		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_6, SET);		//  sw lamp on
	} else if( Input_Status.drv_close == 0 ) {
		Button_Status.drv_close = 0;
//		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_6, RESET);	//  sw lamp on
	}

	if( Input_Status.ps_open == 1 ) {
		Button_Status.ps_open = 1;
//		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5, SET);		//  sw lamp on
	} else if( Input_Status.ps_open == 0 ) {
		Button_Status.ps_open = 0;
//		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5, RESET);	//  sw lamp on
	}

	if( Input_Status.ps_close == 1 ) {
		Button_Status.ps_close = 1;
//		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, SET);		//  sw lamp on
	} else if( Input_Status.ps_close == 0 ) {
		Button_Status.ps_close = 0;
//		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, RESET);	//  sw lamp on
	}

	if( autonomous_sw == 1 ) {
		Button_Status.autonomous = 1;
	} else if( autonomous_sw == 0 ) {
		Button_Status.autonomous = 0;
	}

	if( auto_mode == 1 ) {
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, SET);// autonomous button lamp on // 220419 only use LED 1

	} else if( auto_mode == 0 ) {
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, RESET);	// autonomous button lamp off
\
	}

	/*
	 * cluster trip button function
	 * short press(~2s) : 0x01, long press(2s~) : 0x02
	 */
	/*
	if( Input_Status.cluster_trip == 0 ) {
		if( ++cluster_trip_push_cnt > 100 ) {		// if press over 2s
			cluster_trip_push_cnt = 101;
			cluster_trip_status = 2;
			if( (cluster_trip_status_prev==1) && (cluster_trip_status==2) ) {
				cluster_trip_status_tx = cluster_trip_status;
			}
		} else if( cluster_trip_push_cnt <= 100 ) {
			cluster_trip_status = 1;
		}
	} else if( Input_Status.cluster_trip == 1 ) {
		if( cluster_trip_status == 1 ) {
			cluster_trip_status_tx = cluster_trip_status;
		}
		cluster_trip_status = cluster_trip_push_cnt = 0;
	}
	cluster_trip_status_prev = cluster_trip_status;
	*/

	if( Input_Status.cluster_trip == 1 ){
			cluster_trip_push_cnt++;
			cluster_trip_check_cnt = 0;
	}else if( Input_Status.cluster_trip == 0 ){


		if( cluster_trip_push_cnt == 0){
				cluster_trip_status_tx = 0;
		}else if( cluster_trip_push_cnt >= 5 && cluster_trip_push_cnt < 25){
				cluster_trip_status_tx = 1;
		}else if( cluster_trip_push_cnt >= 25){
				cluster_trip_status_tx = 2;
		}

		if( cluster_trip_check_cnt >= 3 ){
			cluster_trip_push_cnt = 0;
			cluster_trip_check_cnt = 10;
		}else{
			cluster_trip_check_cnt++;
		}

	}


}


void ButtonDataConv(uint8_t *txdata, uint8_t *flag)
{
	txdata[0] = Button_Status.ign1_status | (Button_Status.defogger<<2) | (Button_Status.drv_open<<3) |
			(Button_Status.drv_close<<4) | (Button_Status.ps_open<<5) | (Button_Status.ps_close<<6) | (Button_Status.ign2_status<<7);
	txdata[1] = vehicle_vel;
	txdata[2] = vehicle_vel >> 8;
	txdata[3] = vehicle_dec;
	txdata[4] = vehicle_dec >> 8;
	txdata[5] = brake_stauts | (shift_status<<1) | (epb_status<<3);
	txdata[6] = ems_status | (auto_mode<<1);

	(*flag) = 1;
}




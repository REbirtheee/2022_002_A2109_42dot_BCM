/*
 * door_ctrl.h
 *
 *  Created on: Nov 18, 2021
 *      Author: okskt
 */

#ifndef INC_DOOR_CTRL_H_
#define INC_DOOR_CTRL_H_


#include "main.h"


struct WarningStatus_s{
	uint8_t door_open;
	uint8_t seatbelt;
	uint8_t pbrake;
	uint8_t seat1;
	uint8_t seat2;
	uint8_t seat3;
	uint8_t seat4;
	uint8_t seat5;
	uint8_t seat6;
	uint8_t seat7;
	uint8_t seat8;
}Warning_Status;

struct SeatStatus_s{
	uint8_t seat1;
	uint8_t seat2;
	uint8_t seat3;
	uint8_t seat4;
	uint8_t seat5;
	uint8_t seat6;
	uint8_t seat7;
	uint8_t seat8;
	uint8_t belt1;
	uint8_t belt2;
	uint8_t belt3;
	uint8_t belt4;
	uint8_t belt5;
	uint8_t belt6;
	uint8_t belt7;
	uint8_t belt8;
}Seat_Status;


//uint8_t DRV_door_open;
//uint8_t DRV_door_close;
//uint8_t PS_door_open;
//uint8_t PS_door_close;
//uint8_t DRV_door_open_prev;
//uint8_t DRV_door_close_prev;
//uint8_t PS_door_open_prev;
//uint8_t PS_door_close_prev;

uint8_t door_open_on;
uint8_t door_open_on_cnt;
uint8_t door_close_on;
uint8_t door_close_on_cnt;
uint8_t column_on_seq;
uint8_t column_on_cnt;

uint8_t drv_seatbelt_status;
uint8_t drv_seat_status;
uint8_t seatbelt_R0_C0;
uint8_t seatbelt_R1_C0;
uint8_t seatbelt_R0_C1;
uint8_t seatbelt_R1_C1;
uint8_t seatbelt_R0_C2;
uint8_t seatbelt_R1_C2;
uint8_t seatbelt_R0_C3;
uint8_t seatbelt_R1_C3;
uint8_t seat_R0_C0;
uint8_t seat_R1_C0;
uint8_t seat_R0_C1;
uint8_t seat_R1_C1;
uint8_t seat_R0_C2;
uint8_t seat_R1_C2;
uint8_t seat_R0_C3;
uint8_t seat_R1_C3;
uint8_t seatbelt_tx_data[8];
uint8_t seatbelt_tx_flag;
uint8_t pass_seatbelt_warning;
uint8_t drv_seatbelt_warning;
uint8_t door_open_staus;
uint8_t impact_status;
uint8_t impact_cnt;
uint8_t courtesy_lamp_on;
uint8_t hazard_lamp_on;
uint8_t RF_door_open_sig;
uint8_t RF_door_open_sig_prev;
uint8_t RF_door_open_sig_on;
uint8_t RF_door_open_sig_time;
uint8_t RF_door_open_sig_cnt;
uint8_t RF_door_open_control_on;
uint8_t RF_door_open_control_cnt;
uint8_t RF_door_close_sig;
uint8_t RF_door_close_sig_prev;
uint8_t RF_door_close_sig_on;
uint8_t RF_door_close_sig_time;
uint8_t RF_door_close_sig_cnt;
uint8_t RF_door_close_control_on;
uint8_t RF_door_close_control_cnt;
uint8_t taillamp_on;
uint8_t taillamp_on_door_open;
uint8_t distant_alarm;
uint8_t distant_alarm_prev;
uint8_t distant_alarm_on;
uint8_t distant_alarm_on_cnt;

uint16_t pass_seatbelt_cnt;
uint16_t drv_seatbelt_cnt;
uint8_t door_warning_cnt;
uint8_t pbrake_warning_cnt;
uint16_t distant_volt;
uint16_t sun_volt;


void DoorControl();
void ImpactControl();
void SeatBeltRead();
void SeatbeltWarningControl();
void DoorWarningControl();
void PBrakeWarningControl();
void SeatbeltDataConv(uint8_t *txdata, uint8_t *txflag);
void WarningAlarmControl();
void DistantDoorControl();
void ButtonDataConv(uint8_t *txdata, uint8_t *txflag);


#endif /* INC_DOOR_CTRL_H_ */

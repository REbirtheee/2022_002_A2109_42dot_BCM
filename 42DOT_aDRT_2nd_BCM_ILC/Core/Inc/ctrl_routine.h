/*
 * ctrl_routine.h
 *
 *  Created on: Nov 11, 2021
 *      Author: okskt
 */

#ifndef INC_CTRL_ROUTINE_H_
#define INC_CTRL_ROUTINE_H_


#include "main.h"

#if 0
#define DEBUG_TEST
#endif

#define CAN_ID_LAMPINFO			0x100
#define CAN_ID_CARINFO			0x110
#define CAN_ID_LAMPINFO2		0x121
#define CAN_ID_WIPERINFO		0x132
#define CAN_ID_DOORINFO			0x143
#define CAN_ID_BUTTONINFO		0x113
#define CAN_ID_ILCINFO			0x200
#define CAN_ID_AKIT_CMD			0x210
#define CAN_ID_DEBUG			0x333
#define CAN_ID_LLCINFO			0x020
#define CAN_ID_BCMINFO			0x030

#define CAN_ID_BC2ICINFO		0x18FF1721
#define CAN_ID_DTCINFO			0x18FF1722
#define CAN_ID_SM2ICINFO		0x18FF171F
#define CAN_ID_CC2ICINFO		0x18FF1747
#define CAN_ID_AK2BC			0x18FF1725


CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;
CAN_HandleTypeDef hcan3;

CAN_RxHeaderTypeDef can1_rx_header;
CAN_TxHeaderTypeDef can1_tx_header;
CAN_RxHeaderTypeDef can2_rx_header;
CAN_TxHeaderTypeDef can2_tx_header;
CAN_RxHeaderTypeDef can3_rx_header;
CAN_TxHeaderTypeDef can3_tx_header;


uint8_t can1_rx_data[8];
uint8_t can1_tx_data[8];
uint8_t can2_rx_data[8];
uint8_t can2_tx_data[8];
uint8_t can3_rx_data[8];
uint8_t can3_tx_data[8];
uint8_t bc2ic_tx_data[8];
uint8_t dtc_tx_data[8];
uint8_t sm2ic_tx_data[8];
uint8_t cc2ic_tx_data[8];
uint8_t bcm_tx_data[8];
uint8_t ak2bc_rx_data[8];
uint8_t ak2bc_tx_data[8];

uint8_t timer6_cmd_num;
uint16_t timer6_cmd_num_100ms;
uint16_t timer6_cmd_num_1000ms;
uint8_t can_not_receive_cnt;
uint8_t bc2ic_tx_cnt;
uint8_t bc2ic_tx_flag;
uint8_t dtc_tx_cnt;
uint8_t dtc_tx_flag;
uint8_t sm2ic_tx_cnt;
uint8_t sm2ic_tx_flag;
uint8_t cc2ic_tx_cnt;
uint8_t cc2ic_tx_flag;
uint8_t bcm_tx_flag;
uint8_t ak2bc_tx_flag;

int16_t vehicle_vel;
int16_t vehicle_dec;
uint8_t brake_stauts;
uint8_t epb_status;
uint8_t pbrake_status;
uint8_t shift_status;
uint8_t gear_status;
uint8_t soc_value;
uint8_t regenerative_brake_value;
uint8_t regenerative_brake_mode;
uint8_t charge_mode;
uint8_t car_ready_status;
uint8_t tail_lamp_status;
uint8_t hazard_lamp_status;
uint8_t head_lamp_low_status;
uint8_t head_lamp_high_status;
uint8_t turn_lamp_left_status;
uint8_t turn_lamp_right_status;
uint8_t seatbelt_warning_status;
uint8_t seat_status;
uint8_t door_open_status;
uint8_t charge_door_status;
uint8_t bcm_fail_status;
uint8_t head_low_lamp_fail;
uint8_t stop_lamp_fail;
uint8_t turn_left_lamp_fail;
uint8_t turn_right_lamp_fail;
uint8_t stop_enable;
uint8_t auto_mode;
uint8_t ems_status;
uint8_t alive_cnt;
uint8_t bcm_checksum;
uint8_t stop_enable_prev;
uint8_t wake_up_on;
uint8_t wake_up_cnt;
uint8_t rke_unlock_staus;
uint8_t rke_lock_staus;
uint8_t cluster_can_tx_flag;
uint8_t stop_lamp_on;
uint8_t steer_fail_warning;

struct SteerErrorStatus_s{
	uint8_t system_interal_fail;
	uint8_t system_degrade;
	uint8_t communication_fail;
}Steer_Error_Status;

uint16_t stop_mode_cnt;

uint32_t can1_tx_mailbox;
uint32_t can2_tx_mailbox;
uint32_t can3_tx_mailbox;

void ULCTxMessage(uint32_t id, uint8_t length, uint8_t *data, uint8_t *flag);
void LocalTxMessage(uint32_t id, uint8_t length, uint8_t *data, uint8_t *flag);
void ClusterToILC(CAN_RxHeaderTypeDef *rxheader, uint8_t *rxdata);
void LLCTxMessage(uint32_t id, uint8_t length, uint8_t *data, uint8_t *flag);
void LocalToILC(CAN_RxHeaderTypeDef *rxheader, uint8_t *rxdata);
void LLCToILC(CAN_RxHeaderTypeDef *rxheader, uint8_t *rxdata);
void BC2ICDataConv(uint8_t *txdata, uint8_t *txflag);
void DTCDataConv(uint8_t *txdata, uint8_t *txflag);
void SM2ICDataConv(uint8_t *txdata, uint8_t *txflag);
void CC2ICDataConv(uint8_t *txdata, uint8_t *txflag);
void BCMDataConv(uint8_t *txdata, uint8_t *txflag);
void AkitCmdConv(uint8_t *txdata, uint8_t *rxdata, uint8_t *txflag);
void CANReStart();



#endif /* INC_CTRL_ROUTINE_H_ */

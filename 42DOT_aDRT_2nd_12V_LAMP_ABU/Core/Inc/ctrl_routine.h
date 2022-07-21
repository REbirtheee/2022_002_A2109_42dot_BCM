/*
 * ctrl_routine.h
 *
 *  Created on: Jan 4, 2022
 *      Author: okskt
 */

#ifndef INC_CTRL_ROUTINE_H_
#define INC_CTRL_ROUTINE_H_


#include "main.h"


FDCAN_HandleTypeDef hfdcan1;
FDCAN_HandleTypeDef hfdcan2;


#define CAN_ID_AKIT_CMD			0x210
#define CAN_ID_BUTTONINFO		0x200
#define CAN_ID_24VLAMPINFO		0x100
#define CAN_ID_12VLAMPINFO		0x121
#define CAN_ID_WIPERINFO		0x132
#define CAN_ID_SEATINFO			0x143
#define CAN_ID_DEBUG			0x333

#define CAN_ID_TEST				0x334


#define CAN_ID_USFRONTINFO	0x20
#define CAN_ID_USREARINFO	0x10



FDCAN_RxHeaderTypeDef can1_rx_header;
FDCAN_TxHeaderTypeDef can1_tx_header;


FDCAN_RxHeaderTypeDef can2_rx_header;
FDCAN_TxHeaderTypeDef can2_tx_header;


uint8_t can1_rx_data[8];
uint8_t can1_tx_data[8];
uint8_t can2_rx_data[8];
uint8_t can2_tx_data[8];

uint8_t testdata[8];

uint8_t gear_position;
uint8_t brake_status;
int16_t vehicle_vel;
uint8_t vehicle_vel_prev;
uint8_t pBrake_status;
uint8_t check_pBrake_status;
uint8_t auto_mode;
uint8_t door_open_status_prev;
uint8_t timer6_cmd_num;
uint8_t ign1_status;
uint8_t ign1_status_old;
uint8_t ign1_status_check;
uint8_t stop_enable;

uint16_t stop_mode_cnt;

uint8_t testAVAS;
uint8_t savedata;




void CanTxMessage(uint32_t id, uint32_t length, uint8_t *data, uint8_t *flag);
void CanTx1Message(uint32_t id, uint32_t length, uint8_t *data, uint8_t *flag);
void LocalToABU(FDCAN_RxHeaderTypeDef *rxheader, uint8_t *rxdata);
void usToABU(FDCAN_RxHeaderTypeDef *rxheader, uint8_t *rxdata);
void CANReStart();


#endif /* INC_CTRL_ROUTINE_H_ */

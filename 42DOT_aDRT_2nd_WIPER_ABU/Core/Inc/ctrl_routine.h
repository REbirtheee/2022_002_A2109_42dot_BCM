/*
 * ctrl_routine.h
 *
 *  Created on: Nov 11, 2021
 *      Author: okskt
 */

#ifndef INC_CTRL_ROUTINE_H_
#define INC_CTRL_ROUTINE_H_


#include "main.h"


#define CAN_ID_AKIT_CMD			0x210
#define CAN_ID_BUTTONINFO		0x200
#define CAN_ID_24VLAMPINFO		0x100
#define CAN_ID_12VLAMPINFO		0x121
#define CAN_ID_WIPERINFO		0x132
#define CAN_ID_DOORINFO			0x143
#define CAN_ID_DEBUG			0x333


FDCAN_HandleTypeDef hfdcan1;
FDCAN_HandleTypeDef hfdcan2;

FDCAN_RxHeaderTypeDef can1_rx_header;
FDCAN_TxHeaderTypeDef can1_tx_header;
FDCAN_RxHeaderTypeDef can2_rx_header;
FDCAN_TxHeaderTypeDef can2_tx_header;

uint8_t can1_rx_data[8];
uint8_t can1_tx_data[8];
uint8_t can2_rx_data[8];
uint8_t can2_tx_data[8];
uint8_t spi2_tx_data[8];

uint8_t ign1_status;
uint8_t key_in;
uint8_t vehicle_vel_prev;
uint8_t pBrake_status;
uint8_t door_open_status;
uint8_t timer6_cmd_num;
uint8_t lamp_tx_flag;
uint8_t can_not_receive_cnt;
uint8_t stop_enable;
uint8_t charge_door_status;
uint8_t rke_lock_status;
uint8_t rke_unlock_status;
uint8_t auto_mode;

int16_t vehicle_vel;
uint16_t stop_mode_cnt;

float vehicle_dec;


void CanTxMessage(uint32_t id, uint32_t length, uint8_t *data, uint8_t *flag);
void LocalToABU(FDCAN_RxHeaderTypeDef *rxheader, uint8_t *rxdata);
void CANReStart();


#endif /* INC_CTRL_ROUTINE_H_ */

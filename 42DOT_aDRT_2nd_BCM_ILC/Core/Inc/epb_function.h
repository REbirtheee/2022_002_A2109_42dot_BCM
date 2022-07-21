/*
 * epb_function.h
 *
 *  Created on: Jun 28, 2022
 *      Author: TREEZE20
 */

#ifndef INC_EPB_FUCTION_H_
#define INC_EPB_FUCTION_H_

#include "main.h"


CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;
CAN_HandleTypeDef hcan3;

CAN_RxHeaderTypeDef can1_rx_header;
CAN_TxHeaderTypeDef can1_tx_header;
CAN_RxHeaderTypeDef can2_rx_header;
CAN_TxHeaderTypeDef can2_tx_header;
CAN_RxHeaderTypeDef can3_rx_header;
CAN_TxHeaderTypeDef can3_tx_header;


uint8_t info_0x47f[8];
uint8_t info_0x470[8];
uint8_t info_0x386[8];
uint8_t info_0x5B0[8];
uint8_t info_0x541[8];
uint8_t info_0x507[8];
uint8_t info_0x153[8];
uint8_t info_0x329[8];
uint8_t info_0x260[8];

struct alive_s{
	uint8_t info_0x47f;
	uint8_t info_0x470;
	uint8_t info_0x153_1;
	uint8_t info_0x153_2;
	uint8_t info_0x329;
	uint8_t info_0x260;
}alive;

void EpbCanInfo0x47F();
void EpbCanInfo0x470();
void EpbCanInfo0x507();
void EpbCanInfo0x386();
void EpbCanInfo0x5B0();
void EpbCanInfo0x153();
void EpbCanInfo0x329();
void EpbCanInfo0x260();


#endif /* INC_EPB_FUCTION_H_ */

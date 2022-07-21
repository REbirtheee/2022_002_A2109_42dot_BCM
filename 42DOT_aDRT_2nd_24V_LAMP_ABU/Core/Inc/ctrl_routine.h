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
#define CAN_ID_VEHICLEINFO		0x110
#define CAN_ID_12VLAMPINFO		0x121
#define CAN_ID_WIPERINFO		0x132
#define CAN_ID_SEATINFO			0x143
#define CAN_ID_DEBUG			0x333


FDCAN_RxHeaderTypeDef can1_rx_header;
FDCAN_TxHeaderTypeDef can1_tx_header;
FDCAN_RxHeaderTypeDef can2_rx_header;
FDCAN_TxHeaderTypeDef can2_tx_header;


uint8_t can1_rx_data[8];
uint8_t can1_tx_data[8];
uint8_t can2_rx_data[8];
uint8_t can2_tx_data[8];

uint8_t vehicle_tx_data[8];

uint8_t timer6_cmd_num;
uint8_t ign1_status;
uint8_t ign1_status_prev;
uint8_t ign2_status;
uint8_t ign2_status_prev;
uint8_t key_in;
uint8_t pBrake_status;
int16_t vehicle_vel;
uint8_t brake_status;
uint8_t door_open_status;
uint8_t door_open_status_prev;
uint8_t lamp_tx_flag;
uint8_t taillamp_on;
uint8_t taillamp_on_prev;
uint8_t headlamp_low_on;
uint8_t turnlamp_left_on;
uint8_t turnlamp_right_on;
uint8_t hazard_on;
uint8_t RF_open_lamp;
uint8_t RF_open_lamp_prev;
uint8_t RF_open_lamp_on;
uint8_t RF_open_lamp_on_cnt;
uint8_t RF_close_lamp;
uint8_t RF_close_lamp_prev;
uint8_t RF_close_lamp_on;
uint8_t RF_close_lamp_on_cnt;
uint8_t can_not_receive_cnt;
uint8_t welcome_on;
uint8_t welcome_on_cnt;
uint8_t welcome_off;
uint8_t welcome_off_cnt;
uint8_t head_low_lamp_fail;
uint8_t head_low_lamp_fail_cnt;
uint8_t head_low_lamp_fail_on_cnt;
uint8_t stop_lamp_fail;
uint8_t stop_lamp_fail_cnt;
uint8_t stop_lamp_fail_on_cnt;
uint8_t stop_enable;
uint8_t soc_value;
uint8_t vehicle_charge_status;
uint8_t charge_mode;
uint8_t vehicle_tx_flag;
uint8_t vehicle_curr_rx_flag;
uint8_t charge_door_status;
uint8_t car_ready_status;
uint8_t button_tx_flag;
uint8_t ems_status;
uint8_t auto_mode;

uint16_t stop_mode_cnt;
uint16_t dimmer_volt;
uint16_t hazard_onoff_cnt;
int16_t vehicle_current;
uint16_t vehicle_voltage;


void CanTxMessage(uint32_t id, uint32_t length, uint8_t *data, uint8_t *flag);
void LocalToABU(FDCAN_RxHeaderTypeDef *rxheader, uint8_t *rxdata);
void VehicleDataConv(uint8_t *txdata, uint8_t *flag, uint8_t *curr_rx_flag);
void VehicleToABU(FDCAN_RxHeaderTypeDef *rxheader, uint8_t *rxdata);
void CANReStart();
void StopModeStart();


#endif /* INC_CTRL_ROUTINE_H_ */

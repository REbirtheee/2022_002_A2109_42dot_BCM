/*
 * button_ctrl.h
 *
 *  Created on: Nov 17, 2021
 *      Author: okskt
 */

#ifndef INC_BUTTON_CTRL_H_
#define INC_BUTTON_CTRL_H_


#include "main.h"


#define READ_PIN_DI_0		HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_0)
#define READ_PIN_DI_1		HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_1)
#define READ_PIN_DI_2		HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_2)
#define READ_PIN_DI_3		HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_3)
#define READ_PIN_DI_4		HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_4)
#define READ_PIN_DI_5		HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_5)
#define READ_PIN_DI_6		HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_6)
#define READ_PIN_DI_7		HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_7)
#define READ_PIN_DI_8		HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_8)
#define READ_PIN_DI_9		HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_9)
#define READ_PIN_DI_10		HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_10)
#define READ_PIN_DI_11		HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_11)


struct InputStatus_s {
	uint8_t ign1_status;
	uint8_t auto_light;
	uint8_t defogger;
	uint8_t ps_close;
	uint8_t ps_open;
	uint8_t drv_close;
	uint8_t drv_open;
	uint8_t ign2_status;
	uint8_t autonomous;
	uint8_t cluster_trip;
} Input_Status;

struct InputStatus_Prev_s {
	uint8_t ign1_status;
	uint8_t auto_light;
	uint8_t defogger;
	uint8_t ps_close;
	uint8_t ps_open;
	uint8_t drv_close;
	uint8_t drv_open;
	uint8_t ign2_status;
	uint8_t autonomous;
	uint8_t cluster_trip;
} Input_Status_Prev;

struct InputStatus_Raw_s {
	uint8_t ign1_status;
	uint8_t auto_light;
	uint8_t defogger;
	uint8_t ps_close;
	uint8_t ps_open;
	uint8_t drv_close;
	uint8_t drv_open;
	uint8_t ign2_status;
	uint8_t autonomous;
	uint8_t cluster_trip;
} Input_Status_Raw;

struct InputStatus_Raw_Prev_s {
	uint8_t ign1_status;
	uint8_t auto_light;
	uint8_t defogger;
	uint8_t ps_close;
	uint8_t ps_open;
	uint8_t drv_close;
	uint8_t drv_open;
	uint8_t ign2_status;
	uint8_t autonomous;
	uint8_t cluster_trip;
} Input_Status_Raw_Prev;

struct Chattering_s {
	uint8_t ign1_status_cnt;
	uint8_t auto_light_cnt;
	uint8_t defogger_cnt;
	uint8_t ps_close_cnt;
	uint8_t ps_open_cnt;
	uint8_t drv_close_cnt;
	uint8_t drv_open_cnt;
	uint8_t ign2_status_cnt;
	uint8_t autonomous_cnt;
	uint8_t cluster_trip_cnt;
} Chattering;

struct Button_Status_s {
	uint8_t ign1_status;
	uint8_t auto_light;
	uint8_t defogger;
	uint8_t ps_close;
	uint8_t ps_open;
	uint8_t drv_close;
	uint8_t drv_open;
	uint8_t ign2_status;
	uint8_t autonomous;
	uint8_t cluster_trip;
	uint8_t accel_decel_sw;
	uint8_t drvie_mode_sw;
	uint8_t logging_sw;
	uint8_t marker_sw;
} Button_Status;

uint8_t key_in;
uint8_t button_tx_data[8];
uint8_t button_tx_flag;
uint8_t auto_sw_prev;
uint8_t auto_sw;
uint8_t defogger_sw;
uint8_t autonomous_sw;
uint8_t autonomous_status;
uint8_t cluster_trip_sw;
uint8_t cluster_trip_long;
uint8_t cluster_trip_status;
uint8_t cluster_trip_status_tx;

uint8_t cluster_trip_push_cnt;
uint8_t cluster_trip_check_cnt;

uint16_t cluster_trip_sw_cnt;

uint32_t deffoger_cnt;

void ButtonRead();
void ButtonSWChatt();
void ButtonConrtrol();
void ButtonDataConv(uint8_t *txdata, uint8_t *flag);


#endif /* INC_BUTTON_CTRL_H_ */

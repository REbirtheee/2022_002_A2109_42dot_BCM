/*
 * wiper_ctrl.h
 *
 *  Created on: Nov 16, 2021
 *      Author: okskt
 */

#ifndef INC_WIPER_CTRL_H_
#define INC_WIPER_CTRL_H_


#include "main.h"


#define READ_PIN_DI_0			HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_0)
#define READ_PIN_DI_1			HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_1)
#define READ_PIN_DI_2			HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_2)
#define READ_PIN_DI_3			HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_3)
#define READ_PIN_DI_4			HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1)
#define READ_PIN_DI_5			HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2)
#define READ_PIN_DI_6			HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3)
#define READ_PIN_DI_7			HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4)
#define READ_PIN_DI_8			HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5)
#define READ_PIN_DI_9			HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6)
#define READ_PIN_DI_10			HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7)
#define READ_PIN_DI_11			HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_4)
#define READ_PIN_DI_12			HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_2)
#define READ_PIN_DI_13			HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_3)
#define READ_PIN_DI_14			HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_4)
#define READ_PIN_DI_15			HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_5)


TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim5;


struct InputStatusW_s {
	uint8_t wiper_int;
	uint8_t wiper_low;
	uint8_t wiper_high;
	uint8_t washer;
	uint8_t auto_wiper;
	uint8_t wiper_parking;
	uint8_t key_in;
	uint8_t accessory;
}Input_Status;

struct InputStatus_Raw_W_s {
	uint8_t wiper_int;
	uint8_t wiper_low;
	uint8_t wiper_high;
	uint8_t washer;
	uint8_t auto_wiper;
	uint8_t wiper_parking;
	uint8_t key_in;
	uint8_t accessory;
}Input_Status_Raw;

struct InputStatus_Prev_W_s {
	uint8_t wiper_int;
	uint8_t wiper_low;
	uint8_t wiper_high;
	uint8_t washer;
	uint8_t auto_wiper;
	uint8_t wiper_parking;
	uint8_t key_in;
	uint8_t accessory;
}Input_Status_Prev;

struct WiperStatus_W_s {
	uint8_t wiper_int;
	uint8_t wiper_low;
	uint8_t wiper_high;
	uint8_t washer;
	uint8_t auto_wiper;
	uint8_t wiper_parking;
}Wiper_Status;

struct Chattering_W_s {
	uint8_t wiper_int_cnt;
	uint8_t wiper_low_cnt;
	uint8_t wiper_high_cnt;
	uint8_t washer_cnt;
	uint8_t auto_wiper_cnt;
	uint8_t wiper_parking_cnt;
	uint8_t key_in_cnt;
	uint8_t accessory_cnt;
}Chattering;

struct LampStatus_s {
	uint8_t courtesy_lamp;
}Lamp_Status;

struct AkitCommand_s {
	uint8_t wiper_int_on;
	uint8_t wiper_low_on;
	uint8_t wiper_high_on;
	uint8_t wiper_off;
	uint8_t washer_on;
}Akit_Command;


uint8_t wiper_tx_data[8];

uint8_t washer_on;
uint8_t washer_on_cnt;
uint8_t washer_off;
uint8_t washer_off_cnt;
uint8_t wiper_tx_flag;
uint8_t ign2_status;
uint8_t rain_sens_wiper_low;
uint8_t rain_sens_wiper_high;
uint8_t taillamp_on;
uint8_t stop_lamp_on;
uint8_t dimmer_status;
uint8_t door_open_status_prev;

uint16_t wiper_int_on_cnt;
uint16_t wiper_int_volt;
uint16_t dimmer_volt;


void WiperSWRead(struct InputStatusW_s *input_status);
void WiperSWChatt(struct InputStatusW_s *input_status);
void WiperControl(struct WiperStatus_W_s *wiper_status, struct InputStatusW_s *input_status);
void AutoWiperControl(struct WiperStatus_W_s *wiper_status, struct InputStatusW_s *input_status);
void DimmerLampControl();
void LampControl(struct LampStatus_s *lamp_status);
void WiperDataConv(uint8_t *txdata, uint8_t *flag);


#endif /* INC_WIPER_CTRL_H_ */

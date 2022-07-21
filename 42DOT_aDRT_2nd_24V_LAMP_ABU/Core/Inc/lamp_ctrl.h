/*
 * lamp_ctrl.h
 *
 *  Created on: Jan 4, 2022
 *      Author: okskt
 */

#ifndef INC_LAMP_CTRL_H_
#define INC_LAMP_CTRL_H_


#include "main.h"


#ifdef debug
#include "stdio.h"
#endif


UART_HandleTypeDef huart1;


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
TIM_HandleTypeDef htim4;


struct InputStatus_s {
	uint8_t brake_push_sw;
	uint8_t hazardlamp;
	uint8_t door_open_status;
}Input_Status;

struct InputStatusPrev_s {
	uint8_t brake_push_sw;
	uint8_t hazardlamp;
	uint8_t door_open_status;
}Input_Status_Prev;

struct InputStatusRaw_s {
	uint8_t brake_push_sw;
	uint8_t hazardlamp;
	uint8_t door_open_status;
}Input_Status_Raw;

struct InputStatusRawPrev_s {
	uint8_t door_open_status;
}Input_Status_Raw_Prev;

struct LampStatus_s {
	uint8_t hazard_lamp;
	uint8_t courtesy_lamp;
	uint8_t drl_lamp;
	uint8_t drl_c_lamp;
	uint8_t dimmer_lamp;
	uint8_t stop_lamp;
}Lamp_Status;

struct Chattering_s {
	uint8_t brake_push_sw_cnt;
	uint8_t hazardlamp_cnt;
	uint8_t door_open_status_cnt;
}Chattering;

struct Akit_Command_S{
	uint8_t hazard_cmd;
}akit_command;


uint8_t lamp_rx_data[1];
uint8_t lamp_tx_data[8];

uint8_t hazard_push;
uint8_t hazard_push_prev;
uint8_t hazard_on;
uint8_t drl_center_on;
uint8_t drl_center_on_cnt;
uint8_t drl_center_off_cnt;

uint16_t head_low_lamp_curr;
uint16_t stop_lamp_curr;

float vehicle_dec;


void LampSWRead(struct InputStatus_s *input_status);
void InputSWChatt(struct InputStatus_s *input_status);
void LampDataConv(uint8_t *txdata, uint8_t *flag);
void LampControl(struct LampStatus_s *lamp_status, struct InputStatus_s *input_status);
void WelcomeControl();


#endif /* INC_LAMP_CTRL_H_ */

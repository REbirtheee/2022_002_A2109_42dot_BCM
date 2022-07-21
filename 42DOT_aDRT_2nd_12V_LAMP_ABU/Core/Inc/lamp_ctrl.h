/*
 * lamp_ctrl.h
 *
 *  Created on: Jan 4, 2022
 *      Author: okskt
 */

#ifndef INC_LAMP_CTRL_H_
#define INC_LAMP_CTRL_H_


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


struct InputStatus_s {
	uint8_t taillamp;
	uint8_t headlamp_H;
	uint8_t headlamp_H_passing;
	uint8_t headlamp_L;
	uint8_t turnlamp_RH;
	uint8_t turnlamp_LH;
	uint8_t RKE_lock;
	uint8_t RKE_unlock;
	uint8_t accel_sw;
	uint8_t decel_sw;
	uint8_t drive_mode_sw;
	uint8_t logging_sw;
	uint8_t marker_sw;
	uint8_t door_open_status;
	uint8_t charge_door_status;
}Input_Status;

struct InputStatusPrev_s {
	uint8_t taillamp;
	uint8_t headlamp_H;
	uint8_t headlamp_H_passing;
	uint8_t headlamp_L;
	uint8_t turnlamp_RH;
	uint8_t turnlamp_LH;
	uint8_t RKE_lock;
	uint8_t RKE_unlock;
	uint8_t accel_sw;
	uint8_t decel_sw;
	uint8_t drive_mode_sw;
	uint8_t logging_sw;
	uint8_t marker_sw;
	uint8_t door_open_status;
	uint8_t charge_door_status;
}Input_Status_Prev;

struct InputStatusRaw_s {
	uint8_t taillamp;
	uint8_t headlamp_H;
	uint8_t headlamp_H_passing;
	uint8_t headlamp_L;
	uint8_t turnlamp_RH;
	uint8_t turnlamp_LH;
	uint8_t RKE_lock;
	uint8_t RKE_unlock;
	uint8_t accel_sw;
	uint8_t decel_sw;
	uint8_t drive_mode_sw;
	uint8_t logging_sw;
	uint8_t marker_sw;
	uint8_t door_open_status;
	uint8_t charge_door_status;
}Input_Status_Raw;

struct LampStatus_s {
	uint8_t tail_lamp;
	uint8_t hazard_lamp;
	uint8_t turn_lamp_left;
	uint8_t turn_lamp_right;
	uint8_t head_lamp_low;
	uint8_t head_lamp_high;
	uint8_t drl_lamp;
	uint8_t courtesy_lamp;
}Lamp_Status;

struct Chattering_s {
	uint8_t taillamp_cnt;
	uint8_t headalamp_low_cnt;
	uint8_t headalamp_high_cnt;
	uint8_t headlamp_high_passing_cnt;
	uint8_t turnlamp_left_cnt;
	uint8_t turnlamp_right_cnt;
	uint8_t RKE_lock_cnt;
	uint8_t RKE_unlock_cnt;
	uint8_t accel_sw_cnt;
	uint8_t decel_sw_cnt;
	uint8_t drive_mode_sw_cnt;
	uint8_t logging_sw_cnt;
	uint8_t marker_sw_cnt;
	uint8_t door_open_status_cnt;
	uint8_t charge_door_status_cnt;
}Chattering;

struct DoorStatus_s {
	//	uint8_t door_open_status;
	uint8_t door_opening;
	uint8_t door_closing;
}Door_Status;

struct ButtonStatus_s{
	uint8_t accel_decel_sw;
	uint8_t drive_mode_sw;
	uint8_t logging_sw;
	uint8_t marker_sw;
}Button_Status;

struct AkitCommand_s{
	uint8_t turn_left_on;
	uint8_t turn_right_on;
	uint8_t tail_on;
	uint8_t head_low_on;
	uint8_t head_high_on;
	uint8_t lamp_off;
	uint8_t door_open;
	uint8_t door_close;
}akit_command;

struct UsfrontStatus_s{
	uint8_t us_front_head1;
	uint8_t us_front_head2;
	uint8_t us_front_id;
	uint8_t us_front_ob_detect;
	uint8_t us_front_ob_dist;
	uint8_t us_front_cksm;
	uint8_t us_front_dist_lpf;
	uint8_t us_front_dist_cnt;
}Us_front;

struct UsrearStatus_s{
	uint8_t us_rear_head1;
	uint8_t us_rear_head2;
	uint8_t us_rear_id;
	uint8_t us_rear_ob_detect;
	uint8_t us_rear_ob_dist;
	uint8_t us_rear_cksm;
	uint8_t us_rear_dist_lpf;
	uint8_t us_rear_dist_cnt;
}Us_rear;

uint8_t lamp_tx_data[8];

uint8_t lamp_tx_flag;
uint8_t hazard_sw_on;
uint8_t stop_lamp_on;
uint8_t auto_light_status;
uint8_t charge_door_status;
uint8_t headlamp_L_onoff;
uint8_t headlamp_H_onoff;
uint8_t sidelamp_onoff;
uint8_t turnlamp_left_onoff_cnt;
uint8_t turnlamp_right_onoff_cnt;

uint8_t dec_hazard_on;
uint8_t hazard_dec_cnt;
uint8_t RF_door_open_sig;
uint8_t RF_door_open_sig_on;
uint8_t RF_door_open_sig_on_prev;
uint8_t RF_door_open_sig_cnt;
uint8_t RF_door_close_sig;
uint8_t RF_door_close_sig_on;
uint8_t RF_door_close_sig_on_prev;
uint8_t RF_door_close_sig_cnt;
uint8_t RF_open_hazard_on;
uint8_t RKE_door_lock_status;
uint8_t RF_close_hazard_on;
uint8_t RF_door_open_seq;
uint8_t RF_door_open_seq_cnt;
uint8_t can_not_receive_cnt;
uint8_t auto_tail_on_cnt;
uint8_t auto_tail_off_cnt;
uint8_t auto_head_on_cnt;
uint8_t auto_head_off_cnt;

uint8_t DRV_door_open;
uint8_t DRV_door_close;
uint8_t PS_door_open;
uint8_t PS_door_close;
uint8_t DRV_door_open_prev;
uint8_t DRV_door_close_prev;
uint8_t PS_door_open_prev;
uint8_t PS_door_close_prev;
uint8_t door_open_on;
uint16_t door_open_on_cnt;
uint8_t door_close_on;
uint8_t door_close_on_cnt;
uint8_t distant_door_open;
uint8_t distant_door_open_cnt;
uint8_t door_open_comp;
uint8_t turn_lamp_left_delay_cnt;
uint8_t turn_lamp_left_fail;
uint8_t turn_lamp_left_fail_cnt;
uint8_t turn_lamp_right_fail;
uint8_t turn_lamp_right_fail_cnt;
uint8_t turn_lamp_right_delay_cnt;
uint8_t convertor_status;
uint8_t convertor_on_delay;


uint8_t us_door_closing;
uint8_t us_door_closing_cnt;
uint8_t check_us_door_open;
uint8_t check_us_door_open_cnt;

uint8_t check_us_door_close;
uint8_t check_us_door_close_cnt;
uint8_t us_door_opening_cnt;

uint16_t hazard_onoff_cnt;
uint16_t light_data;
uint16_t distant_data;
uint16_t turn_left_curr;
uint16_t turn_right_curr;
uint16_t door_open_comp_cnt;

uint8_t boardingpass_AVAS_trigger;
uint16_t boardingpass_AVAS_cnt;

uint8_t doorwarning_AVAS_trigger;
uint16_t doorwarning_AVAS_cnt;
uint8_t door_warning_closing_cnt;
uint8_t check_door_warning_sensor_cnt;



uint8_t test_tx_flag;


void LampDataConv(uint8_t *txdata, uint8_t *flag);
void LampSWRead(struct InputStatus_s *input_status);
void InputSWChatt(struct InputStatus_s *input_status);
void LampControl();
//void HazardControl();
void RF_DoorControl();
void DoorControl();
void DistantDoorControl();
void ButtonControl(struct ButtonStatus_s *button_status, struct InputStatus_s *input_status);
void CMS_Control();
void usdoor_control();
void boardingpass_AVAS_control();
void doorwaring_AVAS_control();




#endif /* INC_LAMP_CTRL_H_ */

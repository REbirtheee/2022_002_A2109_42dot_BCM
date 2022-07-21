/*
 * param.h
 *
 *  Created on: May 31, 2022
 *      Author: MSI
 */

#ifndef INC_PARAM_H_
#define INC_PARAM_H_

#include "stm32g4xx_hal.h"
#include "string.h"
#include "stm32g4xx_hal_flash.h"

FDCAN_HandleTypeDef hfdcan1;
FDCAN_HandleTypeDef hfdcan2;
FDCAN_HandleTypeDef hfdcan3;

FDCAN_TxHeaderTypeDef a_can_tx_header;
FDCAN_TxHeaderTypeDef l_can_tx_header;
FDCAN_TxHeaderTypeDef c_can_tx_header;

FDCAN_RxHeaderTypeDef a_can_rx_header;
FDCAN_RxHeaderTypeDef l_can_rx_header;
FDCAN_RxHeaderTypeDef c_can_rx_header;

#define FLASH_STORAGE 0x08005000
#define page_size 0x800

/* diagnosis count number */
#define ADDR_FLASH_PAGE_1		((uint32_t)0x08040000)

/* steer memeory */
#define ADDR_FLASH_PAGE_2		((uint32_t)0x08040800)
#define ADDR_FLASH_PAGE_3		((uint32_t)0x08041000)

/* vehicle memeory */
#define ADDR_FLASH_PAGE_4		((uint32_t)0x08041800)
#define ADDR_FLASH_PAGE_5		((uint32_t)0x08042000)

/* accel memeory */
#define ADDR_FLASH_PAGE_6		((uint32_t)0x08042800)
#define ADDR_FLASH_PAGE_7		((uint32_t)0x08043000)

/* brake memeory */
#define ADDR_FLASH_PAGE_8		((uint32_t)0x08043800)
#define ADDR_FLASH_PAGE_9		((uint32_t)0x08044000)

/* epb shift memeory */
#define ADDR_FLASH_PAGE_10		((uint32_t)0x08044800)
#define ADDR_FLASH_PAGE_11		((uint32_t)0x08045000)

/* imu acc memeory */
#define ADDR_FLASH_PAGE_12		((uint32_t)0x08045800)
#define ADDR_FLASH_PAGE_13		((uint32_t)0x08046000)

/* gyro memeory */
#define ADDR_FLASH_PAGE_14		((uint32_t)0x08046800)
#define ADDR_FLASH_PAGE_15		((uint32_t)0x08047000)


/* vehicle memeory */
#define ADDR_FLASH_PAGE_16		((uint32_t)0x08047800)
#define ADDR_FLASH_PAGE_17		((uint32_t)0x08048000)
#define ADDR_FLASH_PAGE_18		((uint32_t)0x08048800)
#define ADDR_FLASH_PAGE_19		((uint32_t)0x08049000)
#define ADDR_FLASH_PAGE_20		((uint32_t)0x08049800)
#define ADDR_FLASH_PAGE_21		((uint32_t)0x0804A000)
#define ADDR_FLASH_PAGE_22		((uint32_t)0x0804A800)
#define ADDR_FLASH_PAGE_23		((uint32_t)0x0804B000)
#define ADDR_FLASH_PAGE_24		((uint32_t)0x0804B800)
#define ADDR_FLASH_PAGE_25		((uint32_t)0x0804C000)
#define ADDR_FLASH_PAGE_26		((uint32_t)0x0804C800)
#define ADDR_FLASH_PAGE_27		((uint32_t)0x0804D000)
#define ADDR_FLASH_PAGE_28		((uint32_t)0x0804D800)
#define ADDR_FLASH_PAGE_29		((uint32_t)0x0804F000)
#define ADDR_FLASH_PAGE_30		((uint32_t)0x0804F800)

/* accel memeory */
#define ADDR_FLASH_PAGE_31		((uint32_t)0x08050000)
#define ADDR_FLASH_PAGE_32		((uint32_t)0x08050800)
#define ADDR_FLASH_PAGE_33		((uint32_t)0x08051000)
#define ADDR_FLASH_PAGE_34		((uint32_t)0x08051800)
#define ADDR_FLASH_PAGE_35		((uint32_t)0x08052000)
#define ADDR_FLASH_PAGE_36		((uint32_t)0x08052800)
#define ADDR_FLASH_PAGE_37		((uint32_t)0x08053000)
#define ADDR_FLASH_PAGE_38		((uint32_t)0x08053800)
#define ADDR_FLASH_PAGE_39		((uint32_t)0x08054000)
#define ADDR_FLASH_PAGE_40		((uint32_t)0x08054800)
#define ADDR_FLASH_PAGE_41		((uint32_t)0x08055000)
#define ADDR_FLASH_PAGE_42		((uint32_t)0x08055800)
#define ADDR_FLASH_PAGE_43		((uint32_t)0x08056000)
#define ADDR_FLASH_PAGE_44		((uint32_t)0x08056800)
#define ADDR_FLASH_PAGE_45		((uint32_t)0x08057000)

/* brake memeory */
#define ADDR_FLASH_PAGE_46		((uint32_t)0x08057800)
#define ADDR_FLASH_PAGE_47		((uint32_t)0x08058000)
#define ADDR_FLASH_PAGE_48		((uint32_t)0x08058800)
#define ADDR_FLASH_PAGE_49		((uint32_t)0x08059000)
#define ADDR_FLASH_PAGE_50		((uint32_t)0x08059800)
#define ADDR_FLASH_PAGE_51		((uint32_t)0x0805A000)
#define ADDR_FLASH_PAGE_52		((uint32_t)0x0805A800)
#define ADDR_FLASH_PAGE_53		((uint32_t)0x0805B000)
#define ADDR_FLASH_PAGE_54		((uint32_t)0x0805B800)
#define ADDR_FLASH_PAGE_55		((uint32_t)0x0805C000)
#define ADDR_FLASH_PAGE_56		((uint32_t)0x0805C800)
#define ADDR_FLASH_PAGE_57		((uint32_t)0x0805D000)
#define ADDR_FLASH_PAGE_58		((uint32_t)0x0805D800)
#define ADDR_FLASH_PAGE_59		((uint32_t)0x0805F000)
#define ADDR_FLASH_PAGE_60		((uint32_t)0x0805F800)

/* epb shift memeory */
#define ADDR_FLASH_PAGE_61		((uint32_t)0x08060000)
#define ADDR_FLASH_PAGE_62		((uint32_t)0x08060800)
#define ADDR_FLASH_PAGE_63		((uint32_t)0x08061000)
#define ADDR_FLASH_PAGE_64		((uint32_t)0x08061800)
#define ADDR_FLASH_PAGE_65		((uint32_t)0x08062000)
#define ADDR_FLASH_PAGE_66		((uint32_t)0x08062800)
#define ADDR_FLASH_PAGE_67		((uint32_t)0x08063000)
#define ADDR_FLASH_PAGE_68		((uint32_t)0x08063800)
#define ADDR_FLASH_PAGE_69		((uint32_t)0x08064000)
#define ADDR_FLASH_PAGE_70		((uint32_t)0x08064800)
#define ADDR_FLASH_PAGE_71		((uint32_t)0x08065000)
#define ADDR_FLASH_PAGE_72		((uint32_t)0x08065800)
#define ADDR_FLASH_PAGE_73		((uint32_t)0x08066000)
#define ADDR_FLASH_PAGE_74		((uint32_t)0x08066800)
#define ADDR_FLASH_PAGE_75		((uint32_t)0x08067000)

/* imu acc memeory */
#define ADDR_FLASH_PAGE_76		((uint32_t)0x08067800)
#define ADDR_FLASH_PAGE_77		((uint32_t)0x08068000)
#define ADDR_FLASH_PAGE_78		((uint32_t)0x08068800)
#define ADDR_FLASH_PAGE_79		((uint32_t)0x08069000)
#define ADDR_FLASH_PAGE_80		((uint32_t)0x08069800)
#define ADDR_FLASH_PAGE_81		((uint32_t)0x0806A000)
#define ADDR_FLASH_PAGE_82		((uint32_t)0x0806A800)
#define ADDR_FLASH_PAGE_83		((uint32_t)0x0806B000)
#define ADDR_FLASH_PAGE_84		((uint32_t)0x0806B800)
#define ADDR_FLASH_PAGE_85		((uint32_t)0x0806C000)
#define ADDR_FLASH_PAGE_86		((uint32_t)0x0806C800)
#define ADDR_FLASH_PAGE_87		((uint32_t)0x0806D000)
#define ADDR_FLASH_PAGE_88		((uint32_t)0x0806D800)
#define ADDR_FLASH_PAGE_89		((uint32_t)0x0806F000)
#define ADDR_FLASH_PAGE_90		((uint32_t)0x0806F800)

/* imu gyro memeory */
#define ADDR_FLASH_PAGE_91		((uint32_t)0x08070000)
#define ADDR_FLASH_PAGE_92		((uint32_t)0x08070800)
#define ADDR_FLASH_PAGE_93		((uint32_t)0x08071000)
#define ADDR_FLASH_PAGE_94		((uint32_t)0x08071800)
#define ADDR_FLASH_PAGE_95		((uint32_t)0x08072000)
#define ADDR_FLASH_PAGE_96		((uint32_t)0x08072800)
#define ADDR_FLASH_PAGE_97		((uint32_t)0x08073000)
#define ADDR_FLASH_PAGE_98		((uint32_t)0x08073800)
#define ADDR_FLASH_PAGE_99		((uint32_t)0x08074000)
#define ADDR_FLASH_PAGE_100		((uint32_t)0x08074800)
#define ADDR_FLASH_PAGE_101		((uint32_t)0x08075000)
#define ADDR_FLASH_PAGE_102		((uint32_t)0x08075800)
#define ADDR_FLASH_PAGE_103		((uint32_t)0x08076000)
#define ADDR_FLASH_PAGE_104		((uint32_t)0x08076800)
#define ADDR_FLASH_PAGE_105		((uint32_t)0x08077000)

/* etc memeory */
#define ADDR_FLASH_PAGE_106		((uint32_t)0x08077800)
#define ADDR_FLASH_PAGE_107		((uint32_t)0x08078000)
#define ADDR_FLASH_PAGE_108		((uint32_t)0x08078800)
#define ADDR_FLASH_PAGE_109		((uint32_t)0x08079000)
#define ADDR_FLASH_PAGE_110		((uint32_t)0x08079800)
#define ADDR_FLASH_PAGE_111		((uint32_t)0x0807A000)
#define ADDR_FLASH_PAGE_112		((uint32_t)0x0807A800)
#define ADDR_FLASH_PAGE_113		((uint32_t)0x0807B000)
#define ADDR_FLASH_PAGE_114		((uint32_t)0x0807B800)
#define ADDR_FLASH_PAGE_115		((uint32_t)0x0807C000)
#define ADDR_FLASH_PAGE_116		((uint32_t)0x0807C800)
#define ADDR_FLASH_PAGE_117		((uint32_t)0x0807D000)
#define ADDR_FLASH_PAGE_118		((uint32_t)0x0807D800)
#define ADDR_FLASH_PAGE_119		((uint32_t)0x0807F000)
#define ADDR_FLASH_PAGE_120		((uint32_t)0x0807F800)

#define FLASH_USER_START_ADDR	ADDR_FLASH_PAGE_1

#define FLASH_START_ADDR_CNT			ADDR_FLASH_PAGE_1
#define FLASH_START_ADDR_SCC			ADDR_FLASH_PAGE_2


#define FLASH_USER_END_ADDR		ADDR_FLASH_PAGE_120

#define USER_ADDR1				(FLASH_USER_START_ADDR)
#define USER_ADDR2				(FLASH_USER_START_ADDR + 32)

#define false 	0
#define true 	1

uint32_t romdata;

struct ParamSequnce {
	uint8_t timer;

} param_seq;

struct PramVaiable {
	uint8_t read_init;
	uint8_t gnrl_2_raw[2];
	uint8_t gnrl_2_diff[2];
	uint32_t gnrl_2_sum;
	uint8_t gnrl_2_flag;
#if 0
	uint8_t ai_raw[8];

	uint32_t high_ai_sum;
	uint32_t low_ai_sum;
	uint8_t ai_diff[8];
#endif
	uint8_t si_raw[8];
	uint8_t si_flag;
	uint8_t si_write_flag;
	uint8_t si_active_flag;
	uint8_t si_next_flag;
	uint32_t high_si_sum;
	uint32_t low_si_sum;
	uint8_t si_diff[8];

#if 0
	//uint8_t gnrl_1_status_check;
	uint8_t gnrl_1_status_check_update;
	uint8_t gnrl_1_status_cal_disable;
	uint8_t gnrl_1_status_cal_enable;
#endif

	uint8_t gnrl_2_req_index;
	uint8_t gnrl_2_req_write_flag;
	uint8_t gnrl_2_req_active_flag;
	uint8_t gnrl_2_req_checksum_enable;
	uint8_t gnrl_2_req_oper_mode;
	uint8_t gnrl_2_req_comm_enable;

	uint8_t ai_req_index;
	uint8_t ai_req_write_flag;
	uint8_t ai_req_active_flag;
	uint16_t ai_req_long_acc_min;
	uint16_t ai_req_long_acc_max;
	uint16_t ai_req_long_jerk_limit_m0;
	uint16_t ai_req_long_jerk_limit_m1;
	uint8_t ai_req_long_speed_limit_m0;
	uint8_t ai_req_long_speed_limit_m1;

	uint8_t si_req_index;
	uint8_t si_req_write_flag;
	uint8_t si_req_active_flag;
	uint16_t si_req_lat_acc_min;
	uint16_t si_req_lat_acc_max;
	uint16_t si_req_steer_angle_limit_0;
	uint16_t si_req_steer_angle_limit_1;
	uint8_t si_req_steer_angle_rate;

} param;

void ParamUpdate(struct ParamSequnce *seq, struct PramVaiable *param);
void InitSaveDataRead(struct PramVaiable *param);
void SaveDataConvert(struct PramVaiable *param);
void SteerInterfaceParamCaliDataConvert(struct PramVaiable *param);
uint32_t FlashRead(uint32_t address, uint32_t page);
void FlashErase(uint32_t address, uint32_t page, uint8_t number);
void FlashSave(uint32_t address, uint32_t data, uint32_t page);
void ParamErrorHandler(void);
void CountLimit(uint32_t *receive, uint32_t max);
void ParamSequnceTimer(uint8_t *timer, uint8_t init, uint8_t max);
void ParamCanInfo(uint16_t ID, uint32_t size, uint8_t *send, uint8_t *flag);
void GnrlParamSave(struct PramVaiable *param, uint8_t savedata);
void SteerInterfaceParamSave(struct PramVaiable *par);


#endif /* INC_PARAM_H_ */

/*
 * epb_function.c
 *
 *  Created on: Jun 28, 2022
 *      Author: TREEZE20
 */

#include "epb_function.h"


/*
 * number : 7
 * period : 100ms
 * */

void EpbCanInfo0x47F()
{
	can3_tx_header.StdId = 0x47F;
	can3_tx_header.RTR = CAN_RTR_DATA;
	can3_tx_header.IDE = CAN_ID_STD;
	can3_tx_header.DLC = 6;

	alive.info_0x47f++;

    info_0x47f[0] = 0x00;
    info_0x47f[1] = alive.info_0x47f;
    info_0x47f[2] = 0xFF;
    info_0x47f[3] = 0xFA;
    info_0x47f[4] = 0x00;
    info_0x47f[5] = 0x00;

    HAL_CAN_AddTxMessage(&hcan3, &can3_tx_header, info_0x47f, &can3_tx_mailbox);
}

/*
 * number : 8
 * period : 100ms
 * */
void EpbCanInfo0x470()
{
	can3_tx_header.StdId = 0x470;
	can3_tx_header.RTR = CAN_RTR_DATA;
	can3_tx_header.IDE = CAN_ID_STD;
	can3_tx_header.DLC = 8;

	alive.info_0x470++;
	if( alive.info_0x470 == 4) {
		alive.info_0x470 = 0;
	}

	if(alive.info_0x470 == 0) {
		info_0x470[2] = 0x05;
		info_0x470[7] = 0x50;

	} else if ( alive.info_0x470 == 1) {
		info_0x470[2] = 0x01;
		info_0x470[7] = 0x54;
	} else if ( alive.info_0x470 == 2) {
		info_0x470[2] = 0x05;
		info_0x470[7] = 0x58;
	} else if ( alive.info_0x470 == 3) {
		info_0x470[2] = 0x05;
		info_0x470[7] = 0x9C;
	}

	info_0x470[0] = 0x15;
	info_0x470[1] = 0x40;
	info_0x470[3] = 0x04;
	info_0x470[4] = 0x54;
	info_0x470[5] = 0x50;
	info_0x470[6] = 0x54;

	HAL_CAN_AddTxMessage(&hcan3, &can3_tx_header, info_0x470, &can3_tx_mailbox);
}


/*
 * number : 9
 * period : 1000ms
 * */
void EpbCanInfo0x386()
{
	can3_tx_header.StdId = 0x386;
	can3_tx_header.RTR = CAN_RTR_DATA;
	can3_tx_header.IDE = CAN_ID_STD;
	can3_tx_header.DLC = 8;

    info_0x386[0] = 0x00;
    info_0x386[1] = 0xC0;
    info_0x386[2] = 0x00;
    info_0x386[3] = 0x00;
    info_0x386[4] = 0x00;
    info_0x386[5] = 0x40;
    info_0x386[6] = 0x00;
    info_0x386[7] = 0x80;

    HAL_CAN_AddTxMessage(&hcan3, &can3_tx_header, info_0x386, &can3_tx_mailbox);
}

/*
 * number : 10
 * period : 1000ms
 * */
void EpbCanInfo0x5B0()
{
	can3_tx_header.StdId = 0x5B0;
	can3_tx_header.RTR = CAN_RTR_DATA;
	can3_tx_header.IDE = CAN_ID_STD;
	can3_tx_header.DLC = 4;

    info_0x5B0[0] = 0x56;
    info_0x5B0[1] = 0xDC;
    info_0x5B0[2] = 0x09;
    info_0x5B0[3] = 0x00;

    HAL_CAN_AddTxMessage(&hcan3, &can3_tx_header, info_0x5B0, &can3_tx_mailbox);
}

/*
 * number : 11
 * period : 100ms
 * */
void EpbCanInfo0x541(uint8_t _epb_set)
{
	uint8_t _epb_set_con = (_epb_set & 0x38) >> 3;
	can3_tx_header.StdId = 0x541;
	can3_tx_header.RTR = CAN_RTR_DATA;
	can3_tx_header.IDE = CAN_ID_STD;
	can3_tx_header.DLC = 8;

	info_0x541[0] = 0x03;
	info_0x541[1] = 0x00;
	info_0x541[2] = 0x81;
	info_0x541[3] = 0x00;
	info_0x541[4] = 0xE0;
	info_0x541[5] = 0x00;
	info_0x541[6] = 0x00;

	/* released : 0x08 -> 1
	 * applying : 0x38 -> 7
	 * applied : 0x10 -> 2
	 * releasing : 0x30 -> 6
	 * */
    if( (_epb_set_con == 2) || (_epb_set_con == 6) ) {
    	/* applied */
    	info_0x541[7] = 0x14;
    } else if( (_epb_set_con == 1) || (_epb_set_con == 7) ){
    	/* released */
    	info_0x541[7] = 0x04;
    } else {
    	info_0x541[7] = 0x04;
    }

    HAL_CAN_AddTxMessage(&hcan3, &can3_tx_header, info_0x541, &can3_tx_mailbox);
}

/*
 * number : 12
 * period : 100ms
 * */
void EpbCanInfo0x507()
{
	can3_tx_header.StdId = 0x507;
	can3_tx_header.RTR = CAN_RTR_DATA;
	can3_tx_header.IDE = CAN_ID_STD;
	can3_tx_header.DLC = 8;

	info_0x507[0] = 0x00;
	info_0x507[1] = 0x00;
	info_0x507[2] = 0x00;
	info_0x507[3] = 0x01;


	HAL_CAN_AddTxMessage(&hcan3, &can3_tx_header, info_0x507, &can3_tx_mailbox);
}

void EpbCanInfo0x153()
{
    alive.info_0x153_1++;
    alive.info_0x153_2++;

	can3_tx_header.StdId = 0x153;
	can3_tx_header.RTR = CAN_RTR_DATA;
	can3_tx_header.IDE = CAN_ID_STD;
	can3_tx_header.DLC = 8;

    info_0x153[0] = 0x00;
    info_0x153[1] = 0x80;
    info_0x153[2] = 0x10;
    info_0x153[3] = 0xFF;
    info_0x153[4] = 0x00;
    info_0x153[5] = 0xFF;
    info_0x153[6] = alive.info_0x153_1 << 4;
    info_0x153[7] = ( (alive.info_0x153_2 << 4) | 0x0E);

    HAL_CAN_AddTxMessage(&hcan3, &can3_tx_header, info_0x153, &can3_tx_mailbox);
}

void EpbCanInfo0x329()
{
    alive.info_0x329++;
    if(alive.info_0x329 == 20) {
        alive.info_0x329 = 0;
    }

	can3_tx_header.StdId = 0x329;
	can3_tx_header.RTR = CAN_RTR_DATA;
	can3_tx_header.IDE = CAN_ID_STD;
	can3_tx_header.DLC = 8;

    if( alive.info_0x329 < 4) {
        info_0x329[0] = 0x0C;
    } else if( alive.info_0x329 < 8) {
        info_0x329[0] = 0x40;
    } else if( alive.info_0x329 < 12) {
        info_0x329[0] = 0x84;
    } else if( alive.info_0x329 < 16) {
        info_0x329[0] = 0xF1;
    }

    info_0x329[1] = 0xAF;
    info_0x329[2] = 0x7E;
    info_0x329[3] = 0x94;
    info_0x329[4] = 0x0A;
    info_0x329[5] = 0x20;
    info_0x329[6] = 0x00;
    info_0x329[7] = 0x14;

    HAL_CAN_AddTxMessage(&hcan3, &can3_tx_header, info_0x329, &can3_tx_mailbox);
}

void EpbCanInfo0x260()
{
    alive.info_0x260++;

	can3_tx_header.StdId = 0x260;
	can3_tx_header.RTR = CAN_RTR_DATA;
	can3_tx_header.IDE = CAN_ID_STD;
	can3_tx_header.DLC = 8;

    if( alive.info_0x260 == 0) {
        info_0x260[7] = 0x02;
    } else if( alive.info_0x260 == 1) {
        info_0x260[7] = 0x11;
    } else if( alive.info_0x260 == 2) {
        info_0x260[7] = 0x20;
    } else if( alive.info_0x260 == 3) {
        info_0x260[7] = 0x3F;
    }

    info_0x260[0] = 0x17;
    info_0x260[1] = 0x20;
    info_0x260[2] = 0x00;
    info_0x260[3] = 0x30;
    info_0x260[4] = 0x00;
    info_0x260[5] = 0x3E;
    info_0x260[6] = 0x00;

    HAL_CAN_AddTxMessage(&hcan3, &can3_tx_header, info_0x260, &can3_tx_mailbox);
}

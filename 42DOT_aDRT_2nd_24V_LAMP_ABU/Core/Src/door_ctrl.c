/*
 * door_ctrl.c
 *
 *  Created on: Nov 18, 2021
 *      Author: okskt
 */


#include "door_ctrl.h"


/*
 * Door Open Warning Control
 */
void DoorWarningControl()
{
#if 0
	if ( ign1_status == 1 ) {
		if( Door_Status.door_open_status == 1 ) {
			if( vehicle_vel >= 10 ) {
				if( ++door_warning_cnt < 33 ) {
					HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, SET);
				} else if( door_warning_cnt >= 33 ) {
					HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, RESET);
					if( door_warning_cnt == 66 ) {
						door_warning_cnt = 0;
					}
				}
			} else {
				HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, RESET);
				door_warning_cnt = 0;
			}
		} else if( Door_Status.door_open_status == 0 ) {
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, RESET);
			door_warning_cnt = 0;
		}
	} else if( ign1_status == 0 ) {
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, RESET);
		door_warning_cnt = 0;
	}
#endif
	if ( ign1_status == 1 ) {
		if( door_open_status == 1 ) {
			if( (vehicle_vel>=100) || (vehicle_vel<=-100) ) {
				Warning_Status.door_open = 1;
			} else {
				Warning_Status.door_open = 0;
			}
		} else if( door_open_status == 0 ) {
			Warning_Status.door_open = 0;
		}
	} else if( ign1_status == 0 ) {
		Warning_Status.door_open = 0;
	}
}

/*
 * Parking Brake Apply Warning Control
 */
void PBrakeWarningControl()
{
#if 0
	if ( ign1_status == 1 ) {
		if( pBrake_status == 1 ) {
			if( vehicle_vel >= 30 ) {
				if( ++pbrake_warning_cnt < 33 ) {
					HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, SET);
				} else if( pbrake_warning_cnt >= 33 ) {
					HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, RESET);
					if( pbrake_warning_cnt == 66 ) {
						pbrake_warning_cnt = 0;
					}
				}
			} else {
				HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, RESET);
				pbrake_warning_cnt = 0;
			}
		} else if( (pBrake_status==0) && (Door_Status.door_open_status==0) ) {
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, RESET);
			pbrake_warning_cnt = 0;
		}
	} else if( ign1_status == 0 ) {
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, RESET);
		pbrake_warning_cnt = 0;
	}
#endif
	if ( ign1_status == 1 ) {
		if( pBrake_status == 1 ) {
			if( vehicle_vel >= 300 ) {
				Warning_Status.pbrake = 1;
			} else {
				Warning_Status.pbrake = 0;
			}
		} else {
			Warning_Status.pbrake = 0;
		}
	} else if( ign1_status == 0 ) {
		Warning_Status.pbrake = 0;
	}
}

void SeatBeltRead()
{
	if( ign1_status == 1 ) {
		if( column_on_seq == 0 ) {
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, RESET);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, SET);				// Col 0 Drv
			if( column_on_cnt > 2 ) {
				Seat_Status.belt1 = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_4);		// DI 11
				Seat_Status.belt2 = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_2);		// DI 12
				Seat_Status.seat1 = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_3);		// DI 13
				Seat_Status.seat2 = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_4);		// DI 14
			}
		} else if( column_on_seq == 1 ) {
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, RESET);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, SET);				// Col 1 Drv
			if( column_on_cnt > 2 ) {
				Seat_Status.belt3 = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_4);
				Seat_Status.belt4 = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_2);
				Seat_Status.seat3 = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_3);
				Seat_Status.seat4 = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_4);
			}
		} else if( column_on_seq == 2 ) {
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, RESET);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, SET);				// Col 2 Drv
			if( column_on_cnt > 2 ) {
				Seat_Status.belt5 = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_4);
				Seat_Status.belt6 = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_2);
				Seat_Status.seat5 = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_3);
				Seat_Status.seat6 = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_4);
			}
		} else if( column_on_seq == 3 ) {
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, RESET);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, SET);				// Col 3 Drv
			if( column_on_cnt > 2 ) {
				Seat_Status.belt7 = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_4);
				Seat_Status.belt8 = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_2);
				Seat_Status.seat7 = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_3);
				Seat_Status.seat8 = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_4);
			}
		}
		if( ++column_on_cnt > 5 ) {
			if( ++column_on_seq > 3 ) {
				column_on_seq = 0;
			}
			column_on_cnt = 0;
		}
	} else if( ign1_status == 0 ) {
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9, RESET);
		Seat_Status.seat1 = Seat_Status.seat2 = Seat_Status.seat3 = Seat_Status.seat4 =
				Seat_Status.seat5 = Seat_Status.seat6 = Seat_Status.seat7 = Seat_Status.seat8 = 0;
		Seat_Status.belt1 = Seat_Status.belt2 = Seat_Status.belt3 = Seat_Status.belt4 =
				Seat_Status.belt5 = Seat_Status.belt6 = Seat_Status.belt7 = Seat_Status.belt8 = 0;
		column_on_seq = 0;
		column_on_cnt = 0;
	}
}

/*
 * Seatbelt Warning Control
 */
void SeatbeltWarningControl()
{
#if 0
	if( ign1_status == 1 ) {
		if( ((seat_R0_C0==1) && (seatbelt_R0_C0==0)) || ((seat_R1_C0==1) && (seatbelt_R1_C0==0)) ||
				((seat_R0_C1==1) && (seatbelt_R0_C1==0)) || ((seat_R1_C1==1) && (seatbelt_R1_C1==0)) ||
				((seat_R0_C2==1) && (seatbelt_R0_C2==0)) || ((seat_R1_C2==1) && (seatbelt_R1_C2==0)) ||
				((seat_R0_C3==1) && (seatbelt_R0_C3==0)) || ((seat_R1_C3==1) && (seatbelt_R0_C3==0)) ) {
			pass_seatbelt_warning = 1;
			if( (vehicle_vel>30) && (pass_seatbelt_cnt==0) ) {
				HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, SET);
			}
			if( ++pass_seatbelt_cnt > 2000 ) {
				HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, RESET);
				pass_seatbelt_cnt = 1;
			}
		} else {
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, RESET);
			pass_seatbelt_warning = 0;
			pass_seatbelt_cnt = 0;
		}

		if( (drv_seat_status==1) && (drv_seatbelt_status==0) ) {
			drv_seatbelt_warning = 1;
			if( (vehicle_vel>30) && (drv_seatbelt_cnt==0) ) {
				HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, SET);
			}
			if( ++drv_seatbelt_cnt > 2000 ) {
				HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, RESET);
				drv_seatbelt_cnt = 1;
			}
		} else {
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, RESET);
			drv_seatbelt_warning = 0;
			drv_seatbelt_cnt = 0;
		}
	}
#endif
	if( ign1_status == 1 ) {
		if( (Seat_Status.seat1==1) && (Seat_Status.belt1==1) ) {
			Warning_Status.seat1 = 1;
		} else {
			Warning_Status.seat1 = 0;
		}
		if( (Seat_Status.seat2==1) && (Seat_Status.belt2==1) ) {
			Warning_Status.seat2 = 1;
		} else {
			Warning_Status.seat2 = 0;
		}
		if( (Seat_Status.seat3==1) && (Seat_Status.belt3==1) ) {
			Warning_Status.seat3 = 1;
		} else {
			Warning_Status.seat3 = 0;
		}
		if( (Seat_Status.seat4==1) && (Seat_Status.belt4==1) ) {
			Warning_Status.seat4 = 1;
		} else {
			Warning_Status.seat4 = 0;
		}
		if( (Seat_Status.seat5==1) && (Seat_Status.belt5==1) ) {
			Warning_Status.seat5 = 1;
		} else {
			Warning_Status.seat5 = 0;
		}
		if( (Seat_Status.seat6==1) && (Seat_Status.belt6==1) ) {
			Warning_Status.seat6 = 1;
		} else {
			Warning_Status.seat6 = 0;
		}
		if( (Seat_Status.seat7==1) && (Seat_Status.belt7==1) ) {
			Warning_Status.seat7 = 1;
		} else {
			Warning_Status.seat7 = 0;
		}
		if( (Seat_Status.seat8==1) && (Seat_Status.belt8==1) ) {
			Warning_Status.seat8 = 1;
		} else {
			Warning_Status.seat8 = 0;
		}

		if( (Warning_Status.seat1==1) || (Warning_Status.seat2==1) || (Warning_Status.seat3==1) || (Warning_Status.seat4==1) ||
				(Warning_Status.seat5==1) || (Warning_Status.seat6==1) || (Warning_Status.seat7==1) || (Warning_Status.seat8==1) ) {
			Warning_Status.seatbelt = 1;
		} else {
			Warning_Status.seatbelt = 0;
		}
	}
}

/*
 * 충동센서 Control
 */
void ImpactControl()
{
	if( vehicle_vel > 200 ) {
		if( impact_status == 1 ) {
			hazard_lamp_on = 1;
//			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, SET);		// Door Unlock Drv
			if( impact_cnt > 10 ) {
//				HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, SET);		// Door Open Drv
			}
		}
	}
}

void SeatbeltDataConv(uint8_t *txdata, uint8_t *txflag)
{
	if( *txflag ) {
		txdata[0] = (Warning_Status.seat1) | (Warning_Status.seat2<<1) | (Warning_Status.seat3<<2) | (Warning_Status.seat4<<3) |
				(Warning_Status.seat5<<4) | (Warning_Status.seat6<<5) | (Warning_Status.seat7<<6) | (Warning_Status.seat8<<7);
		txdata[1] = Seat_Status.seat1 | (Seat_Status.seat2<<1) | (Seat_Status.seat3<<2) | (Seat_Status.seat4<<3) |
				(Seat_Status.seat5<<4) | (Seat_Status.seat6<<5) | (Seat_Status.seat7<<6) | (Seat_Status.seat8<<7);
		txdata[2] = Seat_Status.belt1 | (Seat_Status.belt2<<1) | (Seat_Status.belt3<<2) | (Seat_Status.belt4<<3) |
				(Seat_Status.belt5<<4) | (Seat_Status.belt6<<5) | (Seat_Status.belt7<<6) | (Seat_Status.belt8<<7);
		txdata[3] = (Warning_Status.door_open<<1) | (Warning_Status.pbrake<<2) | (Warning_Status.seatbelt<<3);
		txdata[4] = distant_volt;
		txdata[5] = distant_volt >> 8;
		txdata[6] = sun_volt;
		txdata[7] = sun_volt >> 8;
	}
}

void WarningAlarmControl()
{
	static uint8_t warning_cnt;

	if( (distant_alarm==1) &&  (distant_alarm_on==0) ) {
		distant_alarm_on = 1;
	}

	if( Warning_Status.door_open == 1 ) {
		if( ++warning_cnt < 33) {
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, SET);
//			HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_8);
		} else if( warning_cnt < 67 ) {
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, RESET);
			if( warning_cnt == 66 ) {
				warning_cnt = 0;
			}
		}
	} else if( Warning_Status.pbrake == 1 ) {
		if( ++warning_cnt < 33) {
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, SET);
//			HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_8);
		} else if( warning_cnt < 67 ) {
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, RESET);
			if( warning_cnt == 66 ) {
				warning_cnt = 0;
			}
		}
	} else if ( Warning_Status.seatbelt == 1 )  {
		if( vehicle_vel > 300 ) {
			if( ++warning_cnt < 33) {
				HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, SET);
//				HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_8);
			} else if( warning_cnt < 67 ) {
				HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, RESET);
				if( warning_cnt == 66 ) {
					warning_cnt = 0;
				}
			}
		} else if( vehicle_vel <= 300 )  {
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, RESET);
		}

	} else if( distant_alarm_on == 1 ) {
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, SET);
		if( ++distant_alarm_on_cnt > 16 ) {
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, RESET);
			distant_alarm_on = 0;
			distant_alarm_on_cnt = 0;
		}

	} else if( (Warning_Status.door_open==0) && (Warning_Status.pbrake==0) && (Warning_Status.seatbelt==0) && (distant_alarm_on==0) ) {
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, RESET);
		warning_cnt = 0;
	}
	distant_alarm_prev = distant_alarm;
}



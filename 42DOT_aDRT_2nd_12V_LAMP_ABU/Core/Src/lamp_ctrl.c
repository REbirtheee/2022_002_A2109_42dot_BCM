/*
 * lamp_ctrl.c
 *
 *  Created on: Jan 4, 2022
 *      Author: okskt
 */


#include "lamp_ctrl.h"

uint8_t boardingpassAVAS_first_step = 0;


void LampSWRead(struct InputStatus_s *input_status)
{
	if( auto_mode == 1 ) {
		Input_Status_Raw.turnlamp_LH = akit_command.turn_left_on;
		Input_Status_Raw.turnlamp_RH = akit_command.turn_right_on;
		Input_Status_Raw.taillamp = akit_command.tail_on;
		Input_Status_Raw.headlamp_L = akit_command.head_low_on;
		if( (akit_command.tail_on==0) && (akit_command.head_low_on==0) ) {
			Input_Status_Raw.headlamp_H = 0;
			Input_Status_Raw.headlamp_H_passing = akit_command.head_high_on;
		} else {
			Input_Status_Raw.headlamp_H_passing = 0;
			Input_Status_Raw.headlamp_H = akit_command.head_high_on;
		}
		if( akit_command.lamp_off == 1 ) {
			Input_Status_Raw.taillamp = Input_Status_Raw.headlamp_L = Input_Status_Raw.headlamp_H = Input_Status_Raw.headlamp_H_passing = 0;
		}
	} else if ( auto_mode == 0 ) {
		Input_Status_Raw.turnlamp_LH = READ_PIN_DI_0;			// DI 0
		Input_Status_Raw.turnlamp_RH = READ_PIN_DI_1;			// DI 1
		Input_Status_Raw.taillamp = READ_PIN_DI_2;				// DI 2
		Input_Status_Raw.headlamp_L = READ_PIN_DI_3;			// DI 3
		Input_Status_Raw.headlamp_H = READ_PIN_DI_5;			// DI 5
		Input_Status_Raw.headlamp_H_passing = READ_PIN_DI_6;	// DI 6
	}

	Input_Status_Raw.charge_door_status = READ_PIN_DI_4;		// DI 3
	if( ign1_status == 0 ) {
		if( pBrake_status == 1 ) {
			Input_Status_Raw.RKE_lock = READ_PIN_DI_7;			// DI 7
			Input_Status_Raw.RKE_unlock = READ_PIN_DI_8;		// DI 8
		}
	}
	Input_Status_Raw.door_open_status = READ_PIN_DI_9;			// DI 9
	Input_Status_Raw.accel_sw = READ_PIN_DI_11;					// DI 11
	Input_Status_Raw.decel_sw = READ_PIN_DI_12;					// DI 12
	Input_Status_Raw.drive_mode_sw = READ_PIN_DI_13;			// DI 13
	Input_Status_Raw.logging_sw = READ_PIN_DI_14;				// DI 14
	Input_Status_Raw.marker_sw = READ_PIN_DI_15^0x01;			// DI 15

	if( Input_Status_Raw.headlamp_L == 1 ) {	// For Simulator
		Input_Status_Raw.taillamp = 1;
	}

	InputSWChatt(input_status);		// Switch Input Chattering Function

	Input_Status_Prev.taillamp = Input_Status_Raw.taillamp;
	Input_Status_Prev.headlamp_L = Input_Status_Raw.headlamp_L;
	Input_Status_Prev.headlamp_H = Input_Status_Raw.headlamp_H;
	Input_Status_Prev.headlamp_H_passing = Input_Status_Raw.headlamp_H_passing;
	Input_Status_Prev.turnlamp_LH = Input_Status_Raw.turnlamp_LH;
	Input_Status_Prev.turnlamp_RH = Input_Status_Raw.turnlamp_RH;
	Input_Status_Prev.RKE_lock = Input_Status_Raw.RKE_lock;
	Input_Status_Prev.RKE_unlock = Input_Status_Raw.RKE_unlock;
	Input_Status_Prev.door_open_status = Input_Status_Raw.door_open_status;
	Input_Status_Prev.accel_sw = Input_Status_Raw.accel_sw;
	Input_Status_Prev.decel_sw = Input_Status_Raw.decel_sw;
	Input_Status_Prev.drive_mode_sw = Input_Status_Raw.drive_mode_sw;
	Input_Status_Prev.logging_sw = Input_Status_Raw.logging_sw;
	Input_Status_Prev.marker_sw = Input_Status_Raw.marker_sw;
	Input_Status_Prev.charge_door_status = Input_Status_Raw.charge_door_status;
}

void InputSWChatt(struct InputStatus_s *input_status)
{
	if( Input_Status_Prev.taillamp == Input_Status_Raw.taillamp ) {
		if( ++Chattering.taillamp_cnt > 2 ) {
			Input_Status.taillamp = Input_Status_Raw.taillamp;
			Chattering.taillamp_cnt = 3;
		}
	} else {
		Chattering.taillamp_cnt = 0;
	}

	if( Input_Status_Prev.headlamp_L == Input_Status_Raw.headlamp_L ) {
		if( ++Chattering.headalamp_low_cnt > 2 ) {
			input_status->headlamp_L = Input_Status_Raw.headlamp_L;
			Chattering.headalamp_low_cnt = 3;
		}
	} else {
		Chattering.headalamp_low_cnt = 0;
	}

	if( Input_Status_Prev.headlamp_H == Input_Status_Raw.headlamp_H ) {
		if( ++Chattering.headalamp_high_cnt > 2 ) {
			input_status->headlamp_H = Input_Status_Raw.headlamp_H;
			Chattering.headalamp_high_cnt = 3;
		}
	} else {
		Chattering.headalamp_high_cnt = 0;
	}

	if( Input_Status_Prev.headlamp_H_passing == Input_Status_Raw.headlamp_H_passing ) {
		if( ++Chattering.headlamp_high_passing_cnt > 2 ) {
			input_status->headlamp_H_passing = Input_Status_Raw.headlamp_H_passing;
			Chattering.headlamp_high_passing_cnt = 3;
		}
	} else {
		Chattering.headlamp_high_passing_cnt = 0;
	}

	if( Input_Status_Prev.turnlamp_LH == Input_Status_Raw.turnlamp_LH ) {
		if( ++Chattering.turnlamp_left_cnt > 2 ) {
			input_status->turnlamp_LH = Input_Status_Raw.turnlamp_LH;
			Chattering.turnlamp_left_cnt = 3;
		}
	} else {
		Chattering.turnlamp_left_cnt = 0;
	}

	if( Input_Status_Prev.turnlamp_RH == Input_Status_Raw.turnlamp_RH ) {
		if( ++Chattering.turnlamp_right_cnt > 2 ) {
			input_status->turnlamp_RH = Input_Status_Raw.turnlamp_RH;
			Chattering.turnlamp_right_cnt = 3;
		}
	} else {
		Chattering.turnlamp_right_cnt = 0;
	}

	if( Input_Status_Prev.RKE_lock == Input_Status_Raw.RKE_lock ) {
		if( ++Chattering.RKE_lock_cnt > 2 ) {
			RF_door_close_sig_on = input_status->RKE_lock = Input_Status_Raw.RKE_lock;
			Chattering.RKE_lock_cnt = 3;
		}
	} else {
		Chattering.RKE_lock_cnt = 0;
	}

	if( Input_Status_Prev.RKE_unlock == Input_Status_Raw.RKE_unlock ) {
		if( ++Chattering.RKE_unlock_cnt > 1 ) {
			RF_door_open_sig_on = input_status->RKE_unlock = Input_Status_Raw.RKE_unlock;
			Chattering.RKE_unlock_cnt = 3;
		}
	} else {
		Chattering.RKE_unlock_cnt = 0;
	}

	if( Input_Status_Prev.door_open_status == Input_Status_Raw.door_open_status ) {
		if( ++Chattering.door_open_status_cnt > 2 ) {
			input_status->door_open_status = Input_Status_Raw.door_open_status;
			Chattering.door_open_status_cnt = 3;
		}
	} else {
		Chattering.door_open_status_cnt = 0;
	}

	if( Input_Status_Prev.accel_sw == Input_Status_Raw.accel_sw ) {
		if( ++Chattering.accel_sw_cnt > 2 ) {
			input_status->accel_sw = Input_Status_Raw.accel_sw;
			Chattering.accel_sw_cnt = 3;
		}
	} else {
		Chattering.accel_sw_cnt = 0;
	}

	if( Input_Status_Prev.decel_sw == Input_Status_Raw.decel_sw ) {
		if( ++Chattering.decel_sw_cnt > 2 ) {
			input_status->decel_sw = Input_Status_Raw.decel_sw;
			Chattering.decel_sw_cnt = 3;
		}
	} else {
		Chattering.decel_sw_cnt = 0;
	}

	if( Input_Status_Prev.drive_mode_sw == Input_Status_Raw.drive_mode_sw ) {
		if( ++Chattering.drive_mode_sw_cnt > 2 ) {
			input_status->drive_mode_sw = Input_Status_Raw.drive_mode_sw;
			Chattering.drive_mode_sw_cnt = 3;
		}
	} else {
		Chattering.drive_mode_sw_cnt = 0;
	}

	if( Input_Status_Prev.logging_sw == Input_Status_Raw.logging_sw ) {
		if( ++Chattering.logging_sw_cnt > 2 ) {
			input_status->logging_sw = Input_Status_Raw.logging_sw;
			Chattering.logging_sw_cnt = 3;
		}
	} else {
		Chattering.logging_sw_cnt = 0;
	}

	if( Input_Status_Prev.marker_sw == Input_Status_Raw.marker_sw ) {
		if( ++Chattering.marker_sw_cnt > 2 ) {
			input_status->marker_sw = Input_Status_Raw.marker_sw;
			Chattering.marker_sw_cnt = 3;
		}
	} else {
		Chattering.marker_sw_cnt = 0;
	}

	if( Input_Status_Prev.charge_door_status == Input_Status_Raw.charge_door_status ) {
		if( ++Chattering.charge_door_status_cnt > 2 ) {
			input_status->charge_door_status = Input_Status_Raw.charge_door_status;
			Chattering.charge_door_status_cnt = 3;
		}
	} else {
		Chattering.charge_door_status_cnt = 0;
	}
}

void LampControl(struct LampStatus_s *lamp_status, struct InputStatus_s *input_status)
{
	static uint8_t speaker_on_cnt = 0;
	static uint8_t speaker_off_cnt = 0;

	/*
	 * 24V Converter Control
	 * if stop lamp, ign1, tail lamp on, then convertor on
	 */
	if( (door_open_on==0) && (door_close_on==0) ) {
		if( (stop_lamp_on==1) || (ign1_status==1) || (input_status->taillamp==1) ) {
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, SET);
			convertor_status = 1;
		} else if( (stop_lamp_on==0) && (ign1_status==0) && (input_status->taillamp==0) ) {
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, RESET);
			convertor_status = 0;
		}
	}

	/*
	 * auto-light on/off
	 * if ign on, tail off, then auto light on
	 */
	if( ign1_status == 1 ) {
		if( auto_mode == 0 ) {
			if( input_status->taillamp == 0 ) {
				auto_light_status = 1;
			} else if( input_status->taillamp == 1 ) {
				auto_light_status = 0;
			}
		} else if( auto_mode == 1 ) {
			if( (akit_command.tail_on==0) && (akit_command.head_low_on==0) && (akit_command.head_high_on==0) && (akit_command.lamp_off==0) ) {
				auto_light_status = 1;
			} else {
				auto_light_status = 0;
			}
		}
	} else if( ign1_status == 0 ) {
		auto_light_status = 0;
	}

	/*
	 *	Tail/Head Lamp Control Function
	 */
	if( input_status->taillamp == 1 ) {
		if( auto_light_status == 0 ) {
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, SET);				// TailLamp ON ( No TailLamp ) -> LicenseLamp ON
			lamp_status->tail_lamp = 1;
		}

	} else if ( input_status->taillamp == 0 ) {
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, RESET);				// Head-High OFF
		if( auto_light_status == 0 ) {
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, RESET);			// TailLamp, Head-Low OFF ( No TailLamp ) -> LicenseLamp OFF
			lamp_status->tail_lamp = 0;
		}
	}

	/*
	 * Head-High Lamp Control
	 */
	if( lamp_status->tail_lamp == 1 ) {
		if( (input_status->headlamp_H == 1) || (input_status->headlamp_H_passing==1) ) {
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, SET);
			lamp_status->head_lamp_high = 1;					// HeadLamp High ON
		} else if( (input_status->headlamp_H==0) && (input_status->headlamp_H_passing==0) ) {
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, RESET);
			lamp_status->head_lamp_high = 0;					// HeadLamp High OFF
		}

		if( input_status->headlamp_H_passing == 0 ) {
			if( auto_light_status == 0 ) {
				if( input_status->headlamp_L == 1 ) {
					lamp_status->head_lamp_low = 1;			// HeadLamp Low ON
				} else if ( input_status->headlamp_L == 0 ) {
					lamp_status->head_lamp_low = 0;			// HeadLamp Low OFF
				}
			}
		} else if( input_status->headlamp_H_passing == 1 ) {
			if( auto_light_status == 0 ) {
				lamp_status->head_lamp_low = 0;				// HeadLamp Low ON
			}
		}

	} else if( lamp_status->tail_lamp == 0 ) {
//		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, RESET);
		lamp_status->head_lamp_low = 0;		// HeadLamp High/Low OFF
		if( input_status->headlamp_H_passing == 1 ) {
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, SET);
			lamp_status->head_lamp_high = 1;
		} else if( input_status->headlamp_H_passing == 0 ) {
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, RESET);
			lamp_status->head_lamp_high = 0;
		}
	}

	/*
	 *	Hazard Lamp Control Function
	 */
	if( hazard_sw_on == 1 ) {
		if( dec_hazard_on == 1 ) {
			dec_hazard_on = 0;
		}
	}
	if( (hazard_sw_on==1) || (dec_hazard_on==1) ) {
		turnlamp_left_onoff_cnt = turnlamp_right_onoff_cnt = 0;
		if( ++hazard_onoff_cnt < 24 ) {
			lamp_status->hazard_lamp = lamp_status->turn_lamp_left = lamp_status->turn_lamp_right = 1;
		} else if ( hazard_onoff_cnt >= 24 ) {
			lamp_status->hazard_lamp = lamp_status->turn_lamp_left = lamp_status->turn_lamp_right = 0;
			if( hazard_onoff_cnt == 47 ) {
				hazard_onoff_cnt = 0;
			}
		}
	} else if( hazard_sw_on == 0 ) {
		lamp_status->hazard_lamp = 0;
		hazard_onoff_cnt = 0;
		if( input_status->turnlamp_LH == 0) {
			lamp_status->turn_lamp_left = 0;
		}
		if( input_status->turnlamp_RH == 0 ) {
			lamp_status->turn_lamp_right = 0;
		}
	}

	if( ign1_status == 1 ) {
		/*
		 * Front-Speaker Control Function
		 */

#if 1 //220707 forward AVAS off -> 220708 modify
		if( vehicle_vel <= 0 ) {								// 0 km/h -> Speaker OFF
			speaker_on_cnt = 0;
			if( ++speaker_off_cnt > 10 ) {
				HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, RESET);
				speaker_off_cnt = 10;
			}
		} else if((vehicle_vel > 0) && (vehicle_vel<=200) && (gear_position != 1)) {	// ~20 km/h && N or D -> Speaker ON
			speaker_off_cnt = 0;
			if( ++speaker_on_cnt > 10 ) {
				HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, SET);
				speaker_on_cnt = 10;
			}
		} else if( vehicle_vel > 200 ) {						// 20~ km/h -> Speaker OFF
			speaker_on_cnt = 0;
			if( ++speaker_off_cnt > 10 ) {
				HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, RESET);
				speaker_off_cnt = 10;
			}
		} else {
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, RESET);
		}
#endif


		/*
		 *	Turn Lamp Control Function
		 */
		if( hazard_sw_on == 0 ) {
			if( input_status->turnlamp_LH == 1 ) {
				if( turn_lamp_left_fail == 0 ) {
					// normal turn lamp function
					if( ++turnlamp_left_onoff_cnt < 24 ) {
						lamp_status->turn_lamp_left = 1;
					} else if ( turnlamp_left_onoff_cnt >= 24 ) {
						lamp_status->turn_lamp_left = 0;
						if( turnlamp_left_onoff_cnt > 47 ) {
							turnlamp_left_onoff_cnt = 0;
						}
					}
				} else if( turn_lamp_left_fail == 1 ) {
					// turn lamp fail function
					if( ++turnlamp_left_onoff_cnt < 6 ) {
						lamp_status->turn_lamp_left = 1;
					} else if ( turnlamp_left_onoff_cnt >= 6 ) {
						lamp_status->turn_lamp_left = 0;
						if( turnlamp_left_onoff_cnt > 12 ) {
							turnlamp_left_onoff_cnt = 0;
						}
					}
				}

			} else if( input_status->turnlamp_LH == 0 ) {
				lamp_status->turn_lamp_left = 0;
				turnlamp_left_onoff_cnt = 0;

				if( input_status->turnlamp_RH == 1 ) {
					if( turn_lamp_right_fail == 0 ) {
						// normal turn lamp function
						if( ++turnlamp_right_onoff_cnt < 24 ) {
							lamp_status->turn_lamp_right = 1;
						} else if ( turnlamp_right_onoff_cnt >= 24 ) {
							lamp_status->turn_lamp_right = 0;
							if( turnlamp_right_onoff_cnt > 47 ) {
								turnlamp_right_onoff_cnt = 0;
							}
						}
					} else if( turn_lamp_right_fail == 1 ) {
						// turn lamp fail function
						if( ++turnlamp_right_onoff_cnt < 6 ) {
							lamp_status->turn_lamp_right = 1;
						} else if ( turnlamp_right_onoff_cnt >= 6 ) {
							lamp_status->turn_lamp_right = 0;
							if( turnlamp_right_onoff_cnt > 12 ) {
								turnlamp_right_onoff_cnt = 0;
							}
						}
					}
				} else if( input_status->turnlamp_RH == 0 ) {
					lamp_status->turn_lamp_right = 0;
					turnlamp_right_onoff_cnt = 0;
				}
			}
		}

		/*
		 *	Auto-light Lamp Control Function
		 */
		if( auto_light_status == 1 ) {
			if( (light_data>350) && (light_data<600) ) {  // 730 ~ 1400
				auto_tail_off_cnt = 0;
				if( ++auto_tail_on_cnt > 25 ) {
					HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, SET);					// TailLamp ON ( No TailLamp ) -> LicenseLamp ON
					lamp_status->tail_lamp = lamp_status->head_lamp_low = 1;	// HeadLamp Low ON
					if( input_status->headlamp_H_passing == 1 ) {
						lamp_status->head_lamp_low = 0;
					}
					auto_tail_on_cnt = 25;
				}
			} else if( light_data >= 600 ) { //1400
				auto_tail_on_cnt = 0;
				if( ++auto_tail_off_cnt > 25 ) {
					HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, RESET);				// TailLamp OFF ( No TailLamp ) -> LicenseLamp OFF
					lamp_status->tail_lamp = lamp_status->head_lamp_low = 0;	// HeadLamp Low OFF
					auto_tail_off_cnt = 25;
				}
			}
			else {
				auto_tail_on_cnt = auto_tail_off_cnt = 0;
			}
		}

		if( gear_position == 1 ) {
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9, SET);		// Back Lamp ON
		} else {
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9, RESET);	// Back Lamp OFF
		}

	} else if ( ign1_status == 0 ) {
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9|GPIO_PIN_12, RESET);		// Back-Lamp, Front-Speaker OFF
	}
}

#if 0
void HazardControl()
{
	if( vehicle_dec < -20 ) {
		if( ++hazard_dec_cnt > 5 ) {
			dec_hazard_on = 1;
		}
	} else {
		hazard_dec_cnt = 0;
	}
}
#endif

void RF_DoorControl()
{
	if( ign1_status == 0 ) {
		if( pBrake_status == 1 ) {
			if( RF_door_open_sig_on == 1 ) {
				if( RF_door_open_sig_on_prev == 0 ) {
					door_open_on = 1;
					convertor_on_delay = 0;
				}
			}

			if( RF_door_close_sig_on == 1 ) {
				if( RF_door_close_sig_on_prev ) {
					door_close_on = 1;
					convertor_on_delay = 0;
				}
			}
		}
	}

	RF_door_open_sig_on_prev = RF_door_open_sig_on;
	RF_door_close_sig_on_prev = RF_door_close_sig_on;
}

void DoorControl()
{
	static uint8_t akit_door_open_prev = 0;
	static uint8_t akit_door_close_prev = 0;

//220623 -> song oper -> ultra sensor X , open trigger -> AVAS -> door open
#if 0
	if( ((vehicle_vel<=50) && (vehicle_vel>=-50)) && ((gear_position == 2 || pBrake_status == 1))) {
		if( ((DRV_door_open==1)&&(DRV_door_open_prev==0)) || ((PS_door_open==1)&&(PS_door_open_prev==0)) || ((akit_door_open_prev!=1)&&(akit_command.door_open==1)) ) {
			//door_open_on = 1;
			Door_Status.door_closing = 0;
			convertor_on_delay = 0;
			check_us_door_open = 1;
		}

  if(doorwarning_AVAS_trigger == 0){
	if(check_us_door_open){
		if(++check_us_door_open_cnt <= 10){
			//if( (Us_rear.us_rear_dist_lpf >= 40 && Us_rear.us_rear_dist_lpf <= 90) || (Us_front.us_front_dist_lpf >= 32 && Us_front.us_front_dist_lpf <=70) ) {  //40cm <-> 90cm
			  if(Us_rear.us_rear_dist_lpf >= 40 && Us_rear.us_rear_dist_lpf <= 90) {	//220523 front sensor delete.
					us_door_closing_cnt++;
						}

					}else{

						check_us_door_open_cnt = 0;
						if(us_door_closing_cnt >= 4){
							// detect
							//220523 -> door open sig -> oper avas
							doorwarning_AVAS_trigger = 1;
							HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, SET); // DW_AVAS_on

						}else{
							door_open_on = 1;
							check_us_door_open = 0;
						}
						us_door_closing_cnt = 0;
				}
			}
  }else{
	 check_us_door_open_cnt = 0;
  }

#endif

  if( ((vehicle_vel<=50) && (vehicle_vel>=-50)) && ((gear_position == 2 || pBrake_status == 1))) {
  		if( ((DRV_door_open==1)&&(DRV_door_open_prev==0)) || ((PS_door_open==1)&&(PS_door_open_prev==0)) || ((akit_door_open_prev!=1)&&(akit_command.door_open==1)) ) {
  			//door_open_on = 1;
  			Door_Status.door_closing = 0;
  			convertor_on_delay = 0;
  			doorwarning_AVAS_trigger = 1;
  			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, SET); // DW_AVAS_on
  		}


		if( door_open_on == 1 ) {
			if( convertor_status == 1 ) {  // 시동 켜져 있을때

				door_close_on_cnt = door_close_on = 0;					// 21_12_30 Update
				HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, RESET);
				HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, SET);
				Door_Status.door_opening = 1;		// 21_12_30 Update

				if( ++door_open_on_cnt > 300 ) {  // 6sec
					HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, RESET);
//					Door_Status.door_opening = 1;	// 21_12_30 Update
					door_open_on = 0;
					door_open_on_cnt = 0;
					distant_door_open = 0;
				}
			} else if( convertor_status == 0 ) {  // 시동 꺼져 있을때,
				HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, SET);
				if( ++convertor_on_delay > 50 ) {
					convertor_on_delay = 50;

					door_close_on_cnt = door_close_on = 0;					// 21_12_30 Update
					HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, RESET);
					HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, SET);
					Door_Status.door_opening = 1;		// 21_12_30 Update

					if( ++door_open_on_cnt > 300 ) {
						HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, RESET);
//						Door_Status.door_opening = 1;	// 21_12_30 Update
						door_open_on = 0;
						door_open_on_cnt = 0;
						distant_door_open = 0;
						convertor_on_delay = 0;
					}
				}
			}
		}

		if( Input_Status.door_open_status == 1 ) {
			if( ((DRV_door_close==1)&&(DRV_door_close_prev==0)) || ((PS_door_close==1)&&(PS_door_close_prev==0)) || ((akit_door_close_prev!=1)&&(akit_command.door_close==1)) ) {
				if( door_open_on == 0 ) {
					door_close_on = 1;
					door_open_comp = 0;
					Door_Status.door_opening = 0;
					convertor_on_delay = 0;
					//check_us_door_close = 1;
				}
			}
		}

		/*
		if(check_us_door_close){
			if(++check_us_door_close_cnt <= 10){
				if( Us_front.us_front_dist_lpf >= 40 && Us_front.us_front_dist_lpf <= 60 ) {  //40cm <-> 90cm
						us_door_opening_cnt++;
							}

						}else{
							check_us_door_close = 0;
							check_us_door_close_cnt = 0;
							if(us_door_opening_cnt >= 3){
								// detect
							}else{
								door_close_on = 1;
							}
							us_door_opening_cnt = 0;
					}
				}
		 */

		if( door_close_on == 1 ) {
			if( convertor_status == 1 ) {
				HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, RESET);
				HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, SET);
				Door_Status.door_closing = 1;			// 21_12_30 Update
				if( ++door_close_on_cnt > 350 ) {
					HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, RESET);
					door_close_on = 0;					// 21_12_30 Update
//					Door_Status.door_closing = 1;		// 21_12_30 Update
					door_close_on_cnt = 0;
				}
			} else if( convertor_status == 0 ) {
				HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, SET);
				if( ++convertor_on_delay > 50 ) {
					convertor_on_delay = 50;

					HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, RESET);
					HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, SET);
					Door_Status.door_closing = 1;			// 21_12_30 Update
					if( ++door_close_on_cnt > 350 ) {
						HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, RESET);
						door_close_on = 0;					// 21_12_30 Update
//						Door_Status.door_closing = 1;		// 21_12_30 Update
						door_close_on_cnt = 0;
						convertor_on_delay = 0;
					}
				}
			}
		}
	}else{ //220523 speed up && R/D -> open fuction All off
		door_open_on = 0;
		Door_Status.door_closing = 0;
		convertor_on_delay = 0;
		check_us_door_open = 0;
	}

	DRV_door_open_prev = DRV_door_open;
	DRV_door_close_prev = DRV_door_close;
	PS_door_open_prev = PS_door_open;
	PS_door_close_prev = PS_door_close;
	akit_door_open_prev = akit_command.door_open;
	akit_door_close_prev = akit_command.door_close;


	if( Door_Status.door_opening == 1 ) {
		if( ++door_open_comp_cnt > 66 ) {
			door_open_comp = 1;
			door_open_comp_cnt = 0;
			Door_Status.door_opening = 0;
		}
	} else if( Door_Status.door_closing == 1 ) {
		door_open_comp = 0;
		door_open_comp_cnt = 0;
	}
}

/*
 * ?��?�� 방�? Control
 */
void DistantDoorControl()
{
	if( Input_Status.door_open_status == 1 ) {
		if( Door_Status.door_closing == 1 ) {
			if( distant_data >= 1200 ) { // 50 ~ 150cm  // 5000
				if( ++distant_door_open_cnt > 3 ) {
					distant_door_open = 1;		// before sensor
					distant_door_open_cnt = 0;
				}
			} else {
				distant_door_open_cnt = 0;
			}
		}
		if( distant_door_open == 1 ) {
			door_open_on = 1;
			Door_Status.door_closing = 0;
			distant_door_open = 0;
		}

	} else if( Input_Status.door_open_status == 0 ) {
		distant_door_open = 0;
		distant_door_open_cnt = 0;
	}
}

void usdoor_control()
{
	if( Input_Status.door_open_status == 0 ) {
		if( Door_Status.door_opening == 1 ) {
			if( Us_rear.us_rear_dist_lpf >= 40 && Us_rear.us_rear_dist_lpf <= 90 ) {
				if( ++us_door_closing_cnt > 3 ) {
					us_door_closing = 1;		// before sensor
					us_door_closing_cnt = 0;
				}
			} else {
				us_door_closing_cnt = 0;
			}
		}
		if( us_door_closing == 1 ) {
			door_close_on = 1; // close sig on
			door_open_on= 0;  // open off
			//Door_Status.door_opening = 0;
			us_door_closing = 0;
			}
		} else if( Input_Status.door_open_status == 1 ) {
			//distant_door_open = 0;
			//distant_door_open_cnt = 0;
		}

}

void ButtonControl(struct ButtonStatus_s *button_status, struct InputStatus_s *input_status)
{
	if( input_status->accel_sw == 1 ) {
		button_status->accel_decel_sw = 1;
	} else if( input_status->decel_sw == 1 ) {
		button_status->accel_decel_sw = 2;
	} else {
		button_status->accel_decel_sw = 0;
	}

	if( input_status->drive_mode_sw == 1 ) {
		button_status->drive_mode_sw = 2;
	} else if( input_status->drive_mode_sw == 0 ) {
		button_status->drive_mode_sw = 1;
	}

	if( input_status->logging_sw == 1 ) {
		button_status->logging_sw = 1;
	} else if( input_status->logging_sw == 0 ) {
		button_status->logging_sw = 2;
	}

	if( input_status->marker_sw == 1 ) {
		button_status->marker_sw = 1;
	} else if( input_status->marker_sw == 0 ) {
		button_status->marker_sw = 0;
	}
}

void LampDataConv(uint8_t *txdata, uint8_t *flag)
{
	txdata[0] = (Lamp_Status.tail_lamp) | (Lamp_Status.hazard_lamp<<1) | (Lamp_Status.turn_lamp_left<<2) | (Lamp_Status.turn_lamp_right<<3) |
			(Lamp_Status.head_lamp_low<<4) | (Lamp_Status.head_lamp_high<<5) | (Input_Status.door_open_status<<6) | (Input_Status.charge_door_status<<7);
	txdata[1] = RF_door_open_sig_on | (RF_door_close_sig_on<<1);
	txdata[2] = turn_left_curr;
	txdata[3] = turn_left_curr>>8;
	txdata[4] = turn_right_curr;
	txdata[5] = turn_right_curr>>8;
	txdata[6] = turn_lamp_left_fail | (turn_lamp_right_fail<<1);
	txdata[7] = Button_Status.accel_decel_sw | (Button_Status.drive_mode_sw<<2) | (Button_Status.logging_sw<<4) | (Button_Status.marker_sw<<6);

	(*flag) = 1;
}

void CMS_Control()
{
	if((Input_Status.door_open_status == 1) || (ign1_status == 1))
	{
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, SET);
	}else{
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, RESET);
	}

}


void boardingpass_AVAS_control()
{
	if( (Input_Status.door_open_status == 1) && (ign1_status == 1) && (boardingpass_AVAS_trigger == 0)) {  // door open check
			if( distant_data >= 600 ) { // 50 ~ 150cm  // 5000 //  before (220516)->1200
				boardingpass_AVAS_trigger = 1;

				if(boardingpassAVAS_first_step){
					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, SET); // BP_AVAS_on
				}else{
					boardingpassAVAS_first_step = 1;
				}
			}
	}

	if(boardingpass_AVAS_trigger){
		boardingpass_AVAS_cnt++;
		if(boardingpass_AVAS_cnt > 240){ // 4sec
			boardingpass_AVAS_cnt = 0;
			boardingpass_AVAS_trigger = 0;
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, RESET); // BP_AVAS_off
		}
	}

	if(ign1_status == 0){
		boardingpassAVAS_first_step = 0;
	}
}

void doorwaring_AVAS_control()
{

	/*
	if( vehicle_vel == 0 && Input_Status.door_open_status == 0 && ign1_status == 1 && (gear_position == 2 || pBrake_status == 1)  ) { // vihicle vel && door close cheek && ign1 on
		if(++check_door_warning_sensor_cnt <= 10){
			if( Us_rear.us_rear_dist_lpf >= 40 && Us_rear.us_rear_dist_lpf <= 90 ) { //  sensor distance check.
				door_warning_closing_cnt ++;
			}
		}else{
			check_door_warning_sensor_cnt = 0;
			if(door_warning_closing_cnt >= 4){
				//detect
				doorwarning_AVAS_trigger = 1;
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, SET); // DW_AVAS_on

			}
			door_warning_closing_cnt = 0;
		}

	}
	*/

	if(doorwarning_AVAS_trigger){
		doorwarning_AVAS_cnt++;
		if(doorwarning_AVAS_cnt == 100){
			door_open_on = 1;
		}
		if(doorwarning_AVAS_cnt > 350){ // 7sec
			doorwarning_AVAS_cnt = 0;
			doorwarning_AVAS_trigger = 0;
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, RESET); // DW_AVAS_off
		}
	}
}










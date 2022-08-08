/*
 * lamp_ctrl.c
 *
 *  Created on: Jan 4, 2022
 *      Author: okskt
 */


#include "lamp_ctrl.h"

#if 0
uint8_t sig_lamp_color_wht_on[11] = {0x02,
		0x32, 0x35, 0x35,			//R
		0x32, 0x34, 0x30,			//G
		0x31, 0x38, 0x30, 0x03};	//B

uint8_t sig_lamp_color_blue_on[11] = {0x02,
		0x30, 0x37, 0x39,
		0x30, 0x37, 0x33,
		0x32, 0x31, 0x38, 0x03};

uint8_t sig_lamp_color_off[11] = {0x02,
		0x30, 0x30, 0x30,
		0x30, 0x30, 0x30,
		0x30, 0x30, 0x30, 0x03};
#else
uint8_t sig_lamp_color_wht_ctl_on[14] = {0x02,
		0x30, 0x30, 0x30,			// R
		0x30, 0x30, 0x30,			// G
		0x30, 0x30, 0x30,			// B
		0x32, 0x35, 0x35, 0x03};	// W

uint8_t sig_lamp_color_blue_ctl_on[14] = {0x02,
		0x30, 0x37, 0x39,
		0x30, 0x37, 0x33,
		0x32, 0x31, 0x38,
		0x30, 0x30, 0x30, 0x03};

uint8_t sig_lamp_color_ctl_off[14] = {0x02,
		0x30, 0x30, 0x30,
		0x30, 0x30, 0x30,
		0x30, 0x30, 0x30,
		0x30, 0x30, 0x30, 0x03};
#endif


void LampSWRead(struct InputStatus_s *input_status)
{
	Input_Status_Raw.brake_push_sw = READ_PIN_DI_0;		// DI 0
	if( auto_mode == 1 ) {
		hazard_on = akit_command.hazard_cmd;
	} else if( auto_mode == 0 ) {
		Input_Status_Raw.hazardlamp = READ_PIN_DI_6;		// DI 6
	}

	InputSWChatt(input_status);

	Input_Status_Prev.brake_push_sw = Input_Status_Raw.brake_push_sw;
	Input_Status_Prev.hazardlamp = Input_Status_Raw.hazardlamp;
	hazard_push_prev = hazard_push;
}

void InputSWChatt(struct InputStatus_s *input_status)
{
	if( Input_Status_Prev.brake_push_sw == Input_Status_Raw.brake_push_sw ) {
		if( ++Chattering.brake_push_sw_cnt > 1 ) {
			input_status->brake_push_sw = Input_Status_Raw.brake_push_sw;
			Chattering.brake_push_sw_cnt = 3;
		}
	} else {
		Chattering.brake_push_sw_cnt = 0;
	}

	if( Input_Status_Prev.hazardlamp == Input_Status_Raw.hazardlamp ) {
		if( ++Chattering.hazardlamp_cnt > 1 ) {
			hazard_push = input_status->hazardlamp = Input_Status_Raw.hazardlamp;
			if( (hazard_push_prev==0) && (hazard_push==1) ) {
				if( hazard_on == 0 ) {
					hazard_on = 1;
				} else if( hazard_on == 1 ) {
					hazard_on = 0;
				}
			}
			Chattering.hazardlamp_cnt = 3;
		}
	} else {
		Chattering.hazardlamp_cnt = 0;
	}
}

void LampControl(struct LampStatus_s *lamp_status, struct InputStatus_s *input_status)
{
	static uint8_t uart_data_tx_cnt;
#if 0		// for signature lamp test
	static uint8_t headlamp_low_on_prev;
	if( (headlamp_low_on_prev==0) && (headlamp_low_on==1) ) {
		HAL_UART_Transmit(&huart1, sig_lamp_color_blue_on, 11, 100);
	} else if( (headlamp_low_on_prev==1) && (headlamp_low_on==0) ) {
		HAL_UART_Transmit(&huart1, sig_lamp_color_wht_on, 11, 100);
	}
	headlamp_low_on_prev = headlamp_low_on;
#else
//	if( headlamp_low_on == 1 ) {
//		if( ++uart_data_tx_cnt > 50 ) {
//			HAL_UART_Transmit(&huart1, sig_lamp_color_blue_on, 11, 100);
//			uart_data_tx_cnt = 0;
//		}
//	} else if( headlamp_low_on == 0 ) {
//		if( ++uart_data_tx_cnt > 50 ) {
//			HAL_UART_Transmit(&huart1, sig_lamp_color_wht_on, 11, 100);
//			uart_data_tx_cnt = 0;
//		}
//	}
#endif

	/*
	 * Head Lamp Low Control
	 */
	if( headlamp_low_on == 1 ) {
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, SET);
	} else if ( headlamp_low_on == 0 ) {
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, RESET);
		head_low_lamp_fail_on_cnt = 0;
	}

#if 0
	if( (RF_open_lamp_on==0) && (RF_close_lamp_on==0) && (welcome_on==0) && (welcome_off==0) ) {
	/*
	 * Hazard Lamp Control
	 */
		if( (taillamp_on==1) && (hazard_on==0) ) {
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, SET);
		} else if( (taillamp_on==0) && (hazard_on==0) ) {
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, RESET);
		}
		if( (hazard_on==1) || (ems_status==1) ) {
			lamp_status->hazard_lamp = 1;
			if( turnlamp_left_on == 1 ) {
				HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10|GPIO_PIN_12, SET);
			} else if( turnlamp_left_on == 0 ) {
				HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10|GPIO_PIN_12, RESET);
			}
			if( turnlamp_right_on == 1 ) {
				HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, SET);
			} else if( turnlamp_right_on == 0 ) {
				HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, RESET);
			}

		} else if( (hazard_on==0) && (ems_status==0) ) {
			lamp_status->hazard_lamp = 0;
			hazard_onoff_cnt = 0;
			if( ign1_status == 1 ) {
				if( turnlamp_left_on == 1 ) {
					HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, RESET);
					HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, SET);
				} else if( turnlamp_left_on == 0 ) {
					HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, RESET);
					if( turnlamp_right_on == 1) {
						HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, SET);
					} else if( turnlamp_right_on == 0 ) {
						HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, RESET);
					}
				}
			} else if( ign1_status == 0 ) {
				HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10 | GPIO_PIN_11, RESET);
			}
		}
	}
#else
	if( (RF_open_lamp_on==0) && (RF_close_lamp_on==0) ) {
		/*
		 * Hazard Lamp Control
		 */
		if( (taillamp_on==1) && (hazard_on==0) ) {
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, SET);
		} else if( (taillamp_on==0) && (hazard_on==0) ) {
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, RESET);
		}
		if( (hazard_on==1) || (ems_status==1) ) {
			lamp_status->hazard_lamp = 1;
			if( turnlamp_left_on == 1 ) {
				HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10|GPIO_PIN_12, SET);
			} else if( turnlamp_left_on == 0 ) {
				HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10|GPIO_PIN_12, RESET);
			}
			if( turnlamp_right_on == 1 ) {
				HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, SET);
			} else if( turnlamp_right_on == 0 ) {
				HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, RESET);
			}

		} else if( (hazard_on==0) && (ems_status==0) ) {
			lamp_status->hazard_lamp = 0;
			hazard_onoff_cnt = 0;
			if( (welcome_on==0) && (welcome_off==0) ) {
				if( ign1_status == 1 ) {
					if( turnlamp_left_on == 1 ) {
						HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, RESET);
						HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, SET);
					} else if( turnlamp_left_on == 0 ) {
						HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, RESET);
						if( turnlamp_right_on == 1) {
							HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, SET);
						} else if( turnlamp_right_on == 0 ) {
							HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, RESET);
						}
					}

				} else if( ign1_status == 0 ) {
					HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10 | GPIO_PIN_11, RESET);
				}
			}
		}
	}
#endif

	/*
	 *	Stop Lamp Control Function
	 */
	if( ign1_status == 0 ) {
		brake_status = 0;		// if ign1_status==0, ignore brake_status from ICCU
	}
	if( (brake_status==1) || (input_status->brake_push_sw==1) ) {
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9, SET);			// Stop Lamp ON
		lamp_status->stop_lamp = 1;
	} else if ( (brake_status==0) && (input_status->brake_push_sw==0) ) {
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9, RESET);		// Stop Lamp OFF
		stop_lamp_fail_on_cnt = lamp_status->stop_lamp = 0;
	}

	/*
	 *	DRL Control Function
	 */
	if( ign1_status == 1 ) {
		if( welcome_on == 0 ) {
			if( (vehicle_vel<30) && (vehicle_vel>-30) ) {
				drl_center_off_cnt = 0;
				if( ++drl_center_on_cnt > 25 ) {
					drl_center_on = 1;
					drl_center_on_cnt = 25;
				}
			} else {
				drl_center_on_cnt = 0;
				if( ++drl_center_off_cnt > 25 ) {
					drl_center_on = 0;
					drl_center_off_cnt = 25;
				}
			}
//			if( ign1_status_prev == 0 )  {
			if( (brake_status==1) || (input_status->brake_push_sw==1) ) {
#if 0
				htim3.Instance->CCR2 = htim4.Instance->CCR3 = 210; // 100% -> 70% -> 70% 220713 song oper

#endif
#if 1 //220725 -> brake -> side DRL bright control
				if( (taillamp_on==1) && (drl_center_on==1) ) {
					htim3.Instance->CCR2 = htim4.Instance->CCR3 = 210; // 100% -> 70% -> 70% 220713 song oper
				}else{
					htim3.Instance->CCR2 = htim4.Instance->CCR3 = 490;
				}
#endif
				htim4.Instance->CCR4 = htim3.Instance->CCR1 = 700; //220714
			}else{
				htim4.Instance->CCR4 = htim3.Instance->CCR1 = 0; //220714
			}
			HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
			HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
			//HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4); //220714
			HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);

			lamp_status->drl_lamp = 1;
//			}
			if( (taillamp_on==1) && (drl_center_on==1) ) {
//			if( taillamp_on_prev == 0 ) {
				if( (brake_status==0) && (input_status->brake_push_sw==0) ) {
					//htim3.Instance->CCR2 = htim4.Instance->CCR3 = htim4.Instance->CCR4 = htim3.Instance->CCR1 = 210; // 30% 100% -> 70% -> 21% 220713 song oper
					htim3.Instance->CCR2 = htim4.Instance->CCR3 = 210; //220725 tail -> back center off
				} else if( brake_status == 1 ) {
					htim3.Instance->CCR1 = htim4.Instance->CCR4 = htim3.Instance->CCR1 = 700; // 100% -> 70% -> 70% 220713 song oper
				}
#if 1
				//HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4); //220516 oper song
				//HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1); //220516 oper song
#endif
				lamp_status->drl_c_lamp = 1;
//			}
			} else if( (taillamp_on==0) || (drl_center_on==0) ) {
				if( taillamp_on_prev == 1 ) {
#if 1
					//HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_4); //220516 oper song
					//HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1); //220516 oper song
#endif
					if( (brake_status==0) && (Input_Status.brake_push_sw==0) ) {
						htim3.Instance->CCR2 = htim4.Instance->CCR3 = 490; // 70% -> 70% -> 49% 220713 song oper
					}
					lamp_status->drl_c_lamp = 0;
				} else {
#if 0
					HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4); //220516 oper song
					HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1); //220516 oper song
#endif
				}
			}

			if( pBrake_status == 1 ){
				htim3.Instance->CCR2 = htim4.Instance->CCR3 = 0;
			}

		}
		/*
		 * Signature Lamp Control
		 */
#if 1
#if 0
		//..220531 oper  sonng

		if( auto_mode == 1 ) {
			if( ++uart_data_tx_cnt > 50 ) {
				HAL_UART_Transmit(&huart1, sig_lamp_color_blue_ctl_on, 14, 100);
				uart_data_tx_cnt = 0;
			}
		} else if( auto_mode == 0 ) {
			if( ++uart_data_tx_cnt > 50 ) {
				HAL_UART_Transmit(&huart1, sig_lamp_color_wht_ctl_on, 14, 100);
				uart_data_tx_cnt = 0;
			}
		}


#endif
#if 1  //..220531 oper  sonng
		if( auto_mode == 1 ) {
			if( ++uart_data_tx_cnt > 50 ) {
				HAL_UART_Transmit(&huart1, sig_lamp_color_ctl_off, 14, 100);
				uart_data_tx_cnt = 0;
			}
		} else if( auto_mode == 0 ) {
			if( ++uart_data_tx_cnt > 50 ) {
				HAL_UART_Transmit(&huart1, sig_lamp_color_ctl_off, 14, 100);
				uart_data_tx_cnt = 0;
			}
		}

#endif
#endif

	} else if ( ign1_status == 0 ) {
		if( ign1_status_prev == 1 ) {
			HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
			HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);
			HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_3);
			HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_4);
			lamp_status->drl_lamp = lamp_status->drl_c_lamp = 0;
		}
		if( (hazard_on==0) && (ems_status==0) && (taillamp_on==0) ) {
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, RESET);
		}
	}

	/*
	 * Door Lock/Unlock Turn Lamp Toggle
	 */
	if( (RF_open_lamp==1) && (RF_open_lamp_prev==0) ) {
		RF_open_lamp_on = 1;
		RF_open_lamp_on_cnt = 0;
	}
	if( (RF_close_lamp==1) && (RF_close_lamp_prev==0) ) {
		RF_close_lamp_on = 1;
		RF_close_lamp_on_cnt = 0;
	}

	if( RF_open_lamp_on == 1 ) {
		RF_close_lamp_on = 0;
		if( ++RF_open_lamp_on_cnt < 12 ) {
			HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
			HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
		} else if( RF_open_lamp_on_cnt < 24) {
			HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);
			HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_3);
		} if( RF_open_lamp_on_cnt == 24 ) {
			RF_open_lamp_on = 0;
			RF_open_lamp_on_cnt = 0;
		}

	} else if( RF_close_lamp_on == 1 ) {
		RF_open_lamp_on = 0;
		if( ++RF_close_lamp_on_cnt < 12 ) {
			HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
			HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
		} else if( RF_close_lamp_on_cnt < 24) {
			HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);
			HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_3);
		} else if( RF_close_lamp_on_cnt < 36 ) {
			HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
			HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
		} else if( RF_close_lamp_on_cnt < 48 ) {
			HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);
			HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_3);
		}
		if( RF_close_lamp_on_cnt >= 60 ) {
			RF_close_lamp_on = 0;
			RF_close_lamp_on_cnt = 0;
		}
	}

	RF_open_lamp_prev = RF_open_lamp;
	RF_close_lamp_prev = RF_close_lamp;
	ign1_status_prev = ign1_status;
	ign2_status_prev = ign2_status;
	taillamp_on_prev = taillamp_on;
}

void WelcomeControl()
{
	static uint8_t ign1_status_prev_;
	static uint8_t ign2_status_prev_;

	if( (ign2_status==1) && (ign2_status_prev_==0) ) {
		welcome_on = 1;
		welcome_off = welcome_off_cnt = 0;
	} else if( (ign1_status==0) && (ign1_status_prev_==1) ) {
		welcome_off = 1;
		welcome_on = welcome_on_cnt = 0;
	}
	ign1_status_prev_ = ign1_status;
	ign2_status_prev_ = ign2_status;

	if( welcome_on == 1 ) {
		if( ++welcome_on_cnt == 1 ) {
			htim3.Instance->CCR1 = htim3.Instance->CCR2 = htim4.Instance->CCR3 = htim4.Instance->CCR4 = 50;
			HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
			HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
			HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
			HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
		}
		else if( welcome_on_cnt < 80 ) {
			htim3.Instance->CCR1 = htim3.Instance->CCR1 + 10;
			htim3.Instance->CCR1 = htim3.Instance->CCR2 + 10;
			htim4.Instance->CCR3 = htim4.Instance->CCR3 + 10;
			htim4.Instance->CCR4 = htim4.Instance->CCR4 + 10;
		}
		else if( welcome_on_cnt == 80 ) {
			htim3.Instance->CCR1 = htim3.Instance->CCR2 = htim4.Instance->CCR3 = htim4.Instance->CCR4 = 1000;
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10 | GPIO_PIN_11, SET);
		} else if( (welcome_on_cnt>=112)  && (welcome_on_cnt<130) ) {
			HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
			HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);
			HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_3);
			HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_4);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10 | GPIO_PIN_11, RESET);
		} else if( (welcome_on_cnt>=130)  && (welcome_on_cnt<148) ) {
			HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
			HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
			HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
			HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10 | GPIO_PIN_11, SET);
		} else if( (welcome_on_cnt>=148)  && (welcome_on_cnt<166) ) {
			HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
			HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);
			HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_3);
			HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_4);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10 | GPIO_PIN_11, RESET);
		} else if( (welcome_on_cnt>=166)  && (welcome_on_cnt<184) ) {
			HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
			HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
			HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
			HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10 | GPIO_PIN_11, SET);
		} else if( (welcome_on_cnt>=184)  && (welcome_on_cnt<202) ) {
			HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
			HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);
			HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_3);
			HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_4);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10 | GPIO_PIN_11, RESET);
		} else if( welcome_on_cnt >=  202 ) {
			welcome_on = welcome_on_cnt = 0;
		}
	} else if( welcome_off == 1 ) {
		if( ++welcome_off_cnt == 1 ) {
			htim3.Instance->CCR1 = htim3.Instance->CCR2 = htim4.Instance->CCR3 = htim4.Instance->CCR4 = 1000;
			HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
			HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);
			HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_3);
			HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_4);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10 | GPIO_PIN_11, RESET);
		} else if( (welcome_off_cnt>=34) && (welcome_off_cnt<68) ) {
			HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
			HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
			HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
			HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10 | GPIO_PIN_11, SET);
		} else if( (welcome_off_cnt>=68) && (welcome_off_cnt<102) ) {
			HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
			HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);
			HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_3);
			HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_4);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10 | GPIO_PIN_11, RESET);
		} else if( (welcome_off_cnt>=102) && (welcome_off_cnt<136) ) {
			HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
			HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
			HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
			HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10 | GPIO_PIN_11, SET);
		} else if( (welcome_off_cnt>=136) && (welcome_off_cnt<154) ) {
			HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
			HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);
			HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_3);
			HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_4);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10 | GPIO_PIN_11, RESET);
		} else if( welcome_off_cnt == 154 ) {
			htim3.Instance->CCR1 = htim3.Instance->CCR2 = htim4.Instance->CCR3 = htim4.Instance->CCR4 = 950;
			HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
			HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
			HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
			HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
		} else if( welcome_off_cnt < 234 ) {
			htim3.Instance->CCR1 = htim3.Instance->CCR1 - 10;
			htim3.Instance->CCR2 = htim3.Instance->CCR2 - 10;
			htim4.Instance->CCR3 = htim4.Instance->CCR3 - 10;
			htim4.Instance->CCR4 = htim4.Instance->CCR4 - 10;
		} else if( welcome_off_cnt >= 234 ) {
			HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
			HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);
			HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_3);
			HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_4);
			welcome_off = welcome_off_cnt = 0;
		}
	}
}

void LampDataConv(uint8_t *txdata, uint8_t *flag)
{
	txdata[0] = Lamp_Status.hazard_lamp | (Lamp_Status.drl_lamp<<2) | (Lamp_Status.drl_c_lamp<<3) |
			(Lamp_Status.stop_lamp<<4);
	txdata[1] = head_low_lamp_curr;
	txdata[2] = head_low_lamp_curr>>8;
	txdata[3] = stop_lamp_curr;
	txdata[4] = stop_lamp_curr>>8;
	txdata[5] = head_low_lamp_fail | (stop_lamp_fail<<1);

	(*flag) = 1;
}



/*
 * wiper_ctrl.c
 *
 *  Created on: Nov 16, 2021
 *      Author: okskt
 */

#include "wiper_ctrl.h"


void WiperSWRead(struct InputStatusW_s *input_status)
{
	static uint8_t washer_prev = 0;

	if( auto_mode == 1 ) {
		Input_Status_Raw.wiper_int = Akit_Command.wiper_int_on;
		Input_Status_Raw.wiper_low = Akit_Command.wiper_low_on;
		Input_Status_Raw.wiper_high = Akit_Command.wiper_high_on;
		Input_Status_Raw.washer = Akit_Command.washer_on;
	} else if( auto_mode == 0 ) {
		Input_Status_Raw.wiper_int = READ_PIN_DI_0;			// DI 0
		Input_Status_Raw.wiper_low = READ_PIN_DI_1;			// DI 1
		Input_Status_Raw.wiper_high = READ_PIN_DI_2;		// DI 2
		Input_Status_Raw.washer = READ_PIN_DI_3;				// DI 3
	}
	Input_Status_Raw.wiper_parking = READ_PIN_DI_7;			// DI 7
	Input_Status_Raw.key_in = READ_PIN_DI_14;				// DI 14
	Input_Status_Raw.accessory = READ_PIN_DI_15;			// DI 15

	WiperSWChatt(input_status);

	/*
	 * Washer Control
	 */
	if( (washer_prev==0) && (Input_Status.washer==1) ) {
		washer_on_cnt = washer_off = 0;
		washer_on = 1;
	} else if( (washer_prev==1) && (Input_Status.washer==0) ) {
		washer_off_cnt = washer_on = 0;
		washer_off = 1;
	}

	if( washer_on == 1 ) {
		if( Input_Status.washer == 1 ) {
			if( ++washer_on_cnt > 15 ) {
				HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9, SET);		// Wiper Low Drive
				washer_on = 0;
				washer_on_cnt = 0;
				washer_off_cnt = 0;
				Wiper_Status.wiper_low = 1;
			}
		} else {
			washer_on = 0;
			washer_on_cnt = 0;
		}
	} else if( washer_off == 1 ) {
		if( Input_Status.washer == 0 ) {
			if( ++washer_off_cnt > 120 ) {
				if( Input_Status_Raw.wiper_parking == 1 ) {
					HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9, RESET);		// Wiper Low OFF
					washer_off = 0;
					Wiper_Status.wiper_low = 0;
				}
			}
		}
	}

	Input_Status_Prev.wiper_int = Input_Status_Raw.wiper_int;
	Input_Status_Prev.wiper_low = Input_Status_Raw.wiper_low;
	Input_Status_Prev.wiper_high = Input_Status_Raw.wiper_high;
	Input_Status_Prev.washer = Input_Status_Raw.washer;
	Input_Status_Prev.key_in = Input_Status_Raw.key_in;
	Input_Status_Prev.accessory = Input_Status_Raw.accessory;
//	Input_Status_Prev.wiper_parking = Input_Status_Raw.wiper_parking;

	washer_prev = Input_Status.washer;
}

void WiperSWChatt(struct InputStatusW_s *input_status)
{
	if( Input_Status_Prev.wiper_int == Input_Status_Raw.wiper_int ) {
		if( ++Chattering.wiper_int_cnt >= 2 ) {
			input_status->wiper_int = Input_Status_Raw.wiper_int;
			Chattering.wiper_int_cnt = 3;
		}
	} else {
		Chattering.wiper_int_cnt = 0;
	}

	if( Input_Status_Prev.wiper_low == Input_Status_Raw.wiper_low ) {
		if( ++Chattering.wiper_low_cnt >= 2 ) {
			input_status->wiper_low = Input_Status_Raw.wiper_low;
			Chattering.wiper_low_cnt = 3;
		}
	} else {
		Chattering.wiper_low_cnt = 0;
	}

	if( Input_Status_Prev.wiper_high == Input_Status_Raw.wiper_high ) {
		if( ++Chattering.wiper_high_cnt >= 2 ) {
			input_status->wiper_high = Input_Status_Raw.wiper_high;
			Chattering.wiper_high_cnt = 3;
		}
	} else {
		Chattering.wiper_high_cnt = 0;
	}

	if( Input_Status_Prev.washer == Input_Status_Raw.washer ) {
		if( ++Chattering.washer_cnt >= 2 ) {
			input_status->washer = Input_Status_Raw.washer;
			Chattering.washer_cnt = 3;
		}
	} else {
		Chattering.washer_cnt = 0;
	}

	if( Input_Status_Prev.key_in == Input_Status_Raw.key_in ) {
		if( ++Chattering.key_in_cnt >= 2 ) {
			input_status->key_in = Input_Status_Raw.key_in;
			Chattering.key_in_cnt = 3;
		}
	} else {
		Chattering.key_in_cnt = 0;
	}

	if( Input_Status_Prev.accessory == Input_Status_Raw.accessory ) {
		if( ++Chattering.accessory_cnt >= 2 ) {
			input_status->accessory = Input_Status_Raw.accessory;
			Chattering.accessory_cnt = 3;
		}
	} else {
		Chattering.accessory_cnt = 0;
	}
#if 0
	if( Input_Status_Prev.wiper_parking == Input_Status_Raw.wiper_parking ) {
		if( ++Chattering.wiper_parking_cnt >= 1 ) {
			Input_Status.wiper_parking = Input_Status_Raw.wiper_parking;
			Chattering.wiper_parking_cnt = 3;
		}
	} else {
		Chattering.wiper_parking_cnt = 0;
	}
#endif
}

void WiperControl(struct WiperStatus_W_s *wiper_status, struct InputStatusW_s *input_status)
{
	/*
	 * cluster ign function
	 */
	if( (ign1_status==1) || (charge_door_status==1) ) {
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, SET);			// Cluster IGN Signal Drv
	} else if( (ign1_status==0) && (charge_door_status==0) ){
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, RESET);		// Cluster IGN Signal Drv
	}

	if( ign1_status == 1 ) {
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, SET);		// SENS 5+ Power Drv

		if( input_status->washer==0 ) {
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, RESET);	// Washer OFF
			wiper_status->washer = 0;

			if( washer_off == 0 ) {
				if( input_status->auto_wiper == 0 ) {
					if( Input_Status.wiper_int == 1 ) {
						/*
						 * Wiper INT Function
						 */
						wiper_status->wiper_int = 1;
						if( wiper_int_volt < 1666 ) {
							if( ++wiper_int_on_cnt < 40) {
								HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9, SET);			// Wiper Low Drive
							} else if( wiper_int_on_cnt < 140 ) {
								if( Input_Status_Raw.wiper_parking == 1 ) {
									HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9, RESET);	// Wiper Low Drive
								}
							} else {
								wiper_int_on_cnt = 0;
							}

						} else if( wiper_int_volt < 3333 ) {
							if( ++wiper_int_on_cnt < 40) {
								HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9, SET);			// Wiper Low Drive
							} else if( wiper_int_on_cnt < 540 ) {
								if( Input_Status_Raw.wiper_parking == 1 ) {
									HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9, RESET);	// Wiper Low Drive
								}
							} else {
								wiper_int_on_cnt = 0;
							}

						} else {
							if( ++wiper_int_on_cnt < 40) {
								HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9, SET);			// Wiper Low Drive
							} else if( wiper_int_on_cnt < 1040 ) {
								if( Input_Status_Raw.wiper_parking == 1 ) {
									HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9, RESET);	// Wiper Low Drive
								}
							} else {
								wiper_int_on_cnt = 0;
							}
						}

					} else if( input_status->wiper_int == 0 ) {
						wiper_status->wiper_int = 0;		// Wiper INT OFF
						wiper_int_on_cnt = 0;
					}

					/*
					 * Wiper Low Control
					 */
					if( input_status->wiper_low == 1 ) {
						HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9, SET);				// Wiper Low Drive
						wiper_status->wiper_low = 1;
					} else if( input_status->wiper_low == 0 ) {
						wiper_status->wiper_low = 0;
						if( (input_status->wiper_int==0) && (input_status->wiper_high==0) ) {
							if( Input_Status_Raw.wiper_parking == 1 ) {
								HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9, RESET);	// Wiper Low OFF
							}
						}
					}

					/*
					 * Wiper High Control
					 */
					if( input_status->wiper_high == 1 ) {
						HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9 | GPIO_PIN_10, SET);		// Wiper Low, High Drive
						wiper_status->wiper_high = 1;
					} else if( input_status->wiper_high == 0 ) {
						wiper_status->wiper_high = 0;
						if( Input_Status_Raw.wiper_parking == 1 ) {
							HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, RESET);				// Wiper High OFF
						}
					}
				}
			}

		} else if( input_status->washer == 1 ) {
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, RESET);	// Wiper High OFF
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, SET);		// Washer ON
			wiper_status->washer = 1;
		}

	} else if( ign1_status == 0 ) {
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, RESET);						// Washer  OFF
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, RESET);						// 5+ Power OFF
		wiper_status->washer = 0;
		if( Input_Status_Raw.wiper_parking == 1 ) {
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9 | GPIO_PIN_10, RESET);		// Wiper Low, High OFF
		}
		wiper_status->wiper_int = wiper_status->wiper_low = wiper_status->wiper_high = 0;
		wiper_int_on_cnt = 0;
	}
}

void DimmerLampControl()
{
#if 0
	if( ign1_status == 1 ) {
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, SET);		// Cluster ILL(+) ON
	} else if( ign1_status == 0 ) {
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, RESET);	// Cluster ILL(+) OFF
	}

	if( taillamp_on == 1 ) {
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, SET);		// Dimmer Control High Drv
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, SET);		// RSI Brightness Control Drv

		if( dimmer_volt < 600 ) {
			htim3.Instance->CCR1 = 100;					// Dimmer Output PWM Duty
			htim3.Instance->CCR2 = 900;					// Cluster Brightness Signal PWM Duty
		} else if( dimmer_volt < 1200 ) {
			htim3.Instance->CCR1 = 200;
			htim3.Instance->CCR2 = 800;
		} else if( dimmer_volt < 1800 ) {
			htim3.Instance->CCR1 = 300;
			htim3.Instance->CCR2 = 700;
		} else if( dimmer_volt < 2400 ) {
			htim3.Instance->CCR1 = 400;
			htim3.Instance->CCR2 = 600;
		} else if( dimmer_volt < 3000 ) {
			htim3.Instance->CCR1 = 500;
			htim3.Instance->CCR2 = 500;
		} else if( dimmer_volt < 3600 ) {
			htim3.Instance->CCR1 = 600;
			htim3.Instance->CCR2 = 400;
		} else if( dimmer_volt < 4200 ) {
			htim3.Instance->CCR1 = 700;
			htim3.Instance->CCR2 = 300;
		} else if( dimmer_volt < 4800 ) {
			htim3.Instance->CCR1 = 800;
			htim3.Instance->CCR2 = 200;
		} else if( dimmer_volt < 5100 ) {
			htim3.Instance->CCR1 = 900;
			htim3.Instance->CCR2 = 100;
		}
		HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);		// Dimmer Output Drv
		HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);		// Cluster Brightness Signal Drv

	} else if( taillamp_on == 0 ) {
		HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);		// Dimmer Output PWM Off
		HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);		// Cluster Brightness PWM Off
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, RESET);	// Dimmer Control High OFF
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, RESET);	// RSI Brightness Control OFF
	}
#else
	if( ign1_status == 1 ) {
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, SET);		// Cluster ILL(+) ON
	} else if( ign1_status == 0 ) {
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, RESET);	// Cluster ILL(+) OFF
	}

	if( taillamp_on == 1 ) {
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, SET);		// Dimmer Control High Drv
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, SET);		// RSI Brightness Control Drv

		htim3.Instance->CCR1 = 100;						// Dimmer Output PWM Duty
		htim3.Instance->CCR2 = 100;						// Cluster Brightness Signal PWM Duty
		HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);		// Dimmer Output Drv
		HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);		// Cluster Brightness Signal Drv

	} else if( taillamp_on == 0 ) {
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, RESET);	// Dimmer Control High OFF
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, RESET);	// RSI Brightness Control OFF

		htim3.Instance->CCR1 = 900;						// Dimmer Output PWM Duty
		htim3.Instance->CCR2 = 900;						// Cluster Brightness Signal PWM Duty
		HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);		// Dimmer Output Drv
		HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);		// Cluster Brightness Signal Drv
	}
#endif
}

void AutoWiperControl(struct WiperStatus_W_s *wiper_status, struct InputStatusW_s *input_status)
{
	static uint8_t auto_wiper_delay = 0;
	static uint8_t rain_sens_wiper_low_on = 0;
	static uint8_t rain_sens_wiper_low_on_cnt = 0;
	static uint8_t rain_sens_wiper_high_on = 0;
	static uint8_t rain_sens_wiper_high_on_cnt = 0;

	if( ign1_status == 1 ) {
		if( (input_status->wiper_int==0) && (input_status->wiper_low==0) && (input_status->wiper_high==0) && (Akit_Command.wiper_off==0) ) {
			input_status->auto_wiper = 1;
		} else {
			input_status->auto_wiper = 0;
			auto_wiper_delay = 0;
		}

		if( input_status->auto_wiper == 1 ) {
			if( (washer_off==0) && (washer_on==0) && (input_status->washer==0) ) {
				HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, SET);						// Rain Sensor Power ON
				if( ++auto_wiper_delay > 10 ) {
					rain_sens_wiper_low = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1);	// read rain sensor output (wiper low)
					rain_sens_wiper_high = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2);	// read rain sensor output (wiper high)

#if 1
					if( rain_sens_wiper_low == 1 ) {
						if( ++rain_sens_wiper_low_on_cnt > 5 ) { //220523 100 -> 50
							rain_sens_wiper_low_on = 1;
						}
					} else if( rain_sens_wiper_low == 0 ) {
						rain_sens_wiper_low_on = rain_sens_wiper_low_on_cnt = 0;
					}
					if( rain_sens_wiper_high == 1 ) {
						if( ++rain_sens_wiper_high_on_cnt > 5 ) { //220523 100 -> 50
							rain_sens_wiper_high_on = 1;
						}
					} else if( rain_sens_wiper_high == 0 ) {
						rain_sens_wiper_high_on = rain_sens_wiper_high_on_cnt = 0;
					}
#endif
#if 0
					if( rain_sens_wiper_low == 1 ) {
							rain_sens_wiper_low_on = 1;
						}

					if( rain_sens_wiper_high == 1 ) {
							rain_sens_wiper_high_on = 1;
						}
#endif

					auto_wiper_delay = 50;
				}

				if( Input_Status_Raw.wiper_parking == 1 ) {
					HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, RESET);				// Rain Sens Parking Sig drv
				} else if( Input_Status_Raw.wiper_parking == 0 ) {
					HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, SET);					// Rain Sens Parking Sig off
				}

				if( rain_sens_wiper_high_on == 1 ) {
					HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9, RESET);				// Wiper Low OFF
					rain_sens_wiper_low_on = 0;
				}

				if( rain_sens_wiper_low_on == 1 ) {
					HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, RESET);				// Wiper High OFF
					HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9, SET);					// Wiper Low Drive
					wiper_status->wiper_low = 1;
				} else if( rain_sens_wiper_low_on == 0 ) {
					if( Input_Status_Raw.wiper_parking == 1 ) {
						HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9, RESET);			// Wiper Low OFF
					}
					wiper_status->wiper_low = 0;
					if( rain_sens_wiper_high_on == 1 ) {
						HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9, RESET);			// Wiper Low OFF
						HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, SET);				// Wiper High Drive
						wiper_status->wiper_high = 1;
					} else if( rain_sens_wiper_high_on == 0 ) {
						if( Input_Status_Raw.wiper_parking == 1 ) {
							HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, RESET);		// Wiper High OFF
						}
						wiper_status->wiper_high = 0;
					}
				}
			}
		} else if( input_status->auto_wiper == 0 ) {
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, RESET);					// Rain Sensor Power OFF
		}
	} else if( ign1_status == 0 ) {
		input_status->auto_wiper = 0;
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, RESET);						// Rain Sensor Power OFF
		auto_wiper_delay = 0;
	}
}

void LampControl(struct LampStatus_s *lamp_status)
{
	/*
	 * Courtesy Lamp Control
	 */
	if( door_open_status == 1 ) {
		lamp_status->courtesy_lamp = 1;
		if( door_open_status_prev == 0 ) {
			htim5.Instance->CCR2 = 1000;
			HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);
		}
	} else if( door_open_status == 0 ) {
		lamp_status->courtesy_lamp = 0;
		if( door_open_status_prev == 1 ) {
			htim5.Instance->CCR2 = 700;
//			htim8.Instance->CCR2 = 500;		// origin
			HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);
		}
		htim5.Instance->CCR2 = htim5.Instance->CCR2 - 6;
		if( htim5.Instance->CCR2 <= 100 ) {
			htim5.Instance->CCR2 = 100;
			HAL_TIM_PWM_Stop(&htim5, TIM_CHANNEL_2);
			lamp_status->courtesy_lamp = 0;
		}
	}
	door_open_status_prev = door_open_status;
}

void WiperDataConv(uint8_t *txdata, uint8_t *flag)
{
	txdata[0] = Wiper_Status.wiper_int | (Wiper_Status.wiper_low<<1) | (Wiper_Status.wiper_high<<2) | (Wiper_Status.washer<<3) |
			(Input_Status.auto_wiper<<4) | (Lamp_Status.courtesy_lamp<<5);
	txdata[1] = wiper_int_volt;
	txdata[2] = wiper_int_volt >> 8;
	txdata[3] = dimmer_volt;
	txdata[4] = dimmer_volt >> 8;

	(*flag) = 1;
}




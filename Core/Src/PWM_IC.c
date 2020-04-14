/*
 * 		PWM_IC.c
 *
 *  	Created on: 7 feb. 2020
 *      Author: MERT OCAL
 *
 *      //	This library is designed especially for STM32F407 Discovery Board TIM2
 *      //	The Input_Capture has been set for 4 channel reading
 *      //
 */

#include "PWM_IC.h"


uint8_t  First_Captured		= 0;
uint32_t Roll_IC_Val1		= 0;
uint32_t Roll_IC_Val2		= 0;
uint32_t Pitch_IC_Val1		= 0;
uint32_t Pitch_IC_Val2		= 0;
uint32_t Throttle_IC_Val1	= 0;
uint32_t Throttle_IC_Val2	= 0;
uint32_t Yaw_IC_Val1		= 0;
uint32_t Yaw_IC_Val2		= 0;
//
extern int receiver_input_channel_1, receiver_input_channel_2, receiver_input_channel_3, receiver_input_channel_4;
extern int receiver_input[5];


void PWM_IC_Start( void )
{
	HAL_TIM_IC_Start_IT( &htim2, TIM_CHANNEL_1 );
}

void PWM_IC_Stop( void )
{
	HAL_TIM_IC_Stop_IT( &htim2, TIM_CHANNEL_1);
	HAL_TIM_IC_Stop_IT( &htim2, TIM_CHANNEL_2);
	HAL_TIM_IC_Stop_IT( &htim2, TIM_CHANNEL_3);
	HAL_TIM_IC_Stop_IT( &htim2, TIM_CHANNEL_4);
}

void HAL_TIM_IC_CaptureCallback( TIM_HandleTypeDef *htim )
{
	if ( htim -> Channel == HAL_TIM_ACTIVE_CHANNEL_1 )   // if the interrupt source is channel1
	{
		if ( First_Captured == false ) 												// if the first value is not captured
		{
			Roll_IC_Val1 = HAL_TIM_ReadCapturedValue( htim, TIM_CHANNEL_1 ); 	// read the first value
			First_Captured = true;  											// set the first captured as true
			__HAL_TIM_SET_CAPTUREPOLARITY( htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING ); 	// Now change the polarity to falling edge
		}

		else if ( First_Captured == true )   // if the first is already captured
		{
			Roll_IC_Val2 = HAL_TIM_ReadCapturedValue( htim, TIM_CHANNEL_1 );  // read second value
			__HAL_TIM_SET_COUNTER( htim, false );  // reset the counter

			if ( Roll_IC_Val2 > Roll_IC_Val1 )
			{
				receiver_input_channel_1 = Roll_IC_Val2 - Roll_IC_Val1;
				receiver_input[1] = receiver_input_channel_1;
			}

			First_Captured = false;	// set it back to false
			__HAL_TIM_SET_CAPTUREPOLARITY( htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING );	// set polarity to rising edge

			HAL_TIM_IC_Stop_IT( &htim2, TIM_CHANNEL_1 );
			HAL_TIM_IC_Start_IT( &htim2, TIM_CHANNEL_2 );
		}
	}

	if ( htim -> Channel == HAL_TIM_ACTIVE_CHANNEL_2 )   // if the interrupt source is channel2
	{
		if ( First_Captured == false ) // if the first value is not captured
		{
			Pitch_IC_Val1 = HAL_TIM_ReadCapturedValue( htim, TIM_CHANNEL_2 ); 	// read the first value
			First_Captured = true;  										// set the first captured as true
			__HAL_TIM_SET_CAPTUREPOLARITY( htim, TIM_CHANNEL_2, TIM_INPUTCHANNELPOLARITY_FALLING ); 	// Now change the polarity to falling edge
		}

		else if ( First_Captured == true )   // if the first is already captured
		{
			Pitch_IC_Val2 = HAL_TIM_ReadCapturedValue( htim, TIM_CHANNEL_2 );  // read second value
			__HAL_TIM_SET_COUNTER( htim, false );  // reset the counter

			if ( Pitch_IC_Val2 > Pitch_IC_Val1 )
			{
				receiver_input_channel_2 = Pitch_IC_Val2 - Pitch_IC_Val1;
				receiver_input[2] = receiver_input_channel_2;
			}

			First_Captured = false; // set it back to false
			__HAL_TIM_SET_CAPTUREPOLARITY( htim, TIM_CHANNEL_2, TIM_INPUTCHANNELPOLARITY_RISING );  // set polarity to rising edge

			HAL_TIM_IC_Stop_IT( &htim2, TIM_CHANNEL_2 );
			HAL_TIM_IC_Start_IT( &htim2, TIM_CHANNEL_3 );
		}
	}

	if ( htim -> Channel == HAL_TIM_ACTIVE_CHANNEL_3 )   // if the interrupt source is channel2
	{
		if ( First_Captured == false ) // if the first value is not captured
		{
			Throttle_IC_Val1 = HAL_TIM_ReadCapturedValue( htim, TIM_CHANNEL_3 ); 	// read the first value
			First_Captured = true;  										// set the first captured as true
			__HAL_TIM_SET_CAPTUREPOLARITY( htim, TIM_CHANNEL_3, TIM_INPUTCHANNELPOLARITY_FALLING ); 	// Now change the polarity to falling edge
		}

		else if ( First_Captured == true )   // if the first is already captured
		{
			Throttle_IC_Val2 = HAL_TIM_ReadCapturedValue( htim, TIM_CHANNEL_3 );  // read second value
			__HAL_TIM_SET_COUNTER(htim, false);  // reset the counter

			if ( Throttle_IC_Val2 > Throttle_IC_Val1 )
			{
				receiver_input_channel_3 = Throttle_IC_Val2 - Throttle_IC_Val1;
				receiver_input[3] = receiver_input_channel_3;
			}

			First_Captured = false; // set it back to false
			__HAL_TIM_SET_CAPTUREPOLARITY( htim, TIM_CHANNEL_3, TIM_INPUTCHANNELPOLARITY_RISING );  // set polarity to rising edge

			HAL_TIM_IC_Stop_IT( &htim2, TIM_CHANNEL_3 );
			HAL_TIM_IC_Start_IT( &htim2, TIM_CHANNEL_4 );
		}
	}

	if ( htim -> Channel == HAL_TIM_ACTIVE_CHANNEL_4 )   // if the interrupt source is channel2
	{
		if ( First_Captured == false ) // if the first value is not captured
		{
			Yaw_IC_Val1 = HAL_TIM_ReadCapturedValue( htim, TIM_CHANNEL_4 ); 	// read the first value
			First_Captured = true;  										// set the first captured as true
			__HAL_TIM_SET_CAPTUREPOLARITY( htim, TIM_CHANNEL_4, TIM_INPUTCHANNELPOLARITY_FALLING ); 	// Now change the polarity to falling edge
		}

		else if ( First_Captured == true )   // if the first is already captured
		{
			Yaw_IC_Val2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4);  // read second value
			__HAL_TIM_SET_COUNTER(htim, false);  // reset the counter

			if ( Yaw_IC_Val2 > Yaw_IC_Val1 )
			{
				receiver_input_channel_4 = Yaw_IC_Val2 - Yaw_IC_Val1;
				receiver_input[4] = receiver_input_channel_4;
			}

			First_Captured = false; // set it back to false
			__HAL_TIM_SET_CAPTUREPOLARITY( htim, TIM_CHANNEL_4, TIM_INPUTCHANNELPOLARITY_RISING );  // set polarity to rising edge

			HAL_TIM_IC_Stop_IT( &htim2, TIM_CHANNEL_4 );
			HAL_TIM_IC_Start_IT( &htim2, TIM_CHANNEL_1 );
		}
	}
}


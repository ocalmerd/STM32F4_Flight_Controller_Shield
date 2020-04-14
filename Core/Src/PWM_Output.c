/*
 * PWM_Output.c
 *
 *  Created on: Mar 22, 2020
 *      Author: MERT OCAL
 */

#include "PWM_Output.h"


void PWM_Output_Start( void )
{
	HAL_TIM_PWM_Start( &htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start( &htim3, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start( &htim3, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start( &htim3, TIM_CHANNEL_4);
}

void PWM_Output_Idle( void )
{
	htim3.Instance->CCR1 = 1000;
	htim3.Instance->CCR2 = 1000;
	htim3.Instance->CCR3 = 1000;
	htim3.Instance->CCR4 = 1000;
}

void PWM_Output_ESC_Calibration( int esc_3 )
{
	htim3.Instance->CCR1 = esc_3;
	htim3.Instance->CCR2 = esc_3;
	htim3.Instance->CCR3 = esc_3;
	htim3.Instance->CCR4 = esc_3;
}

void PWM_Output_ESC( int esc_1, int esc_2, int esc_3, int esc_4 )
{
	htim3.Instance->CCR1 = esc_1;
	htim3.Instance->CCR2 = esc_2;
	htim3.Instance->CCR3 = esc_3;
	htim3.Instance->CCR4 = esc_4;
}


/*
 * 		PWM_IC.h
 *
 *  	Created on: 7 feb. 2020
 *      Author: MERT OCAL
 *
 *      //	This library is designed especially for TIM2
 *      //	The Input_Capture has been set for 4 channel reading
 *      //
 * 
 * 
 *      /////   TIM2 Settings   /////
 * 
 *      //  htim2.Instance = TIM2;
 *      //  htim2.Init.Prescaler = 84-1;													// Depends on the Input Signals
 *      //  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
 *      //  htim2.Init.Period = 10000-1;													// Depends on the Input Signals
 *      //  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
 *      //  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
 *      //  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
 *      //  {
 *      //    Error_Handler();
 *      //  }
 *      //  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
 *      //  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
 *      //  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
 *      //  {
 *      //    Error_Handler();
 *      //  }
 *      //  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
 *      //  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
 *      //  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
 *      //  sConfigIC.ICFilter = 0;
 * 		//
 * 
 */

#ifndef PWM_IC_H_
#define PWM_IC_H_

#include "main.h"
#include "stm32f4xx_it.h"
#include "stdbool.h"

TIM_HandleTypeDef htim2;

void HAL_TIM_IC_CaptureCallback( TIM_HandleTypeDef *htim );

void PWM_IC_Start( void );

void PWM_IC_Stop( void );

#endif /* PWM_IC_H_ */

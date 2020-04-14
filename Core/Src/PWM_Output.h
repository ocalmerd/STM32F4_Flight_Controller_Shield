/*
 * PWM_Output.h
 *
 *  Created on: Mar 22, 2020
 *      Author: MERT OCAL
 */

#ifndef SRC_PWM_OUTPUT_H_
#define SRC_PWM_OUTPUT_H_


#include "main.h"


TIM_HandleTypeDef htim3;


void PWM_Output_Start( void );

void PWM_Output_Idle( void );

void PWM_Output_ESC_Calibration( int esc_3 );

void PWM_Output_ESC( int esc_1, int esc_2, int esc_3, int esc_4 );

#endif /* SRC_PWM_OUTPUT_H_ */

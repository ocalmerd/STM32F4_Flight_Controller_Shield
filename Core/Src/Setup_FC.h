/*
 * setup.h
 *
 *  Created on: 19 mar. 2020
 *      Author: MERT OCAL
 */

#ifndef SRC_SETUP_FC_H_
#define SRC_SETUP_FC_H_

#include "main.h"

#include "PWM_IC.h"
#include "L3GD20H.h"
#include "Flash_RW.h"


UART_HandleTypeDef huart4;


void Main_Setup( void );

void Setup_Gyro_Signalen( void );

void Check_Gyro_Axes( uint8_t movement );

void Register_Min_Max( void );

void Check_Receiver_Inputs( uint8_t movement );


#endif /* SRC_SETUP_FC_H_ */

/*
 * Flight_Control.h
 *
 *  Created on: 19 mar. 2020
 *      Author: MERT OCAL
 */

#ifndef SRC_FLIGHT_CONTROL_H_
#define SRC_FLIGHT_CONTROL_H_

#include "main.h"

#include "PWM_IC.h"
#include "L3GD20H.h"
//
#include "PWM_Output.h"


void Flight_Control_Setup( void );

void Flight_Control_Loop( void );

int Convert_Receiver_Channels( uint8_t function );

void Calculate_PID( void );

void Flight_Gyro_Signalen( void );

#endif /* SRC_FLIGHT_CONTROL_H_ */

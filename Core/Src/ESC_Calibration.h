/*
 * ESC_Calibration.h
 *
 *  Created on: 26 mar. 2020
 *      Author: MERT OCAL
 */

#ifndef SRC_ESC_CALIBRATION_H_
#define SRC_ESC_CALIBRATION_H_

#include "main.h"
//
#include "PWM_IC.h"
#include "PWM_Output.h"


void ESC_Calibration ( void );

int Convert_Receiver_Channel( uint8_t function );

#endif /* SRC_ESC_CALIBRATION_H_ */

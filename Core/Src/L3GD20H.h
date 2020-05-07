/*
 * L3GD20H.h
 *
 *  Created on: 12 feb. 2020
 *      Author: MERT OCAL
 */

#ifndef SRC_L3GD20H_H_
#define SRC_L3GD20H_H_

#include "main.h"


I2C_HandleTypeDef hi2c1;
UART_HandleTypeDef huart4;


#define L3GD20H_ADDRESS_DATAREAD 	0xD7
#define L3GD20H_ADDRESS_DATAWRITE 	0xD6
//
#define L3GD20H_REG_WHO_AM_I 		0x0F
#define L3GD20H_WHO_AM_I 			0xD7
//
#define L3GD20H_REG_CTRL1			0x20
#define L3GD20H_REG_CTRL2			0x21
#define L3GD20H_REG_CTRL3			0x22
#define L3GD20H_REG_CTRL4			0x23
#define L3GD20H_REG_CTRL5			0x24
#define L3GD20H_REG_REFERENCE		0x25
#define L3GD20H_REG_OUT_TEMP		0x26
#define L3GD20H_REG_STATUS			0x27
#define L3GD20H_REG_OUT_X_L			0x28
#define L3GD20H_REG_OUT_X_H			0x29
#define L3GD20H_REG_OUT_Y_L			0x2A
#define L3GD20H_REG_OUT_Y_H			0x2B
#define L3GD20H_REG_OUT_Z_L			0x2C
#define L3GD20H_REG_OUT_Z_H			0x2D
#define L3GD20H_REG_FIFO_CTRL_REG	0x2E
#define L3GD20H_REG_FIFO_SRC_REG	0x2F
#define L3GD20H_REG_INT1_CFG		0x30
#define L3GD20H_REG_INT1_SRC		0x31
#define L3GD20H_REG_INT1_TSH_XH		0x32
#define L3GD20H_REG_INT1_TSH_XL		0x33
#define L3GD20H_REG_INT1_TSH_YH		0x34
#define L3GD20H_REG_INT1_TSH_YL		0x35
#define L3GD20H_REG_INT1_TSH_ZH		0x36
#define L3GD20H_REG_INT1_TSH_ZL		0x37
#define L3GD20H_REG_INT1_DURATION	0x38
#define L3GD20H_REG_LOW_ODR			0x39


void L3GD20H_Init( void );

bool L3GD20H_Connection_Check( void );

void L3GD20H_Read_Gyro_RAW_Outputs( void );


#endif /* SRC_L3GD20H_H_ */

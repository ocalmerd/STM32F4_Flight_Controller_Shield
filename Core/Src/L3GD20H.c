/*
 * L3GD20H.c
 *
 *  Created on: 12 feb. 2020
 *      Author: MERT OCAL
 */

#include "L3GD20H.h"


char uartTX[101];
//
uint8_t L3GD20H_SET_CTRL1  		= 0x0F;
uint8_t L3GD20H_SET_CTRL4  		= 0x90;
//
uint8_t L3GD20H_OUT_X_L 	    = 0;
uint8_t L3GD20H_OUT_X_H         = 0;
uint8_t L3GD20H_OUT_Y_L 	    = 0;
uint8_t L3GD20H_OUT_Y_H	        = 0;
uint8_t L3GD20H_OUT_Z_L	        = 0;
uint8_t L3GD20H_OUT_Z_H	        = 0;
uint8_t L3GD20H_OUT_TEMPERATURE	= 0;
uint8_t L3GD20H_STATUS			= 0;
//
extern int16_t gX_Raw, gY_Raw, gZ_Raw;
//
extern bool gyro_error;


void L3GD20H_Init( void )
{
	uint8_t L3GD20H_WHO_AM_I_Check = 0x00;

	HAL_I2C_Mem_Read(&hi2c1, L3GD20H_ADDRESS_DATAREAD , L3GD20H_REG_WHO_AM_I, 1, &L3GD20H_WHO_AM_I_Check, 1, 100 );

	if ( L3GD20H_WHO_AM_I_Check == L3GD20H_WHO_AM_I )
	{
        HAL_I2C_Mem_Write( &hi2c1, L3GD20H_ADDRESS_DATAWRITE, L3GD20H_REG_CTRL1, 1, &L3GD20H_SET_CTRL1,	1, 10 );
        HAL_I2C_Mem_Write( &hi2c1, L3GD20H_ADDRESS_DATAWRITE, L3GD20H_REG_CTRL4, 1, &L3GD20H_SET_CTRL4, 1, 10 );

		sprintf(uartTX, "                                                                                                    ");
		sprintf(uartTX, "\nL3GD20H is found and settled!\n");
		HAL_UART_Transmit( &huart4, (uint8_t *)uartTX, sizeof(uartTX), 100 );
    }
	else
	{
		sprintf(uartTX, "                                                                                                    ");
		sprintf(uartTX, "\nL3GD20H is NOT found!\n");
		HAL_UART_Transmit( &huart4, (uint8_t *)uartTX, sizeof(uartTX), 100 );

		gyro_error = true;
	}
}

bool L3GD20H_Connection_Check( void )
{
	uint8_t L3GD20H_WHO_AM_I_Check = 0x00;

	HAL_I2C_Mem_Read(&hi2c1, L3GD20H_ADDRESS_DATAREAD , L3GD20H_REG_WHO_AM_I, 1, &L3GD20H_WHO_AM_I_Check, 1, 100 );

	if ( L3GD20H_WHO_AM_I_Check == L3GD20H_WHO_AM_I )
	{
		return true;
	}
	else
	{
		return false;
	}
}

void L3GD20H_Read_Gyro_RAW_Outputs( void )
{
	HAL_I2C_Mem_Read(&hi2c1, L3GD20H_ADDRESS_DATAREAD, L3GD20H_REG_OUT_X_L, 1,  &L3GD20H_OUT_X_L, 1, 100);
	HAL_I2C_Mem_Read(&hi2c1, L3GD20H_ADDRESS_DATAREAD, L3GD20H_REG_OUT_X_H, 1, 	&L3GD20H_OUT_X_H, 1, 100);
	HAL_I2C_Mem_Read(&hi2c1, L3GD20H_ADDRESS_DATAREAD, L3GD20H_REG_OUT_Y_L, 1,  &L3GD20H_OUT_Y_L, 1, 100);
	HAL_I2C_Mem_Read(&hi2c1, L3GD20H_ADDRESS_DATAREAD, L3GD20H_REG_OUT_Y_H, 1,  &L3GD20H_OUT_Y_H, 1, 100);
	HAL_I2C_Mem_Read(&hi2c1, L3GD20H_ADDRESS_DATAREAD, L3GD20H_REG_OUT_Z_L, 1,  &L3GD20H_OUT_Z_L, 1, 100);
	HAL_I2C_Mem_Read(&hi2c1, L3GD20H_ADDRESS_DATAREAD, L3GD20H_REG_OUT_Z_H, 1,  &L3GD20H_OUT_Z_H, 1, 100);

	gX_Raw = (int16_t) (L3GD20H_OUT_X_H << 8 | L3GD20H_OUT_X_L);
	gY_Raw = (int16_t) (L3GD20H_OUT_Y_H << 8 | L3GD20H_OUT_Y_L);
	gZ_Raw = (int16_t) (L3GD20H_OUT_Z_H << 8 | L3GD20H_OUT_Z_L);
}

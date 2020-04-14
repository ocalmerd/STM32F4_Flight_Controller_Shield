/*
 * Setup.c
 *
 *  Created on: 19 mar. 2020
 *      Author: MERT OCAL
 */

#include "Setup_FC.h"

char uartTX[101];

uint8_t eeprom_data_test[12];

static bool error;
uint8_t roll_axis, pitch_axis, yaw_axis;
uint8_t receiver_check_byte, gyro_check_byte;

int center_channel_1, center_channel_2, center_channel_3, center_channel_4;
int high_channel_1, high_channel_2, high_channel_3, high_channel_4;
int low_channel_1, low_channel_2, low_channel_3, low_channel_4;
int cal_int;
uint8_t channel_1_assign, channel_2_assign, channel_3_assign, channel_4_assign;
float gyro_roll_cal, gyro_pitch_cal, gyro_yaw_cal;
static uint8_t eeprom_data[32];
//
extern int receiver_input_channel_1, receiver_input_channel_2, receiver_input_channel_3, receiver_input_channel_4;
//
extern float gyro_pitch, gyro_roll, gyro_yaw;
extern int16_t gX_Raw, gY_Raw, gZ_Raw;


void Main_Setup( void )
{
    sprintf(uartTX, "                                                                                                    ");
    sprintf(uartTX, "\nMove the sticks to the center position within 10 seconds!\n");
    HAL_UART_Transmit( &huart4, (uint8_t *)uartTX, sizeof(uartTX), 100 );
    HAL_Delay( 10000 );

    //Store the central stick positions
    center_channel_1 = receiver_input_channel_1;
    center_channel_2 = receiver_input_channel_2;
    center_channel_3 = receiver_input_channel_3;
    center_channel_4 = receiver_input_channel_4;


    sprintf(uartTX, "                                                                                                    ");
    sprintf(uartTX, "\nMove the sticks to min & max positions then back to center within 10 seconds!\n");
    HAL_UART_Transmit( &huart4, (uint8_t *)uartTX, sizeof(uartTX), 100 );
    HAL_Delay( 1000 );
    //Register the min and max values of the receiver channels
    Register_Min_Max();


    sprintf(uartTX, "                                                                                                    ");
    sprintf(uartTX, "\nMove the throttle stick to full throttle within 10 seconds!\n");
    HAL_UART_Transmit( &huart4, (uint8_t *)uartTX, sizeof(uartTX), 100 );
    HAL_Delay( 10000 );
    Check_Receiver_Inputs( 1 );			//	Move the throttle stick to full throttle and back to center within 10 seconds

    sprintf(uartTX, "                                                                                                    ");
    sprintf(uartTX, "\nMove the roll stick to simulate left wing up within 10 seconds!\n");
    HAL_UART_Transmit( &huart4, (uint8_t *)uartTX, sizeof(uartTX), 100 );
    HAL_Delay( 10000 );
    Check_Receiver_Inputs( 2 );			//	Move the roll stick to simulate left wing up and back to center within 10 seconds

    sprintf(uartTX, "                                                                                                    ");
    sprintf(uartTX, "\nMove the pitch stick to simulate nose up within 10 seconds!\n");
    HAL_UART_Transmit( &huart4, (uint8_t *)uartTX, sizeof(uartTX), 100 );
    HAL_Delay( 10000 );
    Check_Receiver_Inputs( 3 );			//	Move the pitch stick to simulate left wing up and back to center within 10 seconds

    sprintf(uartTX, "                                                                                                    ");
    sprintf(uartTX, "\nMove the yaw stick to simulate nose right within 10 seconds!\n");
    HAL_UART_Transmit( &huart4, (uint8_t *)uartTX, sizeof(uartTX), 100 );
    HAL_Delay( 10000 );
    Check_Receiver_Inputs( 4 );			//	Move the yaw stick to simulate nose right and back to center within 10 seconds


    sprintf(uartTX, "                                                                                                    ");
    sprintf(uartTX, "\nTaking multiple gyro data samples (calibration) during ~8 seconds!\n");
    HAL_UART_Transmit( &huart4, (uint8_t *)uartTX, sizeof(uartTX), 100 );
    HAL_Delay( 5000 );
	//Let's take multiple gyro data samples so we can determine the average gyro offset (calibration).
	for ( cal_int = 0; cal_int < 2000 ; cal_int++ )
	{              													//Take 2000 readings for calibration.
		Setup_Gyro_Signalen();                          	//Read the gyro output.

		gyro_roll_cal += gyro_roll;                               	//Ad roll value to gyro_roll_cal.
		gyro_pitch_cal += gyro_pitch;                              	//Ad pitch value to gyro_pitch_cal.
		gyro_yaw_cal += gyro_yaw;                                  	//Ad yaw value to gyro_yaw_cal.
	}

	//Now that we have 2000 measures, we need to devide by 2000 to get the average gyro offset.
	gyro_roll_cal /= 2000;                                       //Divide the roll total by 2000.
	gyro_pitch_cal /= 2000;                                      //Divide the pitch total by 2000.
	gyro_yaw_cal /= 2000;                                        //Divide the yaw total by 2000.


    sprintf(uartTX, "                                                                                                    ");
    sprintf(uartTX, "\nLift the left side of the quadcopter to a 45 degree angle within 5 seconds!\n");
    HAL_UART_Transmit( &huart4, (uint8_t *)uartTX, sizeof(uartTX), 100 );
	HAL_Delay( 3000 );
	Check_Gyro_Axes( 1 );		//	Lift the left side of the quadcopter to a 45 degree angle within 10 seconds

    sprintf(uartTX, "                                                                                                    ");
    sprintf(uartTX, "\nLift the nose of the quadcopter to a 45 degree angle within 5 seconds!\n");
    HAL_UART_Transmit( &huart4, (uint8_t *)uartTX, sizeof(uartTX), 100 );
	HAL_Delay( 3000 );
	Check_Gyro_Axes( 2 );		//	Lift the nose of the quadcopter to a 45 degree angle within 10 seconds

    sprintf(uartTX, "                                                                                                    ");
    sprintf(uartTX, "\nRotate the nose of the quadcopter 45 degree to the right within 5 seconds!\n");
    HAL_UART_Transmit( &huart4, (uint8_t *)uartTX, sizeof(uartTX), 100 );
	HAL_Delay( 3000 );
	Check_Gyro_Axes( 3 );		//	Rotate the nose of the quadcopter 45 degree to the right within 10 seconds

    eeprom_data[0]  = center_channel_1 & 0b11111111;
    eeprom_data[1]  = center_channel_1 >> 8;
    eeprom_data[2]  = center_channel_2 & 0b11111111;
    eeprom_data[3]  = center_channel_2 >> 8;
    eeprom_data[4]  = center_channel_3 & 0b11111111;
    eeprom_data[5]  = center_channel_3 >> 8;
    eeprom_data[6]  = center_channel_4 & 0b11111111;
    eeprom_data[7]  = center_channel_4 >> 8;
    eeprom_data[8]  = high_channel_1 & 0b11111111;
    eeprom_data[9]  = high_channel_1 >> 8;
    eeprom_data[10] = high_channel_2 & 0b11111111;
    eeprom_data[11] = high_channel_2 >> 8;
    eeprom_data[12] = high_channel_3 & 0b11111111;
    eeprom_data[13] = high_channel_3 >> 8;
    eeprom_data[14] = high_channel_4 & 0b11111111;
    eeprom_data[15] = high_channel_4 >> 8;
    eeprom_data[16] = low_channel_1 & 0b11111111;
    eeprom_data[17] = low_channel_1 >> 8;
    eeprom_data[18] = low_channel_2 & 0b11111111;
    eeprom_data[19] = low_channel_2 >> 8;
    eeprom_data[20] = low_channel_3 & 0b11111111;
    eeprom_data[21] = low_channel_3 >> 8;
    eeprom_data[22] = low_channel_4 & 0b11111111;
    eeprom_data[23] = low_channel_4 >> 8;
    eeprom_data[24] = channel_1_assign;
    eeprom_data[25] = channel_2_assign;
    eeprom_data[26] = channel_3_assign;
    eeprom_data[27] = channel_4_assign;
    eeprom_data[28] = roll_axis;
    eeprom_data[29] = pitch_axis;
    eeprom_data[30] = yaw_axis;
	eeprom_data[31] = 92;


	if( eeprom_data[31] == 92 && eeprom_data[28] == 2 && eeprom_data[29] == 1 && eeprom_data[30] == 131 )
	{
	    sprintf(uartTX, "                                                                                                    ");
	    sprintf(uartTX, "\nValues look good! Saving data to the flash.....!\n");
	    HAL_UART_Transmit( &huart4, (uint8_t *)uartTX, sizeof(uartTX), 100 );


		Flash_EraseSector( 11 );
																				// EEPROM Array to Flash
		Flash_EEPROM_Data_Write( 11, 0x080E0000,  0, &eeprom_data[0]);
		Flash_EEPROM_Data_Write( 11, 0x080E0000,  1, &eeprom_data[1]);
		Flash_EEPROM_Data_Write( 11, 0x080E0000,  2, &eeprom_data[2]);
		Flash_EEPROM_Data_Write( 11, 0x080E0000,  3, &eeprom_data[3]);
		Flash_EEPROM_Data_Write( 11, 0x080E0000,  4, &eeprom_data[4]);
		Flash_EEPROM_Data_Write( 11, 0x080E0000,  5, &eeprom_data[5]);
		Flash_EEPROM_Data_Write( 11, 0x080E0000,  6, &eeprom_data[6]);
		Flash_EEPROM_Data_Write( 11, 0x080E0000,  7, &eeprom_data[7]);
		Flash_EEPROM_Data_Write( 11, 0x080E0000,  8, &eeprom_data[8]);
		Flash_EEPROM_Data_Write( 11, 0x080E0000,  9, &eeprom_data[9]);
		Flash_EEPROM_Data_Write( 11, 0x080E0000, 10, &eeprom_data[10]);
		Flash_EEPROM_Data_Write( 11, 0x080E0000, 11, &eeprom_data[11]);
		Flash_EEPROM_Data_Write( 11, 0x080E0000, 12, &eeprom_data[12]);
		Flash_EEPROM_Data_Write( 11, 0x080E0000, 13, &eeprom_data[13]);
		Flash_EEPROM_Data_Write( 11, 0x080E0000, 14, &eeprom_data[14]);
		Flash_EEPROM_Data_Write( 11, 0x080E0000, 15, &eeprom_data[15]);
		Flash_EEPROM_Data_Write( 11, 0x080E0000, 16, &eeprom_data[16]);
		Flash_EEPROM_Data_Write( 11, 0x080E0000, 17, &eeprom_data[17]);
		Flash_EEPROM_Data_Write( 11, 0x080E0000, 18, &eeprom_data[18]);
		Flash_EEPROM_Data_Write( 11, 0x080E0000, 19, &eeprom_data[19]);
		Flash_EEPROM_Data_Write( 11, 0x080E0000, 20, &eeprom_data[20]);
		Flash_EEPROM_Data_Write( 11, 0x080E0000, 21, &eeprom_data[21]);
		Flash_EEPROM_Data_Write( 11, 0x080E0000, 22, &eeprom_data[22]);
		Flash_EEPROM_Data_Write( 11, 0x080E0000, 23, &eeprom_data[23]);
		Flash_EEPROM_Data_Write( 11, 0x080E0000, 24, &eeprom_data[24]);
		Flash_EEPROM_Data_Write( 11, 0x080E0000, 25, &eeprom_data[25]);
		Flash_EEPROM_Data_Write( 11, 0x080E0000, 26, &eeprom_data[26]);
		Flash_EEPROM_Data_Write( 11, 0x080E0000, 27, &eeprom_data[27]);
		Flash_EEPROM_Data_Write( 11, 0x080E0000, 28, &eeprom_data[28]);
		Flash_EEPROM_Data_Write( 11, 0x080E0000, 29, &eeprom_data[29]);
		Flash_EEPROM_Data_Write( 11, 0x080E0000, 30, &eeprom_data[30]);
		Flash_EEPROM_Data_Write( 11, 0x080E0000, 31, &eeprom_data[31]);


		Flash_EEPROM_Data_Read( 11, 0x080E0000, 31, &eeprom_data_test[0]);

	    sprintf(uartTX, "                                                                                                    ");
	    sprintf(uartTX, "\nSetup is done! Everything is looks OKAY! All data is saved to the flash! Reset!\n");
	    HAL_UART_Transmit( &huart4, (uint8_t *)uartTX, sizeof(uartTX), 100 );
	}
	else if( eeprom_data_test[0] == 92 && eeprom_data[28] != 2 && eeprom_data[29] != 1 && eeprom_data[30] != 131 )
	{
	    sprintf(uartTX, "                                                                                                    ");
	    sprintf(uartTX, "\nSetup is NOT done! Gyro axises are useless! Re-do the setup!\n");
	    HAL_UART_Transmit( &huart4, (uint8_t *)uartTX, sizeof(uartTX), 100 );

		Flash_EraseSector( 11 );
	}
	else
	{
	    sprintf(uartTX, "                                                                                                    ");
	    sprintf(uartTX, "\nSetup is NOT done! All data is useless! Re-do the setup!\n");
	    HAL_UART_Transmit( &huart4, (uint8_t *)uartTX, sizeof(uartTX), 100 );

		Flash_EraseSector( 11 );
	}
}

void Setup_Gyro_Signalen()
{
	L3GD20H_Read_Gyro_RAW_Outputs();

	gyro_roll  = gX_Raw;
	gyro_pitch = gY_Raw;
	gyro_yaw   = gZ_Raw;

	if(cal_int == 2000)							//Only compensate after the calibration
	{
		gyro_roll  -= gyro_roll_cal;
		gyro_pitch -= gyro_pitch_cal;
		gyro_yaw   -= gyro_yaw_cal;
	}
}

void Check_Gyro_Axes( uint8_t movement )
{
	uint8_t trigger_axis = 0;
	float gyro_angle_roll, gyro_angle_pitch, gyro_angle_yaw;

	//Reset all axes
	gyro_angle_roll = 0;
	gyro_angle_pitch = 0;
	gyro_angle_yaw = 0;

	Setup_Gyro_Signalen();

	unsigned int i = 10000;
	while( gyro_angle_roll > -30 && gyro_angle_roll < 30 && gyro_angle_pitch > -30 && gyro_angle_pitch < 30 && gyro_angle_yaw > -30 && gyro_angle_yaw < 30 )
	{
		Setup_Gyro_Signalen();
		i--;

		gyro_angle_roll += gyro_roll * 0.00007;              //0.00007 = 17.5 (md/s) / 250(Hz)
		gyro_angle_pitch += gyro_pitch * 0.00007;
		gyro_angle_yaw += gyro_yaw * 0.00007;

//		HAL_Delay( 4 );	 //Loop is running @ 250Hz. +/-300us is used for communication with the gyro
	}


	//Assign the moved axis to the corresponding function (pitch, roll, yaw)
	if(( gyro_angle_roll < -30 || gyro_angle_roll > 30 ) && gyro_angle_pitch > -30 && gyro_angle_pitch < 30 && gyro_angle_yaw > -30 && gyro_angle_yaw < 30 )
	{
		gyro_check_byte |= 0b00000001;
		if( gyro_angle_roll < 0 )	{	trigger_axis = 0b10000001;	}
		else	{	trigger_axis = 0b00000001;	}
	}

	if(( gyro_angle_pitch < -30 || gyro_angle_pitch > 30 ) && gyro_angle_roll > -30 && gyro_angle_roll < 30 && gyro_angle_yaw > -30 && gyro_angle_yaw < 30 )
	{
		gyro_check_byte |= 0b00000010;
		if( gyro_angle_pitch < 0 )	{	trigger_axis = 0b10000010;	}
		else	{	trigger_axis = 0b00000010;	}
	}

	if(( gyro_angle_yaw < -30 || gyro_angle_yaw > 30 ) && gyro_angle_roll > -30 && gyro_angle_roll < 30 && gyro_angle_pitch > -30 && gyro_angle_pitch < 30 )
	{
		gyro_check_byte |= 0b00000100;
		if( gyro_angle_yaw < 0 )	{	trigger_axis = 0b10000011;	}
		else	{	trigger_axis = 0b00000011;	}
	}

	if( trigger_axis == 0 )
	{
		error = true;
	}

	else
	if( movement == 1 )	{	roll_axis  = trigger_axis;	}
	if( movement == 2 )	{	pitch_axis = trigger_axis;	}
	if( movement == 3 )	{	yaw_axis   = trigger_axis;	}
}

void Register_Min_Max( void )
{
	uint8_t zero = 0;
	low_channel_1 = receiver_input_channel_1;
	low_channel_2 = receiver_input_channel_2;
	low_channel_3 = receiver_input_channel_3;
	low_channel_4 = receiver_input_channel_4;

	while(receiver_input_channel_1 < center_channel_1 + 20 && receiver_input_channel_1 > center_channel_1 - 20)
	{
		for(int i = 10000; i > 0; i-- )	{	}
	}
	
	int k = 20000000;
	while( k > 0 && zero < 15 )
	{
		if(	receiver_input_channel_1 < center_channel_1 + 20 && receiver_input_channel_1 > center_channel_1 - 20)	{	zero |= 0b00000001;	}
		if(	receiver_input_channel_2 < center_channel_2 + 20 && receiver_input_channel_2 > center_channel_2 - 20)	{	zero |= 0b00000010;	}
		if(	receiver_input_channel_3 < center_channel_3 + 20 && receiver_input_channel_3 > center_channel_3 - 20)	{	zero |= 0b00000100;	}
		if(	receiver_input_channel_4 < center_channel_4 + 20 && receiver_input_channel_4 > center_channel_4 - 20)	{	zero |= 0b00001000;	}

		if(	receiver_input_channel_1 < low_channel_1)	{	low_channel_1 = receiver_input_channel_1;	}
		if(	receiver_input_channel_2 < low_channel_2)	{	low_channel_2 = receiver_input_channel_2;	}
		if(	receiver_input_channel_3 < low_channel_3)	{	low_channel_3 = receiver_input_channel_3;	}
		if(	receiver_input_channel_4 < low_channel_4)	{	low_channel_4 = receiver_input_channel_4;	}
		
		if(	receiver_input_channel_1 > high_channel_1)	{	high_channel_1 = receiver_input_channel_1;	}
		if(	receiver_input_channel_2 > high_channel_2)	{	high_channel_2 = receiver_input_channel_2;	}
		if(	receiver_input_channel_3 > high_channel_3)	{	high_channel_3 = receiver_input_channel_3;	}
		if(	receiver_input_channel_4 > high_channel_4)	{	high_channel_4 = receiver_input_channel_4;	}

		k--;
	}
}

void Check_Receiver_Inputs( uint8_t movement )			//Check if a receiver input value is changing within 30 seconds
{
	uint8_t trigger = 0;
	int pulse_length;
	unsigned int i = 3000000;

	while( i > 0 && trigger == 0 )
	{
		HAL_Delay( 250 );

		if(receiver_input_channel_1 > 1750 || receiver_input_channel_1 < 1250)
		{
			trigger = 1;
			receiver_check_byte |= 0b00000001;
			pulse_length = receiver_input_channel_1;
		}

		if(receiver_input_channel_2 > 1750 || receiver_input_channel_2 < 1250)
		{
			trigger = 2;
			receiver_check_byte |= 0b00000010;
			pulse_length = receiver_input_channel_2;
		}

		if(receiver_input_channel_3 > 1750 || receiver_input_channel_3 < 1250)
		{
			trigger = 3;
			receiver_check_byte |= 0b00000100;
			pulse_length = receiver_input_channel_3;
		}

		if(receiver_input_channel_4 > 1750 || receiver_input_channel_4 < 1250)
		{
			trigger = 4;
			receiver_check_byte |= 0b00001000;
			pulse_length = receiver_input_channel_4;
		}

		i--;
	}

	if(trigger == 0)
	{
		error = 1;
	}

	else		//Assign the stick to the function.
	{
		if(movement == 1)
		{
			channel_3_assign = trigger;
			if(pulse_length < 1250)channel_3_assign += 0b10000000;
		}

		if(movement == 2)
		{
			channel_1_assign = trigger;
			if(pulse_length < 1250)channel_1_assign += 0b10000000;
		}

		if(movement == 3)
		{
			channel_2_assign = trigger;
			if(pulse_length < 1250)channel_2_assign += 0b10000000;
		}

		if(movement == 4)
		{
			channel_4_assign = trigger;
     		if(pulse_length < 1250)channel_4_assign += 0b10000000;
		}
	}
}

/*
 * ESC_Calibration.c
 *
 *  Created on: 26 mar. 2020
 *      Author: MERT OCAL
 */

#include "ESC_Calibration.h"


int receiver_input_channel_3;
int receiver_input[5];
//
int esc_1, esc_2, esc_3, esc_4;
//
extern uint8_t eeprom_data[32];


void ESC_Calibration ( void )
{
	PWM_Output_Start();

	while ( true )
	{
		receiver_input_channel_3 = Convert_Receiver_Channel(3);

		esc_3 = receiver_input_channel_3;

		PWM_Output_ESC_Calibration( esc_3 );
	}
}

int Convert_Receiver_Channel( uint8_t function )
{
	uint8_t channel, reverse;                                                       	//First we declare some local variables
	int low, center, high, actual;
	int difference;

	channel = eeprom_data[function + 23] & 0b00000111;                           		//What channel corresponds with the specific function
	if( eeprom_data[function + 23] & 0b10000000 )	{	reverse = 1;	}               //Reverse channel when most significant bit is set
	else	{	reverse = 0;	}                                                       //If the most significant is not set there is no reverse

	actual = receiver_input[channel];                                            		//Read the actual receiver value for the corresponding function
	low	   = ( eeprom_data[channel * 2 + 15] << 8 ) | eeprom_data[channel * 2 + 14];  	//Store the low value for the specific receiver input channel
	center = ( eeprom_data[channel * 2 - 1]  << 8 ) | eeprom_data[channel * 2 - 2 ]; 	//Store the center value for the specific receiver input channel
	high   = ( eeprom_data[channel * 2 + 7]  << 8 ) | eeprom_data[channel * 2 + 6 ];  	//Store the high value for the specific receiver input channel

	if( actual < center )
	{                                                         							//The actual receiver value is lower than the center value
		if( actual < low )	{	actual = low;	}                                       //Limit the lowest value to the value that was detected during setup
		difference = ( (long)(center - actual) * (long)500 ) / (center - low);       	//Calculate and scale the actual value to a 1000 - 2000us value
		if( reverse == 1 )	{	return 1500 + difference;	}                           //If the channel is reversed
		else 	 {	return 1500 - difference;	}                                       //If the channel is not reversed
	}
	else if( actual > center )
	{                                                                        			//The actual receiver value is higher than the center value
		if( actual > high )	{	actual = high;	}                                       //Limit the lowest value to the value that was detected during setup
		difference = ( (long)(actual - center) * (long)500 ) / (high - center);      	//Calculate and scale the actual value to a 1000 - 2000us value
		if( reverse == 1 )	{	return 1500 - difference;	}                           //If the channel is reversed
		else	{	return 1500 + difference;	}                                       //If the channel is not reversed
	}
	else	{	return 1500;	}
}

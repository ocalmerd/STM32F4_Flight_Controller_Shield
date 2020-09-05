/*
 * Flight_Control.c
 *
 *  Created on: 19 mar. 2020
 *      Author: MERT OCAL
 */

#include "Flight_Control.h"


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//PID gain and limit settings
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
float pid_p_gain_roll = 2.4;               	//Gain setting for the roll P-controller 		- (1.6)
float pid_i_gain_roll = 0.008;              //Gain setting for the roll I-controller 		- (0.007)
float pid_d_gain_roll = 120;                //Gain setting for the roll D-controller 		- (100)
int pid_max_roll = 400;                   	//Maximum output of the PID-controller (+/-) 	- (400)

float pid_p_gain_pitch = 2.4;    			//Gain setting for the pitch P-controller		- (1.6)
float pid_i_gain_pitch = 0.008;  			//Gain setting for the pitch I-controller 		- (0.007)
float pid_d_gain_pitch = 120;  				//Gain setting for the pitch D-controller 		- (100)
int pid_max_pitch = 400;          			//Maximum output of the PID-controller (+/-)    - (400)

float pid_p_gain_yaw = 4.0;                	//Gain setting for the pitch P-controller 		- (4.0)
float pid_i_gain_yaw = 0.02;               	//Gain setting for the pitch I-controller  		- (0.02)
float pid_d_gain_yaw = 0.0;                	//Gain setting for the pitch D-controller   	- (0)
int pid_max_yaw = 400;                     	//Maximum output of the PID-controller (+/-) 	- (400)

float pid_error_temp;
float pid_i_mem_roll, pid_roll_setpoint, gyro_roll_input, pid_output_roll, pid_last_roll_d_error;
float pid_i_mem_pitch, pid_pitch_setpoint, gyro_pitch_input, pid_output_pitch, pid_last_pitch_d_error;
float pid_i_mem_yaw, pid_yaw_setpoint, gyro_yaw_input, pid_output_yaw, pid_last_yaw_d_error;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Declaring global variables
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int start, throttle, cal_int = 0;
int receiver_input[5];
double gyro_axis[4], gyro_axis_cal[4];
//
extern int esc_1, esc_2, esc_3, esc_4;
extern int receiver_input_channel_1, receiver_input_channel_2, receiver_input_channel_3, receiver_input_channel_4;
//
extern int16_t gX_Raw, gY_Raw, gZ_Raw;
extern double gyro_pitch, gyro_roll, gyro_yaw;
//
extern uint8_t eeprom_data[32];


void Flight_Control_Setup( void )
{
	HAL_Delay( 3000 );

	//Let's take multiple gyro data samples so we can determine the average gyro offset (calibration).
	for ( cal_int = 0; cal_int < 2000 ; cal_int ++ )				//Take 2000 readings for calibration.
	{
		Flight_Gyro_Signalen();                                           //Read the gyro output.

		gyro_axis_cal[1] += gyro_axis[1];                          //Ad roll value to gyro_roll_cal.
		gyro_axis_cal[2] += gyro_axis[2];                          //Ad pitch value to gyro_pitch_cal.
		gyro_axis_cal[3] += gyro_axis[3];                          //Ad yaw value to gyro_yaw_cal.                                               //Wait 3 milliseconds before the next loop.
	}

	//Now that we have 2000 measures, we need to divide by 2000 to get the average gyro offset.
	gyro_axis_cal[1] /= 2000;                                    //Divide the roll total by 2000.
	gyro_axis_cal[2] /= 2000;                                    //Divide the pitch total by 2000.
	gyro_axis_cal[3] /= 2000;                                    //Divide the yaw total by 2000.

	//Wait until the receiver is active and the throttle is set to the lower position.
	while( receiver_input_channel_3 > 1100 )
	{
		receiver_input_channel_3 = Convert_Receiver_Channels(3);    //Convert the actual receiver signals for throttle to the standard 1000 - 2000us
		receiver_input_channel_4 = Convert_Receiver_Channels(4);    //Convert the actual receiver signals for yaw to the standard 1000 - 2000us
	}

	PWM_Output_Start();
	PWM_Output_Idle();

	start = 0;           //Set start back to 0.
}

void Flight_Control_Loop( void )
{
	receiver_input_channel_1 = Convert_Receiver_Channels(1);      // Convert the actual receiver signals for pitch to the standard 1000 - 2000us.
	receiver_input_channel_2 = Convert_Receiver_Channels(2);      // Convert the actual receiver signals for roll to the standard 1000 - 2000us.
	receiver_input_channel_3 = Convert_Receiver_Channels(3);      // Convert the actual receiver signals for throttle to the standard 1000 - 2000us.
	receiver_input_channel_4 = Convert_Receiver_Channels(4);      // Convert the actual receiver signals for yaw to the standard 1000 - 2000us.

	//Let's get the current gyroscope data and scale it to degrees per second for the PID calculations.
	Flight_Gyro_Signalen();

	gyro_roll_input 	= ( gyro_roll_input * 0.8 )	+ ( ( gyro_roll 	/ 57.14286 ) * 0.2);      // Gyroscope PID input is deg/sec.
	gyro_pitch_input 	= ( gyro_pitch_input * 0.8) + ( ( gyro_pitch    / 57.14286 ) * 0.2);      // Gyroscope PID input is deg/sec.
	gyro_yaw_input		= ( gyro_yaw_input * 0.8  ) + ( ( gyro_yaw 	    / 57.14286 ) * 0.2);      // Gyroscope PID input is deg/sec.

	//For starting the motors: throttle low and yaw left (step 1).
	if( receiver_input_channel_3 < 1100 && receiver_input_channel_4 < 1100 )
	{
//		if ( L3GD20H_Connection_Check() )
//		{
		start = 1;
//		}
	}

	//When yaw stick is back in the center position start the motors (step 2).
	if( start == 1 && receiver_input_channel_3 < 1100 && receiver_input_channel_4 > 1400 )
	{
		start = 2;

		//Reset the PID controllers for a bumpless start.
		pid_i_mem_roll = 0;
		pid_last_roll_d_error = 0;
		pid_i_mem_pitch = 0;
		pid_last_pitch_d_error = 0;
		pid_i_mem_yaw = 0;
		pid_last_yaw_d_error = 0;
	}

	//Stopping the motors: throttle low and yaw right.
	if( start == 2 && receiver_input_channel_3 < 1100 && receiver_input_channel_4 > 1900 )
	{
		start = 0;

		PWM_Output_Idle();
	}

	//The PID set point in degrees per second is determined by the roll receiver input.
	//In the case of dividing by 3 the max roll rate is approximately 164 degrees per second ( (500-8)/3 = 164d/s ).
	pid_roll_setpoint = 0;
	//We need a little dead band of 16us for better results.
	if     ( receiver_input_channel_1 > 1508 )	{	pid_roll_setpoint = (receiver_input_channel_1 - 1508)/3.0;	}
	else if( receiver_input_channel_1 < 1492 )	{	pid_roll_setpoint = (receiver_input_channel_1 - 1492)/3.0;	}

	//The PID set point in degrees per second is determined by the pitch receiver input.
	//In the case of dividing by 3 the max pitch rate is approximately 164 degrees per second ( (500-8)/3 = 164d/s ).
	pid_pitch_setpoint = 0;
	//We need a little dead band of 16us for better results.
	if     ( receiver_input_channel_2 > 1508 )	{	pid_pitch_setpoint = (receiver_input_channel_2 - 1508)/3.0;	}
	else if( receiver_input_channel_2 < 1492 )	{	pid_pitch_setpoint = (receiver_input_channel_2 - 1492)/3.0;	}

	//The PID set point in degrees per second is determined by the yaw receiver input.
	//In the case of dividing by 3 the max yaw rate is approximately 164 degrees per second ( (500-8)/3 = 164d/s ).
	pid_yaw_setpoint = 0;
	//We need a little dead band of 16us for better results.
	if(receiver_input_channel_3 > 1050)
	{ 	//Do not yaw when turning off the motors.
		if     ( receiver_input_channel_4 > 1508 )	{	pid_yaw_setpoint = (receiver_input_channel_4 - 1508)/3.0;	}
		else if( receiver_input_channel_4 < 1492 )	{	pid_yaw_setpoint = (receiver_input_channel_4 - 1492)/3.0;	}
	}

	//PID inputs are known. So we can calculate the pid output.
	Calculate_PID();

	throttle = receiver_input_channel_3;                                      // We need the throttle signal as a base signal.

	if ( start == 2 )
	{                                                 			// The motors are started.
		if ( throttle > 1800 )	{	throttle = 1800;	}                           // We need some room to keep full control at full throttle.
		esc_1 = throttle - pid_output_pitch + pid_output_roll - pid_output_yaw; 	// Calculate the pulse for esc_1 (front-right - CCW)
		esc_2 = throttle + pid_output_pitch + pid_output_roll + pid_output_yaw; 	// Calculate the pulse for esc_2 (rear-right  - CW)
		esc_3 = throttle + pid_output_pitch - pid_output_roll - pid_output_yaw; 	// Calculate the pulse for esc_3 (rear-left   - CCW)
		esc_4 = throttle - pid_output_pitch - pid_output_roll + pid_output_yaw; 	// Calculate the pulse for esc_4 (front-left  - CW)

		if( esc_1 < 1200 )	{	esc_1 = 1200;	}				// Keep the motors running.
		if( esc_2 < 1200 )	{	esc_2 = 1200;	}               // Keep the motors running.
		if( esc_3 < 1200 )	{	esc_3 = 1200;	}               // Keep the motors running.
		if( esc_4 < 1200 )	{	esc_4 = 1200;	}               // Keep the motors running.

		if( esc_1 > 2000 )	{	esc_1 = 2000;	}               // Limit the esc_1 pulse to 2000us.
		if( esc_2 > 2000 )	{	esc_2 = 2000;	}               // Limit the esc_2 pulse to 2000us.
		if( esc_3 > 2000 )	{	esc_3 = 2000;	}               // Limit the esc_3 pulse to 2000us.
		if( esc_4 > 2000 )	{	esc_4 = 2000;	}               // Limit the esc_4 pulse to 2000us.

		PWM_Output_ESC( esc_1, esc_2, esc_3, esc_4 );
	}

	else
	{
		PWM_Output_Idle();
	}
}

void Flight_Gyro_Signalen( void )
{
	L3GD20H_Read_Gyro_RAW_Outputs();

	gyro_axis[1] = gX_Raw;
	gyro_axis[2] = gY_Raw;
	gyro_axis[3] = gZ_Raw;

	if(cal_int == 2000)
	{
		gyro_axis[1] -= gyro_axis_cal[1];                            //Only compensate after the calibration
	    gyro_axis[2] -= gyro_axis_cal[2];                            //Only compensate after the calibration
	    gyro_axis[3] -= gyro_axis_cal[3];                            //Only compensate after the calibration
	}

	gyro_roll  = gyro_axis[eeprom_data[28] & 0b00000011];
	if(	eeprom_data[28] & 0b10000000 )	{	gyro_roll *= -1;	}

	gyro_pitch = gyro_axis[eeprom_data[29] & 0b00000011];
	if(	eeprom_data[29] & 0b10000000 )	{	gyro_pitch *= -1;	}

	gyro_yaw   = gyro_axis[eeprom_data[30] & 0b00000011];
	if(	eeprom_data[30] & 0b10000000 )	{	gyro_yaw *= -1;		}
}

void Calculate_PID( void )
{
	//Roll calculations
	pid_error_temp = gyro_roll_input - pid_roll_setpoint;
	pid_i_mem_roll += pid_i_gain_roll * pid_error_temp;
	if		( pid_i_mem_roll > pid_max_roll )			{	pid_i_mem_roll = pid_max_roll;	}
	else if	( pid_i_mem_roll < pid_max_roll * -1 )		{	pid_i_mem_roll = pid_max_roll * -1;	}

	pid_output_roll = pid_p_gain_roll * pid_error_temp + pid_i_mem_roll + pid_d_gain_roll * (pid_error_temp - pid_last_roll_d_error);
	if		( pid_output_roll > pid_max_roll )			{	pid_output_roll = pid_max_roll;	}
	else if	( pid_output_roll < pid_max_roll * -1 )		{	pid_output_roll = pid_max_roll * -1;	}

	pid_last_roll_d_error = pid_error_temp;


	//Pitch calculations
	pid_error_temp = gyro_pitch_input - pid_pitch_setpoint;
	pid_i_mem_pitch += pid_i_gain_pitch * pid_error_temp;
	if     	( pid_i_mem_pitch > pid_max_pitch )			{	pid_i_mem_pitch = pid_max_pitch;	}
	else if	( pid_i_mem_pitch < pid_max_pitch * -1 ) 	{	pid_i_mem_pitch = pid_max_pitch * -1;	}

	pid_output_pitch = pid_p_gain_pitch * pid_error_temp + pid_i_mem_pitch + pid_d_gain_pitch * (pid_error_temp - pid_last_pitch_d_error);
	if		( pid_output_pitch > pid_max_pitch )		{	pid_output_pitch = pid_max_pitch;	}
	else if	( pid_output_pitch < pid_max_pitch * -1 )	{	pid_output_pitch = pid_max_pitch * -1;	}

	pid_last_pitch_d_error = pid_error_temp;


	//Yaw calculations
	pid_error_temp = gyro_yaw_input - pid_yaw_setpoint;
	pid_i_mem_yaw += pid_i_gain_yaw * pid_error_temp;
	if		( pid_i_mem_yaw > pid_max_yaw )			{	pid_i_mem_yaw = pid_max_yaw;	}
	else if	( pid_i_mem_yaw < pid_max_yaw * -1 )	{	pid_i_mem_yaw = pid_max_yaw * -1;	}

	pid_output_yaw = pid_p_gain_yaw * pid_error_temp + pid_i_mem_yaw + pid_d_gain_yaw * (pid_error_temp - pid_last_yaw_d_error);
	if		( pid_output_yaw > pid_max_yaw )		{	pid_output_yaw = pid_max_yaw;	}
	else if	( pid_output_yaw < pid_max_yaw * -1 )	{	pid_output_yaw = pid_max_yaw * -1;	}

	pid_last_yaw_d_error = pid_error_temp;
}

int Convert_Receiver_Channels( uint8_t function )
{
	uint8_t channel, reverse;                                                       //First we declare some local variables
	int low, center, high, actual;
	int difference;

	channel = eeprom_data[function + 23] & 0b00000111;                           //What channel corresponds with the specific function
	if( eeprom_data[function + 23] & 0b10000000  )	{	reverse = 1;	}            //Reverse channel when most significant bit is set
	else	{	reverse = 0;	}                                                            //If the most significant is not set there is no reverse

	actual = receiver_input[channel];                                            //Read the actual receiver value for the corresponding function
	low 	= ( eeprom_data[channel * 2 + 15] << 8 ) | eeprom_data[channel * 2 + 14]; 	//Store the low value for the specific receiver input channel
	center 	= ( eeprom_data[channel * 2 - 1] << 8 )  | eeprom_data[channel * 2 - 2];	//Store the center value for the specific receiver input channel
	high 	= ( eeprom_data[channel * 2 + 7] << 8 )  | eeprom_data[channel * 2 + 6];   	//Store the high value for the specific receiver input channel

	if( actual < center )
	{                                             	//The actual receiver value is lower than the center value
		if( actual < low )	{	actual = low;	}                              	//Limit the lowest value to the value that was detected during setup
		difference = ( (long)(center - actual) * (long)500 ) / (center - low);  //Calculate and scale the actual value to a 1000 - 2000us value
		if( reverse == 1 )	{	return 1500 + difference;	}                   //If the channel is reversed
		else	{	return 1500 - difference;	}                               //If the channel is not reversed
	}
	else if(actual > center)
	{                                           	//The actual receiver value is higher than the center value
		if(actual > high)	{	actual = high;	}                               //Limit the lowest value to the value that was detected during setup
		difference = ( (long)(actual - center) * (long)500 ) / (high - center); //Calculate and scale the actual value to a 1000 - 2000us value
		if( reverse == 1 )	{	return 1500 - difference;	}                  	//If the channel is reversed
		else	{	return 1500 + difference;	}                               //If the channel is not reversed
	}
	else	{	return 1500;	}
}


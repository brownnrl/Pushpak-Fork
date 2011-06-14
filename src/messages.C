/*
  This library is free software; you can redistribute it and/or modify it under the
  terms of the GNU Lesser General Public License as published by the Free Software
  Foundation; either version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful, but WITHOUT ANY
  WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A 
  PARTICULAR PURPOSE.  See the GNU Lesser General Public License for more details.
*/

// Copyright (c) 2009 Brijesh Sirpatil
#include <inttypes.h>

#include "util.h"
#include "adc.h"
#include "pushpak.h"


#include "HardwareSerial.h"
#include "Print.h"
#include "packet.h"


#define MSG_ADC12B_ID 5
#define MSG_SENSOR_ID 6

#define MSG_READ 1
#define MSG_WRITE 2

#define MSG_READ_RSP 3	//Response to Read command


//! Global Parameters
uint8_t gRun_flag = 0;			//index[1]
uint8_t gRun_mode = 0;			//index[2]
uint8_t gMotor_FF_drive = 45;	//index[3]
uint8_t gMotor_max_drive = 100;	//index[4]
uint8_t gMotor_min_drive = 30;	//index[5]
uint8_t gP_term = 0;			//index[6]
uint8_t gP_term_limit = 0;		//index[7]


//Prototype message structure.
struct msg_sensor_type {
	uint8_t		hdr1;		//header byte 1
	uint8_t		hdr2;		//header byte 2
	uint8_t		id; 		//Message type id	
	uint8_t		length;		//Message length not incudling header, type and length fields.

	uint32_t 	sys_time; 
	int16_t	acc_x;
	int16_t	acc_y;
	int16_t	acc_z;
	int16_t	gyro_x;
	int16_t	gyro_y;
	int16_t	gyro_z;
};

struct msg_ADC12b_type {
	uint8_t		hdr1;		//header byte 1
	uint8_t		hdr2;		//header byte 2
	uint8_t		id; 		//Message type id	
	uint8_t		length;		//Message length not incudling header and type.

	uint32_t 	sys_time;
	int16_t	adc_data[8];
};


void send_msg_ADC12b_samples(void)
{
	
	struct msg_ADC12b_type msg = {0xAB, 0xCD, MSG_ADC12B_ID}; 
	
	msg.length = sizeof(msg_ADC12b_type) - 4;
	msg.sys_time = adc_get_sample_time();
	
	msg.adc_data[0] = gADC_output[0];
	msg.adc_data[1] = gADC_output[1];
	msg.adc_data[2] = gADC_output[2];
	msg.adc_data[3] = gADC_output[3];
	msg.adc_data[4] = gADC_output[4];
	msg.adc_data[5] = gADC_output[5];
	msg.adc_data[6] = gADC_output[6];
	msg.adc_data[7] = gADC_output[7];

	Serial.write((uint8_t*)&msg, sizeof(msg_ADC12b_type));

}

void send_msg_sensor_values(void)
{
	struct msg_sensor_type msg = {0xAB, 0xCD, MSG_SENSOR_ID}; 
	
	msg.length = sizeof(msg_sensor_type) - 4;
	msg.sys_time = adc_get_sample_time();
	
 	msg.acc_x = gADC_output[0];
	msg.acc_y = gADC_output[1];
	msg.acc_z = gADC_output[2];
	msg.gyro_x = gADC_output[3];
	msg.gyro_y = gADC_output[4];
	msg.gyro_z = gADC_output[5];

	Serial.write((uint8_t*)&msg, sizeof(msg_sensor_type));
}


//! Write value into parameter table. The table may contain differnt data types as different locations.
//! The data type is determined by the location.
void parameter_write(uint8_t index, void *ptr)
{
	switch(index)
	{
		case 1:
			gRun_flag = * (uint8_t *) ptr; 	// Start/Stop Command
			break;
		case 2:
			gRun_mode = * (uint8_t *) ptr; 	// RC Control/Autonomous/Test mode
			break;
		case 3:
			gMotor_FF_drive	= * (uint8_t *) ptr; 	//Motor Feed Forward drive value
			break;
		case 4:
			gMotor_max_drive = * (uint8_t *) ptr; 	
			break;
		case 5:
			gMotor_min_drive = * (uint8_t *) ptr; 	
			break;
			
		case 6:
			gP_term = * (uint8_t *) ptr;			// PID control loops P term
			break;
			
		case 7:
			gP_term_limit = * (uint8_t *) ptr;	// PID control loops P term limit, max value
			break;
		
		default:
			break;	
	}
}

//! Returns pointer to parameter being read.
void parameter_read(uint8_t index)
{
	uint8_t buf[5];
	
	void* temp;
	temp = NULL;
	switch(index)
	{
		case 1:
			temp = &gRun_flag; 	// Start/Stop Command
			break;
		case 2:
			temp = &gRun_mode; 	// RC Control/Autonomous/Test mode
			break;
		case 3:
			temp = &gMotor_FF_drive; 	//Motor Feed Forward drive value
			break;
		case 4:
			temp = &gMotor_max_drive; 	
			break;
		case 5:
			temp = &gMotor_min_drive; 	
			break;
			
		case 6:
			temp = &gP_term;			// PID control loops P term
			break;
			
		case 7:
			temp = &gP_term_limit;	// PID control loops P term limit, max value
			break;
		
		default:
			temp = NULL;
			break;	
	}
	
	buf[0] = MSG_READ_RSP;
	buf[1] = * ((uint8_t *) temp);
	buf[2] = * (((uint8_t *) temp) + 1);
	buf[3] = * (((uint8_t *) temp) + 2);
	buf[4] = * (((uint8_t *) temp) + 3);
	
	send_pkt(buf,5);
}



void process_msg(uint8_t *buffer)
{
	switch(buffer[0])
	{
		case MSG_READ :
			parameter_read(buffer[1]);
			break;
		case MSG_WRITE :
			parameter_write(buffer[1], &buffer[2]);
			break;
		default :
			break;
	}
}



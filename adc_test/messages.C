/*
  This library is free software; you can redistribute it and/or modify it under the
  terms of the GNU Lesser General Public License as published by the Free Software
  Foundation; either version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful, but WITHOUT ANY
  WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A 
  PARTICULAR PURPOSE.  See the GNU Lesser General Public License for more details.
*/

// Copyright (c) 2009 Brijesh Sirpatil

#include "pushpak.h"
#include "util.h"
#include "adc.h"

#include "Print.h"
#include "HardwareSerial.h"


#define MSG_ADC12B_ID 2
#define MSG_SENSOR_ID 3


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

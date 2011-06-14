/*
  This library is free software; you can redistribute it and/or modify it under the
  terms of the GNU Lesser General Public License as published by the Free Software
  Foundation; either version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful, but WITHOUT ANY
  WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A 
  PARTICULAR PURPOSE.  See the GNU Lesser General Public License for more details.
*/

// Copyright (c) 2009 Brijesh Sirpatil

#include <util/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#include "pushpak.h"
#include "util.h"
#include "adc.h"

#include "Print.h"
#include "HardwareSerial.h"
#include "messages.h"
#include "Accelerometer.h"
#include "GyroSensor.h"


volatile uint32_t gSystemTime = 0;


//Message structure from Robosavvy.com CHR-6d Digital Inertial Measurement Unit
struct chr6d_msg{
	uint8_t		hdr1; 		
	uint8_t		hdr2; 		
	uint8_t		hdr3; 		

	uint8_t		pkt_type; 		
	uint8_t		length;
	uint8_t		active;
	int16_t	gyro_z;
	int16_t	gyro_y;
	int16_t	gyro_x;
	int16_t	acc_z;
	int16_t	acc_y;
	int16_t	acc_x;
	uint16_t chk_sum;
};

struct chr6d_msg pkt;
///////////////////////////////////////////////////////////////////////////////


void sensor_calibrate()
{
	uint8_t	i;
	const uint8_t AVG_CNT = 100;

	uint32_t gyro_x_zero, gyro_y_zero, gyro_z_zero;
	uint32_t acc_x_zeroG, acc_y_zeroG, acc_z_zeroG;
	
		
	GPIO_SET(GYRO_AUTO_ZERO); //send auto zero pulse
	_delay_us(100);
	GPIO_CLEAR(GYRO_AUTO_ZERO); 
	
	_delay_ms(100); //Gyro needs 10ms to finish auto zero, but we have low pass filter so wait longer

	acc_x_zeroG = 0;
 	acc_y_zeroG = 0;
 	acc_z_zeroG = 0;
 	gyro_x_zero = 0;
 	gyro_y_zero = 0;
 	gyro_z_zero = 0;
	
	for(i=0;i<AVG_CNT;++i) //collect data for 1sec or 100 times
	{
		adc_update();
 		acc_x_zeroG += adc_get_value_mv(ACCL_X_CH);
	 	acc_y_zeroG += adc_get_value_mv(ACCL_Y_CH);
	 	acc_z_zeroG += adc_get_value_mv(ACCL_Z_CH);
	 	
	 	gyro_x_zero += gADC_output[3];
	 	gyro_y_zero += gADC_output[4];
	 	gyro_z_zero += gADC_output[5];		
	
	}	
	
	acc_x_zeroG /= AVG_CNT;
 	acc_y_zeroG /= AVG_CNT;
 	acc_z_zeroG /= AVG_CNT;
 	gyro_x_zero /= AVG_CNT;
 	gyro_y_zero /= AVG_CNT;
 	gyro_z_zero /= AVG_CNT;
	
 	 	
 	//Note for Z axis using the same value as X axis. When sensor board is flat, the Z is either +/- 1G.
	Acclmtr.set_zerog_values((uint16_t)acc_x_zeroG, (uint16_t)acc_y_zeroG, (uint16_t) acc_x_zeroG);	 	 	
 			
	Gyro.set_zero_values((uint16_t)gyro_x_zero, (uint16_t)gyro_y_zero, (uint16_t) gyro_z_zero);	 	 	
}


void setup() 
{

	Serial.begin(115200); 
	Serial.println();
	Serial.println();
	Serial.println();
	Serial.println("Pushpak Quadrotor........");
	
	GPIO_OUTPUT(LED);
	GPIO_OUTPUT(GYRO_AUTO_ZERO);

	adc_initialize(); //Initialize adc at the last as this funtion enable interrupts.
	
	sensor_calibrate();
/*	
	Serial.print("Raw ADC count for 1.1V ref = ");
	Serial.println(adc_raw_ref_val);
	Serial.print("Raw ADC millivolts per count = ");
	Serial.println((float) (1100.0/(float)adc_raw_ref_val));
	
	Serial.print("Accumalated 12bit ADC count for 1.1V ref = ");
	Serial.println(adc_ref_val);
	Serial.print("Accumalated 12bit ADC millivolts per count = ");
	Serial.println((float) (1100.0/(float)adc_ref_val));
		
	Serial.print("Accelerometer calibration values, X, Y, Z: " );
	Serial.print(Acclmtr.m_zerog_x);
	Serial.print(", ");
	Serial.print(Acclmtr.m_zerog_y);
	Serial.print(", ");
	Serial.print(Acclmtr.m_zerog_z);
	Serial.println();

*/
/*	
	Serial.print("Gyro calibration values, X, Y, Z: " );
	Serial.print(Gyro.mX_zero);
	Serial.print(", ");
	Serial.print(Gyro.mY_zero);
	Serial.print(", ");
	Serial.print(Gyro.mZ_zero);
	Serial.println();
*/
	
	Serial.println();
	
	pkt.hdr1 = 's';
	pkt.hdr2 = 'n';
	pkt.hdr3 = 'p';
	pkt.pkt_type = 0xB7;
	pkt.length = 13;
	pkt.active = 0x3F;
	
	int x, y;
	
	x = -1;
	y = -1024;
	
	x = x >> 2;
	y = y >> 2;
	
	Serial.print("x = ");
	Serial.println(x);

	Serial.print("y = ");
	Serial.println(y);
	
	
	//Interrupts are enabled in the 
	//sei(); //enable interrupts

}

void loop() 
{
	uint8_t	i;
	uint16_t chk_sum = 0;
	uint8_t *ptr;
	
	int16_t x,y,z;


//	//use this for loop to reduce the data rate at which data is sent to pc.
 	for(i=0;i<1;++i)
 	{
 		adc_update();
		
		x = adc_get_value_mv(ACCL_X_CH);
		y = adc_get_value_mv(ACCL_Y_CH);
		z = adc_get_value_mv(ACCL_Z_CH);

		Acclmtr.process_ADC_sample(x,y,z);
 		Gyro.process_ADC_sample(gADC_output[3], gADC_output[4], gADC_output[5]);
 	}
 
 	GPIO_TOGGLE(LED); //LED is on Port D, Pin 4

 		
///////////////////////////////////////////////////////////////	
// 	//Send ADC data in ASCII format.
  	Serial.print(adc_get_sample_time());
  	
  	for(i=0;i<6;++i)
  	{
		Serial.print(',');
  		Serial.print((unsigned int) gADC_output[i]);
   	}
  	Serial.println();

///////////////////////////////////////////////////////////////	

///////////////////////////////////////////////////////////////	
// 	//Send Sensor data in ASCII format.
// 	Serial.print(adc_get_sample_time());
//	Serial.print(',');
 
/*
	Serial.print(gADC_output[0]);
	Serial.print(',');
	Serial.print(gADC_output[1]);
	Serial.print(',');
	Serial.print(gADC_output[2]);
	Serial.print(',');
*/ 

/* 
	Serial.print(adc_get_value_mv(ACCL_X_CH));
	Serial.print(',');
	Serial.print(adc_get_value_mv(ACCL_Y_CH));
	Serial.print(',');
	Serial.print(adc_get_value_mv(ACCL_Z_CH));
	Serial.print(',');
*/

//	x = Acclmtr.get_x();
//	y = Acclmtr.get_y();
//	z = Acclmtr.get_z();

//	Serial.print((int)x);
//	Serial.print(',');
//	Serial.print(y);
//	Serial.print(',');
//	Serial.print((int)z);
//	Serial.print(',');


//	Serial.print(Gyro.mX);
//	Serial.print(',');
//	Serial.print(Gyro.mY);
//	Serial.print(',');
//	Serial.print(Gyro.mZ);
//	Serial.print(',');
		
// 	Serial.println();

///////////////////////////////////////////////////////////////	








//	send_msg_sensor_values();

	
// 	//Send data in binary format.
// 	msg.length = sizeof(msg_type1) - 4;
// 	msg.sys_time = adc_get_sample_cnt();
// 	msg.acc_x = gADC_output[0];
// 	msg.acc_y = gADC_output[1];
// 	msg.acc_z = gADC_output[2];
// 	msg.gyro_x = gADC_output[3];
// 	msg.gyro_y = gADC_output[4];
// 	msg.gyro_z = gADC_output[5];
// 
// 	Serial.write((uint8_t*)&msg, sizeof(msg_type1));



///////////////////////////////////////////////////////////
// 	//Send message to robosavvy chr-6d GUI
//  	pkt.acc_x = gADC_output[0];
//  	pkt.acc_y = gADC_output[1];
//  	pkt.acc_z = gADC_output[2];
//  	pkt.gyro_x = gADC_output[3];
//  	pkt.gyro_y = gADC_output[4];
//  	pkt.gyro_z = gADC_output[5];
//  	
//  	
//  	ptr = (uint8_t*)&pkt;
//  	for(i=0;i< (sizeof(chr6d_msg) - 2);++i)
//  	{
//  		chk_sum += ptr[i];	
//  	}
//  	
//  	pkt.chk_sum = chk_sum;
//  	
//  	Serial.write((uint8_t*)&pkt, sizeof(chr6d_msg));
/////////////////////////////////////////////////////////////
}


int main(void)
{
	setup();
    
	for (;;)
	{
		loop();
	}
	    
	return 0;
}


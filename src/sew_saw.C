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
#include <math.h>
#include <util/delay.h>

#include "pushpak.h"
#include "util.h"
#include "adc.h"

#include "Print.h"
#include "HardwareSerial.h"
#include "Accelerometer.h"
#include "GyroSensor.h"
#include "servo_out.h"
#include "messages.h"
#include "packet.h"


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
	sei(); //enable interrupts

	Serial.begin(115200); 
	Serial.println();
	Serial.println();
	Serial.println();
	Serial.println("Pushpak Quadrotor........");
	
	GPIO_OUTPUT(LED);
	GPIO_OUTPUT(GYRO_AUTO_ZERO);

	adc_initialize(); //Initialize adc at the last as this funtion enable interrupts.

//Hardcoding the Zero g Values		
//	sensor_calibrate();
	Acclmtr.set_zerog_values((uint16_t)1658, (uint16_t)1658, (uint16_t)1658);	 	 	
	
	//Need to initialize the servo outputs to properly setup the ESC's
	initialize_servo_out();	
	
	update_servo_out(1,1);
	update_servo_out(2,1);
	update_servo_out(3,1);
	update_servo_out(4,1);

	GPIO_SET(LED);
	_delay_ms(1000); //Needed for ESC setup.
	
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
}


#define FF_VAL 35 		//Feed foward value

#define P_CONST  12.0
#define D_CONST  0.0
#define I_CONST  0.000


#define MAX_P	8
#define MAX_I	0.8
#define MAX_D	20



void loop() 
{
	int32_t x,y,z;
	double pitch, roll, yaw;
	int32_t temp;
	static uint8_t cnt;
	static uint8_t overflow = 0;
	
	float err;
	static float err_d1 = 0, err_d2 = 0; // _d1 = delayed by 1 or previous cycles sample.
	static float p_term, i_term, d_term;
	float corr; //correction
	static float corr_d1 = 0; 
	
	int16_t left_motor, right_motor;
	
 	GPIO_TOGGLE(LED); 	//LED is on Port D, Pin 4
	adc_update(); 		//blocking call
		
	x = adc_get_value_mv(ACCL_X_CH);
	y = adc_get_value_mv(ACCL_Y_CH);
	z = adc_get_value_mv(ACCL_Z_CH);

	Acclmtr.process_ADC_sample((int16_t)x,(int16_t)y,(int16_t)z);
	Gyro.process_ADC_sample(gADC_output[3], gADC_output[4], gADC_output[5]);

	x = Acclmtr.get_x();
	y = Acclmtr.get_y();
	z = Acclmtr.get_z();
  		
	temp = y*y + z*z;
	temp = sqrt(temp);
	pitch =  atan2(x,temp);
	
	temp = x*x + z*z;
	temp = sqrt(temp);
	roll =  atan2(y, temp);

	temp = x*x + y*y;
	temp = sqrt(temp);
	yaw =  atan2(z, temp);

	err = pitch; 

	p_term = P_CONST * err;
	p_term = (p_term > MAX_P) ? MAX_P : p_term;
	p_term = (p_term < -MAX_P) ? -MAX_P : p_term;


	i_term += (I_CONST * err);
	i_term = (i_term > MAX_I) ? MAX_I : i_term;
	i_term = (i_term < -MAX_I) ? -MAX_I : i_term;

	
	corr = p_term + i_term;
	
	//put a threshold
	corr = (corr > 100) ? 100 : corr;
	corr = (corr < -100) ? -100 : corr;
	
	left_motor = FF_VAL - (int16_t) corr;
	right_motor = FF_VAL + (int16_t) corr;

	//saftey limits
	left_motor = (left_motor < 0) ? 0 : left_motor;
	left_motor = (left_motor > 100) ? 100 : left_motor;

	right_motor = (right_motor < 0) ? 0 : right_motor;
	right_motor = (right_motor > 100) ? 100 : right_motor;

	if(gRun_flag == 1)
	{	
		update_servo_out(1,right_motor);
		update_servo_out(3,left_motor);
	}
	
	err_d1 = err;
	err_d2 = err_d1;
	corr_d1 = corr;
	
	overflow = adc_is_data_ready();
	
	uint8_t *ptr;
	do{
		ptr = parse_for_pkt();	//Parse the incomming messages in whatever time is remaning
		
		if(ptr != NULL)
		{
			process_msg(ptr);
		}
		
	}while(!adc_is_data_ready());
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


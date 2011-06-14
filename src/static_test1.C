/*
  This library is free software; you can redistribute it and/or modify it under the
  terms of the GNU Lesser General Public License as published by the Free Software
  Foundation; either version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful, but WITHOUT ANY
  WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A 
  PARTICULAR PURPOSE.  See the GNU Lesser General Public License for more details.
*/

// Copyright (c) 2009 Brijesh Sirpatil

/*


File: static_test1.C

Author: Sandeep Sirpatil

Description: This is a simple test program to do static test. You can control Thrust, roll, pitch and yaw of the quadrotor.  
*/


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
#include "servo_in.h"




void timer1_init()
{

	//Timer 1 is used by or for decoding RC reciever pulses

	//With a clock prescale of 8 we get
	//Minimum time resolution of = 400ns(2.5MHz)
	//Maxiumum time period = 26.2144ms(38.14Hz)
	
	//With a clock prescale of 64 we get
	//Minimum interrupt time period = 3.2us

	TCCR1A = 0;
	TCCR1B = 0;

	// Prescalar 64
	SET_BIT(TCCR1B, CS11);
	SET_BIT(TCCR1B, CS10); 

	//Normal non PWM mode
 	CLR_BIT(TCCR1B, WGM13); 
 	CLR_BIT(TCCR1B, WGM12); 
 	CLR_BIT(TCCR1A, WGM11); 
 	CLR_BIT(TCCR1A, WGM10); 

	//Enable interrupt on timer overflow
 	SET_BIT(TIMSK1, TOIE1); 

}


ISR(TIMER1_OVF_vect)
{
	GPIO_TOGGLE(LED); //LED is on Port D, Pin 4		
}


void setup() 
{

	Serial.begin(115200); 
	Serial.println();
	Serial.println();
	Serial.println();
	Serial.println("Pushpak Quadrotor........");
	
	GPIO_OUTPUT(LED);
	
	initialize_servo_out();	
	initialize_servo_in();
	
	update_servo_out(1,1);
	update_servo_out(2,1);
	update_servo_out(3,1);
	update_servo_out(4,1);

	GPIO_SET(LED);
	_delay_ms(1000);

	timer1_init();
	
	//Interrupts are enabled in the 
	sei(); //enable interrupts

}


// The min and max range of motor thrust that is mapped to the 0-255 range of thrust input command from the servo in.
#define THRUST_MIN 40
#define THRUST_MAX 90

// The max amount of change in thrust that is mapped to the full range of input command. 
#define YAW_MAX    10
#define ROLL_MAX   10
#define PITCH_MAX  10



int8_t yawCmdRaw, rollCmdRaw, pitchCmdRaw; // Raw commands from the servo in. Range  is -127 to +127
uint8_t thrustCmdRaw; // Raw thrust command from servo in. range is 0-255

int16_t yawCmdScaled, rollCmdScaled, pitchCmdScaled; // Scaled to the XXX_MAX range 
uint16_t north_M_thrust, south_M_thrust, east_M_thrust, west_M_thrust; // Calculated motor thrust

uint8_t cnt;

void loop() 
{
	
 	GPIO_TOGGLE(LED); //LED is on Port D, Pin 4

	update_servo_in(); // Read the 4 receiver channels .
	
	yawCmdRaw   = gCh4ServoIn - 127;
	rollCmdRaw  = gCh1ServoIn - 127;
	pitchCmdRaw = gCh2ServoIn - 127;
	thrustCmdRaw  = gCh3ServoIn;  
	
	// Scale the commands input  range to the Thrust MAX range.
	yawCmdScaled   = ((int16_t) yawCmdRaw * YAW_MAX) / 128;
	rollCmdScaled  = ((int16_t) rollCmdRaw * ROLL_MAX) / 128;
	pitchCmdScaled = ((int16_t) pitchCmdRaw * PITCH_MAX) / 128;
	
	// Kind of Collective Thrust
	north_M_thrust = south_M_thrust = east_M_thrust = west_M_thrust = THRUST_MIN + ( ((int16_t) thrustCmdRaw * (THRUST_MAX - THRUST_MIN))/ 255 ) ;
	
	// Pitch command
	north_M_thrust -= pitchCmdScaled;
	south_M_thrust += pitchCmdScaled;
	
	// roll command
	east_M_thrust -= rollCmdScaled;
	west_M_thrust += rollCmdScaled;
	
	// yaw command
	north_M_thrust -= yawCmdScaled;
	south_M_thrust -= yawCmdScaled;
	east_M_thrust += yawCmdScaled;
	west_M_thrust += yawCmdScaled;
	
	update_servo_out(1,south_M_thrust);
	update_servo_out(2,west_M_thrust);
	update_servo_out(3,north_M_thrust);
	update_servo_out(4,east_M_thrust);

	cnt++;
	if(cnt%10 == 0)
	{
		
		Serial.print(int16_t(rollCmdRaw));
		Serial.print(",");
		
		Serial.print(int16_t(pitchCmdRaw));
		Serial.print(",");
		Serial.print(int16_t(thrustCmdRaw));
		Serial.print(",");
		
		Serial.print(int16_t(yawCmdRaw));
		Serial.print(",");

		Serial.print(int16_t(rollCmdScaled));
		Serial.print(",");
		
		Serial.print(int16_t(pitchCmdScaled));
		Serial.print(",");
		Serial.print(int16_t(yawCmdScaled));
		Serial.print("      *");

		Serial.print(int16_t(north_M_thrust));
		Serial.print(",");

		Serial.print(int16_t(south_M_thrust));
		Serial.print(",");
		
		Serial.print(int16_t(east_M_thrust));
		Serial.print(",");
		Serial.print(int16_t(west_M_thrust));
		Serial.print(" *              ");
		Serial.print("\r");
	}
	
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


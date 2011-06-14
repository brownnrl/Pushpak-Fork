/*
  Copyright (c) 2009 Brijesh Sirpatil
  
  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General
  Public License along with this library; if not, write to the
  Free Software Foundation, Inc., 59 Temple Place, Suite 330,
  Boston, MA  02111-1307  USA

*/
#include <util/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>

#include "pushpak.h"
#include "util.h"
#include "servo_in.h"
#include "servo_out.h"
#include "adc.h"

#include "Print.h"
#include "HardwareSerial.h"


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
	Serial.println("Pushpak Quadrotor RC Radio Setup........");
	
	GPIO_OUTPUT(LED);
	GPIO_CLEAR(LED);
	initialize_servo_out();
	initialize_servo_in();

	timer1_init();
	sei(); //enable interrupts

}


void loop() 
{
	uint8_t status;
	

	status = update_servo_in();

	if((status & 0x4) != 0)
	{

		Serial.print((unsigned int) status, HEX);
		Serial.print(" ");
		
		Serial.print((unsigned int)gCh1ServoIn);
		Serial.print(" ");
		Serial.print((unsigned int)gCh2ServoIn);
		Serial.print(" ");
		Serial.print((unsigned int)gCh3ServoIn);
		Serial.print(" ");
		Serial.println((unsigned int)gCh4ServoIn);

	}


	if(gCh1ServoIn < 35)
	{
		gCh1ServoIn = 0;
	}	

	if(gCh2ServoIn < 35)
	{
		gCh2ServoIn = 0;
	}	

	
	if(gCh3ServoIn < 35)
	{
		gCh3ServoIn = 0;
	}	

	if(gCh4ServoIn < 35)
	{
		gCh4ServoIn = 0;
	}	
	

	update_servo_out(1,gCh3ServoIn);
	update_servo_out(2,gCh3ServoIn);
	update_servo_out(3,gCh3ServoIn);
	update_servo_out(4,gCh3ServoIn);		
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


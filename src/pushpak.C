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

//////////////////////////////////////////////////////////////////
// Global variables
volatile uint8_t gSystemError = 0;
volatile uint8_t gSystemTimerTick = 0;
volatile uint8_t gSystemTime = 0;
//////////////////////////////////////////////////////////////////


////////
// Things to be FIXED:
// 1) Giving direct access to gSystemTime is not a good idea. Also currently if it is 16bit value then
//		care has to be taken to access the variable atomically.
// 2) Should the name of variable gSystemTimerTick be changed to gSystemLoopTick, also gSystemTime to gLoopCount??

//////////////////////////////////



void timer1_init()
{
//	uint16_t count;

	//Timer 1 is used by or for:
	//	a) Control Loop Timing and interrupt
	//	b) Decoding RC reciever pulses

	//With a clock prescale of 8 we get
	//Minimum interrupt time period = 400ns(2.5MHz)
	//Maxiumum interrupt time period = 26.2144ms(38.14Hz)
	
	//With a clock prescale of 64 we get
	//Minimum interrupt time period = 3.2us
	

	//Do not use loop rate greater than 500Hz as that will mess up the RC decoding logic.
	//Safe operating loop rate in terms of not conflicting with RC reciever decoding is from
	//lower than 300Hz.

	TCCR1A = 0;
	TCCR1B = 0;

	// Prescalar 64
	SET_BIT(TCCR1B, CS11);
	SET_BIT(TCCR1B, CS10); 

// 	//CTC (Clear Timer on Compare Match)
// 	CLR_BIT(TCCR1B, WGM13); 
// 	SET_BIT(TCCR1B, WGM12); 
// 	CLR_BIT(TCCR1A, WGM11); 
// 	CLR_BIT(TCCR1A, WGM10); 

	//Normal non PWM mode
 	CLR_BIT(TCCR1B, WGM13); 
 	CLR_BIT(TCCR1B, WGM12); 
 	CLR_BIT(TCCR1A, WGM11); 
 	CLR_BIT(TCCR1A, WGM10); 


// 	//Calculate count value to generate given interrupt rate.
// 	//8 is clock prescale used for Timer 1
// 	count = (F_CPU/(CONTROL_LOOP_RATE*8));
// 	OCR1A = count;

// 	TIMSK1 = 0;
// 	//Enable interrupt on Compare Match A
// 	SET_BIT(TIMSK1, OCIE1A); 
 	SET_BIT(TIMSK1, TOIE1); 

}

//Main System timer interrupt. Used for precise control loop timing.
//The main control loop should wait on gSystemTimerTick to be set.
ISR(TIMER1_COMPA_vect){

	gSystemTimerTick = 1;
	++gSystemTime;
}

ISR(TIMER1_OVF_vect){
	GPIO_TOGGLE(LED); //LED is on Port D, Pin 4		
}


void setup() 
{
	//Initialize all the global variables 
	gSystemError = 0;
	gSystemTimerTick = 0;
	gSystemTime = 0;

	Serial.begin(115200); 
	Serial.println("Pushpak Quadrotor........");
	
	GPIO_OUTPUT(LED);
	GPIO_CLEAR(LED);
	initialize_servo_out();
	initialize_servo_in();

	initialize_adc(); //Initialize adc at the last as this funtion enable interrupts.
	timer1_init();
	sei(); //enable interrupts
//	cli(); //Disable interrupts before copying the current RC reciever pulse values


}

extern volatile uint16_t ch1Count, ch2Count, ch3Count, ch4Count; 	// Pulse width.
extern volatile uint16_t minTime;
extern volatile uint16_t maxTime, currTime;
extern volatile uint16_t currCount;

extern uint16_t ch1RisingCount, ch2RisingCount, ch3RisingCount, ch4RisingCount; // Timer count when rising edge occured


void loop() 
{
	uint8_t status;
	uint16_t temp1, temp2, temp3, temp4, temp5;
	
	//Wait for System Timer Tick
	//Could also use the ADC flag to lock the sampling rate to control loop rate.
//	while(gSystemTimerTick == 0);

//	gSystemTimerTick = 0; //clear the flag

//	if((gSystemTime % 4)  == 0)
	{//this will toggle led every 4th iteration
//		GPIO_TOGGLE(LED); //LED is on Port D, Pin 4		
	}
	
/********************************************************************************/
// User code to control the Quadrotor goes here in between these comment lines.
	update_adc_samples();
	status = update_servo_in();

	if((status & 0x4) != 0)
	{//this will toggle led every 4th iteration
		//	Serial.println((int)gCh3ServoIn);

		Serial.print((unsigned int) status, HEX);
		Serial.print(" ");
		Serial.print((unsigned int)ch3Count);
		Serial.print(" ");
		
//		Serial.print((unsigned int)gCh1ServoIn);
//		Serial.print(" ");
//		Serial.print((unsigned int)gCh2ServoIn);
//		Serial.print(" ");
		Serial.print((unsigned int)gCh3ServoIn);
		Serial.print(" ");
//		Serial.println((unsigned int)gCh4ServoIn);

 		ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
		{
// 			temp1 = currCount;
 			temp2 = ch3RisingCount;
// 			temp3 = currTime;
 //			temp4 = minTime;
 //			temp5 = maxTime;
 		}
// 		Serial.print(" ");
// 		Serial.print((unsigned int)temp1); //
// 		Serial.print(" ");
 		Serial.println((unsigned int)temp2);
// 		Serial.print(" ");
// 		Serial.print((unsigned int)temp3);
// 		Serial.print(" ");
// 		Serial.print((unsigned int)temp4);
// 		Serial.print(" ");
// 		Serial.println((unsigned int)temp5);
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



	//End of User code section.
/********************************************************************************/	

	//Status and Error checking code
	
	//Every 30-40ms check if new RC reciever pulse was decoded. If no new pulse was decoded then
	//possible loss of RC radio link.
	//Mask the last 2 bits and check if they are zero. Probably faster than modulo by 4 operation.
// 	if((gSystemTime % 4) == 0)
// 	{
// 		//Check if bottom 4 bits corresponding to 4 channels of RC reciever are set
// 		if((gServoInStatus & 0xF) != 0xF)
// 		{//At least one of the receivers channels is malfunctioning
// 			SET_BIT(gSystemError,RC_LINK_ERROR);	
// 		}
// 	}
// 	
// 	//This should be last line in the control loop code.
// 	//Check if the current iteration of the code has take more time than control loop time period.	
// 	if(gSystemTimerTick == 1)
// 	{
// 		SET_BIT(gSystemError,EXECUTION_TIME_OVERFLOW);	
// 	}
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


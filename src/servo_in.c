/*
  servo_in.c
  
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

#include <inttypes.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#include "pushpak.h"
#include "util.h"
#include "servo_in.h"


//!
//! RC recevier pulse decoding.
//! For decoding RC recevier pulses, the pins are configured to generate an interrupt whenever there is a change
//! logic level. Timer 1 is used as time base to measure the pulse widths. 
//!
//! Hardware Resources used : Timer 1 counter
//!
//! Return value : 16bit Timer 1 counter value directly proportional to pulse width.
//!


//! Global variables used within this file. They should not be visible outside of this file.
uint16_t ch1RisingCount, ch2RisingCount, ch3RisingCount, ch4RisingCount; // Timer count when rising edge occured
volatile uint16_t ch1Count, ch2Count, ch3Count, ch4Count; 	// Pulse width.

//! Exported Global variables 
uint8_t gCh1ServoIn, gCh2ServoIn, gCh3ServoIn, gCh4ServoIn;
volatile uint8_t gServoInStatus = 0; //! If a new pulse is decoded on a channel the corresponding bit is set.

// volatile uint16_t minTime = 65000;
// volatile uint16_t maxTime = 0;
// volatile uint16_t prevTime = 0;
// volatile uint16_t currTime = 0;
// volatile uint16_t currCount = 0;
// uint8_t flag = 0;

void initialize_servo_in()
{
	//Does the compiler intialize global variables to 0?
	gCh1ServoIn = 0;
	gCh2ServoIn = 0;
	gCh3ServoIn = 0;
	gCh4ServoIn = 0;
	
	ch1Count = 0;
	ch2Count = 0;
	ch3Count = 0;
	ch4Count = 0;
	
	ch1RisingCount = 0;
	ch2RisingCount = 0;
	ch3RisingCount = 0;
	ch4RisingCount = 0;
		
	gServoInStatus = 0;
	
	//Set the pins as input pins
	GPIO_INPUT(RC_CH1);
	GPIO_INPUT(RC_CH2);
	GPIO_INPUT(RC_CH3);
	GPIO_INPUT(RC_CH4);
	
	//Configure the appropriate interrupts
	
	//RC channel 1
	//Configure INT2 to trigger on both rising and falling edges
	SET_BIT(EICRA, ISC20);
	CLR_BIT(EICRA, ISC21);
	SET_BIT(EIMSK, INT2);	//enable External Interrupt 2 (INT2)
	
	SET_BIT(PCMSK1, 0);	//Enable Pin change Interrupt on Port B pin 0, for RC channel 2
	SET_BIT(PCMSK3, 5);	//Enable Pin change Interrupt on Port D pin 5, for RC channel 3
	SET_BIT(PCMSK2, 7);	//Enable Pin change Interrupt on Port C pin 7, for RC channel 4
	
	SET_BIT(PCICR, PCIE1);	//Enable interrupt for RC channel 2
	SET_BIT(PCICR, PCIE2);	//Enable interrupt for RC channel 4
	SET_BIT(PCICR, PCIE3);	//Enable interrupt for RC channel 3
	
}

//!
//! Updates the global variable with the latest recieved RC reciever values.
//! 
//!
//!	Returns a byte whose bits indicate a which channels have a new decoded servo pulse since the last time
//! this function was called.
//!

uint8_t update_servo_in()
{
	uint8_t oldSREG;
	uint8_t status;
	
	uint16_t count[4];
	
	oldSREG = SREG;
	cli(); //Disable interrupts before copying the current RC reciever pulse values
	
	status = gServoInStatus;
	gServoInStatus = 0;
	
	count[0] = ch1Count;
	count[1] = ch2Count;
	count[2] = ch3Count;
	count[3] = ch4Count;
		
	SREG = oldSREG;	//restore interupt status


	//Convert the raw timer count into a value ranging from 0 to 255 indicating servo pulse min to max.

	if(status & _BV(0))
	{//check if new pulse for channel 1 was decoded.
	
		if(count[0] > MIN_SERVO_IN_COUNT && count[0] < MAX_SERVO_IN_COUNT)
		{
			// Re-implementing the following function so that it is optimized for 8bit values and constant vaules are
			// calculated at compile time.
			
			// long map(long x, long in_min, long in_max, long out_min, long out_max)
			// {
			//   return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
			// }
		
			//Convert the range from MIN_SERVO_IN_COUNT tp MAX_SERVO_IN_COUNT to 0 to 255.
			gCh1ServoIn = 0 + ((count[0] - MIN_SERVO_IN_COUNT)*(255.0-0)/(MAX_SERVO_IN_COUNT-MIN_SERVO_IN_COUNT));
	
		} 	
		else
		{
			status |= _BV(4);	//set error bit.
		}
	}	
	
	if(status & _BV(1))
	{	
		if(count[1] > MIN_SERVO_IN_COUNT && count[1] < MAX_SERVO_IN_COUNT)
		{
			//Convert the range from MIN_SERVO_IN_COUNT to MAX_SERVO_IN_COUNT to 0 to 255.
			gCh2ServoIn = 0 + ((count[1] - MIN_SERVO_IN_COUNT)*(255.0-0)/(MAX_SERVO_IN_COUNT-MIN_SERVO_IN_COUNT));
		} 	
		else
		{
			status |= _BV(5);	//set error bit.
		}
	}	

	if(status & _BV(2))
	{	
		if((count[2] >= MIN_SERVO_IN_COUNT) && (count[2] <= MAX_SERVO_IN_COUNT))
		{
			//Convert the range from MIN_SERVO_IN_COUNT tp MAX_SERVO_IN_COUNT to 0 to 255.
			gCh3ServoIn = 0 + ((count[2] - MIN_SERVO_IN_COUNT)*(255.0-0)/(MAX_SERVO_IN_COUNT-MIN_SERVO_IN_COUNT));
		} 	
		else
		{
			status |= _BV(6);	//set error bit.
		}
	}	
	
	if(status & _BV(3))
	{	
		if(count[3] > MIN_SERVO_IN_COUNT && count[3] < MAX_SERVO_IN_COUNT)
		{
			//Convert the range from MIN_SERVO_IN_COUNT tp MAX_SERVO_IN_COUNT to 0 to 255.
			gCh4ServoIn = 0 + ((count[3] - MIN_SERVO_IN_COUNT)*(255.0-0)/(MAX_SERVO_IN_COUNT-MIN_SERVO_IN_COUNT));
		} 	
		else
		{
			status |= _BV(7);	//set error bit.
		}
	}	
		
	return status;
}

// ***************** Interrupts *************************

ISR(INT2_vect)
{
	uint16_t fallingCount;
	
	if(GPIO_READ(RC_CH1)==1){ // pin High
		ch1RisingCount = TCNT1;
	}else{
		fallingCount = TCNT1;
		ch1Count = fallingCount - ch1RisingCount;
		
		SET_BIT(gServoInStatus,0); //New pulse is decoded set the corresponding bit in the status
	}

}

ISR(PCINT1_vect)
{
	uint16_t fallingCount;
	
	if(GPIO_READ(RC_CH2)==1){ // pin High
		ch2RisingCount = TCNT1;
	}else{
		fallingCount = TCNT1;
		ch2Count = fallingCount - ch2RisingCount;
		SET_BIT(gServoInStatus,1); //New pulse is decoded set the corresponding bit in the status
	}

}

ISR(PCINT2_vect)
{
	uint16_t fallingCount;

	if(GPIO_READ(RC_CH4)==1){ // pin High
		ch4RisingCount = TCNT1;
	}else{
		fallingCount = TCNT1;
		ch4Count = fallingCount - ch4RisingCount;
		SET_BIT(gServoInStatus,3); //New pulse is decoded set the corresponding bit in the status
	}

}

ISR(PCINT3_vect)
{
	uint16_t fallingCount;

	if(GPIO_READ(RC_CH3)==1){ // pin High
		ch3RisingCount = TCNT1;
	}
	else
	{
		fallingCount = TCNT1;
		ch3Count = fallingCount - ch3RisingCount;
		SET_BIT(gServoInStatus,2); //New pulse is decoded set the corresponding bit in the status
	}

}

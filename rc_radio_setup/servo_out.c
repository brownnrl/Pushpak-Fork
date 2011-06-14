/*
  servo_out.c
  
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

#include "pushpak.h"
#include "util.h"
#include "servo_out.h"

//!
//! Generate RC Servo pulses on four hardware PWM pins.
//!
//!
//! Hardware Resources used : 	Timer 0 counter
//!								Timer 1 counter
//!								Hardware PWM pins
//!

void initialize_servo_out(void)
{
	
	//Set Timer 0 in Fast PWM mode, clock prescale of 256.
	//Gives a frequency of 305.18 and period 3.27ms; 78 steps in 1ms time period

	//Fast PWM, clear on match	
	SET_BIT(TCCR0A, COM0A1);
	CLR_BIT(TCCR0A, COM0A0);
	SET_BIT(TCCR0A, COM0B1);
	CLR_BIT(TCCR0A, COM0B0);
	
	//Fast PWM
	CLR_BIT(TCCR0B, WGM02);
	SET_BIT(TCCR0A, WGM01);
	SET_BIT(TCCR0A, WGM00);
	
	//256 Clock prescale factor
	SET_BIT(TCCR0B, CS02);
	CLR_BIT(TCCR0B, CS01);
	CLR_BIT(TCCR0B, CS00);
	
	//Set Timer 2 in Fast PWM mode, clock prescale of 256.
	//Gives a frequency of 305.18 and period 3.27ms; 78 steps in 1ms time period

	//Fast PWM, clear on match	
	SET_BIT(TCCR2A, COM2A1);
	CLR_BIT(TCCR2A, COM2A0);
	SET_BIT(TCCR2A, COM2B1);
	CLR_BIT(TCCR2A, COM2B0);
	
	//Fast PWM
	CLR_BIT(TCCR2B, WGM22);
	SET_BIT(TCCR2A, WGM21);
	SET_BIT(TCCR2A, WGM20);

	//256 Clock prescale factor
	//Note that register setup is different from Timer 0
	SET_BIT(TCCR2B, CS22);
	SET_BIT(TCCR2B, CS21);
	CLR_BIT(TCCR2B, CS20);

	//Set the pins as output pins
	GPIO_OUTPUT(SERVO_CH1);
	GPIO_OUTPUT(SERVO_CH2);
	GPIO_OUTPUT(SERVO_CH3);
	GPIO_OUTPUT(SERVO_CH4);

		
}

//!
//! Update the servo output pulse with a new value.
//! Parameters : ch = channel id. Supports four channels 1 to 4.
//!				 val = New value of the servo output pulse. val = 0 servo output is minimum(1ms long pulse)
//!						val of 255 servro output is maximum(2ms long pulse)
//!

void update_servo_out(uint8_t ch, uint8_t val)
{
	uint8_t count;

// Re-implementing the following function so that it is optimized for 8bit values.
// long map(long x, long in_min, long in_max, long out_min, long out_max)
// {
//   return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
// }
		
	//Convert the range from 0 to 255 to 1ms to 2ms needed for the servo pulse
	count = MIN_SERVO_OUT_COUNT + ((val - 0)*(MAX_SERVO_OUT_COUNT-MIN_SERVO_OUT_COUNT)/(255-0));
	
	switch(ch)
	{
		case 1 : 
			OCR0A = count;
			break;
		case 2 : 
			OCR0B = count;
			break;
		case 3 : 
			OCR2A = count;
			break;
		case 4 : 
			OCR2B = count;
			break;
		default:
			break;	
	}

} 

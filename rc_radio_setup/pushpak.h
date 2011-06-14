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
#ifndef _PUSHPAK_H
#define _PUSHPAK_H

//Needed to make it compatible with C++ compilers and Arduino environment.
#ifdef __cplusplus
extern "C"{
#endif


//#define F_CPU 20000000

#define CONTROL_LOOP_RATE 100	//! \def Control Loop Rate

//!
//!	System Error codes bit position
//!
#define EXECUTION_TIME_OVERFLOW	0	//! \def Control loop took longer to execute than loop time period.
#define RC_LINK_ERROR			1	//! \def RC Reciever inputs faulty or loss of signal.


///
/// RC Reciever decoding input pins. Define the port and the pin which are being used to
/// decode the RC reciver pulses.
/// CAUTION: the corresponding interrupts are harcoded, if any pin is changed here 
/// the interrupt code should also be modified.
///
#define RC_CH1  B, 2	//! \def Reciever Channel 1 on Port B, Pin 2 using INT2 
#define RC_CH2  B, 0	//! \def Reciever Channel 2 on Port B, Pin 0 using PCINT1
#define RC_CH3  D, 5	//! \def Reciever Channel 3 on Port D, Pin 5 using PCINT3
#define RC_CH4  C, 7	//! \def Reciever Channel 4 on Port C, Pin 7 using PCINT2

///
/// RC Servo pulse output pins.
/// CAUTION: The code uses hardware PWM feature, so these pins cannot be changed to any other IO pin.
//!
#define SERVO_CH1  B, 3	//! \def Servo Channel 1 on Port B, Pin 3 using Timer 0 PWM A output 
#define SERVO_CH2  B, 4	//! \def Servo Channel 2 on Port B, Pin 4 using Timer 0 PWM B output 
#define SERVO_CH3  D, 7	//! \def Servo Channel 3 on Port D, Pin 5 using Timer 2 PWM A output 
#define SERVO_CH4  D, 6	//! \def Servo Channel 4 on Port C, Pin 7 using Timer 2 PWM B output 


//!
//! ADC channels
//!
#define ACCL_X	0	//! \def Accelerometer X axis
#define ACCL_Y	1	//! \def Accelerometer Y axis
#define ACCL_Z	2	//! \def Accelerometer Z axis

#define GYRO_X	3	//! \def Gyro X axis
#define GYRO_Y	4	//! \def Gyro Y axis
#define GYRO_Z	5	//! \def Gyro Z axis

#define SONAR	6	//! \def Sonar sensor
#define BATTERY	7	//! \def Battery voltage monitoring

#define LED	D, 4			//! \def LED is on Port D, pin 4.
#define GYRO_AUTO_ZERO B, 1	//! \def Auto zero pin of IDG500 gyro. (X & Y axis)


typedef uint8_t boolean;
typedef uint8_t byte;

//sbi and cbi has been deprecated, but is is used in the hardwareSeria.C file which has been taken from
//Arduino frame work.

#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

#ifdef __cplusplus
} // extern "C"
#endif

#endif // _PUSHPAK_H

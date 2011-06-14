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
#ifndef _SERVO_IN_H
#define _SERVO_IN_H

//Needed to make it compatible with C++ compilers and Arduino environment.
#ifdef __cplusplus
extern "C"{
#endif


#define MIN_SERVO_IN_COUNT	312	//Count value for minimum Servo pulse(1ms pulse)
#define MAX_SERVO_IN_COUNT	625 //Count value for maximum Servo pulse(2ms pulse)

//!
//! Global variables that contains current RC Reciever input pulse width values. These variables
//! are upated when update_servo_in() is called.
//!
extern uint8_t gCh1ServoIn, gCh2ServoIn, gCh3ServoIn, gCh4ServoIn ;
extern volatile uint8_t gServoInStatus; //! If a new pulse is decoded on a channel the corresponding bit is set.


void initialize_servo_in();
uint8_t update_servo_in();


#ifdef __cplusplus
} // extern "C"
#endif

#endif // _SERVO_IN_H

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
#ifndef _SERVO_OUT_H
#define _SERVO_OUT_H

//Needed to make it compatible with C++ compilers and Arduino environment.
#ifdef __cplusplus
extern "C"{
#endif

//! Currently Timer 0 and Timer 1 are setup with clock prescale factor of 256.
//! Time period per count of timer = 1/(F_CPU/256) = 256/F_CPU
//! Minimum servo pulse is 1ms long = 1e-3/(256/F_CPU) = 78.125 counts
//! Maximum servo pulse is 2ms long = 2e-3/(256/F_CPU) = 156.25 counts

#define MIN_SERVO_OUT_COUNT	78	//Count to generate minimum Servo pulse(1ms pulse)
#define MAX_SERVO_OUT_COUNT	156 //Count to generate maximum Servo pulse(2ms pulse)


void initialize_servo_out();
void update_servo_out(uint8_t ch, uint8_t val);

#ifdef __cplusplus
} // extern "C"
#endif

#endif // _SERVO_OUT_H

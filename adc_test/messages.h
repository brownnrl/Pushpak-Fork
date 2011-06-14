/*
  This library is free software; you can redistribute it and/or modify it under the
  terms of the GNU Lesser General Public License as published by the Free Software
  Foundation; either version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful, but WITHOUT ANY
  WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A 
  PARTICULAR PURPOSE.  See the GNU Lesser General Public License for more details.
*/

// Copyright (c) 2009 Brijesh Sirpatil

#ifndef messages_h
#define messages_h

#include <inttypes.h>


void send_msg_ADC12b_samples(void);
void send_msg_sensor_values(void);


//message.C is compiled as C++ file so extern "C" is not required.
#ifdef __cplusplus
extern "C"{
#endif





#ifdef __cplusplus
} // extern "C"
#endif

#endif	//messages_h

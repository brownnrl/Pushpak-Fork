/*
  This library is free software; you can redistribute it and/or modify it under the
  terms of the GNU Lesser General Public License as published by the Free Software
  Foundation; either version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful, but WITHOUT ANY
  WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A 
  PARTICULAR PURPOSE.  See the GNU Lesser General Public License for more details.
*/

// Copyright (c) 2010 Brijesh Sirpatil

#ifndef messages_h
#define messages_h

#include <inttypes.h>


extern uint8_t gRun_flag;	//index[1]
extern uint8_t gRun_mode;	//index[2]


void send_msg_ADC12b_samples(void);
void send_msg_sensor_values(void);


void process_msg(uint8_t *buffer);


//message.C is compiled as C++ file so extern "C" is not required.
#ifdef __cplusplus
extern "C"{
#endif

#ifdef __cplusplus
} // extern "C"
#endif

#endif	//messages_h

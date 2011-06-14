/*
  This library is free software; you can redistribute it and/or modify it under the
  terms of the GNU Lesser General Public License as published by the Free Software
  Foundation; either version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful, but WITHOUT ANY
  WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A 
  PARTICULAR PURPOSE.  See the GNU Lesser General Public License for more details.
*/

// Copyright (c) 2009 Brijesh Sirpatil

#ifndef GyroSensor_h
#define GyroSensor_h

#include <inttypes.h>


class GyroSensor
{
	
   public:
	int16_t mX;
	int16_t mY;
	int16_t mZ;  		
	
	int16_t mX_zero;
	int16_t mY_zero;
	int16_t mZ_zero;
	
 
  public:
  	GyroSensor();
  	
  	void set_zero_values(uint16_t x, uint16_t y, uint16_t z);
	void process_ADC_sample(uint16_t x, uint16_t y, uint16_t z);  

};

                      
extern GyroSensor Gyro; //preinstantiated Gyro object.

#endif	//GyroSensor_h

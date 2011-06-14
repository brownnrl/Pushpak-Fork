/*
  This library is free software; you can redistribute it and/or modify it under the
  terms of the GNU Lesser General Public License as published by the Free Software
  Foundation; either version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful, but WITHOUT ANY
  WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A 
  PARTICULAR PURPOSE.  See the GNU Lesser General Public License for more details.
*/

// Copyright (c) 2009 Brijesh Sirpatil

#include "GyroSensor.h"

GyroSensor::GyroSensor()
{
	//Store precomputed Sensor reading. This values must be updated during sensor calibrartion.
	mX_zero = 1234;
	mY_zero = 1234;
	mZ_zero = 1234;
	
	mX = 0;
	mY = 0;
	mZ = 0;
}

void GyroSensor::set_zero_values(uint16_t x, uint16_t y, uint16_t z)
{
	mX_zero = x;
	mY_zero = y;
	mZ_zero = z;
}

void GyroSensor::process_ADC_sample(uint16_t a, uint16_t b, uint16_t c)
{
	mX = a - mX_zero;
	mY = b - mY_zero;
	mZ = c - mZ_zero;
}  


// Preinstantiate Objects //////////////////////////////////////////////////////

GyroSensor Gyro;

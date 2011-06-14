/*
  This library is free software; you can redistribute it and/or modify it under the
  terms of the GNU Lesser General Public License as published by the Free Software
  Foundation; either version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful, but WITHOUT ANY
  WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A 
  PARTICULAR PURPOSE.  See the GNU Lesser General Public License for more details.
*/

// Copyright (c) 2009 Brijesh Sirpatil

#include "Accelerometer.h"

Accelerometer::Accelerometer()
{
	//Store precomputed Sensor reading. This values must be updated during sensor calibrartion.
	m_zerog_x = 1234;
	m_zerog_y = 1234;
	m_zerog_z = 1234;

	m_x = 0;
	m_y = 0;
	m_z = 0;
}


/// Sets ADC values corresponding to stable stationary zero g value. Note that it uses raw ADC values and not 
/// voltage levels. 
void Accelerometer::set_zerog_values(uint16_t x, uint16_t y, uint16_t z)
{
	m_zerog_x = x;
	m_zerog_y = y;
	m_zerog_z = z;
}

void Accelerometer::process_ADC_sample(uint16_t adc_x, uint16_t adc_y, uint16_t adc_z)
{
	int32_t xx, yy, zz; //stores intemediate vaules in mV centered around zero


	xx = (int32_t)m_zerog_x - adc_x; //inverting x axis to match orientation format
	yy = adc_y - (int32_t)m_zerog_y;
	zz = adc_z - (int32_t)m_zerog_z;
	
	//Convert from milli Volts to milli g units.
	//divide value by 1024, since the conversion factor was multiplied by 1024 to reduce round off errors.
	xx = (xx * m_mV2mg) >> 10;
	yy = (yy * m_mV2mg) >> 10;
	zz = (zz * m_mV2mg) >> 10;

	m_x = (int16_t) xx; //to match the orientation such clockwise rotation is +ve
	m_y = (int16_t) yy;
	m_z = (int16_t) zz;

}  

int16_t	Accelerometer::get_x()
{
	return m_x;
}

int16_t	Accelerometer::get_y()
{
	return m_y;
}

int16_t	Accelerometer::get_z()
{
	return m_z;
}

// Preinstantiate Objects //////////////////////////////////////////////////////
Accelerometer Acclmtr;

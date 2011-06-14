/*
  This library is free software; you can redistribute it and/or modify it under the
  terms of the GNU Lesser General Public License as published by the Free Software
  Foundation; either version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful, but WITHOUT ANY
  WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A 
  PARTICULAR PURPOSE.  See the GNU Lesser General Public License for more details.
*/

// Copyright (c) 2009 Brijesh Sirpatil

#ifndef Accelerometer_h
#define Accelerometer_h

#include <inttypes.h>


class Accelerometer
{
public:
	//Needs to be static so that it can be initialized.

	//The sensitivity of MMA7260QT in 1.5g is 800mV/g.
	//Which when converted to mg/mV comes out to 1.25 mg/mV. If integer representation is used
	//then it becomes 1mV/mg which gives a large round off error.
	//To minimize round of error multiply the conversion factor by 1024 (power of 2) and divide the
	//final result by 1024.
	static const int16_t m_mV2mg = (uint16_t) ((1000.0/800.0)*1024); //conversion factor, from milli Volts to milli g
 
	int16_t m_x;
	int16_t m_y;
	int16_t m_z;

	int16_t m_zerog_x;
	int16_t m_zerog_y;
	int16_t m_zerog_z;

  	Accelerometer();
  	
  	void set_zerog_values(uint16_t x, uint16_t y, uint16_t z);
	void process_ADC_sample(uint16_t x, uint16_t y, uint16_t z); 

	///Returns measured g values in milli g units.
	int16_t	get_x();
	int16_t	get_y();
	int16_t	get_z();

};

                      
extern Accelerometer Acclmtr; //preinstantiated Accelerometer object.

#endif	//Accelerometer_h

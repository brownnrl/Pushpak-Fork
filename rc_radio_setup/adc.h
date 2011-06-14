/*
  ADC.h
  
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

#ifndef ADC_h
#define ADC_h

#include <inttypes.h>

#ifdef __cplusplus
extern "C"{
#endif


#define NUM_ADC_CH 				8	//! \def Number of ADC channels to sample
#define MAX_ACCUMLATION_COUNT	15	//! \def Number of samples to accumlate and generate one output.

#define ADC_REF_EXT 	0	//! \def ADC reference voltage is set to VREF pin(external).
#define ADC_REF_AVCC	1	//! \def ADC reference voltage is set to analog supply voltage (AVCC).
#define ADC_REF_INT11	2	//! \def ADC reference voltage is set to Internal 1.1V source.
#define ADC_REF_INT25	3	//! \def ADC reference voltage is set to Internal 2.56V source.


extern volatile uint16_t gADC_output[NUM_ADC_CH];	//! \def Copy of the final accumlated output result which is accessed by user programs.
extern volatile uint16_t gADC_curr[NUM_ADC_CH];		//! \def current ADC sample without any accumlation or averaging
extern volatile uint8_t gADC_new_output;			//! \def Flag to indicate when accumlation is finished and new output is available. This flag can used


void initialize_adc();
void update_adc_samples(void);

#ifdef __cplusplus
} // extern "C"
#endif

#endif

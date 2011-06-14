/*
  This library is free software; you can redistribute it and/or modify it under the
  terms of the GNU Lesser General Public License as published by the Free Software
  Foundation; either version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful, but WITHOUT ANY
  WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A 
  PARTICULAR PURPOSE.  See the GNU Lesser General Public License for more details.
*/

// Copyright (c) 2009 Brijesh Sirpatil


#include <util/delay.h>
#include <inttypes.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#include "adc.h"

/// This library provides access to Analog to Digital Converter(ADC) using auto
/// trigger functionality. Library uses oversampling to increase ADC bit resolution to 12bits.

/// The ADC clock prescale is set to 128, which provides 156.250KHz ADC clock.
/// ADC requires 13 clock cycles to sample data. So sampling rate is 156.250/13 = 12.0192KHz.
/// This library samples all the 8 channels in continous loop using interrupts and autotrigger
/// functionality. Individual channel sample rate = 12.0192/8 = 1.5024KHz. After accumalating 16 samples the final 
/// sample rate is 1.5024/16 = 93.9 Hz. 
///
/// With 16x oversampling and accumlation we have increased bit resoultion as defined by equation
/// samples to be accumlated = 2^(4n) where n is number of additional bit resoultion.
///
/// Accumlation of 16 samples results in bit growth out of 4 bits out of which only 2 bits are useful.
///	So in the final output LSB 2 bits are dropped.
///


#define MAX_ACCUMLATION_COUNT	16	//! \def Number of samples to accumlate and generate one output.
#define LSB_DROP_CNT			2	//! \def Number of bits to be dropped after accumlation to match the true increase in ADC resolution.

//ADC refrence voltage bit mask
#define ADC_REF_EXT 	0	//! \def ADC reference voltage is set to VREF pin(external).
#define ADC_REF_AVCC	1	//! \def ADC reference voltage is set to analog supply voltage (AVCC).
#define ADC_REF_INT11	2	//! \def ADC reference voltage is set to Internal 1.1V source.
#define ADC_REF_INT25	3	//! \def ADC reference voltage is set to Internal 2.56V source.

#define ADC_INP_11REF	0x1E	//! \def ADC input is internal 1.1 voltage source
#define ADC_INP_GND		0x1F	//! \def ADC input is Ground.


volatile uint16_t gADC_output[NUM_ADC_CH];			//Contains latest accumalted and updated sample
volatile uint32_t gSample_cnt;

uint16_t adc_ref_val, adc_raw_ref_val;
uint16_t adc_conv_factor;
#define FACTOR_MULTIPLE 10			//number of bits the ADC conversion factor is left shifted.



//These global variables should be visible only in this file.
static uint8_t gADC_ch;
static volatile uint16_t gADC_acc[NUM_ADC_CH]; 		//Contains the intermediate accumlator values
static volatile uint16_t gADC_sample[NUM_ADC_CH];	//Contains the final accumlated output result

volatile uint8_t gADC_new_output = 0;	//Flag to indicate when accumlation is finished and new output is available. This flag can used
										//to synchronise the control loop to excat same rate as ADC sampling rate. The ADC sampling rate is
										//100.16Hz.
//***************************************************
//Temporary variables for testing and characterization of the board. Delete for normal operation
volatile uint16_t gADC_curr[NUM_ADC_CH];		//current ADC sample without any accumlation or averaging
volatile uint16_t gADC_curr_output[NUM_ADC_CH];	//current ADC sample without any accumlation or averaging

//***************************************************

void adc_set_ref(uint8_t ref)
{
	//bits 7 and 6 select the refrence voltage.
	ADMUX = (ADMUX & 0x3F) | (ref << 6);	
}

void adc_set_ch(uint8_t ch)
{
	//bits 4 downto 0 are used to select channel.
	ADMUX = (ADMUX & 0xE0) | (0x1F & ch);	
}

///
/// Gives the number of times ADC has cycled through and produced sample since reset.
/// This count is equivalent to system time count if the system loop is locked with the ADC samples.
///
uint32_t adc_get_sample_time()
{
	uint8_t oldSREG;
	uint32_t temp;

	oldSREG = SREG;
	cli();	//disable interrupts while copying the ADC data
	
	temp = gSample_cnt;
		
	// reenable interrupts.
	SREG = oldSREG;

	return temp;
}

void adc_initialize( )
{
	uint8_t i;
	uint8_t high, low;
	
	float factor;
	 
	gSample_cnt = 0;
	//Clock prescale to 128.
	ADCSRA |= _BV(ADPS2);	//Clock Prescale
	ADCSRA |= _BV(ADPS1);	//Clock Prescale
	ADCSRA |= _BV(ADPS0);	//Clock Prescale

	//Set the reference voltage. For Pushpak Quadrotor board the refrence has to be VREF.
	adc_set_ref(ADC_REF_EXT);
	ADCSRA |= _BV(ADEN);	//Enable ADC.
	
	//By sampling the internal 1.1 voltage source, we can precisely calculate the value of external ADC
	//refernce voltage. This value can then be used to convert ADC readings to accurate voltage readings. 
	
	//Get use internal 1.1V to calibrate the ADC refrence voltage
	adc_set_ch(ADC_INP_11REF);

	_delay_ms(100); //delay needed to let the voltages stablize and get accurate reading of 1.1V refrence.

	adc_ref_val = 0;
	adc_raw_ref_val = 0;	

	//Sample and accumalate the internal refernce voltage
	for(i=0;i<MAX_ACCUMLATION_COUNT;++i)
	{	
		ADCSRA |= _BV(ADSC);	//start conversion
		
		// ADSC is cleared when the conversion finishes
		while (bit_is_set(ADCSRA, ADSC));
	
		low = ADCL;
		high = ADCH;
		
		adc_raw_ref_val = (high << 8 ) | low;
		adc_ref_val = adc_ref_val + adc_raw_ref_val;
	}
	
	//Accumlation increase bit count, but actual increase in ADC bit resolution is lesser.
	//Drop the addition bits
	adc_ref_val = adc_ref_val >> LSB_DROP_CNT; 
	
	//Calculate milli Volts per count.
	//To get better resolution multiply the conversion by 1024 or 10bits.
	//Perform floating point operations and then convert to integer value
	
	factor = (1100.0* (1 << FACTOR_MULTIPLE))/adc_ref_val; 	//The refrence voltage inside AVR is 1.1V which 1100 in mV
	factor = factor + 0.5; 					//To round the value to nearest integer
	adc_conv_factor = (uint16_t) factor;
	
/**********************************************************************************************/
//Initialize for Autotrigger functionality.

	//disable it while setting up the Auto Trigger functionality
	ADCSRA &= ~(_BV(ADEN));
	
	//Set the reference voltage. For Pushpak Quadrotor board the refrence has to be VREF.
	adc_set_ref(ADC_REF_EXT);
	adc_set_ch(0);

	ADCSRA |= _BV(ADATE);	//Enable Auto Trigger
	ADCSRA |= _BV(ADIE);	//Enable Interrupt
	ADCSRA |= _BV(ADEN);	//Enable ADC.
		
	//initialize all the variables
	for(i = 0; i<NUM_ADC_CH; i++)
	{
		gADC_acc[i] = 0;
		gADC_sample[i] = 0;
	}
	
	_delay_ms(100); //delay needed to let the voltages stablize.
			
	sei();			/* enable interrupts if they are by chance not enabled yet */

	//Start one conversion to set the ball rolling.
	ADCSRA |= _BV(ADSC);

	//According to datasheet we have to wait for at least 1 ADC clock cycle after starting a conversion
	//to change the channel. ADC clock has time period of 8uS so have to wait for that time.
	//Following dummy loop is for the delay. By using gADC_ch for delay loop counter, ensuring that
	//delay loop does not get optimized out.	
 	gADC_ch = 70;
 	while(gADC_ch != 1)
 	{
 		--gADC_ch;
 	}
	
	gADC_ch = 1;
	adc_set_ch(gADC_ch);
}

//Function returns 1 when a new sample is ready. A new sample is ready once the given number of samples are
///accumlated.
uint8_t adc_is_data_ready()
{
	return gADC_new_output;
}

/// Function blocks till a new sample is available. It copies the accumlated output to an array.
/// This copy is necessary since the ADC samples are continously being accumlated by the interrupts. 
/// ADC ouput after accumalation is 12bits.
void adc_update(void)
{
	uint8_t i;
	uint8_t oldSREG;
		
	while(gADC_new_output == 0);
	gADC_new_output = 0; //clear the flag
		
	oldSREG = SREG;
	cli();	//disable interrupts while copying the ADC data
	
	for(i=0; i<NUM_ADC_CH; i++)
	{
		gADC_output[i] = gADC_sample[i] >> 2;	//Divide by 4 as the LSB 2 bits contain no useful information/
		gADC_curr_output[i] = gADC_curr[i] >> 2; //test code
	}
	
	// reenable interrupts.
	SREG = oldSREG;
}


/// Returns ADC sample in millivolts.
int16_t adc_get_value_mv( uint8_t ch)
{
	int32_t temp1, temp2	;
	//adc_conv_factor is 2^FACTOR_MULTIPLE larger than actual value. Did that for precision
	//So multiply by conversion factor and then perform division by left shift to get true value in mV.

	temp1 = gADC_output[ch];
	temp2 = (temp1 * 743);
	temp2 = temp2 >> FACTOR_MULTIPLE;

	return (int16_t) temp2;
}

/// Returns ADC sample.
int16_t adc_get_sample( uint8_t ch)
{
	return gADC_output[ch];
}


///ADC conversion complete ISR.
ISR(ADC_vect)
{
	uint8_t low, high;
	uint8_t i;
	static uint8_t gADC_acc_cnt = 0;	//Accumlation count	
	
//	PORTB |= _BV(PIN5);	//Drive LED to measure the ISR execution time.
	
	low = ADCL;		// we have to read ADCL first; doing so locks both ADCL and ADCH until ADCH is read. 
	high = ADCH;	

	//Accumalate the current ADC sample.
	//In normal operation if the gADC_ch == 2, currently ADC is sampling channel 2 and result from channel 1 is available
	//read the ADC value and accumlate/average.		
	if(gADC_ch == 0)
	{//special condition, when Ch == 0, then channel count has wrapped around hence the result present is from that of highest channel number.
		gADC_acc[NUM_ADC_CH - 1] += (high << 8) | low; 
		gADC_curr[NUM_ADC_CH - 1] = (high << 8) | low; 	   
	}
	else
	{
		gADC_acc[gADC_ch-1] += (high << 8) | low;
		gADC_curr[gADC_ch-1] = (high << 8) | low;  	   			
	}

	//Check if the number of accumlated samples has reached the max accumlation count.
	//This "if" statement could have been merged with above "if". Keeping them seperate for clarity.
    if(gADC_ch == 0)
    {//finished cycling through all the channels increment accumlation count
    	++gADC_acc_cnt;
		if(gADC_acc_cnt == MAX_ACCUMLATION_COUNT)
		{//Finished accumlating max number of samples, copy the value into output result array
			
			for(i = 0; i<NUM_ADC_CH; i++)
			{
				gADC_sample[i] = gADC_acc[i];
				gADC_acc[i] = 0;		//reset the accumlator register
			}
			
			gADC_acc_cnt = 0;
			gADC_new_output = 1;
			++gSample_cnt;
		}    
    }
   
    ++gADC_ch;
    if(gADC_ch == NUM_ADC_CH)
    {//finished cycling through all the channels
	   	gADC_ch=0; //cycle back to first channel
	}    

	//Write to channel mux register at the end so that the required 1 ADC clock time delay after start of conversion is met.
    //select the next channel     
	adc_set_ch(gADC_ch);

//	PORTB &= (~_BV(PIN5));	//Drive LED to measure the ISR execution time.
}

/*
  adc.c
  
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

#include <inttypes.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#include "adc.h"

/// This library provides access to Analog to Digital Converter(ADC) using auto trigger functionality. Libray internally samples
/// at much higher rate. It then acumalates multiple samples to provide final 100Hz out sample rate.


//These global variables should be visible only in this file.
static uint8_t gADC_ch;
static uint8_t gADC_acc_cnt;						// Accumlation count
//static uint8_t gADC_max_acc_cnt;					// Maximum number of samples to accumlate
static volatile uint16_t gADC_acc[NUM_ADC_CH]; 		//Contains the intermediate accumlator values
static volatile uint16_t gADC_sample[NUM_ADC_CH];	//Contains the final accumlated output result
volatile uint16_t gADC_output[NUM_ADC_CH];

volatile uint8_t gADC_new_output = 0;	//Flag to indicate when accumlation is finished and new output is available. This flag can used
										//to synchronise the control loop to excat same rate as ADC sampling rate. The ADC sampling rate is
										//100.16Hz.
//***************************************************
//Temporary variables for testing and characterization of the board. Delete for normal operation
volatile uint16_t gADC_curr[NUM_ADC_CH];		//current ADC sample without any accumlation or averaging
volatile uint16_t gADC_curr_output[NUM_ADC_CH];	//current ADC sample without any accumlation or averaging

//***************************************************

/// The ADC clock prescale is set to 128, which provides 156.250KHz ADC clock.
/// ADC requires 13 clock cycles to sample data. So sampling rate is 156.250/13 = 12.0192KHz.
/// This library samples all the 8 channels in continous loop using interrupts and autotrigger functionality. 
/// Individual channel sample rate = 12.0192/8 = 1.5024KHz. With a 100Hz loop rate each individual channel will be sampled 15 times 
/// in one loop iteration. Hence we have 15 times oversampling. The 15 samples are accumlated and presented as the finaly output.

/// The library does not divide the result by 15 and get average value. Using the accumlated value we get advantage of 2 bits of
/// increased ADC resolution. So now we have a 12bit ADC. 

void initialize_adc( )
{
	uint8_t i;

	//disable it while setting up the Auto Trigger functionality
	ADCSRA &= ~(_BV(ADEN));
	
	//Set the reference voltage. For Pushpak Quadrotor board the refrence has to be VREF.
	//Set the mux for first channel.
	//LSB 4 bits determine the channel, reset them for first channel
	ADMUX = (ADC_REF_EXT << 6) | (0x07 & 0);	
	
	ADCSRA = 0;	
	ADCSRA |= _BV(ADATE);	//Enable Auto Trigger
	ADCSRA |= _BV(ADIE);	//Enable Interrupt
	ADCSRA |= _BV(ADEN);	//Enable ADC.
	//Clock prescale to 128.
	ADCSRA |= _BV(ADPS2);	//Clock Prescale
	ADCSRA |= _BV(ADPS1);	//Clock Prescale
	ADCSRA |= _BV(ADPS0);	//Clock Prescale
	
	
	//initialize all the variables
	gADC_acc_cnt = 0;
	for(i = 0; i<NUM_ADC_CH; i++)
	{
		gADC_acc[i] = 0;
		gADC_sample[i] = 0;
	}
			
	sei();			/* enable interrupts */

	/*
	* Start one conversion to set the ball rolling.
	*/
	ADCSRA |= _BV(ADSC);

	//According to datasheet we have to wait for at least 1 ADC clock cycle after starting a conversion
	//to change the channel. ADC clock has time period of 8uS so have to wait for that time.
	//Following dummy loop is for the delay. By using gADC_ch for delay loop counter, ensuring that
	//delay loop does not get optimized out.	
// 	gADC_ch = 70;
// 	while(gADC_ch != 1)
// 	{
// 		--gADC_ch;
// 	}
	
	gADC_ch = 1;
	ADMUX = (ADMUX & 0xF8) | (0x07 & gADC_ch);
}


/// Function updated the local copy of ADC results with current results. This copy is necessary since the
/// the ADC results are continously being updated by the interrupts. 
void update_adc_samples(void)
{
	uint8_t i;
	uint8_t oldSREG;
		
	oldSREG = SREG;
	cli();	//disable interrupts while copying the ADC data
	
	for(i=0; i<NUM_ADC_CH; i++)
	{
		gADC_output[i] = gADC_sample[i];

		gADC_curr_output[i] = gADC_curr[i]; //test code

	}
	
	// reenable interrupts.
	SREG = oldSREG;
}




/*
 * ADC conversion complete.  Fetch the 10-bit value, and feed the
 * PWM with it.
 */
ISR(ADC_vect)
{
	uint8_t low, high;
	uint8_t i;
	//to measure the execution time of ISR. Comment out for normal operation.
	//On Pushpak Quadrotor board, LED is connected to this pin.
//	PORTB |= _BV(PIN5);	
	
	// we have to read ADCL first; doing so locks both ADCL and ADCH until ADCH is read. 
	low = ADCL;
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
			gADC_acc_cnt = 0;
			for(i = 0; i<NUM_ADC_CH; i++)
			{
				gADC_sample[i] = gADC_acc[i];
				gADC_acc[i] = 0;		//reset the accumlator register
			}
			gADC_new_output = 1;
		}    
    }
   
    ++gADC_ch;
    if(gADC_ch == NUM_ADC_CH)
    {//finished cycling through all the channels
	   	gADC_ch=0; //cycle back to first channel
	}    
      
    //Write to channel mux register at the end so that the required 1 ADC clock time delay after start of conversion is met.
    //select the next channel     
	ADMUX = (ADMUX & 0xF8) | (0x07 & gADC_ch);

	//to measure the execution time of ISR. Comment out for normal operation.	
//	PORTB &= (~_BV(PIN5));
}

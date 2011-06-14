/*
  This library is free software; you can redistribute it and/or modify it under the
  terms of the GNU Lesser General Public License as published by the Free Software
  Foundation; either version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful, but WITHOUT ANY
  WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A 
  PARTICULAR PURPOSE.  See the GNU Lesser General Public License for more details.

  Copyright (c) 2010 Brijesh Sirpatil
*/


#include <inttypes.h>

#include "util.h"
#include "pushpak.h"
#include "HardwareSerial.h"
#include "Print.h"

#define HEADER_SIZE 3			//2bytes of signature, 1byte size
#define MAX_PAYLOAD_SIZE 60		//Ensure the serial ports have enough buffer space, MAX_PAYLOAD_SIZE + header size should not overflow serial port buffers
#define PACKET_ERROR_BIT 0x80

uint8_t status = 0;


//Message parsing state machine
enum parser_state_mc { 	STATE_IDLE, 	//Idle state
						STATE_HDR1, 	//Signature byte 1 has been recieved correctly
						STATE_HDR2, 	//Signature byte 2 has been recieved correctly
						STATE_PAYLOAD	//Collecting the packet payload and execute packet
					}; 

//! Parses the recieved data stream for valid packets. Receives valid packets, strips the header information
//! and puts payload in the buffer. When complete payload is received returns pointer to start of the payload.
//! Function does not block or wait for message to be received, returns NULL pointer when there is no valid packet.					
//! Call the function repeatedly to decode the incomming data stream.
//! Process the returned payload before calling the this function again.
					
uint8_t* parse_for_pkt()
{
	static uint8_t buffer[MAX_PAYLOAD_SIZE]; //temp buffer to hold the message
	static uint8_t	size = 0;
	static uint8_t	cnt = 0;
	static parser_state_mc parser_state = STATE_IDLE;
	parser_state_mc parser_next_state;
	
	uint8_t byte;
	
	uint8_t flag = false;
	
	if(Serial.data_available() == 0)
	{//no bytes to process
		return NULL;
	}
	
	byte = Serial.read();
	
	switch(parser_state)
	{
		case STATE_IDLE:
			if(byte == 0xAB)
			{
				parser_next_state = STATE_HDR1;				
			}
			else
			{
				status |= PACKET_ERROR_BIT; //note or'ing the bits
				parser_next_state = STATE_IDLE;	
			}
			break;
			
		case STATE_HDR1:
			if(byte == 0xCD)
			{
				parser_next_state = STATE_HDR2;				
			}
			else
			{
				status |= PACKET_ERROR_BIT; //note or'ing the bits
				parser_next_state = STATE_IDLE;	
			}
			break;

		case STATE_HDR2:
			if(byte < MAX_PAYLOAD_SIZE)
			{
				size = byte;
				cnt = 0;	
				parser_next_state = STATE_PAYLOAD;			
			}
			else
			{
				status |= PACKET_ERROR_BIT; //note or'ing the bits
				parser_next_state = STATE_IDLE;	
			}
			break;
			
		case STATE_PAYLOAD:
			buffer[cnt] = byte;
			cnt++;
			if(cnt < size)
			{
				parser_next_state = STATE_PAYLOAD;				
			}
			else
			{
				flag = true;	//Full payload received
				cnt = 0;
				size = 0;
				parser_next_state = STATE_IDLE;	
			}
			break;
							
		default :
			parser_next_state = STATE_IDLE;
			break;
	}
	
	parser_state = parser_next_state;
	
	if(flag == true)
		return &buffer[0];
	else
		return NULL;
	
}

uint8_t send_pkt(uint8_t *buf, uint8_t len)
{
	if(len > MAX_PAYLOAD_SIZE)
	{
		return false;
	}
	
	if(Serial.space_available() <  (len + HEADER_SIZE))
	{
		return false;
	}
	
	Serial.write(0xAB);		//header byte 1
	Serial.write(0xCD);		//header byte 2
	Serial.write(len);
	
	for(uint8_t i=0; i<len; ++i)
	{
		Serial.write(buf[i]);
	}
	
	return true;
}

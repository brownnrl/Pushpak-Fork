/*
  HardwareSerial.cpp - Hardware serial library for Wiring
  Copyright (c) 2006 Nicholas Zambetti.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
  
  Modified 23 November 2006 by David A. Mellis
  
  Modified February 18, 2010 by Brijesh
  Modified code so that data transmission is not interrupt based and uses buffers.
  
*/

#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include <util/atomic.h>

#include "pushpak.h"
#include "HardwareSerial.h"
	

// Define constants and variables for buffering incoming serial data.  We're
// using a ring buffer (I think), in which rx_buffer_head is the index of the
// location to which to write the next incoming character and rx_buffer_tail
// is the index of the location from which to read.

//WARNING: Buffer sizes MUST be power of 2.
#define RX_BUFFER_SIZE 64
#define TX_BUFFER_SIZE 64

#define RX_BUFFER_MASK (RX_BUFFER_SIZE - 1)
#define TX_BUFFER_MASK (TX_BUFFER_SIZE - 1)

#if (RX_BUFFER_SIZE & RX_BUFFER_MASK)
#    error RX buffer size is not a power of 2
#endif
#if (TX_BUFFER_SIZE & TX_BUFFER_MASK)
#    error TX buffer size is not a power of 2
#endif

#if (RX_BUFFER_SIZE > 256 )		//head and tail counters data types limits the buffer size
#    error RX buffer size is too large.
#endif
#if (TX_BUFFER_SIZE > 256 )
#    error TX buffer size is too large.
#endif

/////////////////////////////////////////////////////
//RX buffers

struct rx_ring_buffer {
  uint8_t buffer[RX_BUFFER_SIZE];
  volatile uint8_t head;
  volatile uint8_t tail;
};

rx_ring_buffer rx_buffer0 = { { 0 }, 0, 0 };

////////////////////////////////////////////////////
//Tx Buffers

struct tx_ring_buffer {
  uint8_t buffer[TX_BUFFER_SIZE];
  volatile uint8_t head;
  volatile uint8_t tail;
};

tx_ring_buffer tx_buffer0 = { { 0 }, 0, 0 };


/////////////////////////////////////////////////////////////////////////

inline void store_char(unsigned char c, rx_ring_buffer *rx_buffer)
{
  uint8_t i = (rx_buffer->head + (uint8_t) 1) & (uint8_t)RX_BUFFER_MASK;

  // if we should be storing the received character into the location
  // just before the tail (meaning that the head would advance to the
  // current location of the tail), we're about to overflow the buffer
  // and so we don't write the character or advance the head.
  if (i != rx_buffer->tail) {
    rx_buffer->buffer[rx_buffer->head] = c;
    rx_buffer->head = i;
  }
}

#if defined (__AVR_ATmega644P__)

	SIGNAL(SIG_USART_RECV)
	{
	  unsigned char c = UDR0;
	  store_char(c, &rx_buffer0);
	}
	
	
	//SIGNAL(SIG_USART1_RECV)
	//{
	//  unsigned char c = UDR1;
	//  store_char(c, &rx_buffer1);
	//}

#endif


///////////////////////////////////////////////////////////////////////
//Transmission ISR's

// inline void send_char(volatile uint8_t *ucsrb, uint8_t udrie, volatile uint8_t *udr, tx_ring_buffer *tx_buffer)
// {
// 	if (tx_buffer->head == tx_buffer->tail)
// 	{
// 	// Buffer is empty, disable the interrupt
// 	*ucsrb &= ~(1 << udrie);
// 	}
// 	else
// 	{
// 	tx_buffer->tail = (tx_buffer->tail + (uint8_t)1) & (uint8_t)TX_BUFFER_MASK;
// 	*udr = tx_buffer->buffer[tx_buffer->tail];
// 	}  
// }

SIGNAL(SIG_USART_DATA)
{
  if (tx_buffer0.head == tx_buffer0.tail)
  {
    // Buffer is empty, disable the interrupt
    UCSR0B &= ~(1 << UDRIE0);
  }
  else
  {
    tx_buffer0.tail = (tx_buffer0.tail + (uint8_t)1) & (uint8_t)TX_BUFFER_MASK;
    UDR0 = tx_buffer0.buffer[tx_buffer0.tail];
  }  
}

// SIGNAL(SIG_USART1_DATA)
// {
//   if (tx_buffer1.head == tx_buffer1.tail)
//   {
//     // Buffer is empty, disable the interrupt
//     UCSRB1 &= ~(1 << UDRIE1);
//   }
//   else
//   {
//     tx_buffer1.tail = (tx_buffer1.tail + (uint8_t)1) & (uint8_t)TX_BUFFER_MASK;
//     UDR0 = tx_buffer1.buffer[tx_buffer1.tail];
//   }  
// }


// Constructors ////////////////////////////////////////////////////////////////

HardwareSerial::HardwareSerial(rx_ring_buffer *rx_buffer, tx_ring_buffer *tx_buffer, 
  volatile uint8_t *ubrrh, volatile uint8_t *ubrrl,
  volatile uint8_t *ucsra, volatile uint8_t *ucsrb,
  volatile uint8_t *udr,
  uint8_t rxen, uint8_t txen, uint8_t rxcie, uint8_t udrie, uint8_t u2x)
{
  _rx_buffer = rx_buffer;
  _tx_buffer = tx_buffer;
  _ubrrh = ubrrh;
  _ubrrl = ubrrl;
  _ucsra = ucsra;
  _ucsrb = ucsrb;
  _udr = udr;
  _rxen = rxen;
  _txen = txen;
  _rxcie = rxcie;
  _udrie = udrie;
  _u2x = u2x;
}

// Public Methods //////////////////////////////////////////////////////////////

void HardwareSerial::begin(long baud)
{
  uint16_t baud_setting;
  bool use_u2x;

  // U2X mode is needed for baud rates higher than (CPU Hz / 16)
  if (baud > F_CPU / 16) {
    use_u2x = true;
  } else {
    // figure out if U2X mode would allow for a better connection
    
    // calculate the percent difference between the baud-rate specified and
    // the real baud rate for both U2X and non-U2X mode (0-255 error percent)
    uint8_t nonu2x_baud_error = abs((int)(255-((F_CPU/(16*(((F_CPU/8/baud-1)/2)+1))*255)/baud)));
    uint8_t u2x_baud_error = abs((int)(255-((F_CPU/(8*(((F_CPU/4/baud-1)/2)+1))*255)/baud)));
    
    // prefer non-U2X mode because it handles clock skew better
    use_u2x = (nonu2x_baud_error > u2x_baud_error);
  }
  
  if (use_u2x) {
    *_ucsra = 1 << _u2x;
    baud_setting = (F_CPU / 4 / baud - 1) / 2;
  } else {
    *_ucsra = 0;
    baud_setting = (F_CPU / 8 / baud - 1) / 2;
  }

  // assign the baud_setting, a.k.a. ubbr (USART Baud Rate Register)
  *_ubrrh = baud_setting >> 8;
  *_ubrrl = baud_setting;

  // Flush buffers
  _tx_buffer->head = _tx_buffer->tail = 0;
  _rx_buffer->head = _rx_buffer->tail = 0;
  
  sbi(*_ucsrb, _rxen);
  sbi(*_ucsrb, _txen);
  sbi(*_ucsrb, _rxcie);
}

uint8_t HardwareSerial::data_available(void)
{
  return (_rx_buffer->head - _rx_buffer->tail) & (uint8_t)RX_BUFFER_MASK;
}

uint8_t HardwareSerial::read(void)
{
	uint8_t temp;	
	// if the head isn't ahead of the tail, we don't have any characters
	if (_rx_buffer->head == _rx_buffer->tail)
	{
		return 0;
	} 
	else
	{
		temp = _rx_buffer->buffer[_rx_buffer->tail];	
		_rx_buffer->tail = (_rx_buffer->tail + (uint8_t) 1) % RX_BUFFER_MASK;  
		return temp;
	}
}

void HardwareSerial::flush()
{
  // don't reverse this or there may be problems if the RX interrupt
  // occurs after reading the value of rx_buffer_head but before writing
  // the value to rx_buffer_tail; the previous value of rx_buffer_head
  // may be written to rx_buffer_tail, making it appear as if the buffer
  // don't reverse this or there may be problems if the RX interrupt
  // occurs after reading the value of rx_buffer_head but before writing
  // the value to rx_buffer_tail; the previous value of rx_buffer_head
  // may be written to rx_buffer_tail, making it appear as if the buffer
  // were full, not empty.
    
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
	{
	  _rx_buffer->head = _rx_buffer->tail;
	}
}

//! Returns the empty space available in transmit buffers.
uint8_t HardwareSerial::space_available(void)
{
	uint8_t temp;
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
	{
		temp = (_tx_buffer->head - _tx_buffer->tail) & (uint8_t)TX_BUFFER_MASK;
		temp = TX_BUFFER_MASK - temp;
	}
	return temp;
}

void HardwareSerial::write(uint8_t data)
{
	
  uint8_t tmp_head;
 
  // Calculate new head position
   tmp_head = (_tx_buffer->head + (uint8_t)1) & (uint8_t)TX_BUFFER_MASK;

  // Block until there's room in the buffer
  // XXX: this may block forever if someone externally disabled the transmitter
  //      or the DRE interrupt and there's data in the buffer. Careful!
 
  while (tmp_head == _tx_buffer->tail);

  //Store the data and then advance the head
 
  _tx_buffer->buffer[tmp_head] = data;
  _tx_buffer->head = tmp_head;

  *_ucsrb |= (1 << _udrie); // Enable Data Register Empty interrupt
}


// Preinstantiate Objects //////////////////////////////////////////////////////

HardwareSerial Serial(&rx_buffer0, &tx_buffer0, &UBRR0H, &UBRR0L, &UCSR0A, &UCSR0B, &UDR0, RXEN0, TXEN0, RXCIE0, UDRIE0, U2X0);



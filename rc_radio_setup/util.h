/*
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

#ifndef UTIL_H
#define UTIL_H

#ifdef __cplusplus
extern "C"{
#endif

//!
//! For bit manipulation of internal registers.
//!

#define 	SET_BIT(i, n)   ((i) |= (1 << (n)))
//#define 	GET_BIT(i, n)   ((i) & (1 << (n)))
#define 	GET_BIT(i, n)   ( ((i) & (1 << (n))) >> (n) )
#define 	CLR_BIT(i, n)   ((i) &= ~(1 << (n)))

//!
//! For accessing and controling input/output pins. Since I/O operations require accessing three
//! different registers, DDRx, PORTx and PINx these #defines provide a clean interface.
//!
#define G_INPUT(port,pin) DDR ## port &= ~(1<<pin)
#define G_OUTPUT(port,pin) DDR ## port |= (1<<pin)
#define G_CLEAR(port,pin) PORT ## port &= ~(1<<pin)
#define G_SET(port,pin) PORT ## port |= (1<<pin)
#define G_TOGGLE(port,pin) PORT ## port ^= (1<<pin)
//#define G_READ(port,pin) (PIN ## port & (1<<pin))
#define G_READ(port,pin) ((PIN ## port & (1<<pin)) >> pin)
#define G_READ_N(port,pin) !((PIN ## port & (1<<pin)) >> pin)

#define GPIO_INPUT(...)    G_INPUT(__VA_ARGS__) 	//! Make the pin as an input pin.
#define GPIO_OUTPUT(...)   G_OUTPUT(__VA_ARGS__)	//! Make the pin as an outout pin.
#define GPIO_CLEAR(...)    G_CLEAR(__VA_ARGS__) 	//! Write 0 to the output pin.
#define GPIO_SET(...)      G_SET(__VA_ARGS__)		//! Write 1 to the output pin.
#define GPIO_TOGGLE(...)   G_TOGGLE(__VA_ARGS__)	//! Toggle the output pin.
#define GPIO_READ(...)     G_READ(__VA_ARGS__)		//! Read the value of input pin.	
#define GPIO_READ_N(...)   G_READ_N(__VA_ARGS__)	//! Read the value of input pin and give inverted result.






#ifdef __cplusplus
} // extern "C"
#endif

#endif // UTIL_H

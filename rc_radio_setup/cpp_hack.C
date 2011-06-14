//!
//! Following function need to be defined to be able to use C++ using GCC.
//! Refer to the following AVRFreaks forum thread
//!	http://www.avrfreaks.net/index.php?name=PNphpBB2&file=viewtopic&t=59453&start=all&postdays=0&postorder=asc
//!
#include <stdlib.h> 
#include "cpp_hack.h"


int __cxa_guard_acquire(__guard *g) {return !*(char *)(g);};
void __cxa_guard_release (__guard *g) {*(char *)g = 1;};
void __cxa_guard_abort (__guard *) {}; 
void __cxa_pure_virtual(void) {}; 
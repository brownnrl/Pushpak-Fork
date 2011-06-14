#ifndef CPP_HACK_H
#define CPP_HACK_H

//!
//! Following function need to be defined to be able to use C++ using GCC.
//! Refer to the following AVRFreaks forum thread
//!	http://www.avrfreaks.net/index.php?name=PNphpBB2&file=viewtopic&t=59453&start=all&postdays=0&postorder=asc
//!

__extension__ typedef int __guard __attribute__((mode (__DI__)));

extern "C" int __cxa_guard_acquire(__guard *);
extern "C" void __cxa_guard_release (__guard *);
extern "C" void __cxa_guard_abort (__guard *); 

extern "C" void __cxa_pure_virtual(void); 

#endif //CPP_HACK_H
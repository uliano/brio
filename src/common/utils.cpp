/*
 * File:   utils.cpp
 * Author: uliano
 *
 * Ported from uliano/AVR-Multislope (lib/core/src/utils.cpp).
 */

// those two functions are needed (on AVR) to avoid linking errors
// that arise with virtual destructors in abstract classes
void operator delete(void*) noexcept {}
void operator delete(void*, unsigned int) noexcept {}

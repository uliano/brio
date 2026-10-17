// mcu: avr128da28 avr128da32 avr128da48 avr128da64 avr64ea28 avr64ea48
// The AVR DA and the AVR EA have no OPAMP peripheral at all: the header
// compiles to nothing there, so naming any of it must fail.
#include "avrdx/opamp.hpp"
using namespace brio;
void f() { (void)Opamp<0>::init(OpampConfig{}); }

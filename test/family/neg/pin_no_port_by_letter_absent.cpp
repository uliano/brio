// mcu: avr128db28 avr128db32 avr128db48 avr128da28 avr128da32 avr128da48
// A constant letter naming a port the package has not got (PORTG is
// 64-pin only) is refused where the call is compiled, instead of
// falling back to PORTA.
#include "avrdx/pin.hpp"
using namespace brio;
void f() { port_by_letter('G').OUTSET = 1; }

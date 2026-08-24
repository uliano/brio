// mcu: avr128db32 avr128da32
// TWI1 ALT2 sits on PB2/PB3 and 32-pin packages have no PORTB: the
// device headers list DEFAULT and ALT1 for TWI1 there and nothing else.
#include "avrdx/clock.hpp"
#include "avrdx/twi.hpp"
using namespace brio;
void f() { (void)TwiHost<1, TwiRoute::alt2>::init(Clock<ClockSource::internal, 24'000'000>{}); }

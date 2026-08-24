// mcu: avr128db28 avr128db32 avr128da28 avr128da32
// A Dual mode client needs the route's SECOND pin pair. TWI0 ALT1's is
// PC6/PC7, bonded from 48 pins up - below that the route works but the
// dual client does not exist.
#include "avrdx/clock.hpp"
#include "avrdx/twi.hpp"
using namespace brio;
void f() {
    (void)TwiClient<0, TwiRoute::alt1, true>::init(
        Clock<ClockSource::internal, 24'000'000>{}, {.address = 0x40});
}

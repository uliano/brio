// mcu: samc21e18a samc21g18a samc21j18a
// AVR's 1024 Hz tick is exactly the rate this ticker refuses: it does
// not divide 1000, so millis() could not be exact.
#include "samc/ticker.hpp"
using namespace brio;
using Rate1024 = BasicTicker<1024>;
void f() { (void)Rate1024::ticks(); }

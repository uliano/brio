// mcu: ch32v203rb
// The CH32V203RB's oscillator is 32 MHz and nothing else - its load
// capacitors are built in - so the 8 MHz crystal every other part of the
// family takes is refused on it.
#include "ch32vx03/clock.hpp"

using Eight = brio::Clock<brio::ClockSource::crystal, 8'000'000, 8'000'000>;
void f() { (void)Eight::init(); }

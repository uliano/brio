// mcu: ch32v006k8 ch32v003f4
// A crystal is named with its rate: ClockSource::crystal without the
// third parameter must be REFUSED.
#include "ch32v00x/clock.hpp"

using Xtal = brio::Clock<brio::ClockSource::crystal, 24'000'000>;
void f() { (void)Xtal::init(); }

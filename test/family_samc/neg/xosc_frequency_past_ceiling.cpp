// A crystal past the oscillator's own 32 MHz ceiling (DS60001479M 20.2)
// must be refused at compile time when the configuration is known then.
// mcu: samc21e18a samc21g18a samc21j18a
#include "samc/clock.hpp"

using namespace brio;

void bad() {
    (void)Xosc::init<XoscConfig{.hz = 40'000'000}>();
}

// mcu: ch32v203f6 ch32v203g6 ch32v203k6 ch32v203k8 ch32v203c6 ch32v203c8
// The HSE oscillator of these parts takes 3 to 25 MHz: a 30 MHz crystal
// is above the range the datasheet states for them (table 4-11).
#include "ch32v203/clock.hpp"

using TooFast = brio::Clock<brio::ClockSource::crystal, 30'000'000, 30'000'000>;
void f() { (void)TooFast::init(); }

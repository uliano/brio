// mcu: ch32v006k8 ch32v003f4
// The HSE takes 4 to 25 MHz (RM 3.3.2): a 30 MHz crystal must be REFUSED.
#include "ch32v00x/clock.hpp"

using Xtal = brio::Clock<brio::ClockSource::crystal, 30'000'000, 30'000'000>;
void f() { (void)Xtal::init(); }

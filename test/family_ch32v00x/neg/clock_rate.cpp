// mcu: ch32v006k8
// 10 MHz is not the root divided by any HPRE divider: refused.
#include "ch32v00x/clock.hpp"

using Odd = brio::Clock<brio::ClockSource::internal, 10'000'000>;
void f() { (void)Odd::init(); }

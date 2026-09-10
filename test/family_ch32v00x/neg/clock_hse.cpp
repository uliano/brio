// mcu: ch32v006k8
// HSE is named in ClockSource and not implemented: asking for it is a
// compile error with an explanation, not a wrong clock.
#include "ch32v00x/clock.hpp"

using Xtal = brio::Clock<brio::ClockSource::crystal, 24'000'000>;
void f() { (void)Xtal::init(); }

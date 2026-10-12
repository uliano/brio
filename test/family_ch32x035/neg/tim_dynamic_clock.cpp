// mcu: ch32x035f8
// A timer counts HCLK cycles: a DynamicClock would move every period at
// once, so tim_clock_hz() takes a static clock only.
#include "ch32x035/clock.hpp"
#include "ch32x035/tim.hpp"

using Boot = brio::Clock<brio::ClockSource::internal, 48'000'000>;
using Dyn = brio::DynamicClock<Boot>;
uint32_t f() { return brio::tim_clock_hz(Dyn{}); }

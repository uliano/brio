// mcu: ch32v006k8
// A timer counts HCLK and holds its periods in those cycles: a
// DynamicClock must be REFUSED by tim_clock_hz.
#include "ch32v00x/clock.hpp"
#include "ch32v00x/tim.hpp"

using Boot = brio::Clock<brio::ClockSource::pll, 48'000'000>;
using Dyn = brio::DynamicClock<Boot>;
constexpr uint32_t hz = brio::tim_clock_hz(Dyn{});

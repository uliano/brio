// The ring oscillator has no exact rate, so it cannot be a Clock's hz.
#include "rp2350/clock.hpp"
using Bad = brio::Clock<brio::ClockSource::internal, 11'000'000>;
void f() { (void)Bad::init(); }

// The ring oscillator's rate is not a truth: refused as a clock source.
#include "rp2040/clock.hpp"
using Bad = brio::Clock<brio::ClockSource::internal, 6'500'000>;
void f() { (void)Bad::init(); }

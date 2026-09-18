// There are four GPIO clock outputs, GPOUT0..3.
#include "rp2350/clock.hpp"
using Bad = brio::ClockOut<4>;
void f() { (void)Bad::init(brio::GpoutSource::clk_sys, 1u); }

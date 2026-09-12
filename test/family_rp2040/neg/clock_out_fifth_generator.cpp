// Must FAIL: the RP2040 has four GPIO clock outputs (GPOUT0..3 on GP21,
// GP23, GP24, GP25); a fifth is refused at compile time.
#include "rp2040/clock.hpp"

void fifth() { (void)brio::ClockOut<4>::init(brio::GpoutSource::clk_sys, 125); }

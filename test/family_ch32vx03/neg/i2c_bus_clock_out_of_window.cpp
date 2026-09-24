// mcu: ch32v203f6 ch32v203f8 ch32v203g6 ch32v203g8 ch32v203k6 ch32v203k8 ch32v203c6 ch32v203c8 ch32v203rb ch32v303cb ch32v303rb ch32v303rc ch32v303vc
// A BUS CLOCK THE CHAPTER CANNOT STATE: CTLR2.FREQ carries PB1 in whole
// megahertz and RM 19.12.2 confines the field to 4..60 MHz, so the 72
// MHz this stratum allows PB1 to reach has no legal timing at all - the
// peripheral's own ceiling, and a constant that names it must not
// compile.
#include "ch32vx03/i2c.hpp"

inline constexpr brio::I2cTiming too_fast =
    *brio::i2c_timing_for(72'000'000UL, brio::I2cSpeed::standard_100k);
void f() { (void)too_fast.freq_mhz; }

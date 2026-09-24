// mcu: ch32v203f6 ch32v203f8 ch32v203g6 ch32v203g8 ch32v203k6 ch32v203k8 ch32v203c6 ch32v203c8 ch32v203rb ch32v303cb ch32v303rb ch32v303rc ch32v303vc
// A RISE TIME THE REGISTER CANNOT HOLD: RTR is six bits and holds
// (rise x Fpclk) + 1 (RM 19.12.9), so two microseconds of rise at 48
// MHz of PB1 is 97 - past the field - and there is no timing to write.
#include "ch32v203/i2c.hpp"

inline constexpr brio::I2cTiming slow_wire = *brio::i2c_timing_for(
    48'000'000UL, brio::I2cSpeed::standard_100k, brio::I2cDuty::ratio_2, 2000);
void f() { (void)slow_wire.trise; }

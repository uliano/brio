// mcu: samc21e18a samc21g18a samc21j18a
// GAIN's reset value is zero, and a zero GAIN is NOT "no gain": 43.8.13
// makes GAIN "the number of GCLK_TSENS periods that will be used for a
// measurement cycle" and the field is 24 bits, so the value that reads as
// zero behaves as 2^24. Measured on the bench: a 699 ms conversion (2 x
// 2^24 periods at 48 MHz, to the millisecond) whose result is the gain
// term amplified about two hundredfold. 43.5.9 says the production values
// "must be loaded ... to achieve specified accuracy"; the reset state is
// not merely inaccurate, it is a trap.

#include "samc/tsens.hpp"

using namespace brio;

constexpr TsensConfig bad_cfg{};   // the default IS the uncalibrated one

void use() { (void)Tsens::init<bad_cfg>(0); }

// An I2C bus is two lines: a task that names only the clock is not one.
// mcu: stm32f446xx stm32f412zx
#include "stm32f4/fmpi2c.hpp"
using namespace brio;
constexpr FmpI2cPins pins{.scl = {'C', 6, PinFunction::af4}};
using Bus = FmpI2cHost<1, pins>;
void f() { (void)Bus::init(Clock<ClockSource::hsi, 16'000'000>{}); }

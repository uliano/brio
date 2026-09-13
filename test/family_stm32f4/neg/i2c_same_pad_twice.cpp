// SCL and SDA on one pad is not a bus: the second claim would take the
// pad away from the first, silently.
// mcu: stm32f429xx stm32f446xx stm32f411xe
#include "stm32f4/i2c.hpp"
using namespace brio;
constexpr I2cPins pins{.scl = {'B', 6, PinFunction::af4}, .sda = {'B', 6, PinFunction::af4}};
using Bus = I2cHost<1, pins>;
void f() { (void)Bus::init(Clock<ClockSource::hsi, 16'000'000>{}); }

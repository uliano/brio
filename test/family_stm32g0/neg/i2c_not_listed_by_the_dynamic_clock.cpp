// mcu: stm32g0b1xx stm32g071xx stm32g031xx
// An I2cHost initialized with a DynamicClock that does not list it among
// its Users would keep a TIMINGR table solved against a kernel rate that
// has since moved - and TIMINGR is the whole bus: SCL would run at
// whatever the old rate made it, silently.
#include "stm32g0/i2c.hpp"
using namespace brio;
constexpr I2cPins pins{
    .scl = {'B', 8, PinFunction::af6},
    .sda = {'B', 9, PinFunction::af6},
};
using Host = I2cHost<1, pins>;
using Fast = Clock<ClockSource::pll, 64'000'000>;
using Slow = Clock<ClockSource::internal, 16'000'000, PowerRegime::range2>;
using SysClock = DynamicClock<Rates<Fast, Slow>>;   // Host not among the users
void use() { (void)Host::init(SysClock{}); }

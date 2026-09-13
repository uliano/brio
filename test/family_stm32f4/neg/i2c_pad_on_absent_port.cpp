// Port F is bonded on the 144-pin classes; the F411's line has A, B, C,
// D, E and H, so an I2C2 on PF0/PF1 there is refused rather than
// configuring a port the device header does not declare.
// mcu: stm32f411xe
#include "stm32f4/i2c.hpp"
using namespace brio;
constexpr I2cPins pins{.scl = {'F', 1, PinFunction::af4}, .sda = {'F', 0, PinFunction::af4}};
using Bus = I2cHost<2, pins>;
void f() { (void)Bus::init(Clock<ClockSource::hsi, 16'000'000>{}); }

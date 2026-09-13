// An I2C link is two wires: a clock with no data line is not one, and
// unlike an SPI's MISO the data pad has no write-only excuse.
// mcu: stm32f429xx stm32f446xx stm32f411xe
#include "stm32f4/i2c.hpp"
using namespace brio;
constexpr I2cPins pins{.scl = {'B', 6, PinFunction::af4}};
using Bus = I2cHost<1, pins>;
void f() { (void)Bus::init(Clock<ClockSource::hsi, 16'000'000>{}); }

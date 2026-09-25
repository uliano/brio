// No device header of this pack carries a request mapping, so the
// reserve keys the I2C slice on the part class - and a class whose
// reference manual was not read has no table. An engine there is
// REFUSED: this is the same cell that is correct on the F429, the F446,
// the F411 and the F469.
// mcu: stm32f412zx stm32f401xe
#include "stm32f4/dma.hpp"
#include "stm32f4/i2c.hpp"
using namespace brio;
constexpr I2cPins pins{.scl = {'B', 6, PinFunction::af4}, .sda = {'B', 7, PinFunction::af4}};
using Bus = I2cHost<1, pins, DmaTxEngine<1, 6, 1>, DmaRxEngine<1, 0, 1>>;
void f() { (void)Bus::init(Clock<ClockSource::hsi, 16'000'000>{}); }

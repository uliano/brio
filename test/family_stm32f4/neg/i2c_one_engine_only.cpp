// A write-then-read tenure carries bytes both ways inside one bus
// tenure, so a transmit engine with no receive one would leave its read
// phase without a mover.
// mcu: stm32f429xx stm32f446xx stm32f411xe
#include "stm32f4/dma.hpp"
#include "stm32f4/i2c.hpp"
using namespace brio;
constexpr I2cPins pins{.scl = {'B', 6, PinFunction::af4}, .sda = {'B', 7, PinFunction::af4}};
using Bus = I2cHost<1, pins, DmaTxEngine<1, 6, 1>>;
void f() { (void)Bus::init(Clock<ClockSource::hsi, 16'000'000>{}); }

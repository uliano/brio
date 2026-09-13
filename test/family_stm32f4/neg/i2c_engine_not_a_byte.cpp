// This data register is one byte wide and the Request's buffers are
// bytes, so an engine whose element is a half-word would move two bytes
// per request out of a register that holds one.
// mcu: stm32f429xx stm32f446xx stm32f411xe
#include "stm32f4/dma.hpp"
#include "stm32f4/i2c.hpp"
using namespace brio;
constexpr I2cPins pins{.scl = {'B', 6, PinFunction::af4}, .sda = {'B', 7, PinFunction::af4}};
using Bus = I2cHost<1, pins, DmaTxEngine<1, 6, 1, uint16_t>, DmaRxEngine<1, 0, 1, uint16_t>>;
void f() { (void)Bus::init(Clock<ClockSource::hsi, 16'000'000>{}); }

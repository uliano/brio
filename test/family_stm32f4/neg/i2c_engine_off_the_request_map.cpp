// I2C1's transmit request is DMA1 stream 6 or stream 7, both on channel
// 1: stream 6 on channel 4 is USART2_TX's cell and is refused before the
// board is powered rather than sitting on a stream nothing asks.
// mcu: stm32f429xx stm32f446xx stm32f411xe
#include "stm32f4/dma.hpp"
#include "stm32f4/i2c.hpp"
using namespace brio;
constexpr I2cPins pins{.scl = {'B', 6, PinFunction::af4}, .sda = {'B', 7, PinFunction::af4}};
using Bus = I2cHost<1, pins, DmaTxEngine<1, 6, 4>, DmaRxEngine<1, 0, 1>>;
void f() { (void)Bus::init(Clock<ClockSource::hsi, 16'000'000>{}); }

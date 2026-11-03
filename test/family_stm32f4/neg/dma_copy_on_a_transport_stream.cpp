// A copy engine owns its stream through its own tag (no request paces
// it), so a copy on DMA2's stream 7, USART1's transmit stream, is two
// owners on one slot (docs/design/dma.md, "The ownership rule").
// mcu: stm32f429xx stm32f446xx stm32f411xe
#include "stm32f4/dma.hpp"
#include "stm32f4/usart.hpp"
using namespace brio;
constexpr UartPins pins{.tx = {'A', 9, PinFunction::af7}, .rx = {'A', 10, PinFunction::af7}};
using Serial = Uart<1, pins, 64, 256, DmaTxEngine<2, 7, 4>>;
using Copy = DmaCopyEngine<2, 7>;
void f() {
    (void)Serial::init(Clock<ClockSource::hsi, 16'000'000>{}, 9600);
    Copy::arm();
}

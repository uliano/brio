// A peripheral reaches only the (controller, stream, channel) cells the
// request mapping gives it: USART1's transmit request is DMA2 stream 7
// channel 4, so an engine on DMA1 stream 7 channel 4 is refused before
// the board is powered rather than sitting on a stream nothing asks.
// mcu: stm32f429xx stm32f446xx stm32f411xe
#include "stm32f4/dma.hpp"
#include "stm32f4/usart.hpp"
using namespace brio;
constexpr UartPins pins{.tx = {'A', 9, PinFunction::af7}, .rx = {'A', 10, PinFunction::af7}};
using U = Uart<1, pins, 64, 256, DmaTxEngine<1, 7, 4>>;
void f() { (void)U::init(Clock<ClockSource::hsi, 16'000'000>{}, 9600); }

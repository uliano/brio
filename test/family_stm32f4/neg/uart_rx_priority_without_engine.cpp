// THE RECEIVE ENGINE'S LEVEL IS A RECEIVE ENGINE'S. UartOptions::rx_priority
// ranks the stream a receive ring runs on (docs/design/dma.md); on a port
// with no receive engine there is no stream to rank, and a level named
// there must be REFUSED rather than ignored.
// mcu: stm32f429xx stm32f446xx stm32f411xe
#include "stm32f4/usart.hpp"
using namespace brio;
constexpr UartPins pins{.tx = {'A', 9, PinFunction::af7}, .rx = {'A', 10, PinFunction::af7}};
constexpr UartOptions low_ring{.rx_priority = DmaPriority::low};
using U = Uart<1, pins, 64, 256, NoDmaEngine, NoDmaEngine, low_ring>;
void f() { (void)U::init(Clock<ClockSource::hsi, 16'000'000>{}, 9600); }

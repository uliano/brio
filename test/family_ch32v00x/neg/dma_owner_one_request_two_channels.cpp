// mcu: ch32v006k8 ch32v003f4
// One channel a request (the claim's other half, util/dma_owner.hpp): the
// console's transmit engine binds USART1's transmit request to channel 4, so
// a second engine armed for the same request on channel 3 is the request on
// two channels - REFUSED, the error naming USART1's transmit and both
// channels. (Table 8-2 wires the request to channel 4 alone, so no transport
// can do this; a program arming an engine itself can name any owner.)
#include "ch32v00x/clock.hpp"
#include "ch32v00x/dma.hpp"
#include "ch32v00x/platform.hpp"
#include "ch32v00x/usart.hpp"

using Clk = brio::Clock<brio::ClockSource::pll, 48'000'000>;
using Serial = brio::Uart<1, brio::Ch32v00xPlatform<>, 64, 64, brio::DmaTxEngine<4>>;
volatile uint8_t sink;
void f() {
    (void)Serial::init(Clk{}, 115200);
    brio::DmaTxEngine<3>::arm<brio::DmaOwner<brio::DmaPeriph::usart, 1, brio::DmaSignal::tx>>(&sink);
}

// mcu: ch32x035r8 ch32x035c8 ch32x035g8u ch32x035g8r ch32x035f8 ch32x035f7 ch32x033f8
// One channel a request (the claim's other half, util/dma_owner.hpp): the
// console's transmit engine binds USART2's transmit request to channel 7, so
// a second engine armed for the same request on channel 5 is the request on
// two channels - REFUSED, the error naming USART2's transmit and both
// channels. (Figure 9-1 wires the request to channel 7 alone, so no
// transport can do this; a program arming an engine itself can name any
// owner.)
#include "ch32x035/clock.hpp"
#include "ch32x035/dma.hpp"
#include "ch32x035/platform.hpp"
#include "ch32x035/usart.hpp"

using Clk = brio::Clock<brio::ClockSource::internal, 48'000'000>;
using Serial = brio::Uart<2, brio::Ch32x035Platform<>, 64, 64, brio::UartFormat{}, brio::DmaTxEngine<7>>;
volatile uint8_t sink;
void f() {
    (void)Serial::init(Clk{}, 115200);
    brio::DmaTxEngine<5>::arm<brio::DmaOwner<brio::DmaPeriph::usart, 2, brio::DmaSignal::tx>>(&sink);
}

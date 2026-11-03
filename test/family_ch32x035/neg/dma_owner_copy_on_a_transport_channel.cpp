// mcu: ch32x035r8 ch32x035c8 ch32x035g8u ch32x035g8r ch32x035f8 ch32x035f7 ch32x033f8
// One owner a DMA channel: the console's transmit engine owns channel 7 for
// USART2's request, so a copy engine armed on channel 7 in the same image is
// a second owner - REFUSED, the error naming DmaSlotId<1, 7>, the copy's tag
// and USART2's transmit.
#include "ch32x035/clock.hpp"
#include "ch32x035/dma.hpp"
#include "ch32x035/platform.hpp"
#include "ch32x035/usart.hpp"

using Clk = brio::Clock<brio::ClockSource::internal, 48'000'000>;
using Serial = brio::Uart<2, brio::Ch32x035Platform<>, 64, 64, brio::UartFormat{}, brio::DmaTxEngine<7>>;
void f() {
    (void)Serial::init(Clk{}, 115200);
    brio::DmaCopyEngine<7>::arm();
}

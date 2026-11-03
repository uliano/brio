// mcu: ch32v006k8 ch32v003f4
// One owner a DMA channel: the console's transmit engine owns channel 4 for
// USART1's request, so a copy engine armed on channel 4 in the same image is
// a second owner - REFUSED, the error naming DmaSlotId<1, 4>, the copy's tag
// and USART1's transmit.
#include "ch32v00x/clock.hpp"
#include "ch32v00x/dma.hpp"
#include "ch32v00x/platform.hpp"
#include "ch32v00x/usart.hpp"

using Clk = brio::Clock<brio::ClockSource::pll, 48'000'000>;
using Serial = brio::Uart<1, brio::Ch32v00xPlatform<>, 64, 64, brio::DmaTxEngine<4>>;
void f() {
    (void)Serial::init(Clk{}, 115200);
    brio::DmaCopyEngine<4>::arm();
}

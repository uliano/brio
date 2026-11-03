// mcu: ch32v006k8 ch32v003f4
// Whoever programs a channel owns it: the console's receive engine owns
// channel 5 for USART1's receive, so a block programmed by hand on channel 5
// through DmaChannel's verbs (the hand tag, its default owner) is a second
// owner - REFUSED, the error naming DmaSlotId<1, 5>.
#include "ch32v00x/clock.hpp"
#include "ch32v00x/dma.hpp"
#include "ch32v00x/platform.hpp"
#include "ch32v00x/usart.hpp"

using Clk = brio::Clock<brio::ClockSource::pll, 48'000'000>;
using Serial = brio::Uart<1, brio::Ch32v00xPlatform<>, 64, 64, brio::NoDmaEngine, brio::DmaRxEngine<5>>;
uint8_t cell;
void f() {
    (void)Serial::init(Clk{}, 115200);
    brio::DmaChannel<5>::stop();
}

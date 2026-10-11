// mcu: ch32x035r8 ch32x035c8 ch32x035g8u ch32x035g8r ch32x035f8 ch32x035f7 ch32x033f8
// A receive engine runs the whole receive ring as one circular block, and
// the ring's indices wrap by a mask: a ring of 100 bytes is refused.
#include "ch32x035/clock.hpp"
#include "ch32x035/dma.hpp"
#include "ch32x035/platform.hpp"
#include "ch32x035/usart.hpp"

using P = brio::Ch32x035Platform<>;
using Serial = brio::Uart<2, P, 100, 64, brio::UartFormat{}, brio::NoDmaEngine, brio::DmaRxEngine<6>>;
void f() {
    constexpr brio::Clock<brio::ClockSource::internal, 48'000'000> clock;
    (void)Serial::init(clock, 115200);
}

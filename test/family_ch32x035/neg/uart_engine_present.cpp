// mcu: ch32x035r8 ch32x035c8 ch32x035g8u ch32x035g8r ch32x035f8 ch32x035f7 ch32x033f8
// This stratum has no DMA controller driver, so a transport's engine
// slots take NoDmaEngine alone: an engine that says it is present is
// refused.
#include "ch32x035/clock.hpp"
#include "ch32x035/platform.hpp"
#include "ch32x035/usart.hpp"

struct PretendEngine {
    static constexpr bool present = true;
};

using P = brio::Ch32x035Platform<>;
using Serial = brio::Uart<2, P, 64, 64, brio::UartFormat{}, PretendEngine>;
void f() {
    constexpr brio::Clock<brio::ClockSource::internal, 48'000'000> clock;
    (void)Serial::init(clock, 115200);
}

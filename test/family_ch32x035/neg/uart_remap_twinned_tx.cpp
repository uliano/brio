// mcu: ch32x035r8 ch32x035c8 ch32x035g8u ch32x035g8r ch32x033f8
// USART4's code 2 puts its TX on PC16, which shares its package pin with
// PC11 on every part but the CH32X035F8U6 and may not be driven (table
// 2-1, note 4): a transport on that column is refused.
#include "ch32x035/clock.hpp"
#include "ch32x035/platform.hpp"
#include "ch32x035/usart.hpp"

using P = brio::Ch32x035Platform<>;
using Serial = brio::Uart<4, P, 64, 64, brio::UartFormat{}, brio::NoDmaEngine, brio::NoDmaEngine, 2>;
void f() {
    constexpr brio::Clock<brio::ClockSource::internal, 48'000'000> clock;
    (void)Serial::init(clock, 115200);
}

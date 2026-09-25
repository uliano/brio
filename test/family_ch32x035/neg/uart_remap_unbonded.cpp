// mcu: ch32x035g8u ch32x035g8r ch32x035f8 ch32x035f7 ch32x033f8
// USART2's code 1 is PA20/PA19, which only the two LQFP packages bond: a
// transport on that column is refused where its pads are not pins.
#include "ch32x035/clock.hpp"
#include "ch32x035/platform.hpp"
#include "ch32x035/usart.hpp"

using P = brio::Ch32x035Platform<>;
using Serial = brio::Uart<2, P, 64, 64, brio::UartFormat{}, brio::NoDmaEngine, brio::NoDmaEngine, 1>;
void f() {
    constexpr brio::Clock<brio::ClockSource::internal, 48'000'000> clock;
    (void)Serial::init(clock, 115200);
}

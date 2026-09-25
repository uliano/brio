// mcu: ch32x035r8 ch32x035c8 ch32x035g8u ch32x035g8r ch32x035f8 ch32x035f7 ch32x033f8
// M counts the parity bit in the word, so seven data bits exist only with
// a parity bit: a 7N1 transport is refused.
#include "ch32x035/clock.hpp"
#include "ch32x035/platform.hpp"
#include "ch32x035/usart.hpp"

using P = brio::Ch32x035Platform<>;
using Serial = brio::Uart<2, P, 64, 64,
                          brio::UartFormat{brio::UartBits::seven, brio::UartParity::none, brio::UartStop::one}>;
void f() {
    constexpr brio::Clock<brio::ClockSource::internal, 48'000'000> clock;
    (void)Serial::init(clock, 115200);
}

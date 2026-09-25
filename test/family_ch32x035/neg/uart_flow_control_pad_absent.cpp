// mcu: ch32x035g8u ch32x035g8r ch32x035f8
// USART4's default column puts RTS on PA8, which the 28-pin and QFN20
// packages do not bond: the flow-control pair on that column is refused
// there.
#include "ch32x035/clock.hpp"
#include "ch32x035/platform.hpp"
#include "ch32x035/usart.hpp"

using P = brio::Ch32x035Platform<>;
using Serial = brio::Uart<4, P, 64, 64, brio::UartFormat{}, brio::NoDmaEngine, brio::NoDmaEngine, 0,
                          brio::UartOptions{false, true, false}>;
void f() {
    constexpr brio::Clock<brio::ClockSource::internal, 48'000'000> clock;
    (void)Serial::init(clock, 115200);
}

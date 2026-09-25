// mcu: ch32x035r8 ch32x035c8 ch32x035g8u ch32x035g8r ch32x035f8 ch32x035f7 ch32x033f8
// A USART's divisor counts HCLK, so under a dynamic clock the port must be
// among the users the clock rebases: one that is not is refused.
#include "ch32x035/clock.hpp"
#include "ch32x035/platform.hpp"
#include "ch32x035/ticker.hpp"
#include "ch32x035/usart.hpp"

using P = brio::Ch32x035Platform<>;
using Boot = brio::Clock<brio::ClockSource::internal, 48'000'000>;
using SysClock = brio::DynamicClock<Boot, brio::Ticker>;
using Serial = brio::Uart<2, P>;
void f() {
    constexpr SysClock clock;
    (void)Serial::init(clock, 115200);
}

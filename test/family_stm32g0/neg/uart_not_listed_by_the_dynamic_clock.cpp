// mcu: stm32g0b1xx stm32g071xx stm32g031xx
// A Uart on PCLK initialized with a DynamicClock that does not list it
// would keep its old baud after a switch: clock_follows refuses it at
// init(clock).
#include "stm32g0/clock.hpp"
#include "stm32g0/usart.hpp"
using namespace brio;
constexpr UartPins u2{.tx = {'A', 2, PinFunction::af1}, .rx = {'A', 3, PinFunction::af1}};
using Console = Uart<2, u2>;
struct Other { static void rebase(uint32_t) {} };
using Dyn = DynamicClock<Rates<Clock<ClockSource::pll, 64'000'000>>, Other>;
bool f() { return Console::init(Dyn{}, 115200); }

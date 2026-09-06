// mcu: stm32g0b1xx stm32g071xx stm32g031xx stm32g030xx
// SysTick's reload is a function of the CPU rate: a DynamicClock that
// does not list the counter (or the ticker) among its users is refused
// where the counter is started.
#include "stm32g0/clock.hpp"
#include "stm32g0/ticker.hpp"
using namespace brio;
struct Other { static void rebase(uint32_t) {} };
using Dyn = DynamicClock<Rates<Clock<ClockSource::pll, 64'000'000>>, Other>;
bool f() { return SysTickCounter::start(Dyn{}); }

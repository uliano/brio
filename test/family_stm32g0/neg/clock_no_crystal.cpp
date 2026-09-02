// mcu: stm32g0b1xx stm32g071xx stm32g031xx
// Only internal and pll are implemented; the declared-but-unbuilt roots
// must be a compile error, never a silently wrong clock.
#include "stm32g0/clock.hpp"
using namespace brio;
void f() { (void)Clock<ClockSource::crystal, 8'000'000>::init(); }

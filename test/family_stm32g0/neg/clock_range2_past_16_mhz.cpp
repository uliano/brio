// mcu: stm32g0b1xx stm32g071xx stm32g031xx stm32g030xx
// VCORE Range 2 serves SYSCLK up to 16 MHz (RM0444 4.1.4): a faster
// rate in that regime is refused where the rate is spelled.
#include "stm32g0/clock.hpp"
using Bad = brio::Clock<brio::ClockSource::pll, 32'000'000, brio::PowerRegime::range2>;
bool f() { return Bad::init(); }

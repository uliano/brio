// mcu: stm32g0b1xx stm32g071xx stm32g031xx stm32g030xx
// Low-power run wants SYSCLK at or below 2 MHz (RM0444 4.3.2): a rate
// above it in that regime is refused where the rate is spelled.
#include "stm32g0/clock.hpp"
using Bad = brio::Clock<brio::ClockSource::internal, 4'000'000, brio::PowerRegime::low_power_run>;
bool f() { return Bad::init(); }

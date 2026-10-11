// mcu: stm32g0b1xx stm32g071xx stm32g031xx
// The tracker refuses a timer clock under 8 MHz: its input filter's six
// samples at fDTS / 8 would outlast half an LSE period.
#include "stm32g0/clock.hpp"
#include "stm32g0/lse_tracker.hpp"

using SlowClock = brio::Clock<brio::ClockSource::internal, 4'000'000>;

void refused() { (void)brio::LseTracker<1, 2>::start(SlowClock{}); }

// delay family smoke TU: the SysTick microsecond busy-wait. There is
// nothing package-dependent in it (SysTick is the core's), so this TU
// only proves the header compiles everywhere and the conversion folds
// with a compile-time Clock.
#include "samc/delay.hpp"

using namespace brio;

using SysClock = Clock<ClockSource::internal, 48'000'000>;

void use() {
    constexpr SysClock clock;
    (void)delay_us(clock, 30u);
    uint32_t runtime_us = 100;
    (void)delay_us(clock, runtime_us);
}

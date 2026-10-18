// mcu: avr64ea48 avr64ea28
// The window compares RESULT[15:0]: an unscaled sum of 32 samples is
// 17 bits wide, so a window on RESULT is refused at compile time.
#include "avrxt/adc.hpp"
#include "avrxt/clock.hpp"
using namespace brio;
void f() {
    constexpr Clock<ClockSource::internal, 20'000'000> clk;
    (void)Adc<0>::init<AdcConfig{.accumulate = 32, .window = AdcWindow::inside}>(clk);
}

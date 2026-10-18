// mcu: avr64ea48
// Sign chopping works in the accumulating modes only (DS40002443A
// 31.3.3.5.2, 31.5.10).
#include "avrxt/adc.hpp"
#include "avrxt/clock.hpp"
using namespace brio;
void f() {
    constexpr Clock<ClockSource::internal, 20'000'000> clk;
    (void)Adc<0>::init<AdcConfig{.chopping = true}>(clk);
}

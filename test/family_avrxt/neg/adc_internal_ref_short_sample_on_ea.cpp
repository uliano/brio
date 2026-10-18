// mcu: avr64ea48 avr64ea28
// With an internal reference and no PGA, SAMPDUR >= 4 us x fCLK_ADC
// (DS40002443A 31.3.3.6): at 2 MHz that is 8, and the default 2 is
// refused for a static clock.
#include "avrxt/adc.hpp"
#include "avrxt/clock.hpp"
using namespace brio;
void f() {
    constexpr Clock<ClockSource::internal, 20'000'000> clk;
    (void)Adc<0>::init<AdcConfig{.reference = Ref::v2048}>(clk);
}

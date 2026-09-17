// mcu: ch32v203rb
// THE SIX CHANNELS ABOVE NINE, on the one part that bonds them. The pad
// map is the family's - ADC_IN10..15 are PC0..PC5 - but only the 64-pin
// package brings port C's low pins out, which is why this TU is its own
// and why `AnalogIn<Pin<'C', 0>>` on any other part is a compile error
// (the neg TU beside it).
//
// The channels are the second half of what datasheet table 2-1's
// "16@1" says: sixteen channels on ONE converter, where every part
// below has nine or ten on two. So this is also where the sample-time
// register a smaller part never touches gets written - SAMPTR1 holds
// SMP10 upward, SAMPTR2 the rest.
#include "ch32v203/adc.hpp"
#include "ch32v203/platform.hpp"

using namespace brio;

using SysClock = Clock<ClockSource::pll, 96'000'000>;

static_assert(device::adc_channel_count == 16u);
static_assert(device::adc_count == 1u);

using In10 = AnalogIn<Pin<'C', 0>>;
using In15 = AnalogIn<Pin<'C', 5>>;
static_assert(In10::channel == 10u);
static_assert(In15::channel == 15u);
static_assert(Adc<1>::input_code(In10{}) == 10u);

void adc_high_channels() {
    SysClock clock;
    (void)Adc<1>::init(clock);
    In10::claim();
    In15::claim();
    // SAMPTR1's own half of the ladder: channels 10 and above.
    (void)Adc<1>::sample_time(10, AdcSampleTime::cycles7_5);
    (void)Adc<1>::sample_time(15, AdcSampleTime::cycles239_5);
    (void)Adc<1>::sample_time(15);
    Adc<1>::select(In10{});
    Adc<1>::select(In15{});
    const uint8_t order[6] = {10, 11, 12, 13, 14, 15};
    (void)Adc<1>::sequence(order, 6);
    (void)Adc<1>::watchdog(AdcWatchdogConfig{.channel = 15});
    In10::release();
}

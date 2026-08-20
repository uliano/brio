// VREF/DAC/ADC family smoke TU. The DB-only internal inputs
// (vdd_div10/vddio2_div10) are gated by the MVIO marker: on the DA
// they do not exist at all (the mux codes are reserved there).
#include "avrdx/adc.hpp"
#include "avrdx/dac.hpp"
#include "avrdx/vref.hpp"
#include "avrdx/clock.hpp"

using namespace brio;

using Boot = Clock<ClockSource::internal, 24'000'000>;
using Dyn = DynamicClock<Boot, Adc<0>>;

void analog_common() {
    constexpr Boot clk;
    (void)Adc<0>::init<AdcConfig{.reference = Ref::v2048, .prescaler = AdcPresc::div16,
                                 .accumulate = 4, .debug_run = true}>(clk);
    (void)Adc<0>::init(Dyn{}, AdcConfig{.reference = Ref::v2048, .prescaler = AdcPresc::div16});
    Adc<0>::select(AdcInput::dacref0);
    (void)Adc<0>::select(AdcInput::gnd, AdcInput::dac0);
    (void)Adc<0>::select(AdcInput::temp, AdcInput::temp);   // refused at run time (false)
    Adc<0>::window_signed(Adc<0>::Window::below, -500, 500);
    Adc<0>::rebase(12'000'000);
    (void)Adc<0>::clock_ok();
    static_assert(!adc_neg_valid(AdcInput::temp));
    static_assert(adc_neg_valid(AdcInput::dac0));
    Dac<0>::init({.reference = Ref::v2048});
    Dac<0>::set(512);
    (void)Dac<0>::code();
    Vref::ac(Ref::v2048, true);
#ifdef MVIO
    Adc<0>::select(AdcInput::vdd_div10);       // DB only
#endif
}

// VREF/DAC/ADC family smoke TU, one half per ADC engine. The DB-only
// internal inputs (vdd_div10/vddio2_div10) are gated by the MVIO marker:
// on the DA they do not exist at all (the mux codes are reserved there).
// The AVR EA's half instantiates the differential ADC's surface: the
// pairs, the PGA, the accumulation modes, the window, the paces, the
// configured form the AnalogSampler checks.
#include <avr/io.h>
#if !defined(ADC_PGAEN_bm)
#include "avrxt/adc.hpp"
#include "avrxt/dac.hpp"
#include "avrxt/vref.hpp"
#include "avrxt/clock.hpp"

using namespace brio;

using Boot = Clock<ClockSource::internal, has_oschf_frqsel ? 24'000'000 : 20'000'000>;
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

#else // ADC_PGAEN_bm: the AVR EA

#include "avrxt/adc.hpp"
#include "avrxt/dac.hpp"
#include "avrxt/vref.hpp"
#include "avrxt/clock.hpp"
#include "kernel/platform.hpp"
#include "util/analog_sampler.hpp"
#include "avrxt/platform.hpp"

using namespace brio;

using Boot = Clock<ClockSource::internal, 20'000'000>;
using Dyn = DynamicClock<Boot, Adc<0>>;
using A = Adc<0>;
using Pd1 = AnalogIn<Pin<'D', 1>>;
using Pc3 = AnalogIn<Pin<'C', 3>>;         // AIN31, a negative too on this converter
using Pa2 = AnalogIn<Pin<'A', 2>>;         // AIN22

constexpr AdcConfig scaled_burst{.reference = Ref::v2048, .sample_duration = 8, .accumulate = 256,
                                 .scaled = true, .window = AdcWindow::inside};
constexpr AdcConfig pga_burst{.reference = Ref::vdd, .prescaler = AdcPresc::div4, .sample_duration = 11,
                              .accumulate = 16, .pga = true, .gain = AdcGain::x16, .chopping = true};
constexpr AdcConfig series_sum{.reference = Ref::vdd, .accumulate = 1024, .series = true,
                               .window = AdcWindow::below, .window_source = AdcWindowSource::sample};

void analog_common() {
    constexpr Boot clk;
    (void)A::init<AdcConfig{.reference = Ref::v2048, .sample_duration = 8, .debug_run = true}>(clk);
    (void)A::init<scaled_burst>(clk);
    (void)A::init<pga_burst>(clk);
    (void)A::init<series_sum>(clk);
    (void)A::init(Dyn{}, AdcConfig{.reference = Ref::v4096, .sample_duration = 8, .low_latency = true});
    A::claim(AdcDiff<Pd1{}, Pc3{}>{});
    A::select(Pd1{});
    A::select(AdcInput::vdd_div10);
    A::select(AdcDiff<Pd1{}, AdcInput::gnd>{});
    A::select(AdcDiff<Pa2{}, AdcDacref::dacref1>{});
    A::select(AdcDiff<AdcInput::dac0, AdcInput::gnd>{});
    static_assert(A::input_code(AdcDiff<Pd1{}, Pc3{}>{}) == (1 | adc_diff_flag));
    static_assert(Pc3::code == 31 && Pa2::code == 22);
    (void)A::read();
    (void)A::read_sum();
    (void)A::sum_signed();
    (void)A::sample();
    (void)A::window_signed(A::Window::below, -500, 500);
    A::window_thresholds(100, 200);
    (void)A::error();
    A::start_on(EventChannel<2>{});
    A::start_on_select(true);
    A::rebase(10'000'000);
    (void)A::clock_ok();
    (void)A::temp_kelvin(2000);
    static_assert(adc_result_bits(series_sum) == 22 && adc_result_bits(scaled_burst) == 16);
    static_assert(!adc_negative_ok(AdcInput::temp) && adc_negative_ok(AdcDacref::dacref0));
    static_assert(!adc_config_valid(AdcConfig{.accumulate = 64, .window = AdcWindow::above}));
    static_assert(!adc_config_valid(AdcConfig{.resolution = AdcRes::bits8, .accumulate = 4}));
    static_assert(!adc_clock_valid(20'000'000, AdcConfig{.reference = Ref::v1024}));   // SAMPDUR 2 < 8
    static_assert(adc_period_cycles(20'000'000, AdcConfig{.prescaler = AdcPresc::div4,
                                                          .sample_duration = 20}) == 144);
    Dac<0>::init({.reference = Ref::v2048, .range = DacRange::full});
    Dac<0>::set(512);
    Dac<0>::disable();
    Dac<0>::enable();
    Vref::ac(Ref::v2048, true);
}

// The configured form in a sampler: a scaled sum fits Sampled's 16 bits.
using Conv = AdcConfigured<0, scaled_burst>;
using Sampler = AnalogSampler<Conv, AvrPlatform, Subscribers<>, Pd1{}, AdcInput::temp,
                              AdcDiff<Pd1{}, AdcInput::dac0>{}>;
void sampler_common() {
    (void)Conv::init(Boot{});
    Sampler::init();
}

#endif // ADC_PGAEN_bm

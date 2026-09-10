// ADC family smoke TU: ch32v00x/adc.hpp's converter and every pad it
// takes, under util/analog_sampler.hpp's AnalogSampler - instantiation
// only, no main(), no hardware. The arithmetic is pinned in the header;
// what this fixture adds is the util contract: Adc is an AnalogConverter
// and each AnalogIn / AdcInput a SamplerInput.
#include "ch32v00x/adc.hpp"
#include "ch32v00x/clock.hpp"
#include "ch32v00x/platform.hpp"
#include "util/analog_sampler.hpp"

using namespace brio;

using P = Ch32v00xPlatform<>;
using SysClock = Clock<ClockSource::pll, 48'000'000>;

using In3 = AnalogIn<Pin<'D', 2>>;
using In7 = AnalogIn<Pin<'D', 4>>;
static_assert(In3::channel == 3 && In7::channel == 7);
static_assert(AnalogConverter<Adc>);
static_assert(SamplerInput<Adc, In3{}> && SamplerInput<Adc, AdcInput::vrefint>);
static_assert(Adc::input_code(In7{}) == 7 && Adc::input_code(AdcInput::opa) == 9);

struct Nobody {};
using Sampler = AnalogSampler<Adc, P, Subscribers<>, In3{}, AdcInput::vrefint, In7{}>;

void converter_verbs() {
    constexpr SysClock clock;
    (void)Adc::init(clock, {.prescaler_code = adc_prescaler_for(48'000'000, 24'000'000)->code,
                            .continuous = true, .scan = true, .low_power = false, .input_buffer = true, .dma = true});
    (void)Adc::adcclk_hz(clock);
    (void)Adc::powered(); Adc::power(false); Adc::power(true);
    (void)Adc::sample_time(3, AdcSampleTime::cycles28_5); (void)Adc::sample_time(3);
    Adc::sample_time_all(AdcSampleTime::cycles239_5);
    (void)Adc::conversion_half_cycles(8);
    const uint8_t order[3] = {3, 8, 7};
    (void)Adc::sequence(order, 3); (void)Adc::sequence_length();
    Adc::select(In3{}); Adc::select(AdcInput::vrefint); Adc::select_channel(9);
    (void)Adc::selected();
    Adc::start(); (void)Adc::converting(); (void)Adc::ready();
    (void)Adc::result(); (void)Adc::result_counts();
    uint16_t v;
    (void)Adc::read(v); (void)Adc::read(); (void)Adc::read_settled(4);
    (void)Adc::supply_mv(1489); (void)Adc::millivolts(2048, 3300);
    Adc::trigger(AdcTrigger::tim1_trgo); (void)Adc::trigger();
    Adc::injected_trigger(AdcInjectedTrigger::tim3_cc1); Adc::injected_trigger_enable(false);
    Adc::continuous(false); Adc::dma(false);
    (void)Adc::injected_sequence(order, 2); (void)Adc::injected_offset(3, 100);
    Adc::injected_start(); (void)Adc::injected_ready(); (void)Adc::injected_result(3);
    (void)Adc::watchdog0({.low = 100, .high = 4000, .channel = 8, .interrupt = true, .reset_on_fault = true});
    Adc::watchdog0_off();
    (void)Adc::watchdog(1, 0, 4095, true); (void)Adc::watchdog_result(1); Adc::clear_watchdog_result(2);
    Adc::watchdog_scan(true);
    (void)Adc::flags(); (void)Adc::flag(AdcFlag::converted); Adc::clear_flags(AdcFlag::all);
    Adc::interrupts(Adc::converted_interrupt | Adc::injected_interrupt | Adc::watchdog_interrupt, true);
    (void)Adc::isr();
    (void)Adc::config();
    (void)Adc::data_address();
    In3::claim(); In3::release();
    Adc::release();
}

void sampler() {
    Sampler::init();
    Sampler::dispatch(Sampled{100, 3});
    Sampler::dispatch(SamplerTick{});
}

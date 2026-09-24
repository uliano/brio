// ADC family smoke TU: the converter every part has, the channel map,
// the sample times and their arithmetic, both groups, the watchdog, the
// triggers, and the converter surface util/analog_sampler.hpp asks for.
//
// TWO PART FACTS DIVIDE THIS FAMILY HERE, and they are the whole reason
// the sweep exists: HOW MANY CONVERTERS (two up to the CH32V203C8, ONE
// on the 128 KB part, which spends the second unit on six more
// channels) and WHICH PADS a package bonds. Neither is in this file -
// adc_dual.cpp is the second converter's, on the parts that have it,
// and adc_channels16.cpp the six port-C channels', on the part that
// bonds them - so what is here compiles for all nine.
//
// A CLOCK THE CONVERTER CAN LIVE ON is the third thing every part
// shares: PCLK2 is HCLK here, ADCPRE divides by 8 at most, and the
// converter is rated at 14 MHz - so the TU instantiates init() on a
// 96 MHz tree (ADCCLK 12 MHz) and a neg TU proves the 144 MHz one is
// refused on the line that asked.
#include "ch32v203/adc.hpp"
#include "ch32v203/dma_engine.hpp"
#include "ch32v203/platform.hpp"
#include "util/analog_sampler.hpp"

using namespace brio;

using P = Ch32v203Platform<>;
/// The tree this family's converter is in specification on.
using SysClock = Clock<ClockSource::pll, 96'000'000>;
static_assert(SysClock::adc_in_spec);
static_assert(SysClock::adc_hz == 12'000'000UL);
static_assert(SysClock::pclk2_hz == 96'000'000UL);
/// And the one it is not: the rate this stratum's own console runs at.
static_assert(!Clock<ClockSource::pll, 144'000'000>::adc_in_spec);
static_assert(Clock<ClockSource::pll, 144'000'000>::adc_hz == 18'000'000UL);

// ---- the converter's shape ------------------------------------------------
static_assert(adc_bits == 12 && adc_steps == 4096u && adc_max_count == 4095u);
static_assert(adc_channels == 18 && adc_pad_channels == 16);
static_assert(adc_temperature_channel == 16 && adc_vrefint_channel == 17);
static_assert(adc_regular_slots == 16 && adc_injected_slots == 4);
static_assert(sizeof(AdcRegs) == 0x58);
static_assert(adc_base_for(1) == 0x40012400UL && adc_base_for(2) == 0x40012800UL);
static_assert(adc_base_for(3) == 0u);

// ---- the sampling times (12.3.4) and what they cost ------------------------
static_assert(adc_sample_half_cycles(AdcSampleTime::cycles1_5) == 3u);
static_assert(adc_sample_half_cycles(AdcSampleTime::cycles28_5) == 57u);
static_assert(adc_sample_half_cycles(AdcSampleTime::cycles239_5) == 479u);
static_assert(adc_conversion_half_cycles(AdcSampleTime::cycles1_5) == 25u);
static_assert(adc_conversion_half_cycles(AdcSampleTime::cycles239_5) == 501u);
static_assert(adc_conversion_ns(AdcSampleTime::cycles1_5, 12'000'000UL) == 1041u);
static_assert(adc_conversion_ns(AdcSampleTime::cycles239_5, 12'000'000UL) == 20'875u);
static_assert(adc_conversion_ns(AdcSampleTime::cycles1_5, 0) == 0u);
// Datasheet table 4-28: the source impedance each sampling time settles.
static_assert(adc_max_source_ohms(AdcSampleTime::cycles1_5) == 400u);
static_assert(adc_max_source_ohms(AdcSampleTime::cycles13_5) == 11'400u);
static_assert(adc_max_source_ohms(AdcSampleTime::cycles55_5) == 50'000u);
static_assert(adc_max_source_ohms(AdcSampleTime::cycles71_5) == 0u);
static_assert(adc_sample_shortest == AdcSampleTime::cycles1_5);
static_assert(adc_sample_longest == AdcSampleTime::cycles239_5);

// ---- the pad map (the family's; the package decides which exist) -----------
static_assert(adc_channel_of(Pad{'A', 0}) == 0u);
static_assert(adc_channel_of(Pad{'A', 7}) == 7u);
static_assert(adc_channel_of(Pad{'B', 0}) == 8u);
static_assert(adc_channel_of(Pad{'B', 1}) == 9u);
static_assert(adc_channel_of(Pad{'C', 0}) == 10u);
static_assert(adc_channel_of(Pad{'C', 5}) == 15u);
static_assert(adc_channel_of(Pad{'A', 8}) == 0xFFu);
static_assert(adc_channel_of(Pad{'B', 2}) == 0xFFu);
static_assert(adc_channel_of(Pad{'C', 13}) == 0xFFu);
static_assert(adc_channel_of(Pad{'D', 0}) == 0xFFu);
static_assert(adc_channel_of(Pad{}) == 0xFFu);

// ---- the reference, which is the supply on this family ---------------------
static_assert(ref_mv(Ref::vdda) == 3300u);
static_assert(ref_mv(Ref::vdda, 3000) == 3000u);
static_assert(adc_mv(2048, adc_steps, ref_mv(Ref::vdda)) == 1650u);
static_assert(adc_vrefint_mv == 1200u);
static_assert(adc_temperature_v25_mv == 1400u);
static_assert(adc_temperature_slope_uv_per_c == 4300u);

// ---- the configurations the chapter refuses --------------------------------
static_assert(adc_config_valid(AdcConfig{}));
static_assert(adc_config_valid(AdcConfig{.scan = true, .discontinuous = 8}));
static_assert(!adc_config_valid(AdcConfig{.discontinuous = 9}));
static_assert(!adc_config_valid(AdcConfig{.discontinuous = 3, .injected_discontinuous = true}));
static_assert(!adc_config_valid(AdcConfig{.auto_injected = true, .injected_discontinuous = true}));
static_assert(!adc_config_valid(AdcConfig{.gain = AdcGain::x16}));
static_assert(adc_config_valid(AdcConfig{.input_buffer = true, .gain = AdcGain::x16}));
static_assert(adc_gain_factor(AdcGain::x1) == 1u && adc_gain_factor(AdcGain::x4) == 4u);
static_assert(adc_gain_factor(AdcGain::x16) == 16u && adc_gain_factor(AdcGain::x64) == 64u);

// ---- the watchdog ----------------------------------------------------------
static_assert(adc_watchdog_config_valid(AdcWatchdogConfig{}));
static_assert(adc_watchdog_config_valid(AdcWatchdogConfig{.low = 100, .high = 100}));
static_assert(!adc_watchdog_config_valid(AdcWatchdogConfig{.low = 101, .high = 100}));
static_assert(!adc_watchdog_config_valid(AdcWatchdogConfig{.high = 4096}));
static_assert(!adc_watchdog_config_valid(AdcWatchdogConfig{.channel = 18}));
static_assert(adc_watchdog_config_valid(AdcWatchdogConfig{.channel = 17}));

// ---- the triggers: code 110 is EXTI's line here, not TIM8's ----------------
static_assert(static_cast<uint8_t>(AdcTrigger::tim1_cc1) == 0u);
static_assert(static_cast<uint8_t>(AdcTrigger::tim3_trgo) == 4u);
static_assert(static_cast<uint8_t>(AdcTrigger::exti11) == 6u);
static_assert(static_cast<uint8_t>(AdcTrigger::software) == 7u);
static_assert(static_cast<uint8_t>(AdcInjectedTrigger::tim1_trgo) == 0u);
static_assert(static_cast<uint8_t>(AdcInjectedTrigger::exti15) == 6u);
static_assert(static_cast<uint8_t>(AdcInjectedTrigger::software) == 7u);
static_assert(adc_regular_exti_line == 11 && adc_injected_exti_line == 15);

// ---- the dual modes --------------------------------------------------------
static_assert(adc_dual_mode_valid(AdcDualMode::independent));
static_assert(adc_dual_mode_valid(AdcDualMode::alternate_trigger));
static_assert(!adc_dual_mode_valid(static_cast<AdcDualMode>(10)));
static_assert(static_cast<uint8_t>(AdcDualMode::regular_simultaneous) == 6u);

// ---- what a part decides ---------------------------------------------------
static_assert(Adc<1>::instance == 1);
static_assert(Adc<1>::has_dma && Adc<1>::has_internal_sources);
static_assert(Adc<1>::dma_channel == dma_request_channel(DmaRequest::adc1));
static_assert(Adc<1>::irq() == Irq::adc1_2);
static_assert(Adc<1>::has_dual_mode == (device::adc_count >= 2u));
// The datasheets' tables read back: nine, ten or sixteen bonded
// channels. On the CH32V203 the sixteen-channel part is the one with a
// single converter; every CH32V303 has two, with sixteen channels on
// every package but the LQFP48.
static_assert(device::adc_channel_count == 9u || device::adc_channel_count == 10u ||
              device::adc_channel_count == 16u);
static_assert(device::device_class == DeviceClass::v30x_d8 ||
              (device::adc_channel_count == 16u) == (device::adc_count == 1u));
static_assert(device::device_class != DeviceClass::v30x_d8 || device::adc_count == 2u);

// ---- the inputs the sampler walks ------------------------------------------
// PA1 is ADC_IN1 and every package of the series bonds it (the smallest
// one has PA0..PA7), so it is the pad this TU may name outright.
using Vin = AnalogIn<Pin<'A', 1>>;
static_assert(Vin::channel == 1u);
static_assert(Adc<1>::input_code(Vin{}) == 1u);
static_assert(Adc<1>::input_code(AdcInput::temperature) == 16u);
static_assert(Adc<1>::input_code(AdcInput::vrefint) == 17u);
static_assert(AnalogConverter<Adc<1>>);
static_assert(SamplerInput<Adc<1>, Vin{}>);
static_assert(SamplerInput<Adc<1>, AdcInput::vrefint>);

/// One subscriber, so the sampler's publish() has somewhere to go.
struct Monitor : Fsm<Monitor, AnalogSample> {
    static inline EventQueue<Event, 4, P> queue;
    static inline uint32_t seen = 0;

    static void init() { start(&only); }
    static void dispatch(const Event& e) { Fsm::dispatch(e); }

    static Status only(const Event& e) {
        return match(e,
            [](Entry) { return handled(); },
            [](Exit) { return handled(); },
            [](AnalogSample s) { seen += s.value; return handled(); });
    }
};

using Subs = Subscribers<Monitor>;
using Sampler = AnalogSampler<Adc<1>, P, Subs, Vin{}, AdcInput::vrefint, AdcInput::temperature>;
static_assert(Sampler::input_count == 3);
static_assert(ActiveObject<Sampler>);

// ---- every verb of the resource --------------------------------------------
void adc_verbs() {
    SysClock clock;
    Vin::claim();
    Vin::release();

    (void)Adc<1>::init(clock);
    (void)Adc<1>::init(clock, AdcConfig{.scan = true,
                                        .dma = true,
                                        .input_buffer = true,
                                        .gain = AdcGain::x4,
                                        .internal_sources = true});
    Adc<1>::bus_clock(true);
    (void)Adc<1>::bus_clock();
    Adc<1>::reset();
    (void)Adc<1>::calibrate();
    (void)Adc<1>::calibrate(1000);
    (void)Adc<1>::calibration_code();
    (void)Adc<1>::calibrating();
    Adc<1>::power(true);
    (void)Adc<1>::powered();
    (void)Adc<1>::regs().STATR;
    (void)Adc<1>::data_address();

    (void)Adc<1>::sample_time(0, AdcSampleTime::cycles7_5);
    (void)Adc<1>::sample_time(17, AdcSampleTime::cycles239_5);
    (void)Adc<1>::sample_time(18, AdcSampleTime::cycles1_5);
    (void)Adc<1>::sample_time(0);
    (void)Adc<1>::sample_time(17);
    Adc<1>::sample_time_all(adc_sample_longest);
    (void)Adc<1>::conversion_half_cycles(1);

    const uint8_t order[4] = {1, 16, 17, 1};
    (void)Adc<1>::sequence(order, 4);
    (void)Adc<1>::sequence(nullptr, 1);
    (void)Adc<1>::sequence(order, 0);
    (void)Adc<1>::sequence_length();
    (void)Adc<1>::sequence_channel(0);
    (void)Adc<1>::sequence_channel(16);

    Adc<1>::select(Vin{});
    Adc<1>::select(AdcInput::vrefint);
    Adc<1>::select_channel(3);
    (void)Adc<1>::selected();
    Adc<1>::start();
    (void)Adc<1>::converting();
    (void)Adc<1>::ready();
    (void)Adc<1>::data();
    (void)Adc<1>::result();
    (void)Adc<1>::result_counts();
    (void)Adc<1>::follower_counts();
    uint16_t counts = 0;
    (void)Adc<1>::read(counts);
    (void)Adc<1>::read();
    (void)Adc<1>::read_settled(2);

    (void)Adc<1>::vdda_mv(1490);
    (void)Adc<1>::vdda_mv(0);
    (void)Adc<1>::millivolts(2048, 3300);
    (void)Adc<1>::temperature_centi_c(1737, 3300);
    (void)Adc<1>::temperature_centi_c(1737, 0);

    Adc<1>::trigger(AdcTrigger::tim3_trgo);
    Adc<1>::trigger(AdcTrigger::exti11);
    (void)Adc<1>::trigger();
    Adc<1>::trigger_enable(false);
    Adc<1>::injected_trigger(AdcInjectedTrigger::tim1_trgo);
    (void)Adc<1>::injected_trigger();
    Adc<1>::injected_trigger_enable(true);
    Adc<1>::continuous(true);
    (void)Adc<1>::continuous();
    (void)Adc<1>::dma(true);
    (void)Adc<1>::dma();
    (void)Adc<1>::internal_sources(true);
    (void)Adc<1>::internal_sources();
    (void)Adc<1>::gain(AdcGain::x16);
    (void)Adc<1>::gain(AdcGain::x16, false);
    (void)Adc<1>::gain();
    (void)Adc<1>::input_buffer();

    const uint8_t injected[3] = {1, 16, 17};
    (void)Adc<1>::injected_sequence(injected, 3);
    (void)Adc<1>::injected_sequence(injected, 5);
    (void)Adc<1>::injected_length();
    (void)Adc<1>::injected_offset(0, 100);
    (void)Adc<1>::injected_offset(4, 100);
    (void)Adc<1>::injected_offset(0);
    Adc<1>::injected_start();
    (void)Adc<1>::injected_ready();
    (void)Adc<1>::injected_converting();
    (void)Adc<1>::injected_result(0);
    (void)Adc<1>::injected_result(4);
    int16_t signed_counts = 0;
    (void)Adc<1>::injected_read(signed_counts);

    (void)Adc<1>::watchdog(AdcWatchdogConfig{.low = 100,
                                             .high = 3000,
                                             .channel = 1,
                                             .injected_group = true,
                                             .interrupt = true});
    Adc<1>::watchdog_thresholds(0, adc_max_count);
    (void)Adc<1>::watchdog_low();
    (void)Adc<1>::watchdog_high();
    (void)Adc<1>::watchdog_channel();
    Adc<1>::watchdog_off();

    // The dual modes are adc_dual.cpp's: on the part with one converter
    // `dual()` does not compile, which is the point of that file.
    (void)Adc<1>::dual();

    (void)Adc<1>::flags();
    (void)Adc<1>::flag(AdcFlag::converted);
    Adc<1>::clear_flags(AdcFlag::all);
    Adc<1>::interrupts(Adc<1>::converted_interrupt | Adc<1>::watchdog_interrupt, true);
    (void)Adc<1>::interrupts();
    (void)Adc<1>::isr();
    (void)Adc<1>::config().scan;
    Adc<1>::release();
}

/// The sampler over this converter: init, both paces, the ISR glue's
/// event. Instantiation only.
void adc_sampler() {
    Monitor::init();
    Sampler::init();
    Sampler::start_every(100);
    (void)Sampler::running_every();
    Sampler::dispatch(Sampler::Event{Sampled{1234, Adc<1>::selected()}});
    Monitor::dispatch(Monitor::Event{AnalogSample{0, 1234}});
    Sampler::stop();
    (void)Sampler::unknown_inputs();
}

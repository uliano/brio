// Family smoke TU for samc/adc.hpp: every verb must COMPILE on the E, G
// and J 18A headers (tools/check_samc.sh sweeps all three). The two
// converters are identical at the register level; what differs across
// the family is PAD BONDING, and it differs a lot - the E bonds NO PORT
// B pad to either ADC, which leaves ADC1 there with AIN10 and AIN11 and
// nothing else - so every claim about a pad is asserted PER VARIANT out
// of the device header's own AIN symbols.

#include <stdint.h>

#include "samc/adc.hpp"
#include "samc/clock.hpp"
#include "samc/platform_sam.hpp"
#include "util/analog_sampler.hpp"

using namespace brio;

// ---- geometry, from the device header's own constants ----------------------

static_assert(adc_count() == 2);
static_assert(Adc<0>::external_channels == 12);
static_assert(Adc<1>::external_channels == 12);
static_assert(Adc<0>::is_host && !Adc<0>::is_client);
static_assert(Adc<1>::is_client && !Adc<1>::is_host);
static_assert(Adc<0>::needs_calibration && Adc<1>::needs_calibration);
static_assert(Adc<0>::gclk_id == 33 && Adc<1>::gclk_id == 34);

// ---- the vocabularies this peripheral publishes ----------------------------

static_assert(Adc<0>::resrdy_generator == 0x43);
static_assert(Adc<0>::winmon_generator == 0x44);
static_assert(Adc<1>::resrdy_generator == 0x45);
static_assert(Adc<1>::winmon_generator == 0x46);
static_assert(Adc<0>::start_event_user == 28 && Adc<0>::flush_event_user == 29);
static_assert(Adc<1>::start_event_user == 30 && Adc<1>::flush_event_user == 31);
static_assert(Adc<0>::dma_trigger_resrdy == 42);
static_assert(Adc<1>::dma_trigger_resrdy == 43);

// ---- the clock arithmetic --------------------------------------------------

static_assert(adc_presc_divisor(AdcPresc::div2) == 2);
static_assert(adc_presc_divisor(AdcPresc::div256) == 256);
static_assert(adc_clock_hz(48'000'000, AdcPresc::div32) == 1'500'000);
// At a 48 MHz generator DIV2 leaves table 45-22's 16 MHz ceiling behind
// and DIV4 is the fastest legal setting.
static_assert(!adc_clock_in_range(48'000'000, AdcPresc::div2));
static_assert(adc_clock_in_range(48'000'000, AdcPresc::div4));
static_assert(adc_clock_in_range(48'000'000, AdcPresc::div256));    // 187.5 kHz
static_assert(!adc_clock_in_range(1'000'000, AdcPresc::div8));      // 125 kHz, below the floor
// 750 kHz is nearer 1 MHz than 1.5 MHz is, and the chooser says so.
static_assert(adc_presc_for(48'000'000, 1'000'000) == AdcPresc::div64);
static_assert(adc_presc_for(48'000'000, 1'500'000) == AdcPresc::div32);

// SAMPCTRL.SAMPLEN is (time * f) - 1, rounded up: 10 us at 1.5 MHz is
// 15 cycles, so SAMPLEN 14.
static_assert(adc_samplen_for_ns(1'500'000, adc_intref_sampling_ns) == 14);
static_assert(adc_samplen_for_ns(1'000'000, 1'000) == 0);
static_assert(adc_samplen_for_ns(16'000'000, 10'000) == 63);   // clamped

// ---- the RESSEL / AVGCTRL interplay ----------------------------------------

static_assert(adc_sample_steps(AdcRes::bits12) == 4096);
static_assert(adc_sample_steps(AdcRes::bits16) == 4096);   // 12-bit samples
static_assert(adc_sample_steps(AdcRes::bits10) == 1024);
static_assert(adc_sample_steps(AdcRes::bits8) == 256);
static_assert(adc_sample_bits(AdcRes::bits16) == 12);

static_assert(adc_average_count(AdcAverage::samples64) == 64);
static_assert(adc_auto_shift(AdcAverage::samples16) == 0);
static_assert(adc_auto_shift(AdcAverage::samples32) == 1);
static_assert(adc_auto_shift(AdcAverage::samples1024) == 6);
static_assert(adc_adjres_for_average(AdcAverage::samples8) == 3);
static_assert(adc_adjres_for_average(AdcAverage::samples1024) == 4);
static_assert(adc_adjres_for_oversampling(4) == 0);
static_assert(adc_oversampling_average(4) == AdcAverage::samples256);

// Table 38-2: a true average comes back at 12 bits whatever N is.
static_assert(adc_result_steps(AdcConfig{.resolution = AdcRes::bits16,
                                         .average = AdcAverage::samples64,
                                         .adjust = 4}) == 4096);
static_assert(adc_result_steps(AdcConfig{.resolution = AdcRes::bits16,
                                         .average = AdcAverage::samples1024,
                                         .adjust = 4}) == 4096);
// Table 38-3: 256 samples with ADJRES 0 is a 16-bit result.
static_assert(adc_result_steps(AdcConfig{.resolution = AdcRes::bits16,
                                         .average = AdcAverage::samples256,
                                         .adjust = 0}) == 65536);
static_assert(adc_result_steps(AdcConfig{}) == 4096);
static_assert(adc_result_steps(AdcConfig{.resolution = AdcRes::bits8}) == 256);

// ---- the conversion-time arithmetic (table 45-22) --------------------------

static_assert(adc_sample_cycles(AdcConfig{}) == 13);
static_assert(adc_sample_cycles(AdcConfig{.sample_length = 5}) == 18);
static_assert(adc_sample_cycles(AdcConfig{.resolution = AdcRes::bits8}) == 10);
static_assert(adc_sample_cycles(AdcConfig{.resolution = AdcRes::bits8,
                                          .differential = true}) == 9);
static_assert(adc_sample_cycles(AdcConfig{.offset_compensation = true}) == 16);
static_assert(adc_sample_cycles(AdcConfig{.resolution = AdcRes::bits10,
                                          .differential = true,
                                          .offset_compensation = true}) == 14);
// 38.6.2.14: the correction's 13 cycles are charged PER CONVERSION in
// single mode and ONCE in free-running mode - the distinction the bench
// caught the first version of this arithmetic getting wrong.
static_assert(adc_conversion_cycles(AdcConfig{.correction = true}) == 26);
static_assert(adc_conversion_cycles(AdcConfig{.free_running = true,
                                              .correction = true}) == 13);
static_assert(adc_conversion_cycles(AdcConfig{.resolution = AdcRes::bits16,
                                              .average = AdcAverage::samples16,
                                              .adjust = 4}) == 13 * 16);
static_assert(adc_result_hz(48'000'000, AdcConfig{}) == 1'500'000 / 13);

// ---- what must be refused --------------------------------------------------

static_assert(adc_config_valid(0, AdcConfig{}));
// THE TRAP: more than one sample per result needs RESSEL = 16BIT.
static_assert(!adc_config_valid(0, AdcConfig{.average = AdcAverage::samples16,
                                             .adjust = 4}));
static_assert(adc_config_valid(0, AdcConfig{.resolution = AdcRes::bits16,
                                            .average = AdcAverage::samples16,
                                            .adjust = 4}));
// 38.8.12: OFFCOMP fixes the sampling period, so SAMPLEN must be zero.
static_assert(!adc_config_valid(0, AdcConfig{.sample_length = 3,
                                             .offset_compensation = true}));
// 38.6.3.2: rail-to-rail needs that four-cycle sampling period.
static_assert(!adc_config_valid(0, AdcConfig{.rail_to_rail = true}));
static_assert(adc_config_valid(0, AdcConfig{.rail_to_rail = true,
                                            .offset_compensation = true}));
// The pair is not symmetric.
static_assert(!adc_config_valid(0, AdcConfig{.client_enable = true}));
static_assert(adc_config_valid(1, AdcConfig{.client_enable = true}));
static_assert(!adc_config_valid(1, AdcConfig{.dual = AdcDual::interleave}));
static_assert(adc_config_valid(0, AdcConfig{.dual = AdcDual::interleave}));
// Reserved reference codes and out-of-range fields.
static_assert(!ref_valid(static_cast<Ref>(6)));
static_assert(!adc_config_valid(0, AdcConfig{.reference = static_cast<Ref>(9)}));
static_assert(!adc_config_valid(0, AdcConfig{.adjust = 8}));
static_assert(!adc_config_valid(0, AdcConfig{.sample_length = 64}));
static_assert(!adc_config_valid(0, AdcConfig{.gain_correction = 0x1000}));
static_assert(!adc_config_valid(0, AdcConfig{.offset_correction = 2048}));
// Inverting an event input nobody listens to.
static_assert(!adc_config_valid(0, AdcConfig{.events = {.invert_start = true}}));

// The negative multiplexer is SIX pads wide where the positive one is
// twelve.
static_assert(adc_negative_valid(AdcNegative::ain5));
static_assert(adc_negative_valid(AdcNegative::ground));
static_assert(!adc_negative_valid(static_cast<AdcNegative>(6)));
static_assert(!adc_negative_valid(static_cast<AdcNegative>(0x17)));
static_assert(adc_muxpos_valid(0x0B) && !adc_muxpos_valid(0x0C));
static_assert(adc_muxpos_valid(0x19) && adc_muxpos_valid(0x1B));
static_assert(!adc_muxpos_valid(0x1D));

// ---- the reference arithmetic ----------------------------------------------

static_assert(ref_mv(Ref::vddana, 5100) == 5100);
static_assert(ref_mv(Ref::vddana_div2, 5100) == 2550);
static_assert(ref_mv(Ref::vddana_div1p6, 5100) == 3188);      // 5/8, rounded
static_assert(ref_mv(Ref::intref, 0, VrefLevel::v2_048) == 2048);
static_assert(ref_mv(Ref::vrefa) == 0);                        // not known

// ---- PAD BONDING, per variant ----------------------------------------------
//
// PA04..PA11 reach ADC0 on every package, and PA08/PA09 ALSO reach ADC1
// under different AIN numbers - the overlap that makes the map need the
// instance as a key.

static_assert(Adc<0>::ain_of('A', 4) == 4);
static_assert(Adc<0>::ain_of('A', 8) == 8);
static_assert(Adc<1>::ain_of('A', 8) == 10);
static_assert(Adc<0>::ain_of('A', 9) == 9);
static_assert(Adc<1>::ain_of('A', 9) == 11);
static_assert(Adc<0>::ain_of('A', 12) < 0);      // not an analog pad at all
static_assert(Adc<1>::ain_of('A', 4) < 0);       // PA04 is ADC0's alone
static_assert(Adc<0>::ain_exists(4) && Adc<0>::ain_exists(11));
static_assert(Adc<1>::ain_exists(10) && Adc<1>::ain_exists(11));

#if defined(__SAMC21J18A__)
// The J bonds every one of the twenty-four rows.
static_assert(Adc<0>::ain_of('B', 8) == 2 && Adc<1>::ain_of('B', 8) == 4);
static_assert(Adc<1>::ain_of('B', 0) == 0);
static_assert(Adc<1>::ain_of('B', 4) == 6 && Adc<1>::ain_of('B', 7) == 9);
static_assert(Adc<0>::ain_exists(2) && Adc<1>::ain_exists(0) &&
              Adc<1>::ain_exists(6));
#elif defined(__SAMC21G18A__)
// The G bonds PB02/PB03 and PB08/PB09 and nothing else of PORT B.
static_assert(Adc<0>::ain_of('B', 8) == 2 && Adc<1>::ain_of('B', 8) == 4);
static_assert(Adc<1>::ain_of('B', 2) == 2);
static_assert(Adc<1>::ain_of('B', 0) < 0);       // no PB00 on the G
static_assert(Adc<1>::ain_of('B', 4) < 0);       // no PB04 either
static_assert(Adc<0>::ain_exists(2));
static_assert(!Adc<1>::ain_exists(0) && !Adc<1>::ain_exists(6));
#elif defined(__SAMC21E18A__)
// The E bonds NO PORT B pad to either converter: ADC0 loses AIN2/AIN3
// and ADC1 is down to AIN10 and AIN11.
static_assert(Adc<0>::ain_of('B', 8) < 0);
static_assert(!Adc<0>::ain_exists(2) && !Adc<0>::ain_exists(3));
static_assert(Adc<0>::ain_exists(0) && Adc<0>::ain_exists(11));
static_assert(!Adc<1>::ain_exists(0) && !Adc<1>::ain_exists(5));
static_assert(Adc<1>::ain_exists(10) && Adc<1>::ain_exists(11));
#endif

// ---- the input tags and their codes ----------------------------------------

using PadA8 = Pin<'A', 8>;
using PadA9 = Pin<'A', 9>;
static_assert(Adc<0>::input_code(AnalogIn<PadA8>{}) == 8);
static_assert(Adc<1>::input_code(AnalogIn<PadA8>{}) == 10);
static_assert(Adc<0>::input_code(AdcInput::intref) == 0x19);
static_assert(Adc<0>::input_code(AdcInput::scaled_supply) == 0x1B);

// ---- util/analog_sampler.hpp's own contract, on this silicon ---------------
//
// THE POINT OF THE CAMPAIGN: the converter concept must be satisfiable
// with nothing in util/ changed. If any of these three fail, the sampler
// needs an adapter and the util pass has a question to answer.

static_assert(AnalogConverter<Adc<0>>);
static_assert(AnalogConverter<Adc<1>>);
static_assert(SamplerInput<Adc<0>, AnalogIn<PadA8>{}>);
static_assert(SamplerInput<Adc<0>, AdcInput::intref>);
static_assert(SamplerInput<Adc<1>, AnalogIn<PadA9>{}>);

// ---- every verb, instantiated ----------------------------------------------

struct Sink;
using Subs = Subscribers<>;

using Sampler = AnalogSampler<Adc<0>, SamPlatform, Subs, AdcInput::intref,
                              AnalogIn<PadA8>{}>;

void smoke() {
    constexpr AdcConfig cfg{
        .reference = Ref::vddana,
        .prescaler = AdcPresc::div32,
        .sample_length = 5,
    };
    (void)Adc<0>::init(0, cfg, 48'000'000);
    (void)Adc<0>::init<cfg>(0, 48'000'000);
    (void)Adc<0>::config();
    (void)Adc<0>::load_calibration();
    (void)Adc<0>::calibration();
    (void)Adc<0>::bias_reference_buffer();
    (void)Adc<0>::bias_comparator();
    (void)Adc<0>::sample_steps();
    (void)Adc<0>::result_steps();
    (void)Adc<0>::result_shift();
    (void)Adc<0>::reference();

    Adc<0>::select(AnalogIn<PadA8>{});
    Adc<0>::select(AdcInput::scaled_supply);
    (void)Adc<0>::select_sync(AdcInput::intref);
    (void)Adc<0>::select_negative(AdcNegative::ground);
    (void)Adc<0>::selected();
    (void)Adc<0>::selected_negative();
    Adc<0>::claim_pad<PadA8>();
    Adc<0>::release_pad<PadA8>();

    Adc<0>::sequence(0x0Fu);
    (void)Adc<0>::sequence();
    (void)Adc<0>::sequence_busy();
    (void)Adc<0>::sequence_state();

    Adc<0>::start();
    Adc<0>::flush();
    (void)Adc<0>::ready();
    (void)Adc<0>::overrun();
    (void)Adc<0>::result();
    (void)Adc<0>::result_signed();
    uint16_t v = 0;
    (void)Adc<0>::read(v);
    (void)Adc<0>::read();
    Adc<0>::discard(5);
    (void)Adc<0>::warm_up_conversions();

    (void)Adc<0>::window(AdcWindow::inside, 100, 200);
    (void)Adc<0>::window_signed(AdcWindow::outside, -100, 200);
    (void)Adc<0>::window_off();
    (void)Adc<0>::window_mode();
    (void)Adc<0>::window_hit();
    (void)Adc<0>::window_flag();

    (void)Adc<0>::gain_correction(0x800);
    (void)Adc<0>::gain_correction();
    (void)Adc<0>::offset_correction(-3);
    (void)Adc<0>::offset_correction();
    (void)Adc<0>::correction_enable(true);
    (void)Adc<0>::free_running(true);
    (void)Adc<0>::free_running();

    Adc<0>::arm(Adc<0>::flag_resrdy);
    Adc<0>::disarm(Adc<0>::flag_overrun);
    (void)Adc<0>::armed();
    (void)Adc<0>::flags();
    Adc<0>::clear_flags(Adc<0>::flag_winmon);
    (void)Adc<0>::isr();
    (void)Adc<0>::resrdy();

    (void)Adc<0>::event_config(AdcEventControl{.result_out = true});
    (void)Adc<0>::event_config();
    (void)Adc<0>::start_on(0, EventChannelConfig{.generator = 1});
    (void)Adc<0>::flush_on(1, EventChannelConfig{.generator = 2});
    (void)Adc<0>::stop_events();

    (void)Adc<0>::enable(false);
    (void)Adc<0>::enabled();
    (void)Adc<0>::sync_busy();
    (void)Adc<0>::reset();
    Adc<0>::release();

    // The client half, and the sampler over the pair's other converter.
    (void)Adc<1>::init(0, AdcConfig{.prescaler = AdcPresc::div32}, 48'000'000);
    (void)Adc<1>::init(0, AdcConfig{.prescaler = AdcPresc::div32,
                                    .client_enable = true},
                       48'000'000);
    Adc<1>::release();

    Sampler::init();
    Sampler::start_every(16);
    (void)Sampler::running_every();
    (void)Sampler::unknown_inputs();
    Sampler::stop();
}

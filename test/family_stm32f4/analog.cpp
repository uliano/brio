// The analog converters' family smoke TU (RM0090 ch. 13 and 14, RM0390
// ch. 13 and 14, RM0383 ch. 11 - which has no DAC chapter at all).
//
// What differs across the family here is bigger than in most chapters:
// one converter or three, a DAC or none, and a trigger space whose codes
// die with the timers behind them. So this TU is mostly about the
// RESERVE's derivations, and it asks the header the same question twice
// wherever two independent symbols can be made to agree - which is what a
// family fixture is for.
#include "stm32f4/adc.hpp"
#include "stm32f4/dac.hpp"
#include "stm32f4/dma.hpp"
#include "stm32f4/dma_engine.hpp"

using namespace brio;

// ---- how many converters, and the block they share ---------------------------

static_assert(adc_present(1), "every STM32F4 has ADC1");
static_assert(!adc_present(0) && !adc_present(4), "the family numbers them 1..3");
static_assert(adc_instances() == 1u || adc_instances() == 3u,
              "a part has ADC1 alone or all three - never two");
static_assert(adc_present(2) == adc_present(3),
              "ADC2 and ADC3 come and go together on this family");
static_assert(adc_instances() == (adc_present(2) ? 3u : 1u));

// THE COMMON BLOCK IS ASKED FOR TWICE. The pack names its base macro
// after the converters that share it - ADC1_COMMON_BASE where there is
// one, ADC123_COMMON_BASE where there are three - and 13.14's register
// map puts it at ADC1's base + 0x300 either way. Two derivations, one
// answer.
static_assert(adc_common_base() != 0u);
static_assert(adc_common_base() == adc_base(1) + 0x300u,
              "13.14: the common block sits at ADC1's base + 0x300");

// One enable bit per converter, and ONE reset line for all of them.
static_assert(adc_clock_mask(1) != 0u);
static_assert(adc_present(2) == (adc_clock_mask(2) != 0u));
static_assert(adc_clock_mask(3) != adc_clock_mask(1) || !adc_present(3));
static_assert(adc_reset_mask != 0u);
static_assert(adc_channel_count == 19, "channels 0..18, the width of AWDCH and every SQx field");

// ---- the clock -----------------------------------------------------------------

static_assert(adc_prescaler_divisor(AdcPrescaler::div2) == 2);
static_assert(adc_prescaler_divisor(AdcPrescaler::div8) == 8);
static_assert(adc_clock_hz(90'000'000u, AdcPrescaler::div4) == 22'500'000u);
// The chooser: the smallest division that stays under the ceiling, and
// div8 when none does.
static_assert(adc_prescaler_for(90'000'000u) == AdcPrescaler::div4,
              "PCLK2/2 is 45 MHz, past the 36 MHz ceiling");
static_assert(adc_prescaler_for(60'000'000u) == AdcPrescaler::div2);
static_assert(adc_prescaler_for(90'000'000u, adc_max_hz_low_supply) == AdcPrescaler::div6);
static_assert(adc_prescaler_for(400'000'000u) == AdcPrescaler::div8,
              "no code reaches it - the refusal is init()'s, not the chooser's");

// ---- resolutions and times ------------------------------------------------------

static_assert(adc_sample_steps(AdcRes::bits12) == 4096);
static_assert(adc_sample_steps(AdcRes::bits6) == 64);
static_assert(adc_sar_cycles(AdcRes::bits12) == 12 && adc_sar_cycles(AdcRes::bits6) == 6);
static_assert(adc_sample_cycles(AdcSampleTime::cycles3) == 3);
static_assert(adc_sample_cycles(AdcSampleTime::cycles480) == 480);
// 13.7's four minimum conversion times, spelled by the arithmetic.
static_assert(adc_conversion_cycles(AdcRes::bits12, AdcSampleTime::cycles3) == 15);
static_assert(adc_conversion_cycles(AdcRes::bits10, AdcSampleTime::cycles3) == 13);
static_assert(adc_conversion_cycles(AdcRes::bits8, AdcSampleTime::cycles3) == 11);
static_assert(adc_conversion_cycles(AdcRes::bits6, AdcSampleTime::cycles3) == 9);
static_assert(adc_conversion_cycles(AdcRes::bits12, AdcSampleTime::cycles480) == 492);

// ---- the triggers, and the timers behind them -----------------------------------
//
// THE TIMER DECIDES. RM0383 table 42 spells the F411's two TIM8 codes
// "Reserved" and the F410 keeps four timers out of eight - and the
// reserve reads that off the timers' own base-address macros, so the
// claim below is the header's and not a per-part list's.
static_assert(adc_trigger_valid(AdcTrigger::exti11), "the pad's line is every part's");
static_assert(adc_trigger_valid(AdcInjectedTrigger::exti15));
static_assert(adc_trigger_timer(AdcTrigger::tim8_trgo) == 8);
static_assert(adc_trigger_timer(AdcTrigger::exti11) == 0);
static_assert(adc_trigger_timer(AdcInjectedTrigger::tim4_trgo) == 4);
static_assert(adc_regular_exti_line == 11 && adc_injected_exti_line == 15);
#if defined(TIM8_BASE)
static_assert(adc_trigger_valid(AdcTrigger::tim8_trgo));
static_assert(adc_trigger_valid(AdcInjectedTrigger::tim8_cc4));
#else
static_assert(!adc_trigger_valid(AdcTrigger::tim8_cc1));
static_assert(!adc_trigger_valid(AdcInjectedTrigger::tim8_cc2));
#endif
#if defined(TIM2_BASE)
static_assert(adc_trigger_valid(AdcTrigger::tim2_trgo));
#else
static_assert(!adc_trigger_valid(AdcTrigger::tim2_trgo), "the F410 has no TIM2");
#endif

// ---- the internal channels ------------------------------------------------------
//
// The manual's numbers, and the reserve says whether a manual was read.
// Where it was, VREFINT is channel 17 and the sensor is 16 or 18 - and
// the sensor sharing VBAT's channel is exactly the case where it is 18.
static_assert(adc_internal_facts().known == (adc_input_channel(AdcInput::vrefint) != 0xFFu));
static_assert(!adc_internal_facts().known || adc_input_channel(AdcInput::vrefint) == 17u);
static_assert(!adc_internal_facts().known || adc_input_channel(AdcInput::vbat) == 18u);
static_assert(!adc_internal_facts().known ||
              adc_input_channel(AdcInput::temperature) == 16u ||
              adc_input_channel(AdcInput::temperature) == 18u);
static_assert(adc_internal_facts().sensor_shares_vbat ==
                  (adc_internal_facts().known &&
                   adc_input_channel(AdcInput::temperature) ==
                       adc_input_channel(AdcInput::vbat)),
              "the shared-channel flag IS the two numbers being equal");
static_assert(adc_internal_facts().vbat_divider == (adc_internal_facts().known
                                                        ? (adc_internal_facts().sensor_shares_vbat ? 4 : 2)
                                                        : 0),
              "13.11: /2 where the sensor has a channel of its own, /4 where it does not");
static_assert(adc_input_valid(AdcInput::vrefint) == adc_internal_facts().known);
static_assert(AdcCommon::vbat_divider() == adc_internal_facts().vbat_divider);
static_assert(AdcCommon::sensor_shares_vbat() == adc_internal_facts().sensor_shares_vbat);

// ---- the pad maps ----------------------------------------------------------------

static_assert(adc12_channel_of('A', 0) == 0 && adc12_channel_of('A', 7) == 7);
static_assert(adc12_channel_of('B', 0) == 8 && adc12_channel_of('B', 1) == 9);
static_assert(adc12_channel_of('C', 0) == 10 && adc12_channel_of('C', 5) == 15);
static_assert(adc12_channel_of('A', 8) == 0xFFu, "PA8 is not an analog input");
static_assert(adc12_channel_of('D', 0) == 0xFFu);
// ADC3's own map shares the first four and the middle four with the
// others and puts the rest on port F.
static_assert(adc3_channel_of('A', 3) == 3 && adc3_channel_of('C', 3) == 13);
static_assert(adc3_channel_of('F', 6) == 4 && adc3_channel_of('F', 10) == 8);
static_assert(adc3_channel_of('F', 3) == 9 && adc3_channel_of('F', 5) == 15);
static_assert(adc3_channel_of('A', 4) == 0xFFu, "PA4 is ADC1/ADC2's IN4, not ADC3's");

using In4 = AnalogIn<Pin<'A', 4>>;
using In5 = AnalogIn<Pin<'A', 5>>;
using In10 = AnalogIn<Pin<'C', 0>>;
static_assert(In4::channel == 4 && In5::channel == 5 && In10::channel == 10);
static_assert(Adc<1>::input_code(In4{}) == 4);
static_assert(Adc<1>::input_code(AdcInput::vrefint) == adc_input_channel(AdcInput::vrefint));

// A pad off the default map states its channel - ADC3's port F inputs are
// the case that needs it, and only the big packages bond them.
#if defined(GPIOF_BASE)
using In3f6 = AnalogIn<Pin<'F', 6>, adc3_channel_of('F', 6)>;
static_assert(In3f6::channel == 4);
#endif

// ---- the config, and what it refuses ----------------------------------------------

constexpr AdcConfig default_cfg{};
static_assert(adc_config_valid(default_cfg));
static_assert(adc_result_steps(default_cfg) == 4096);

constexpr AdcConfig both_discontinuous{.discontinuous = true, .injected_discontinuous = true};
static_assert(!adc_config_valid(both_discontinuous), "13.3.11: never both groups");

constexpr AdcConfig auto_and_disc{.discontinuous = true, .auto_injected = true};
static_assert(!adc_config_valid(auto_and_disc), "13.3.10's own note");

constexpr AdcConfig auto_and_trigger{.auto_injected = true,
                                     .injected_trigger_edge = AdcEdge::rising};
static_assert(!adc_config_valid(auto_and_trigger),
              "13.3.10: an auto-injected group takes no external trigger");

constexpr AdcConfig disc_nine{.discontinuous = true, .discontinuous_count = 9};
static_assert(!adc_config_valid(disc_nine), "DISCNUM is three bits");

constexpr AdcConfig dds_without_dma{.dma_continuous_requests = true};
static_assert(!adc_config_valid(dds_without_dma));

constexpr AdcConfig exti_triggered{.trigger = AdcTrigger::exti11,
                                   .trigger_edge = AdcEdge::rising};
static_assert(adc_config_valid(exti_triggered), "a pad's line is every part's");

// ---- the multi-ADC modes -----------------------------------------------------------

static_assert(adc_multi_valid(AdcMulti::independent));
static_assert(adc_multi_converters(AdcMulti::dual_regular) == 2);
static_assert(adc_multi_converters(AdcMulti::triple_regular) == 3);
static_assert(adc_multi_valid(AdcMulti::dual_regular) == (adc_instances() >= 2u));
static_assert(adc_multi_valid(AdcMulti::triple_interleaved) == (adc_instances() >= 3u));
static_assert(!adc_multi_valid(static_cast<AdcMulti>(0x03)), "13.13.16 leaves 00011 Reserved");
static_assert(!adc_multi_valid(static_cast<AdcMulti>(0x1F)));
static_assert(AdcCommon::multi_valid(AdcMulti::dual_regular) == (adc_instances() >= 2u));
static_assert(AdcCommon::instances == adc_instances());

// ---- the flags ----------------------------------------------------------------------

static_assert(AdcFlag::interrupting ==
              (AdcFlag::watchdog | AdcFlag::converted | AdcFlag::injected_converted |
               AdcFlag::overrun),
              "table 89: four of the six can interrupt");
static_assert((AdcFlag::all & (AdcFlag::started | AdcFlag::injected_started)) != 0u);

// ---- where the DMA requests sit --------------------------------------------------------
//
// The analog slice of the request mapping, in the same shape and with the
// same refusal as the serial one: a placement is a CELL and a class whose
// manual was not read has no table at all.

static_assert(!adc_dma_placements(1).known || adc_dma_placements(1).count == 2,
              "RM0090 table 44: ADC1 is DMA2's stream 0 and stream 4, channel 0 both");
static_assert(!adc_dma_placements(1).known || adc_dma_placements(1).at[0].controller == 2,
              "every ADC request is DMA2's");
static_assert(adc_dma_placement_valid(1, 2, 0, 0) == adc_dma_placements(1).known);
static_assert(adc_dma_placement_valid(1, 2, 4, 0) == adc_dma_placements(1).known);
static_assert(!adc_dma_placement_valid(1, 1, 5, 7), "the DAC's cell is not the ADC's");
static_assert(!adc_dma_placement_valid(1, 2, 0, 2), "stream 0 channel 2 is ADC3's");
static_assert(adc_dma_placement_valid(3, 2, 0, 2) == (adc_present(3) && adc_dma_placements(3).known),
              "ADC1 and ADC3 CONTEND for stream 0, on two different channels");
static_assert(!adc_dma_placement_valid(2, 2, 2, 1) || adc_present(2));

static_assert(dac_dma_placements(0).known == (dac_present() && dac_channel_facts().known));
static_assert(!dac_dma_placements(0).known || dac_dma_placements(0).count == 1);
static_assert(dac_dma_placement_valid(0, 1, 5, 7) == dac_dma_placements(0).known,
              "RM0090 table 43: DAC1 is DMA1's stream 5, channel 7");
static_assert(dac_dma_placement_valid(1, 1, 6, 7) == dac_dma_placements(1).known);
static_assert(!dac_dma_placement_valid(0, 2, 0, 0), "an ADC cell is not the DAC's");

// The placement check as a driver offers it: an absent engine is always
// placed, a real one only on its own cells.
using AdcStream = DmaRxEngine<2, 0, 0, uint16_t>;
using WrongStream = DmaRxEngine<2, 1, 0, uint16_t>;
static_assert(Adc<1>::engine_placed<NoDmaEngine>());
static_assert(Adc<1>::engine_placed<AdcStream>() == adc_dma_placements(1).known);
static_assert(!Adc<1>::engine_placed<WrongStream>());
#if defined(DAC_BASE)
using DacStream = DmaTxEngine<1, 5, 7, uint16_t>;
static_assert(Dac::engine_placed<NoDmaEngine>(0));
static_assert(Dac::engine_placed<DacStream>(0) == dac_dma_placements(0).known);
static_assert(!Dac::engine_placed<DacStream>(1), "stream 5 is channel 1's, not channel 2's");
#endif

// ---- the reference --------------------------------------------------------------------

static_assert(ref_mv(Ref::vref_pin) == 3300, "the boards' rail, and the caller may say another");
static_assert(ref_mv(Ref::vref_pin, 3000) == 3000);
static_assert(adc_mv(2048, 4096, ref_mv(Ref::vref_pin)) == 1650);

// ---- the factory measurements -----------------------------------------------------------

static_assert(AdcFactory::characterization_mv == 3300, "the F4's points are at 3.3 V");
static_assert(AdcFactory::ts_cal1_celsius == 30 && AdcFactory::ts_cal2_celsius == 110);
static_assert(AdcFactory::vrefint_cal_address == 0x1FFF7A2AUL);
static_assert(AdcFactory::ts_cal2_address == AdcFactory::ts_cal1_address + 2u);

// ---- every verb, once --------------------------------------------------------------------

using Adc1 = Adc<1>;
static_assert(Adc1::is_master);
static_assert(Adc1::irq() == ADC_IRQn);

void adc_common_verbs() {
    AdcCommon::reset();
    AdcCommon::prescaler(AdcPrescaler::div4);
    (void)AdcCommon::prescaler();
    (void)AdcCommon::adc_hz(90'000'000u);
    AdcCommon::internal_sources(true);
    (void)AdcCommon::internal_sources();
    AdcCommon::vbat(false);
    (void)AdcCommon::vbat();
    (void)AdcCommon::multi(AdcMulti::independent);
    (void)AdcCommon::multi();
    AdcCommon::multi_dma(AdcMultiDma::off, false);
    (void)AdcCommon::multi_dma();
    (void)AdcCommon::interleave_delay(static_cast<uint8_t>(8));
    (void)AdcCommon::interleave_delay();
    (void)AdcCommon::status();
    (void)AdcCommon::status(1);
    (void)AdcCommon::data();
    (void)AdcCommon::data_low();
    (void)AdcCommon::data_high();
    (void)AdcCommon::data_address();
    AdcCommon::release();
}

void adc_verbs() {
    constexpr Clock<ClockSource::hsi, 16'000'000u> clock;
    static const uint8_t seq[3] = {4, 5, 17};

    Adc1::bus_clock(true);
    (void)Adc1::bus_clock();
    (void)Adc1::init(clock, default_cfg);
    (void)Adc1::init(clock, default_cfg, adc_max_hz_low_supply);
    (void)Adc1::config_valid(default_cfg);
    (void)Adc1::configure(default_cfg);
    (void)Adc1::config().resolution;
    (void)Adc1::power_on(clock);
    (void)Adc1::powered();
    (void)Adc1::regs().SR;
    (void)Adc1::data_address();

    (void)Adc1::sample_time(4, AdcSampleTime::cycles480);
    (void)Adc1::sample_time(4);
    Adc1::sample_time_all(AdcSampleTime::cycles15);
    (void)Adc1::conversion_cycles(4);

    (void)Adc1::regular_sequence(seq, 3);
    (void)Adc1::regular_sequence_unchecked(seq, 3);
    (void)Adc1::sequence_length();
    (void)Adc1::sequence_channel(1);
    (void)Adc1::select_channel(4);
    Adc1::select(In4{});
    Adc1::select(AdcInput::vrefint);
    (void)Adc1::select_sync(In5{});
    (void)Adc1::select_sync(AdcInput::temperature);
    (void)Adc1::selected();

    (void)Adc1::injected_sequence(seq, 2);
    (void)Adc1::injected_sequence_unchecked(seq, 2);
    (void)Adc1::injected_length();
    (void)Adc1::injected_slot_channel(4);
    (void)Adc1::injected_offset(1, 0x200);
    (void)Adc1::injected_offset(1);
    (void)Adc1::injected_result(1);
    Adc1::start_injected();
    (void)Adc1::injected_ready();
    int16_t inj = 0;
    (void)Adc1::read_injected(inj, 16);

    (void)Adc1::converting();
    Adc1::start();
    (void)Adc1::ready();
    (void)Adc1::started();
    (void)Adc1::overrun();
    (void)Adc1::result();
    uint16_t v = 0;
    (void)Adc1::read(v, 16);
    (void)Adc1::read(16);
    (void)Adc1::read_settled(2, 16);
    Adc1::stop(16);

    (void)Adc1::result_steps();
    (void)Adc1::vdda_mv(1500);
    (void)Adc1::temperature_centi_c(1000, 3300);
    (void)Adc1::temperature_centi_c_typical(800);

    (void)Adc1::watchdog(0x100, 0xF00, true, false, true, 4);
    Adc1::watchdog_off();
    (void)Adc1::watchdog_thresholds(0x080, 0xF80);
    (void)Adc1::watchdog_high();
    (void)Adc1::watchdog_low();
    (void)Adc1::watchdog_channel();

    (void)Adc1::flags();
    (void)Adc1::flag(AdcFlag::converted);
    Adc1::clear_flags(AdcFlag::all);
    Adc1::interrupts(AdcFlag::converted | AdcFlag::overrun, true);
    (void)Adc1::armed();
    (void)Adc1::isr();

    In4::claim();
    In4::release();
    Adc1::release();
}

// The slaves exist only where the header says so, and the ONLY thing that
// differs about them is that they are not the master.
#if defined(ADC2_BASE)
static_assert(!Adc<2>::is_master && !Adc<3>::is_master);
static_assert(Adc<2>::irq() == Adc<1>::irq(), "one vector for all three");
static_assert(adc_base(2) == adc_base(1) + 0x100u && adc_base(3) == adc_base(1) + 0x200u,
              "13.14's global register map");

void adc_slave_verbs() {
    Adc<2>::bus_clock(true);
    (void)Adc<2>::configure(default_cfg);
    (void)Adc<2>::select_channel(5);
    (void)Adc<2>::read(16);
    Adc<3>::bus_clock(true);
    (void)Adc<3>::select_channel(10);
    Adc<2>::release();
    Adc<3>::release();
}
#endif

// ---- the DAC ----------------------------------------------------------------------------
//
// The vocabulary is every part's; the resource exists only where the
// header declares the block.

static_assert(dac_trigger_valid(DacTrigger::software) && dac_trigger_valid(DacTrigger::exti9));
static_assert(dac_trigger_timer(DacTrigger::tim6_trgo) == 6);
static_assert(dac_trigger_timer(DacTrigger::exti9) == 0);
static_assert(dac_exti_line == 9);
static_assert(dac_wave_amplitude(0) == 1 && dac_wave_amplitude(3) == 15);
static_assert(dac_wave_amplitude(11) == 4095 && dac_wave_amplitude(15) == 4095);
static_assert(dac_lfsr_preload == 0x0AAAu);
static_assert(dac_pad_port(0) == 'A' && dac_pad_pin(0) == 4);
static_assert(dac_pad_port(1) == 'A' && dac_pad_pin(1) == 5);
#if defined(TIM7_BASE)
static_assert(dac_trigger_valid(DacTrigger::tim7_trgo));
#else
static_assert(!dac_trigger_valid(DacTrigger::tim7_trgo));
#endif

constexpr DacChannelConfig dac_default{};
static_assert(dac_channel_config_valid(dac_default));
constexpr DacChannelConfig wave_untriggered{.wave = DacWave::triangle};
static_assert(!dac_channel_config_valid(wave_untriggered),
              "14.5.1: WAVEx is only used with TENx set");
constexpr DacChannelConfig dma_on_software{.triggered = true,
                                           .trigger = DacTrigger::software,
                                           .dma = true};
static_assert(!dac_channel_config_valid(dma_on_software),
              "14.3.7: a software trigger raises no DMA request");
constexpr DacChannelConfig amplitude_off_the_field{
    .triggered = true, .wave = DacWave::noise, .amplitude = 16};
static_assert(!dac_channel_config_valid(amplitude_off_the_field));

// The DAC's presence and its channel count come from two different
// authorities, and the fixture states which: the block from the HEADER,
// the count from the MANUAL - so a part whose manual nobody read has the
// block and one usable channel.
static_assert(dac_present() == (dac_channel_facts().channels != 0u));
static_assert(!dac_channel_facts().known || dac_channel_facts().channels == 2u);
static_assert(dac_channel_facts().known || dac_channel_facts().channels <= 1u);

#if defined(DAC_BASE)
static_assert(Dac::steps == 4096);
static_assert(Dac::channels == dac_channel_facts().channels);
static_assert(Dac::channels_known == dac_channel_facts().known);
static_assert(Dac::irq() == TIM6_DAC_IRQn);
static_assert(Dac::channel_valid(0));
static_assert(dac_code(1650, Dac::steps, ref_mv(Ref::vref_pin)) == 2048);

void dac_verbs() {
    Dac::init();
    (void)Dac::bus_clock();
    (void)Dac::regs().CR;
    Dac::claim_pad<Pin<'A', 4>>();
    (void)Dac::configure(0, dac_default);
    (void)Dac::config(0).buffered;
    (void)Dac::enable(0, true);
    (void)Dac::enabled(0);
    (void)Dac::wave(0, DacWave::none);
    (void)Dac::wave(0);
    (void)Dac::write(0, 2048);
    (void)Dac::write_left(0, 0x8000);
    (void)Dac::write8(0, 128);
    (void)Dac::write_dual(1, 2);
    (void)Dac::write_dual_left(0x1000, 0x2000);
    (void)Dac::write_dual8(1, 2);
    (void)Dac::code(0);
    (void)Dac::output(0);
    (void)Dac::trigger(0);
    Dac::trigger_both();
    (void)Dac::data_address_12r(0);
    (void)Dac::data_address_12l(0);
    (void)Dac::data_address_8r(0);
    (void)Dac::data_address_dual_12r();
    (void)Dac::stop_dma(0);
    (void)Dac::flags();
    Dac::clear_flags(0u);
    (void)Dac::underrun(0);
    (void)Dac::clear_underrun(0);
    Dac::interrupts(0, false);
    (void)Dac::isr();
    Dac::release_pad<Pin<'A', 4>>();
    Dac::release();
}
#endif

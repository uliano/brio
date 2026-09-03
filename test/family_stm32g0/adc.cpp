// Family smoke TU: stm32g0/adc.hpp and stm32g0/vref.hpp (RM0444 ch. 15
// and 17) on each of the three headers the desk's boards span.
// Instantiation only - no main(), no hardware.
//
// WHAT THIS FIXTURE IS REALLY FOR. Every part of this family carries
// exactly one ADC with the same nineteen channels, so - unlike the
// timers - the converter itself is not what differs. What DOES differ is
// its VECTOR: on the G0B1 and G071 classes it is shared with the
// comparators (ADC1_COMP_IRQn), and on the G031 class, which has no
// comparator at all, the ADC has a line of its own under another name.
// A wrong answer there would be a silent Default_Handler spin, which is
// exactly what this file exists to make impossible.
//
// The arithmetic - full scales, sampling and conversion times, the
// oversampler's own full scale, the refusals - is constexpr, so it is
// checked here rather than at the bench.

#include <stdint.h>

#include "stm32g0/adc.hpp"
#include "stm32g0/platform_stm32.hpp"
#include "stm32g0/vref.hpp"
#include "util/analog.hpp"
#include "util/analog_sampler.hpp"

using namespace brio;

// ---- presence and geometry, as the three headers state it ------------------
static_assert(adc_present(), "every STM32G0 has the one ADC");
static_assert(adc_channels() == 19, "nineteen multiplexed channels (15.3.8)");
static_assert(adc_temperature_channel() == 12 && adc_vrefint_channel() == 13 &&
                  adc_vbat_channel() == 14,
              "the three internal channels are 12, 13 and 14");
static_assert(adc_common_base() != 0, "ADC_CCR lives in its own common block");
static_assert(vrefbuf_present(), "every header of this pack declares VREFBUF");

#if defined(STM32G0B1xx) || defined(STM32G071xx)
static_assert(adc_irq() == ADC1_COMP_IRQn,
              "where comparators exist the ADC shares their vector (table 61)");
#else
static_assert(adc_irq() == ADC1_IRQn,
              "the G031 class has no comparator, so the ADC's line is its own");
#endif

// ---- the arithmetic --------------------------------------------------------
static_assert(adc_sample_steps(AdcRes::bits12) == 4096);
static_assert(adc_sample_steps(AdcRes::bits10) == 1024);
static_assert(adc_sample_steps(AdcRes::bits8) == 256);
static_assert(adc_sample_steps(AdcRes::bits6) == 64);

static_assert(adc_sar_half_cycles(AdcRes::bits12) == 25, "12.5 cycles (table 76)");
static_assert(adc_sar_half_cycles(AdcRes::bits6) == 13, "6.5 cycles");

static_assert(adc_sample_half_cycles(AdcSampleTime::cycles1_5) == 3);
static_assert(adc_sample_half_cycles(AdcSampleTime::cycles160_5) == 321);

static_assert(adc_presc_divisor(AdcPresc::div1) == 1);
static_assert(adc_presc_divisor(AdcPresc::div6) == 6);
static_assert(adc_presc_divisor(AdcPresc::div256) == 256);
static_assert(adc_presc_divisor(static_cast<AdcPresc>(12)) == 0, "a Reserved PRESC code");

static_assert(adc_oversampling_ratio(AdcOversampling::x2) == 2);
static_assert(adc_oversampling_ratio(AdcOversampling::x256) == 256);

// 15.3.9's own example: 1.5 + 12.5 = 14 cycles at 12 bits.
constexpr AdcConfig plain{};
static_assert(adc_conversion_half_cycles(plain, 0) == 28);
static_assert(adc_result_steps(plain) == 4096);

// Table 79, one row of it: 16x oversampling with a 4-bit shift is back
// at a 12-bit full scale, and with no shift it is 16 times as wide.
constexpr AdcConfig ovs16_shift4{
    .oversampling = true, .oversampling_ratio = AdcOversampling::x16,
    .oversampling_shift = 4};
static_assert(adc_result_steps(ovs16_shift4) == 4096);
constexpr AdcConfig ovs16_shift0{
    .oversampling = true, .oversampling_ratio = AdcOversampling::x16,
    .oversampling_shift = 0};
static_assert(adc_result_steps(ovs16_shift0) == 65536, "truncated to sixteen bits");

// The second sampling time, chosen per channel through SMPSEL.
constexpr AdcConfig two_times{.sample1 = AdcSampleTime::cycles1_5,
                              .sample2 = AdcSampleTime::cycles160_5,
                              .sample2_channels = 1u << 13};
static_assert(adc_sample_time_of(two_times, 0) == AdcSampleTime::cycles1_5);
static_assert(adc_sample_time_of(two_times, 13) == AdcSampleTime::cycles160_5);
static_assert(adc_conversion_half_cycles(two_times, 13) == 321 + 25);

// The clock arithmetic, both faces.
static_assert(adc_clock_hz(64'000'000UL, 16'000'000UL, plain) == 32'000'000UL);
constexpr AdcConfig async_div2{.clock_mode = AdcClockMode::async,
                              .prescaler = AdcPresc::div2};
static_assert(adc_clock_hz(64'000'000UL, 16'000'000UL, async_div2) == 8'000'000UL);

// ---- the refusals ----------------------------------------------------------
static_assert(adc_config_valid(plain));
static_assert(!adc_config_valid({.continuous = true, .discontinuous = true}),
              "15.4.1 forbids the pair");
static_assert(!adc_config_valid({.prescaler = static_cast<AdcPresc>(15)}));
static_assert(!adc_config_valid({.left_aligned = true, .oversampling = true}),
              "15.8.1: ALIGN is ignored while oversampling, so it is refused");
static_assert(!adc_config_valid({.oversampling = true, .oversampling_shift = 9}));
static_assert(!adc_config_valid({.dma_circular = true}), "circular without DMAEN");
static_assert(!adc_config_valid({.sample2_channels = 1u << 19}), "past the channel count");

// ---- the reference vocabulary (stm32g0/vref.hpp) ---------------------------
static_assert(ref_mv(Ref::buffer_2v048) == 2048);
static_assert(ref_mv(Ref::buffer_2v5) == 2500);
static_assert(ref_mv(Ref::external, 3300) == 3300);
static_assert(ref_mv(Ref::external) == 0, "an unknown external reference says so");
static_assert(ref_valid(Ref::external) && !ref_valid(static_cast<Ref>(7)));

// ---- util/analog.hpp on this converter's numbers ---------------------------
static_assert(adc_mv(2048, 4096, 3300) == 1650);
static_assert(adc_mv(65535, adc_result_steps(ovs16_shift0), 3300) == 3300);

// ---- a pad on a channel, and the sampler's contract ------------------------
using PadA4 = Pin<'A', 4>;
using In4 = AnalogIn<PadA4, 4>;
static_assert(In4::channel == 4);
static_assert(Adc::input_code(In4{}) == 4);
static_assert(Adc::input_code(AdcInput::vrefint) == 13);
static_assert(AnalogConverter<Adc>, "the ADC satisfies util/analog_sampler.hpp");
static_assert(SamplerInput<Adc, AdcInput::vrefint>);
static_assert(SamplerInput<Adc, In4{}>);

static_assert(adc_input_valid(AdcInput::temperature) && adc_input_valid(AdcInput::vbat));

using SysClock = Clock<ClockSource::pll, 64'000'000>;
constexpr SysClock clock;

// ---- every verb instantiated ------------------------------------------------
void use() {
    (void)Adc::init(clock, plain);
    (void)Adc::init(clock, async_div2, 16'000'000UL);
    Adc::bus_clock(true);
    (void)Adc::bus_clock();
    Adc::reset();
    Adc::async_source(AdcAsyncSource::hsi16);
    (void)Adc::async_source();
    (void)Adc::clock_ok();

    (void)Adc::regulator();
    (void)Adc::regulator_on(clock);
    Adc::regulator_off();
    (void)Adc::calibrate();
    (void)Adc::calibration();
    (void)Adc::set_calibration(0x40);
    (void)Adc::enable();
    (void)Adc::enabled();
    (void)Adc::disable();

    (void)Adc::configure(plain);
    Adc::configure_while_enabled(plain);
    (void)Adc::config();

    Adc::vrefint(true);
    Adc::temperature(true);
    Adc::vbat(false);
    (void)Adc::vrefint();
    (void)Adc::temperature();
    (void)Adc::vbat();

    In4::claim();
    Adc::select(In4{});
    Adc::select(AdcInput::vrefint);
    (void)Adc::select_sync(In4{});
    (void)Adc::select_sync(AdcInput::temperature);
    (void)Adc::select_channel(7);
    (void)Adc::sequence((1u << 4) | (1u << 13));
    static const uint8_t order[3] = {13, 4, 0};
    (void)Adc::sequence_ordered(order, 3);
    (void)Adc::selection();
    (void)Adc::selected();

    Adc::start();
    (void)Adc::converting();
    (void)Adc::stop();
    (void)Adc::ready();
    (void)Adc::sequence_done();
    (void)Adc::result();
    uint16_t v = 0;
    (void)Adc::read(v);
    (void)Adc::read();
    (void)Adc::read_settled(4);

    (void)Adc::result_steps();
    (void)Adc::sample_steps();
    (void)Adc::conversion_half_cycles(13);
    (void)Adc::vdda_mv(1500);
    (void)Adc::temperature_centi_c(1000, 3300);
    (void)AdcFactory::plausible();
    (void)AdcFactory::vrefint_cal();
    (void)AdcFactory::ts_cal1();
    (void)AdcFactory::ts_cal2();

    (void)Adc::watchdog1(100, 3000, true, 13);
    (void)Adc::watchdog1_off();
    (void)Adc::watchdog2(1u << 13, 0, 4095);
    (void)Adc::watchdog3(0, 0, 4095);
    (void)Adc::watchdog2_channels();
    (void)Adc::watchdog3_channels();
    (void)Adc::watchdog_thresholds(1, 0, 4095);
    (void)Adc::watchdog_thresholds(2, 0, 4095);
    (void)Adc::watchdog_thresholds(3, 0, 4095);
    (void)Adc::watchdog_thresholds(4, 0, 4095);

    (void)Adc::flags();
    (void)Adc::flag(AdcFlag::converted);
    Adc::clear_flags(AdcFlag::overrun);
    (void)Adc::overrun();
    Adc::interrupts(AdcFlag::converted, true);
    (void)Adc::interrupts();
    (void)Adc::isr();
    (void)Adc::dma_request;
    (void)Adc::data_address();
    (void)Adc::irq();
    Adc::release();
    In4::release();

    // The reference buffer: every verb, and the refusal that guards it.
    Vref::init();
    (void)Vref::bus_clock();
    (void)Vref::enabled();
    (void)Vref::high_impedance();
    (void)Vref::ready();
    (void)Vref::scale();
    (void)Vref::reference();
    (void)Vref::trim();
    (void)Vref::enable({.scale = VrefScale::v2_5, .board_vref_pin_is_free = false});
    Vref::hold();
    Vref::disable();
    Vref::release();
}

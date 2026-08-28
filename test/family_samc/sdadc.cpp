// Family smoke TU for samc/sdadc.hpp: every verb must COMPILE on the E,
// G and J 18A headers (tools/check_samc.sh sweeps all three). This
// peripheral has ONE instance everywhere, but its pad map is the most
// package-dependent in the stratum - the E bonds ONE differential pair,
// the G two, only the J all three - so what this file asserts per
// variant is that bonding, out of the device header's own symbols rather
// than out of a claim.

#include <stdint.h>

#include "samc/sdadc.hpp"
#include "samc/clock.hpp"
#include "samc/platform_sam.hpp"

using namespace brio;

// ---- geometry, from the device header's own constants ----------------------

static_assert(sdadc_count() == 1);
static_assert(Sdadc::channels == 3);
static_assert(Sdadc::gclk_id == 35);
static_assert(Sdadc::half_steps == 32768);

// ---- the vocabularies this peripheral publishes ----------------------------

static_assert(Sdadc::resrdy_generator == 71);
static_assert(Sdadc::winmon_generator == 72);
static_assert(Sdadc::start_event_user == 32);
static_assert(Sdadc::flush_event_user == 33);
static_assert(Sdadc::dma_trigger_resrdy == 44);

// ---- the pads: THE PACKAGE VARIATION, per variant --------------------------
//
// Pair 0 is PA06/PA07 on every variant; pair 1 is PB08/PB09 and absent on
// the E; pair 2 is PB06/PB07 and exists on the J alone. VREFB is PA04
// everywhere.

static_assert(sdadc_pair_exists(0));
static_assert(!sdadc_pair_exists(3));

static_assert(Sdadc::negative_port(0) == 'A' && Sdadc::negative_pin(0) == 6);
static_assert(Sdadc::positive_port(0) == 'A' && Sdadc::positive_pin(0) == 7);

static_assert(sdadc_inn_pad_exists<'A', 6>);
static_assert(sdadc_inp_pad_exists<'A', 7>);
// A pad is one polarity or the other, never both.
static_assert(!sdadc_inp_pad_exists<'A', 6>);
static_assert(!sdadc_inn_pad_exists<'A', 7>);
static_assert(!sdadc_inn_pad_exists<'A', 2>);

static_assert(sdadc_vrefb_pad_exists<'A', 4>);
static_assert(!sdadc_vrefb_pad_exists<'A', 3>);

// Function B on this family, taken from the header's MUX_ symbol and not
// from a letter written down here.
static_assert(Sdadc::negative_function('A', 6) == static_cast<int>(PinFunction::b));
static_assert(Sdadc::positive_function('A', 7) == static_cast<int>(PinFunction::b));
static_assert(Sdadc::vrefb_function('A', 4) == static_cast<int>(PinFunction::b));

#if defined(__SAMC21E18A__)
static_assert(!sdadc_pair_exists(1), "the E bonds no PORT B pad to the SDADC");
static_assert(!sdadc_pair_exists(2));
static_assert(!sdadc_inn_pad_exists<'B', 8>);
#elif defined(__SAMC21G18A__)
static_assert(sdadc_pair_exists(1), "the G adds pair 1 on PB08/PB09");
static_assert(!sdadc_pair_exists(2), "pair 2 is a J-only pad pair");
static_assert(Sdadc::negative_port(1) == 'B' && Sdadc::negative_pin(1) == 8);
static_assert(Sdadc::positive_port(1) == 'B' && Sdadc::positive_pin(1) == 9);
#elif defined(__SAMC21J18A__)
static_assert(sdadc_pair_exists(1) && sdadc_pair_exists(2),
              "the J carries all three pairs");
static_assert(Sdadc::negative_port(2) == 'B' && Sdadc::negative_pin(2) == 6);
static_assert(Sdadc::positive_port(2) == 'B' && Sdadc::positive_pin(2) == 7);
// The pads pair 1 and 2 share with the SAR converters, which is what
// makes a wireless cross-check possible at all.
static_assert(sdadc_inn_pad_exists<'B', 8> && sdadc_inp_pad_exists<'B', 9>);
static_assert(sdadc_inn_pad_exists<'B', 6> && sdadc_inp_pad_exists<'B', 7>);
#endif

// ---- the reference vocabulary ----------------------------------------------
//
// FOUR codes and NONE Reserved - the only reference field of the three
// converters with no illegal value.

static_assert(sdadc_ref_valid(SdadcRef::intref));
static_assert(sdadc_ref_valid(SdadcRef::vrefb));
static_assert(sdadc_ref_valid(SdadcRef::dac));
static_assert(sdadc_ref_valid(SdadcRef::vddana));
static_assert(!sdadc_ref_valid(static_cast<SdadcRef>(4)));

static_assert(sdadc_ref_mv(SdadcRef::vddana, 5100) == 5100);
static_assert(sdadc_ref_mv(SdadcRef::vrefb, 2500) == 2500);
static_assert(sdadc_ref_mv(SdadcRef::intref, 5100, VrefLevel::v4_096) == 4096);

// ---- the clock, LINEAR and not the header's powers of two ------------------

static_assert(sdadc_prescaler_divisor(0) == 2);
static_assert(sdadc_prescaler_divisor(1) == 4);
static_assert(sdadc_prescaler_divisor(2) == 6);     // NOT 8
static_assert(sdadc_prescaler_divisor(3) == 8);
static_assert(sdadc_prescaler_divisor(255) == 512); // 39.5.3's own endpoint

static_assert(sdadc_clock_hz(48'000'000, 3) == 6'000'000);
static_assert(sdadc_sampling_hz(48'000'000, 3) == 1'500'000);
static_assert(sdadc_clock_in_range(48'000'000, 3));
static_assert(!sdadc_clock_in_range(48'000'000, 2));   // 8 MHz, past table 45-26
static_assert(!sdadc_clock_in_range(48'000'000, 24));  // 960 kHz, under it
static_assert(sdadc_prescaler_for(48'000'000, 6'000'000) == 3);
static_assert(sdadc_prescaler_for(48'000'000, 1'000'000) == 23);

// ---- OSR and the rate arithmetic -------------------------------------------

static_assert(sdadc_osr_value(SdadcOsr::osr64) == 64);
static_assert(sdadc_osr_value(SdadcOsr::osr1024) == 1024);
static_assert(sdadc_osr_valid(SdadcOsr::osr1024));
static_assert(!sdadc_osr_valid(static_cast<SdadcOsr>(5)));

// 39.6.3.3's own worked example: 1 MHz generator, OSR 64, PRESCALER 0 ->
// a conversion every 512 us, 1953 sps.
static_assert(sdadc_conversion_cycles(SdadcConfig{.free_running = true}) == 1024);
static_assert(sdadc_result_hz(1'000'000,
                              SdadcConfig{.prescaler = 0,
                                          .osr = SdadcOsr::osr64,
                                          .free_running = true}) == 1953);
static_assert(sdadc_conversion_us(1'000'000,
                                  SdadcConfig{.prescaler = 0,
                                              .osr = SdadcOsr::osr64,
                                              .free_running = true}) == 512);
// A SINGLE conversion costs SKPCNT + 1 of those windows.
static_assert(sdadc_conversion_us(1'000'000,
                                  SdadcConfig{.prescaler = 0,
                                              .osr = SdadcOsr::osr64,
                                              .skip_count = 2}) == 1536);

// ---- the post-processing, IN RAW 24-BIT UNITS -------------------------------
//
// The formula's `Data0` is the 24-bit filter output, not the 16-bit datum
// 39.6.3.4 claims - measured, and it is why an OFFSETCORR of 256 moves
// `result()` by exactly one count.

static_assert(sdadc_raw_per_count == 256);
static_assert(sdadc_raw_half_steps == 8388608);
static_assert(sdadc_gain_permille(1, 0) == 1000);
static_assert(sdadc_gain_permille(3, 1) == 1500);
static_assert(sdadc_corrected(1000, 0, 1, 0) == 1000);
static_assert(sdadc_corrected(1000, 100, 1, 0) == 1100);
static_assert(sdadc_corrected(1000, 0, 3, 1) == 1500);
static_assert(sdadc_corrected(1000, 0, 0, 0) == 0);         // gain zero kills it
static_assert(sdadc_corrected(8000000, 0, 2, 0) == 8388607);  // and it saturates
static_assert(sdadc_corrected(-8000000, 0, 2, 0) == -8388608);

// ---- the result's placement (39.8.19: signed, 24 bits wide) ----------------

static_assert(sdadc_result_of(0x000000u) == 0);
static_assert(sdadc_result_of(0x7FFF00u) == 32767);
static_assert(sdadc_result_of(0x800000u) == -32768);
static_assert(sdadc_result_of(0xFFFF00u) == -1);
static_assert(sdadc_result_of(0xFF0000u) == -256);
static_assert(sdadc_raw_signed(0x7FFFFFu) == 8388607);
static_assert(sdadc_raw_signed(0x800000u) == -8388608);
static_assert(sdadc_raw_signed(0x000000u) == 0);
static_assert(sdadc_raw_signed(0xFFFFFFu) == -1);
// The two views of one register: the datum is the raw value over 256.
static_assert(sdadc_result_of(0x7FFFFFu) == sdadc_raw_signed(0x7FFFFFu) / 256);
static_assert(sdadc_threshold_word(0x1234) == 0x123400u);
static_assert(sdadc_threshold_word(-1) == 0xFFFF00u);

// ---- the refusals that are the chapter's rules -----------------------------

static_assert(sdadc_config_valid(SdadcConfig{}));
// An internal reference without its buffer (39.8.2's Note, erratum 1.8.10).
static_assert(!sdadc_config_valid(SdadcConfig{.reference = SdadcRef::intref}));
static_assert(!sdadc_config_valid(SdadcConfig{.reference = SdadcRef::dac}));
static_assert(sdadc_config_valid(SdadcConfig{.reference = SdadcRef::dac,
                                             .reference_buffer = true}));
// The external pin needs no buffer.
static_assert(sdadc_config_valid(SdadcConfig{.reference = SdadcRef::vrefb}));
// A GAINCORR of zero multiplies every result away.
static_assert(!sdadc_config_valid(SdadcConfig{.gain_correction = 0}));
static_assert(!sdadc_config_valid(SdadcConfig{.gain_correction = 0x4000}));
static_assert(!sdadc_config_valid(SdadcConfig{.osr = static_cast<SdadcOsr>(5)}));
static_assert(!sdadc_config_valid(SdadcConfig{.skip_count = 16}));
// 39.6.2.3's invalid first samples: a SINGLE conversion must skip two
// decimation windows. Free running warms up once and is exempt.
static_assert(!sdadc_config_valid(SdadcConfig{.skip_count = 0}));
static_assert(!sdadc_config_valid(SdadcConfig{.skip_count = 1}));
static_assert(sdadc_config_valid(SdadcConfig{.skip_count = 0, .free_running = true}));
static_assert(!sdadc_config_valid(SdadcConfig{.window = static_cast<SdadcWindow>(5)}));
static_assert(!sdadc_config_valid(SdadcConfig{.shift_correction = 16}));
static_assert(!sdadc_config_valid(SdadcConfig{.bias_control = 0x40}));
static_assert(!sdadc_config_valid(SdadcConfig{.events = {.invert_start = true}}));
static_assert(!sdadc_config_valid(SdadcConfig{.events = {.invert_flush = true}}));
// A sequence bit for a pair this package does not bond.
static_assert(sdadc_config_valid(SdadcConfig{.sequence = 0x1}));
#if defined(__SAMC21E18A__)
static_assert(!sdadc_config_valid(SdadcConfig{.sequence = 0x3}));
#else
static_assert(sdadc_config_valid(SdadcConfig{.sequence = 0x3}));
#endif

// ---- every verb must compile ------------------------------------------------

constexpr SdadcConfig fixture_cfg{
    .reference = SdadcRef::vddana,
    .prescaler = 3,
    .osr = SdadcOsr::osr256,
    .chopper = true,
};

void use() {
    (void)Sdadc::init(0, fixture_cfg, 48'000'000);
    (void)Sdadc::init<fixture_cfg>(0, 48'000'000);
    (void)Sdadc::select(0);
    (void)Sdadc::select<0>();
    (void)Sdadc::selected();
    (void)Sdadc::pair_exists(1);

    Sdadc::claim_negative<Pin<'A', 6>>();
    Sdadc::claim_positive<Pin<'A', 7>>();
    Sdadc::claim_vrefb<Pin<'A', 4>>();
    Sdadc::release_pad<Pin<'A', 6>>();

    (void)Sdadc::start();
    (void)Sdadc::flush();
    (void)Sdadc::ready();
    (void)Sdadc::overrun();
    (void)Sdadc::result();
    (void)Sdadc::result_raw();
    (void)Sdadc::result24();
    int16_t v = 0;
    (void)Sdadc::read(v);
    (void)Sdadc::read();
    (void)Sdadc::next(v);
    Sdadc::discard(2);

    (void)Sdadc::free_running(true);
    (void)Sdadc::free_running();
    (void)Sdadc::window(SdadcWindow::inside, -100, 100);
    (void)Sdadc::window_raw(SdadcWindow::inside, -25600, 25600);
    (void)Sdadc::window_off();
    (void)Sdadc::window_mode();
    (void)Sdadc::window_hit();
    (void)Sdadc::window_flag();

    (void)Sdadc::offset_correction(-1000);
    (void)Sdadc::offset_correction();
    (void)Sdadc::gain_correction(3);
    (void)Sdadc::gain_correction();
    (void)Sdadc::shift_correction(1);
    (void)Sdadc::shift_correction();

    (void)Sdadc::sequence(0x1);
    (void)Sdadc::sequence();
    (void)Sdadc::sequence_busy();
    (void)Sdadc::sequence_state();

    (void)Sdadc::analog_control(fixture_cfg);
    (void)Sdadc::analog_control();
    (void)Sdadc::chopper();
    (void)Sdadc::reference(fixture_cfg);
    (void)Sdadc::reference_register();
    (void)Sdadc::reference_buffer();
    (void)Sdadc::reference_mv(5100);
    (void)Sdadc::to_mv(1000, 5100);

    Sdadc::arm(Sdadc::flag_resrdy);
    Sdadc::disarm(Sdadc::flag_resrdy);
    (void)Sdadc::armed();
    (void)Sdadc::flags();
    Sdadc::clear_flags(Sdadc::flag_overrun);
    (void)Sdadc::isr();
    (void)Sdadc::resrdy();

    (void)Sdadc::event_config(SdadcEventControl{.start_in = true});
    (void)Sdadc::event_config();
    (void)Sdadc::start_on(0, EventChannelConfig{.path = EventPath::asynchronous});
    (void)Sdadc::flush_on(1, EventChannelConfig{.path = EventPath::asynchronous});
    (void)Sdadc::stop_events();

    (void)Sdadc::enable(false);
    (void)Sdadc::enabled();
    (void)Sdadc::sync_busy();
    (void)Sdadc::sync_wait(0);
    (void)Sdadc::reset();
    Sdadc::release();
}

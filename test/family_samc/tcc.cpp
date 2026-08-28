// Family smoke TU for samc/tcc.hpp: every verb must COMPILE on the E, G
// and J 18A headers (tools/check_samc.sh sweeps all three).
//
// The TCC block is where this family stops being uniform. All three
// variants carry all three instances, but the INSTANCES differ from each
// other in five ways at once - counter width, channel count, output
// count, which of the five waveform-extension units they implement, and
// which generic clock channel they share - and what varies per PACKAGE
// is which pads carry a waveform output, on WHICH of the two peripheral
// functions. Both sets of facts are the device header's own TCCn_* and
// PIN_P<pad><fn>_TCC<n>_WO<k> constants, and this fixture asserts them
// so that a reader does not have to take the driver's word for any of it.

#include <stdint.h>

#include "samc/clock.hpp"
#include "samc/device_tables.hpp"
#include "samc/pin.hpp"
#include "samc/tcc.hpp"
#include "util/pwm_channel.hpp"

using namespace brio;

// ---- the geometry, instance by instance ------------------------------------

static_assert(tcc_count() == 3, "this family implements TCC0..TCC2");

static_assert(Tcc<0>::cc_count == TCC0_CC_NUM && Tcc<0>::cc_count == 4);
static_assert(Tcc<1>::cc_count == TCC1_CC_NUM && Tcc<1>::cc_count == 2);
static_assert(Tcc<2>::cc_count == TCC2_CC_NUM && Tcc<2>::cc_count == 2);

static_assert(Tcc<0>::wo_count == 8 && Tcc<1>::wo_count == 4 &&
              Tcc<2>::wo_count == 2);

// 36.8.15: TCC0 and TCC1 are 24-bit counters, TCC2 is 16-bit, and "the
// excess bits are read zero" on the narrow one.
static_assert(Tcc<0>::counter_bits == 24 && Tcc<0>::max_count == 0xFFFFFFul);
static_assert(Tcc<2>::counter_bits == 16 && Tcc<2>::max_count == 0xFFFFul);

// 36.5.3, in one sentence: "TCC0 and TCC1 share a peripheral clock
// generator".
static_assert(Tcc<0>::gclk_id == TCC0_GCLK_ID && Tcc<1>::gclk_id == TCC0_GCLK_ID);
static_assert(Tcc<2>::gclk_id != Tcc<0>::gclk_id);

// 36.6.4: TCC0 is the host of the host/client pair, TCC1 the client -
// the only instance that may set CTRLA.MSYNC - and TCC2 is neither.
static_assert(Tcc<0>::is_pair_host && !Tcc<0>::is_pair_client);
static_assert(Tcc<1>::is_pair_client && !Tcc<1>::is_pair_host);
static_assert(!Tcc<2>::is_pair_host && !Tcc<2>::is_pair_client);

// The five optional units, each from its own header constant.
static_assert(Tcc<0>::has_dead_time && Tcc<0>::has_output_matrix &&
              Tcc<0>::has_swap && Tcc<0>::has_pattern && Tcc<0>::has_dithering);
static_assert(!Tcc<1>::has_dead_time && !Tcc<1>::has_output_matrix &&
              !Tcc<1>::has_swap && Tcc<1>::has_pattern && Tcc<1>::has_dithering);
static_assert(!Tcc<2>::has_dead_time && !Tcc<2>::has_output_matrix &&
              !Tcc<2>::has_swap && !Tcc<2>::has_pattern && !Tcc<2>::has_dithering);

// TCCn_EXT is the header's own one-word summary of those five bits, and
// this is the decoding - OTMX, DTI, SWAP, PG, DITHERING from bit 0 up.
constexpr uint8_t ext_of(uint8_t n) {
    return static_cast<uint8_t>((tcc_has_output_matrix(n) ? 1u : 0u) |
                                (tcc_has_dead_time(n) ? 2u : 0u) |
                                (tcc_has_swap(n) ? 4u : 0u) |
                                (tcc_has_pattern(n) ? 8u : 0u) |
                                (tcc_has_dithering(n) ? 16u : 0u));
}
static_assert(ext_of(0) == TCC0_EXT && TCC0_EXT == 31);
static_assert(ext_of(1) == TCC1_EXT && TCC1_EXT == 24);
static_assert(ext_of(2) == TCC2_EXT && TCC2_EXT == 0);

// 36.6.3.7: slice x drives the pair (WO[x], WO[x + WO_NUM/2]), so an
// instance has one slice per channel but never more than half its
// outputs - and none at all without the dead-time unit.
static_assert(Tcc<0>::slice_count == 4);
static_assert(Tcc<1>::slice_count == 0 && Tcc<2>::slice_count == 0);

static_assert(Tcc<0>::irq() == TCC0_IRQn && Tcc<2>::irq() == TCC2_IRQn);

// ---- the EVSYS and DMAC vocabularies ---------------------------------------
//
// The generator codes are NOT evenly spaced: TCC0 spends seven of them
// (OVF, TRG, CNT and four MCs), TCC1 and TCC2 five each. Reading them
// from the header's own EVENT_ID_* constants is what keeps that from
// being an arithmetic bug.
static_assert(Tcc<0>::overflow_generator == EVENT_ID_GEN_TCC0_OVF);
static_assert(Tcc<0>::retrigger_generator == EVENT_ID_GEN_TCC0_TRG);
static_assert(Tcc<0>::count_generator == EVENT_ID_GEN_TCC0_CNT);
static_assert(Tcc<0>::match_generator(3) == EVENT_ID_GEN_TCC0_MC_3);
static_assert(Tcc<1>::overflow_generator == EVENT_ID_GEN_TCC1_OVF);
static_assert(Tcc<2>::match_generator(1) == EVENT_ID_GEN_TCC2_MC_1);
static_assert(Tcc<1>::overflow_generator - Tcc<0>::overflow_generator == 7);
static_assert(Tcc<2>::overflow_generator - Tcc<1>::overflow_generator == 5);

static_assert(Tcc<0>::event_user(0) == EVENT_ID_USER_TCC0_EV_0);
static_assert(Tcc<0>::event_user(1) == EVENT_ID_USER_TCC0_EV_1);
static_assert(Tcc<0>::match_user(2) == EVENT_ID_USER_TCC0_MC_2);
static_assert(Tcc<2>::event_user(0) == EVENT_ID_USER_TCC2_EV_0);
// The recoverable faults ARE the first two channel event inputs.
static_assert(Tcc<0>::fault_user(TccFault::a) == EVENT_ID_USER_TCC0_MC_0);
static_assert(Tcc<0>::fault_user(TccFault::b) == EVENT_ID_USER_TCC0_MC_1);

static_assert(Tcc<0>::dma_trigger_overflow == TCC0_DMAC_ID_OVF);
static_assert(Tcc<0>::dma_trigger_match(3) == TCC0_DMAC_ID_MC3);
static_assert(Tcc<2>::dma_trigger_match(1) == TCC2_DMAC_ID_MC1);

// ---- the refusals ----------------------------------------------------------

// ERRATUM 1.21.10: ALOCK is not functional and has no workaround, so
// the driver never writes it.
static_assert(!tcc_config_valid(0, TccConfig{.auto_lock = true}),
              "erratum 1.21.10: the ALOCK feature is not functional");

// Dithering exists on TCC0 and TCC1 only.
static_assert(tcc_config_valid(0, TccConfig{.resolution = TccResolution::dither64}));
static_assert(tcc_config_valid(1, TccConfig{.resolution = TccResolution::dither64}));
static_assert(!tcc_config_valid(2, TccConfig{.resolution = TccResolution::dither16}));

// 36.6.4: CTRLA.MSYNC is "only for TCC Client instance".
static_assert(tcc_config_valid(1, TccConfig{.host_sync = true}));
static_assert(!tcc_config_valid(0, TccConfig{.host_sync = true}));
static_assert(!tcc_config_valid(2, TccConfig{.host_sync = true}));

// A capture-enable bit past the channels an instance has.
static_assert(tcc_config_valid(0, TccConfig{.capture_enable = 0xF}));
static_assert(!tcc_config_valid(1, TccConfig{.capture_enable = 0x4}));

// RAMP2C is a variant-L encoding (36.8.17) and is refused everywhere.
static_assert(tcc_wave_valid(0, TccWaveConfig{.ramp = TccRamp::ramp2}));
static_assert(!tcc_wave_valid(0, TccWaveConfig{.ramp = TccRamp::ramp2_critical}));

// The swap unit and the output matrix are TCC0's alone here.
static_assert(tcc_wave_valid(0, TccWaveConfig{.swap = 0x3}));
static_assert(!tcc_wave_valid(1, TccWaveConfig{.swap = 0x1}));
static_assert(!tcc_wave_valid(0, TccWaveConfig{.polarity = 0xF0}));
static_assert(!tcc_wave_valid(2, TccWaveConfig{.circular_cc = 0x4}));

static_assert(tcc_wave_ext_valid(0, TccWaveExtConfig{
    .output_matrix = TccOutputMatrix::broadcast_cc0}));
static_assert(!tcc_wave_ext_valid(1, TccWaveExtConfig{
    .output_matrix = TccOutputMatrix::broadcast_cc0}));
static_assert(tcc_wave_ext_valid(0, TccWaveExtConfig{.dead_time_enable = 0x8}));
static_assert(!tcc_wave_ext_valid(0, TccWaveExtConfig{.dead_time_enable = 0x10}));
static_assert(!tcc_wave_ext_valid(1, TccWaveExtConfig{.dead_time_enable = 0x1}));

// DRVCTRL masks name OUTPUTS, of which TCC2 has two.
static_assert(tcc_drive_valid(0, TccDriveConfig{.invert = 0xFF}));
static_assert(!tcc_drive_valid(2, TccDriveConfig{.invert = 0x4}));
static_assert(!tcc_drive_valid(0, TccDriveConfig{.filter0 = 0x10}));

// A fault's capture channel must exist.
static_assert(tcc_fault_valid(0, TccFaultConfig{.capture_channel = 3}));
static_assert(!tcc_fault_valid(1, TccFaultConfig{.capture_channel = 2}));
static_assert(!tcc_fault_valid(0, TccFaultConfig{.filter_value = 0x10}));

// Pattern generation: TCC0 and TCC1 have it, TCC2 does not.
static_assert(tcc_pattern_valid(0, 0xFF, 0x0F));
static_assert(tcc_pattern_valid(1, 0x0F, 0x03));
static_assert(!tcc_pattern_valid(1, 0x10, 0x00));
static_assert(!tcc_pattern_valid(2, 0x01, 0x01));

// 36.6.2.7: a capture action needs a channel enabled in capture mode.
static_assert(!tcc_event_config_valid(
                  0, TccConfig{},
                  TccEventConfig{.action1 = TccEvent1Action::period_pulse_width}),
              "a PPW capture with no capture channel captures nothing");
static_assert(tcc_event_config_valid(
    0, TccConfig{.capture_enable = 0x3},
    TccEventConfig{.action1 = TccEvent1Action::period_pulse_width}));
static_assert(!tcc_event_config_valid(
    0, TccConfig{}, TccEventConfig{.action0 = TccEvent0Action::stamp}));
static_assert(!tcc_event_config_valid(2, TccConfig{},
                                      TccEventConfig{.match_out = 0x4}));

// The EVCTRL.MCEIx bit a recoverable fault's own input needs (36.6.3.5).
static_assert(tcc_fault_input_bit(TccFault::a) == 0x1);
static_assert(tcc_fault_input_bit(TccFault::b) == 0x2);

// ---- the dithering arithmetic ----------------------------------------------

static_assert(tcc_dither_bits(TccResolution::none) == 0);
static_assert(tcc_dither_frame(TccResolution::dither64) == 64);
static_assert(tcc_dither(100, 0, TccResolution::none) == 100);
static_assert(tcc_dither(100, 32, TccResolution::dither64) == ((100u << 6) | 32u));
static_assert(tcc_dither(100, 8, TccResolution::dither16) == ((100u << 4) | 8u));
// The extra-cycle count is masked to the frame it belongs to.
static_assert(tcc_dither(1, 0xFF, TccResolution::dither32) == ((1u << 5) | 31u));

static_assert(tcc_prescaler_divisor(TccPrescaler::div1024) == 1024);
static_assert(tcc_waveform_is_dual_slope(TccWaveform::dual_slope_both));
static_assert(!tcc_waveform_is_dual_slope(TccWaveform::normal_pwm));
static_assert(tcc_event1_is_capture(TccEvent1Action::pulse_width_period));
static_assert(tcc_capture_is_advanced(TccFaultCapture::capture_max));
static_assert(!tcc_capture_is_advanced(TccFaultCapture::capture));

// ---- the pad table: keyed by pad AND function, per package -----------------

// THE FACT THAT MAKES THIS MAP DIFFERENT FROM THE TC'S: the same pad
// carries two different outputs of two different instances, one under
// each of the two peripheral functions.
static_assert(TccWo<Pin<'A', 8>, PinFunction::e>::timer == 0 &&
              TccWo<Pin<'A', 8>, PinFunction::e>::output == 0);
static_assert(TccWo<Pin<'A', 8>, PinFunction::f>::timer == 1 &&
              TccWo<Pin<'A', 8>, PinFunction::f>::output == 2);
static_assert(TccWo<Pin<'A', 16>, PinFunction::e>::timer == 2);
static_assert(TccWo<Pin<'A', 16>, PinFunction::f>::timer == 0 &&
              TccWo<Pin<'A', 16>, PinFunction::f>::output == 6);
static_assert(TccWo<Pin<'A', 22>, PinFunction::f>::timer == 0 &&
              TccWo<Pin<'A', 22>, PinFunction::f>::output == 4);

// A pad with no TCC output on that function, and one with none at all.
static_assert(!tcc_wo_exists<'A', 22, 'e'>);
static_assert(!tcc_wo_exists<'B', 23, 'e'> && !tcc_wo_exists<'B', 23, 'f'>);

#if defined(__SAMC21E18A__)
// The E package bonds no PA12/PA13 and no PORTB pad below PB30 at all.
static_assert(!tcc_wo_exists<'A', 12, 'e'> && !tcc_wo_exists<'A', 12, 'f'>);
static_assert(!tcc_wo_exists<'B', 10, 'f'> && !tcc_wo_exists<'B', 30, 'e'>);
static_assert(!tcc_wo_exists<'A', 20, 'f'>);
#elif defined(__SAMC21G18A__)
static_assert(tcc_wo_exists<'A', 12, 'e'> && tcc_wo_exists<'A', 12, 'f'>);
static_assert(tcc_wo_exists<'B', 10, 'f'> && !tcc_wo_exists<'B', 16, 'f'>);
static_assert(!tcc_wo_exists<'B', 30, 'e'>);
#else
// The J bonds the widest map, including the console pads: PB30 is
// TCC0/WO0 under E and TCC1/WO2 under F.
static_assert(TccWo<Pin<'B', 30>, PinFunction::e>::timer == 0);
static_assert(TccWo<Pin<'B', 30>, PinFunction::f>::timer == 1 &&
              TccWo<Pin<'B', 30>, PinFunction::f>::output == 2);
static_assert(TccWo<Pin<'B', 16>, PinFunction::f>::timer == 0 &&
              TccWo<Pin<'B', 16>, PinFunction::f>::output == 4);
// One output, several pads: TCC0/WO4 is on PA14, PA22, PB10 and PB16.
static_assert(TccWo<Pin<'A', 14>, PinFunction::f>::output == 4);
static_assert(TccWo<Pin<'B', 10>, PinFunction::f>::output == 4);
#endif

// ---- the tasks satisfy the util contract -----------------------------------

static_assert(PwmChannel<TccPwm<Tcc<0>, 0, 999>>);
static_assert(PwmChannel<TccPwm<Tcc<2>, 1, 255>>);
static_assert(PwmChannel<TccPairPwm<Tcc<0>, 0, 1999>>);
static_assert(TccPwm<Tcc<0>, 0, 999>::max == 999);
static_assert(TccPairPwm<Tcc<0>, 1, 1999>::low_output == 1 &&
              TccPairPwm<Tcc<0>, 1, 1999>::high_output == 5);

// ---- every verb, compiled --------------------------------------------------

void resource_verbs() {
    using T = Tcc<0>;
    constexpr TccConfig cfg{.prescaler = TccPrescaler::div64,
                            .prescaler_sync = TccPrescalerSync::resync,
                            .resolution = TccResolution::dither64,
                            .capture_enable = 0x3,
                            .run_standby = true,
                            .dma_one_shot = true,
                            .count_down = true,
                            .one_shot = true,
                            .lock_update = true};

    (void)T::init(0);
    T::bus_clock(true);
    (void)T::clock(0);
    (void)T::reset();
    (void)T::configure(cfg);
    (void)T::wave(TccWaveConfig{.waveform = TccWaveform::dual_slope_both,
                                .ramp = TccRamp::ramp2,
                                .polarity = 0x5,
                                .swap = 0x3,
                                .circular_cc = 0x1,
                                .circular_period = true});
    (void)T::wave_extension(TccWaveExtConfig{
        .output_matrix = TccOutputMatrix::cc0_then_cc1,
        .dead_time_enable = 0x3,
        .dead_time_low = 40,
        .dead_time_high = 60});
    (void)T::drive(TccDriveConfig{.invert = 0x11,
                                  .fault_output_enable = 0xFF,
                                  .fault_output_value = 0x0F,
                                  .filter0 = 3,
                                  .filter1 = 4});
    (void)T::fault(TccFault::a,
                   TccFaultConfig{.source = TccFaultSource::event,
                                  .halt = TccFaultHalt::software,
                                  .capture = TccFaultCapture::capture,
                                  .capture_channel = 2,
                                  .keep = true,
                                  .qualify = true,
                                  .restart = true,
                                  .blank = TccFaultBlank::rising_edge,
                                  .blank_value = 20,
                                  .filter_value = 5});
    (void)T::fault(TccFault::b,
                   TccFaultConfig{.source = TccFaultSource::inverted_event,
                                  .halt = TccFaultHalt::hardware});
    (void)T::event_config(cfg,
                          TccEventConfig{.action0 = TccEvent0Action::fault,
                                         .action1 = TccEvent1Action::period_pulse_width,
                                         .input0_enable = true,
                                         .input1_enable = true,
                                         .invert0 = true,
                                         .invert1 = true,
                                         .match_in = 0x3,
                                         .match_out = 0xF,
                                         .overflow_out = true,
                                         .retrigger_out = true,
                                         .count_out = true,
                                         .count_select = TccCountEvent::boundary});

    (void)T::ctrla();
    (void)T::evctrl();
    (void)T::wave_reg();
    (void)T::wave_ext_reg();
    (void)T::drive_reg();
    (void)T::fault_reg(TccFault::b);
    (void)T::syncbusy();
    (void)T::waveform();
    (void)T::ramp();
    (void)T::enable(true);
    (void)T::enabled();

    (void)T::command(TccCommand::dma_one_shot);
    (void)T::command(TccCommand::none);   // cancels, through CTRLBCLR
    (void)T::ramp_index_command(TccRampIndexCommand::off);
    (void)T::retrigger();
    (void)T::stop();
    (void)T::update();
    (void)T::ramp_index_command(TccRampIndexCommand::force_b);
    (void)T::count_down(true);
    (void)T::counting_down();
    (void)T::lock_update(true);
    (void)T::update_locked();
    (void)T::one_shot(false);

    (void)T::read_sync();
    (void)T::count();
    (void)T::count_raw();
    (void)T::set_count(1234);

    (void)T::period();
    (void)T::set_period(999);
    (void)T::period_buffer();
    (void)T::set_period_buffer(888);
    (void)T::cc(0);
    (void)T::set_cc(0, 100);
    (void)T::cc_buffer(0);
    (void)T::set_cc_buffer(0, 200);

    (void)T::pattern(0x0F, 0x05);
    (void)T::pattern_buffer(0x0F, 0x0A);
    (void)T::pattern_reg();

    (void)T::sync_busy(TCC_SYNCBUSY_ENABLE_Msk);
    (void)T::cc_sync_busy(0);
    (void)T::period_sync_busy();
    (void)T::pattern_sync_busy();

    (void)T::status();
    (void)T::stopped();
    (void)T::ramp_index();
    (void)T::is_client();
    (void)T::period_buffer_valid();
    (void)T::cc_buffer_valid(1);
    (void)T::pattern_buffer_valid();
    (void)T::compare_output(0);
    (void)T::clear_buffer_valid(TCC_STATUS_CCBUFV0_Msk);

    (void)T::fault_input(TccFault::a);
    (void)T::fault_state(TccFault::b);
    (void)T::clear_fault_state(TccFault::a);
    (void)T::non_recoverable_input(0);
    (void)T::non_recoverable_state(1);
    (void)T::clear_non_recoverable(0);
    (void)T::debug_fault_state();
    (void)T::clear_debug_fault();
    (void)T::update_fault_state();
    (void)T::clear_update_fault();

    (void)T::flags();
    T::clear_flags(T::overflow_flag | T::retrigger_flag | T::count_flag |
                   T::error_flag | T::update_fault_flag | T::debug_fault_flag |
                   T::fault_a_flag | T::fault_b_flag |
                   T::non_recoverable_flag(1) | T::match_flag(3) |
                   T::recoverable_flag(TccFault::b));
    T::arm(T::overflow_flag);
    T::disarm(T::overflow_flag);
    (void)T::armed();
    (void)T::isr();

    T::debug_run(true);
    T::fault_on_debug(true);
    (void)T::debug_reg();
    T::release();
}

void narrow_instance_verbs() {
    // TCC2: 16 bits, two channels, two outputs, no extension at all -
    // the same surface, refusing what it does not have.
    using T = Tcc<2>;
    (void)T::init(0);
    (void)T::configure(TccConfig{.prescaler = TccPrescaler::div16});
    (void)T::wave(TccWaveConfig{.waveform = TccWaveform::normal_pwm});
    (void)T::set_period(0xFFFF);
    (void)T::set_cc_buffer(1, 0x8000);
    (void)T::enable(true);
    T::release();
}

void task_verbs() {
    using Low = TccWo<Pin<'A', 8>, PinFunction::e>;    // TCC0 / WO0
    using High = TccWo<Pin<'A', 22>, PinFunction::f>;  // TCC0 / WO4
    Low::claim();
    High::claim();

    (void)TccPwm<Tcc<0>, 0, 999>::setup(TccPrescaler::div8);
    TccPwm<Tcc<0>, 0, 999>::duty(500);
    (void)TccPairPwm<Tcc<0>, 0, 1999>::setup(TccPrescaler::div1, 40, 40);
    TccPairPwm<Tcc<0>, 0, 1999>::duty(1000);

    High::release();
    Low::release();
}

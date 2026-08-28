// Family smoke TU for samc/tsens.hpp: every verb must COMPILE on the E,
// G and J 18A headers (tools/check_samc.sh sweeps all three). This
// peripheral has ONE instance and NO PADS AT ALL (43.5.1 is "Not
// applicable"), so unlike the three converters there is no pad map to
// vary - which is exactly what this file asserts, per variant, out of
// the device header's own symbols rather than out of a claim.
//
// The arithmetic asserted below is the chapter's own: 43.8.10's worked
// example (2500 = 25 C, -2500 = -25 C) and 43.6.1's note that the
// factory GAIN and OFFSET belong to a 48 MHz GCLK_TSENS.

#include <stdint.h>

#include "samc/tsens.hpp"
#include "samc/clock.hpp"
#include "samc/platform_sam.hpp"

using namespace brio;

// ---- geometry, from the device header's own constants ----------------------

static_assert(tsens_count() == 1);
static_assert(Tsens::gclk_id == 5);
static_assert(Tsens::pac_id == 12);

// ---- the vocabularies this peripheral publishes ----------------------------

static_assert(Tsens::window_generator == 30);
// USER 0: the first row of table 29-3, and the only START user on this
// family that accepts every propagation path.
static_assert(Tsens::start_event_user == 0);
static_assert(Tsens::dma_trigger_resrdy == 1);

// ---- the datum: signed 24 bits, two's complement -----------------------------

static_assert(tsens_field_mask == 0x00FFFFFFUL);
static_assert(tsens_signed(0x000000UL) == 0);
static_assert(tsens_signed(0x0009C4UL) == 2500);      // 43.8.10: T = 25 C
static_assert(tsens_signed(0xFFF63CUL) == -2500);     // 43.8.10: T = -25 C
static_assert(tsens_signed(0x7FFFFFUL) == tsens_value_max);
static_assert(tsens_signed(0x800000UL) == tsens_value_min);

static_assert(tsens_field(2500) == 0x0009C4UL);
static_assert(tsens_field(-2500) == 0xFFF63CUL);
static_assert(tsens_field(tsens_value_min) == 0x800000UL);

static_assert(tsens_value_fits(0));
static_assert(tsens_value_fits(tsens_value_max));
static_assert(tsens_value_fits(tsens_value_min));
static_assert(!tsens_value_fits(tsens_value_max + 1));
static_assert(!tsens_value_fits(tsens_value_min - 1));

static_assert(tsens_milli_celsius(2500) == 25000);
static_assert(tsens_milli_celsius(-2500) == -25000);

// ---- the clock IS the ruler ---------------------------------------------------
//
// VALUE - OFFSET is proportional to GAIN / f_GCLK, so halving the clock
// doubles the span and halving the GAIN puts it back.

static_assert(tsens_calibration_gclk_hz == 48'000'000UL);
// A reading of 2500 taken at 48 MHz is already what it should be.
static_assert(tsens_rescale(2500, -27315, 48'000'000UL) == 2500);
// The same reading taken at 24 MHz is twice the span from the offset, so
// rescaling it halves that span: -27315 + (2500 + 27315) / 2 = -12408.
static_assert(tsens_rescale(2500, -27315, 24'000'000UL) == -12408);
// A zero span is a zero span at any rate.
static_assert(tsens_rescale(-27315, -27315, 12'000'000UL) == -27315);
// A rate of zero is not a rate; the value comes back untouched.
static_assert(tsens_rescale(2500, -27315, 0) == 2500);

static_assert(tsens_gain_for(1000, 48'000'000UL) == 1000);
static_assert(tsens_gain_for(1000, 24'000'000UL) == 500);
static_assert(tsens_gain_for(1000, 96'000'000UL) == 2000);
// Rounded to nearest, not truncated: 1001 x 24/48 = 500.5 -> 501.
static_assert(tsens_gain_for(1001, 24'000'000UL) == 501);
// And clamped to the 24-bit field rather than wrapping into nothing.
static_assert(tsens_gain_for(0xFFFFFFUL, 96'000'000UL) == tsens_field_mask);

// ---- the window vocabulary ----------------------------------------------------

static_assert(tsens_window_valid(TsensWindow::disabled));
static_assert(tsens_window_valid(TsensWindow::hysteresis_below));
static_assert(!tsens_window_valid(static_cast<TsensWindow>(7)));   // Reserved

static_assert(tsens_window_needs_ordered_pair(TsensWindow::inside));
static_assert(tsens_window_needs_ordered_pair(TsensWindow::hysteresis_above));
static_assert(tsens_window_needs_ordered_pair(TsensWindow::hysteresis_below));
// `outside` is deliberately NOT in the set: 43.8.3 and the device
// header's own enumerator comment describe it with the thresholds in
// opposite orders, so nothing here refuses either.
static_assert(!tsens_window_needs_ordered_pair(TsensWindow::outside));
static_assert(!tsens_window_needs_ordered_pair(TsensWindow::above));

// ---- the configuration's rules -------------------------------------------------

constexpr TsensCalibration made_up{.gain = 3000, .offset = -27315, .tcal = 20, .fcal = 30};
static_assert(made_up.programmed());
static_assert(!TsensCalibration{}.programmed());
static_assert(!TsensCalibration{.gain = tsens_field_mask}.programmed());
static_assert(made_up.cal_word() == ((20u << 8) | 30u));

static_assert(tsens_config_valid(TsensConfig{.calibration = made_up}));
// A default configuration carries GAIN 0 - the reset value - and a zero
// GAIN is 2^24, not none (a 699 ms conversion, amplified two hundredfold).
static_assert(!tsens_config_valid(TsensConfig{}));
static_assert(!tsens_config_valid(TsensConfig{
    .calibration = made_up, .window = static_cast<TsensWindow>(7)}));
static_assert(!tsens_config_valid(TsensConfig{.calibration = made_up,
                                              .window = TsensWindow::inside,
                                              .window_lower = 5000,
                                              .window_upper = 1000}));
static_assert(tsens_config_valid(TsensConfig{.calibration = made_up,
                                             .window = TsensWindow::inside,
                                             .window_lower = 1000,
                                             .window_upper = 5000}));
// The crossed pair is accepted for `outside` alone, and knowingly.
static_assert(tsens_config_valid(TsensConfig{.calibration = made_up,
                                             .window = TsensWindow::outside,
                                             .window_lower = 5000,
                                             .window_upper = 1000}));
static_assert(!tsens_config_valid(TsensConfig{.calibration = made_up,
                                              .window = TsensWindow::above,
                                              .window_lower = tsens_value_max + 1}));
static_assert(!tsens_config_valid(TsensConfig{
    .calibration = made_up, .events = TsensEventControl{.invert_start = true}}));
static_assert(tsens_config_valid(TsensConfig{
    .calibration = made_up,
    .events = TsensEventControl{.start_in = true, .invert_start = true}}));

static_assert(tsens_event_control_valid(TsensEventControl{}));
static_assert(tsens_event_control_valid(TsensEventControl{.window_out = true}));
static_assert(!tsens_event_control_valid(TsensEventControl{.invert_start = true}));

// ---- every verb, instantiated ---------------------------------------------------

constexpr TsensConfig smoke_cfg{
    .calibration = made_up,
    .free_running = true,
    .window = TsensWindow::hysteresis_above,
    .window_lower = 2000,
    .window_upper = 6000,
    .run_standby = true,
    .debug_run = true,
    .events = TsensEventControl{.start_in = true, .window_out = true},
};

void use() {
    (void)Tsens::init(0, smoke_cfg);
    (void)Tsens::init<smoke_cfg>(0);
    (void)Tsens::bus_clock(true);
    (void)Tsens::clock(0);
    (void)Tsens::sync_busy();
    (void)Tsens::sync_wait(TSENS_SYNCBUSY_ENABLE_Msk);
    (void)Tsens::reset();
    (void)Tsens::enable(true);
    (void)Tsens::enabled();
    (void)Tsens::config();

    (void)Tsens::calibration(TsensCalibration::factory());
    (void)Tsens::calibration();
    (void)Tsens::offset();
    (void)Tsens::gain();

    Tsens::start();
    (void)Tsens::flags();
    Tsens::clear_flags();
    Tsens::arm(Tsens::flag_result_ready | Tsens::flag_window);
    Tsens::disarm(Tsens::flag_all);
    (void)Tsens::armed();
    (void)Tsens::result_ready();
    (void)Tsens::overrun();
    (void)Tsens::window_hit();
    (void)Tsens::overflow_flag();
    (void)Tsens::overflowed();
    (void)Tsens::value_raw();
    (void)Tsens::value();
    (void)Tsens::read();
    (void)Tsens::measure();
    (void)Tsens::measure_average(10);
    (void)Tsens::rescaled(2500, 24'000'000UL);
    (void)Tsens::isr();

    (void)Tsens::window(TsensWindow::below, 0, 4000);
    (void)Tsens::window();
    (void)Tsens::window_lower();
    (void)Tsens::window_upper();
    (void)Tsens::free_running(true);
    (void)Tsens::free_running();

    (void)Tsens::event_config(TsensEventControl{.start_in = true});
    (void)Tsens::event_config();
    // ALL THREE PATHS are legal into this user (table 29-3), which is the
    // difference from the DAC's and the SDADC's identical-looking verbs.
    (void)Tsens::start_on(0, EventChannelConfig{.path = EventPath::asynchronous});
    (void)Tsens::start_on(0, EventChannelConfig{.path = EventPath::synchronous,
                                                .edge = EventEdge::rising});
    (void)Tsens::start_on(0, EventChannelConfig{.path = EventPath::resynchronized,
                                                .edge = EventEdge::rising});
    (void)Tsens::stop_events();
    (void)Tsens::irq();
    Tsens::release();
}

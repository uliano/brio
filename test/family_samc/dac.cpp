// Family smoke TU for samc/dac.hpp: every verb must COMPILE on the E, G
// and J 18A headers (tools/check_samc.sh sweeps all three). This
// peripheral has ONE instance and its two analog pads are PORT A pads
// every variant bonds, so unlike the ADC's there is no pad map to vary -
// which is exactly what this file asserts, per variant, out of the
// device header's own symbols rather than out of a claim.

#include <stdint.h>

#include "samc/dac.hpp"
#include "samc/clock.hpp"
#include "samc/platform_sam.hpp"

using namespace brio;

// ---- geometry, from the device header's own constants ----------------------

static_assert(dac_count() == 1);
static_assert(Dac::gclk_id == 36);
static_assert(Dac::steps == 1024);
static_assert(Dac::dither_steps == 16384);

// ---- the vocabularies this peripheral publishes ----------------------------

static_assert(Dac::empty_generator == 79);
static_assert(Dac::start_event_user == 38);
static_assert(Dac::dma_trigger_empty == 45);

// ---- the pads: PA02 is VOUT, PA03 is VREFA, on every variant ---------------

static_assert(dac_vout_pad_exists<'A', 2>);
static_assert(dac_vrefa_pad_exists<'A', 3>);
static_assert(!dac_vout_pad_exists<'A', 3>);
static_assert(!dac_vout_pad_exists<'B', 2>);
// Function B on this family, taken from the header's MUX_ symbol and not
// from a letter written down here.
static_assert(Dac::vout_function('A', 2) == static_cast<int>(PinFunction::b));
static_assert(Dac::vrefa_function('A', 3) == static_cast<int>(PinFunction::b));

// THE ZERO-LENGTH WIRE erratum 1.8.9 asks for: VOUT and ADC0/AIN0 are
// the same pad, so the "external wire" the workaround wants is already
// on the die.
static_assert(Dac::adc_input_pad_port == 'A' && Dac::adc_input_pad_pin == 2);

// ---- the reference vocabulary ----------------------------------------------

static_assert(dac_ref_valid(DacRef::intref));
static_assert(dac_ref_valid(DacRef::vddana));
static_assert(dac_ref_valid(DacRef::vrefa));
static_assert(!dac_ref_valid(static_cast<DacRef>(3)));

static_assert(dac_ref_mv(DacRef::vddana, 5100) == 5100);
static_assert(dac_ref_mv(DacRef::vrefa, 2500) == 2500);
static_assert(dac_ref_mv(DacRef::intref, 5100, VrefLevel::v1_024) == 1024);
static_assert(dac_ref_mv(DacRef::intref, 5100, VrefLevel::v4_096) == 4096);

// ---- table 41-1's placement -------------------------------------------------

static_assert(dac_data_word(512, false, false) == 512);
static_assert(dac_data_word(512, true, false) == (512u << 6));
static_assert(dac_data_word(1023, false, false) == 1023);
static_assert(dac_data_word(1023, true, false) == 0xFFC0);
// clamped, never spilling into a neighbouring field
static_assert(dac_data_word(2000, false, false) == 1023);
static_assert(dac_data_word(2000, true, false) == 0xFFC0);
// 14-bit dithered values
static_assert(dac_data_word(0x1234, false, true) == 0x1234);
static_assert(dac_data_word(0x1234, true, true) == (0x1234u << 2));
static_assert(dac_data_word(20000, false, true) == 16383);

// ---- the configuration rules ------------------------------------------------

static_assert(dac_config_valid(DacConfig{}));
static_assert(!dac_config_valid(DacConfig{.reference = static_cast<DacRef>(3)}));
// 41.6.8.3: dithering IS the event-driven mode.
static_assert(!dac_config_valid(DacConfig{.dither = true}));
static_assert(dac_config_valid(DacConfig{.dither = true,
                                         .events = {.start_in = true}}));
// an inverted event input nothing listens to
static_assert(!dac_config_valid(DacConfig{.events = {.invert_start = true}}));
static_assert(dac_config_valid(DacConfig{.events = {.start_in = true,
                                                    .invert_start = true}}));

static_assert(dac_event_control_valid(DacEventControl{.empty_out = true}));

// ---- the timing constants ---------------------------------------------------

static_assert(dac_conversion_ns == 2857);
static_assert(dac_startup_ns == 3000);

// ---- every runtime verb has to compile --------------------------------------

using Vout = Pin<'A', 2>;
using Vrefa = Pin<'A', 3>;

constexpr DacConfig runtime_cfg{
    .reference = DacRef::vddana,
    .external_output = true,
    .internal_output = true,
};

void use() {
    (void)Dac::init(0, runtime_cfg);
    (void)Dac::init<runtime_cfg>(0);
    (void)Dac::enable(false);
    (void)Dac::enabled();
    (void)Dac::reset();
    (void)Dac::clock(0);
    Dac::bus_clock(true);
    (void)Dac::sync_wait(DAC_SYNCBUSY_ENABLE_Msk);
    (void)Dac::sync_busy();
    (void)Dac::ready();
    (void)Dac::wait_ready();

    Dac::claim_vout<Vout>();
    Dac::claim_vrefa<Vrefa>();
    Dac::release_pad<Vout>();

    (void)Dac::set(512);
    Dac::buffer(256);
    (void)Dac::buffer_sync(256);
    (void)Dac::buffer_pending();
    (void)Dac::code();
    (void)Dac::set_mv(2500, 5100);
    (void)Dac::code_mv(512, 5100);

    (void)Dac::control_b(runtime_cfg);
    (void)Dac::control_b();
    (void)Dac::external_output(true);
    (void)Dac::external_output();
    (void)Dac::internal_output();
    (void)Dac::config();
    (void)Dac::reference();

    Dac::arm(Dac::flag_empty);
    Dac::disarm(Dac::flag_underrun);
    (void)Dac::armed();
    (void)Dac::flags();
    Dac::clear_flags(Dac::flag_empty | Dac::flag_underrun);
    (void)Dac::empty();
    (void)Dac::underrun();
    (void)Dac::isr();

    (void)Dac::event_config(DacEventControl{.empty_out = true});
    (void)Dac::event_config();
    (void)Dac::start_on(0, EventChannelConfig{.generator = 1,
                                              .path = EventPath::asynchronous});
    (void)Dac::stop_events();
    (void)Dac::irq();
    (void)Dac::regs().DAC_STATUS;
    Dac::release();
}

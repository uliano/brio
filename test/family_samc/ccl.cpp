// Family smoke TU for samc/ccl.hpp: every verb must COMPILE on the E, G
// and J 18A headers (tools/check_samc.sh sweeps all three), and the pad
// map has to come out DIFFERENT on each - which is the whole point of
// this chapter's per-package half.
//
// THE PACKAGE FACT THIS FILE PINS: the block always has four LUTs
// (CCL_LUT_NUM is 4 on every variant), but on the E and the G LUT3 has
// no pad of any kind - IN[9..11] and OUT[3] are bonded on the J alone.
// So a LUT can exist, be configurable and be perfectly useful while
// being unreachable from outside the chip, and only `if constexpr` on
// `Lut<3>::has_input_pad` may branch on it.

#include <stdint.h>

#include "samc/ccl.hpp"
#include "samc/clock.hpp"
#include "samc/platform_sam.hpp"

using namespace brio;

// ---- geometry, from the device header's own instance parameters -------------

static_assert(ccl_lut_count() == 4);
static_assert(ccl_seq_count() == 2);
static_assert(ccl_io_count() == 12);
static_assert(Ccl::lut_count == 4);
static_assert(Ccl::sequencer_count == 2);
static_assert(Ccl::input_count == 12);
static_assert(Ccl::gclk_id == 38);
static_assert(Ccl::pac_id == 87);   // bridge C (87 / 32 = 2), STATUS bit 23

// The two INSEL codes 37.8.3 marks "only available on SAM C20/C21 N
// variants" are absent from every header in this pack.
static_assert(!ccl_has_alt2_tc());
static_assert(!ccl_has_async_event());

// ---- the vocabularies this peripheral publishes ------------------------------

static_assert(Ccl::output_generator(0) == 0x52);
static_assert(Ccl::output_generator(3) == 0x55);
static_assert(Ccl::input_user(0) == 40);
static_assert(Ccl::input_user(3) == 43);
static_assert(Lut<0>::event_generator == 0x52);
static_assert(Lut<3>::event_user == 43);

// ---- 37.6.2.4's four formulas, evaluated per LUT ------------------------------

static_assert(Lut<0>::ac_source == 0 && Lut<3>::ac_source == 3);
static_assert(Lut<0>::tc_source == 0 && Lut<3>::tc_source == 3);
static_assert(Lut<0>::alt_tc_source == 1 && Lut<3>::alt_tc_source == 4);
static_assert(Lut<0>::tcc_source == 0 && Lut<3>::tcc_source == 0);   // 3 % 3
static_assert(Lut<0>::sercom_source == 0 && Lut<3>::sercom_source == 3);
static_assert(Lut<0>::link_source == 1 && Lut<3>::link_source == 0);  // wraps
static_assert(Lut<0>::pair == 0 && Lut<1>::pair == 0);
static_assert(Lut<2>::pair == 1 && Lut<3>::pair == 1);
static_assert(Lut<0>::is_even && !Lut<1>::is_even);

// ---- the truth table helpers ---------------------------------------------------
//
// TRUTH[k] is the output for pattern k with IN[0] as the LSB (table 37-1).

static_assert(lut_truth([](bool, bool, bool) { return false; }) == 0x00);
static_assert(lut_truth([](bool, bool, bool) { return true; }) == 0xFF);
static_assert(lut_truth([](bool a, bool, bool) { return a; }) == 0xAA);
static_assert(lut_truth([](bool, bool b, bool) { return b; }) == 0xCC);
static_assert(lut_truth([](bool, bool, bool c) { return c; }) == 0xF0);
static_assert(lut_truth([](bool a, bool b, bool) { return a && b; }) == 0x88);
static_assert(lut_truth([](bool a, bool b, bool) { return a || b; }) == 0xEE);
static_assert(lut_truth([](bool a, bool b, bool) { return a != b; }) == 0x66);
static_assert(lut_truth([](bool a, bool b, bool c) { return (a != b) != c; }) == 0x96);

static_assert(lut_truth_pass(0) == 0xAA);
static_assert(lut_truth_pass(1) == 0xCC);
static_assert(lut_truth_pass(2) == 0xF0);
static_assert(lut_truth_invert(0) == 0x55);
static_assert(lut_truth_invert(2) == 0x0F);

// ---- the configuration's rules --------------------------------------------------

static_assert(lut_input_available(LutInput::masked));
static_assert(lut_input_available(LutInput::tcc));      // 0x8: the chapter has it
static_assert(lut_input_available(LutInput::sercom));
static_assert(!lut_input_available(LutInput::alt2_tc));      // N variants only
static_assert(!lut_input_available(LutInput::async_event));  // N variants only

static_assert(lut_input_is_event(LutInput::event));
static_assert(!lut_input_is_event(LutInput::io));

static_assert(ccl_lut_config_valid(0, LutConfig{}));
// A LUT past the last is refused even with a legal configuration.
static_assert(!ccl_lut_config_valid(4, LutConfig{}));
// The edge detector without a filter or a synchronizer (37.6.2.6).
static_assert(!ccl_lut_config_valid(
    0, LutConfig{.filter = LutFilter::none, .edge_detect = true}));
static_assert(ccl_lut_config_valid(
    0, LutConfig{.filter = LutFilter::sync, .edge_detect = true}));
static_assert(ccl_lut_config_valid(
    0, LutConfig{.filter = LutFilter::filter, .edge_detect = true}));
// An event input source with LUTEI clear points the multiplexer at a
// line nothing feeds.
static_assert(!ccl_lut_config_valid(0, LutConfig{.in1 = LutInput::event}));
static_assert(ccl_lut_config_valid(
    0, LutConfig{.in1 = LutInput::event, .event_in = true}));
// INVEI inverting an event nobody listens to.
static_assert(!ccl_lut_config_valid(0, LutConfig{.invert_event_in = true}));
static_assert(ccl_lut_config_valid(
    0, LutConfig{.event_in = true, .invert_event_in = true}));
// The N-variant codes, refused wherever they are asked for.
static_assert(!ccl_lut_config_valid(0, LutConfig{.in0 = LutInput::alt2_tc}));
static_assert(!ccl_lut_config_valid(0, LutConfig{.in2 = LutInput::async_event}));

static_assert(lut_sequencer_valid(LutSequencer::none));
static_assert(lut_sequencer_valid(LutSequencer::rs_latch));
static_assert(!lut_sequencer_valid(static_cast<LutSequencer>(5)));   // Reserved

// The register word, field by field, against the device header's masks.
static_assert(ccl_lutctrl_word(LutConfig{}, false) == 0u);
static_assert(ccl_lutctrl_word(LutConfig{}, true) == CCL_LUTCTRL_ENABLE_Msk);
static_assert(ccl_lutctrl_word(LutConfig{.in0 = LutInput::io,
                                         .in1 = LutInput::ac,
                                         .in2 = LutInput::tcc,
                                         .truth = 0x96,
                                         .filter = LutFilter::filter,
                                         .edge_detect = true,
                                         .event_in = true,
                                         .invert_event_in = true,
                                         .event_out = true},
                               true) == 0x96'78'54'A2UL);

// ---- the pad map, and the package half of this chapter -------------------------
//
// PA04..PA11, PA16..PA19, PA22..PA25, PA30/PA31 carry the CCL on every
// variant of this family; PORT B carries it only where the package
// bonds those pads.

static_assert(ccl_in_line('A', 4) == 0);      // CCL0/IN[0]
static_assert(ccl_in_line('A', 16) == 0);     // ... and so is PA16
static_assert(ccl_in_line('A', 5) == 1);
static_assert(ccl_in_line('A', 6) == 2);
static_assert(ccl_in_line('A', 8) == 3);      // CCL1/IN[0]
static_assert(ccl_in_line('A', 30) == 3);     // ... and so is PA30
static_assert(ccl_in_line('A', 22) == 6);     // CCL2/IN[0]
static_assert(ccl_in_line('A', 24) == 8);
static_assert(ccl_in_line('A', 7) < 0);       // PA07 is an OUTPUT pad
static_assert(ccl_out_lut('A', 7) == 0);
static_assert(ccl_out_lut('A', 11) == 1);
static_assert(ccl_out_lut('A', 19) == 0);
static_assert(ccl_out_lut('A', 25) == 2);
static_assert(ccl_out_lut('A', 4) < 0);
static_assert(ccl_out_lut('A', 12) < 0);      // no CCL function at all

static_assert(ccl_in_exists<'A', 16> && ccl_out_exists<'A', 19>);
static_assert(!ccl_in_exists<'A', 12>);

// LUT0, LUT1 and LUT2 reach pads on every variant.
static_assert(Lut<0>::has_input_pad && Lut<0>::has_output_pad);
static_assert(Lut<1>::has_input_pad && Lut<1>::has_output_pad);
static_assert(Lut<2>::has_input_pad && Lut<2>::has_output_pad);

#if defined(__SAMC21J18A__)
// The J is the only variant that bonds LUT3 at all.
static_assert(Lut<3>::has_input_pad && Lut<3>::has_output_pad);
static_assert(ccl_in_line('B', 14) == 9 && ccl_in_line('B', 16) == 11);
static_assert(ccl_out_lut('B', 17) == 3);
static_assert(ccl_in_line('B', 6) == 6 && ccl_in_line('B', 0) == 1);
static_assert(ccl_out_lut('B', 2) == 0 && ccl_out_lut('B', 9) == 2);
#else
// E AND G: LUT3 EXISTS AND HAS NO PIN. Events, a link or a sequencer
// are the only ways in and out of it.
static_assert(Ccl::lut_count == 4);
static_assert(!Lut<3>::has_input_pad && !Lut<3>::has_output_pad);
static_assert(ccl_in_line('B', 14) < 0 && ccl_out_lut('B', 17) < 0);
#endif

#if defined(__SAMC21E18A__)
// The E bonds no PORT B pad to the CCL at all.
static_assert(ccl_in_line('B', 8) < 0 && ccl_in_line('B', 22) < 0);
static_assert(ccl_out_lut('B', 2) < 0 && ccl_out_lut('B', 23) < 0);
#else
// The G adds PB08..PB11, PB22/PB23 and PB02; the J adds the rest.
static_assert(ccl_in_line('B', 8) == 8 && ccl_in_line('B', 22) == 0);
static_assert(ccl_out_lut('B', 9) == 2 && ccl_out_lut('B', 23) == 0);
static_assert(ccl_out_lut('B', 2) == 0 && ccl_out_lut('B', 11) == 1);
#endif

// ---- the pad types, instantiated on pads every variant carries -----------------

using In0 = CclIn<Pin<'A', 16>>;
using In1 = CclIn<Pin<'A', 17>>;
using In2 = CclIn<Pin<'A', 18>>;
using Out0 = CclOut<Pin<'A', 19>>;

static_assert(In0::lut == 0 && In0::input == 0);
static_assert(In1::lut == 0 && In1::input == 1);
static_assert(In2::lut == 0 && In2::input == 2);
static_assert(Out0::lut == 0);
static_assert(In0::function == PinFunction::i && Out0::function == PinFunction::i);

// PA08 and PA30 are the same input line reached through two pads.
static_assert(CclIn<Pin<'A', 8>>::line == CclIn<Pin<'A', 30>>::line);

// ---- every verb, instantiated ---------------------------------------------------

constexpr LutConfig smoke_cfg{
    .in0 = LutInput::io,
    .in1 = LutInput::ac,
    .in2 = LutInput::event,
    .truth = lut_truth([](bool a, bool b, bool c) { return (a && b) || c; }),
    .filter = LutFilter::filter,
    .edge_detect = true,
    .event_in = true,
    .invert_event_in = true,
    .event_out = true,
};

void use() {
    (void)Ccl::init();
    (void)Ccl::init(0);
    Ccl::bus_clock(true);
    (void)Ccl::clock(0);
    Ccl::unclock();
    Ccl::reset();
    (void)Ccl::resetting();
    Ccl::enable(true);
    (void)Ccl::enabled();
    Ccl::restate_enable();
    (void)Ccl::run_standby(true);
    (void)Ccl::run_standby();
    (void)Ccl::regs();
    (void)Ccl::sequencer(0, LutSequencer::d_flip_flop);
    (void)Ccl::sequencer(1, LutSequencer::rs_latch);
    (void)Ccl::sequencer(0);

    (void)Lut<0>::configure(smoke_cfg);
    (void)Lut<0>::configure(smoke_cfg, false);
    (void)Lut<0>::config();
    (void)Lut<0>::config_valid(smoke_cfg);
    Lut<0>::enable(true);
    Lut<1>::enable(false);
    (void)Lut<0>::enabled();
    (void)Lut<0>::truth(0x96);
    (void)Lut<0>::truth();
    (void)static_cast<uint32_t>(Lut<0>::ctrl());
    // Table 29-3 grants this user the ASYNCHRONOUS path and nothing else.
    (void)Lut<0>::listen(0, EventChannelConfig{.path = EventPath::asynchronous});
    Lut<0>::unlisten();

    // LUT3 is configurable on every variant, pads or no pads.
    (void)Lut<3>::configure(LutConfig{.in0 = LutInput::link,
                                      .truth = lut_truth_pass(0)});
    Lut<3>::enable(true);

    In0::claim();
    In1::claim(PinPull::up);
    In2::claim(PinPull::down);
    Out0::claim();
    (void)Out0::read();
    In0::release();
    Out0::release();

    // The pads LUT3 has on the J and nowhere else, behind the gate that
    // makes the instance usable on the packages that lack them.
    if constexpr (Lut<3>::has_output_pad) {
#if defined(__SAMC21J18A__)
        CclOut<Pin<'B', 17>>::claim();
        CclIn<Pin<'B', 14>>::claim();
#endif
    }

    Ccl::release();
}

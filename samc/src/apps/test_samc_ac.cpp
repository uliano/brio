// test_samc_ac - the reference bench suite for samc/ac.hpp.
//
// A test_<target>_<subject> suite is a menu of single-letter tests over
// the console, judged by tools/bench.py's "ALL: N pass, M fail" grammar
// (util/testbench.hpp owns that grammar). It is a REFERENCE test: it is
// meant to keep passing through every later restructuring of the driver
// under it. `ac_sync_probe` stays what it is - a PROBE that answered one
// timing question - and is not touched by this file.
//
// NOTHING TO WIRE, on the technique the probe proved: a comparator's
// analog input is a DIRECT connection to the pad, so a pad left under
// PORT and driven as an ordinary output is a rail the AC can measure.
// (40.6.3 asks for the pad's digital driver to be disabled for analog
// use; keeping it enabled is exactly the deviation that makes a wireless
// test possible, and the AC reads what PORT drives.) The other voltage
// in the room is each comparator's own 64-step VDD scaler.
//
// THE ONE THING THIS BOARD CANNOT PRODUCE is an analog level strictly
// between the rails on a PIN: there is no DAC driver, and the bandgap
// needs SUPC.VREF, which no driver here turns on. Letter c says so and
// works around it by making the SCALER the window's shared input signal
// instead of a pad - which reaches all three window states.
//
// What is exercised, letter by letter:
//   a  the block: geometry, the EVSYS codes it publishes, and every
//      refusal - per-package input legality, the two chapter rules that
//      are not about pads, and EVCTRL's enable-protection
//   b  one comparator against its own VDD scaler, both ways, with the
//      CMP0 pad read back through PORT
//   c  WINDOW MODE: all three WSTATE values, and the chapter-exact
//      shared-pin shape for the two a rail-driven pad can reach
//   d  the four window interrupt selections, each fired and each held
//      silent
//   e  the AC as an EVSYS GENERATOR: a comparator flip and a window
//      transition, each moving a DMA block
//   f  the AC as an EVSYS USER: an EIC pin edge starting a single-shot
//      comparison through SOC0 - a user table 29-3 marks ASYNCHRONOUS
//      PATH ONLY
//
// build: boards = c21j
// build: monitor_speed = 115200

#include <stdint.h>

#include "samc/ac.hpp"
#include "samc/clock.hpp"
#include "samc/dmac.hpp"
#include "samc/eic.hpp"
#include "samc/evsys.hpp"
#include "samc/nvic.hpp"
#include "samc/pin.hpp"
#include "samc/sercom.hpp"
#include "samc/ticker.hpp"
#include "util/print.hpp"
#include "util/testbench.hpp"

using SysClock = brio::Clock<brio::ClockSource::internal, 48'000'000>;
constexpr SysClock clock;

namespace {

using namespace brio;

constexpr UartPads console_pads{
    .tx = SercomPad::pad0,
    .rx = SercomPad::pad1,
    .tx_pin = {'B', 30, PinFunction::d},
    .rx_pin = {'B', 31, PinFunction::d},
};
using Serial = Uart<5, console_pads>;
constexpr Serial serial;
using Led = Pin<'B', 23>;

TestBench<Serial> bench;

using brio::crlf;
using brio::print;

// ---------------------------------------------------------------------------
// The pads
//
// PA04 = AIN0 and PA05 = AIN1 are COMP0/COMP1's first two analog inputs
// and both are free on the bench board; PA12 carries CMP0 (function H),
// which is how a comparator output is read back with no wire; PA16 is
// EXTINT0, letter f's event source.
// ---------------------------------------------------------------------------
using Ain0 = Pin<'A', 4>;
using Ain1 = Pin<'A', 5>;
using Cmp0Pad = Pin<'A', 12>;
using EicPad = Pin<'A', 16>;
using EicLine = ExtInt<EicPad>;

using Comp0 = AcComparator<0>;
using Comp1 = AcComparator<1>;
using Window0 = AcWindow<0>;

// GCLK_AC on generator 0 (the 48 MHz main clock): this suite measures
// what the comparators DO, not how long they take, so the fastest
// sampling clock is the right one - ac_sync_probe owns the timing.
constexpr uint8_t ac_gen = 0;

// The event fabric, as in test_samc_evsys and test_samc_eic: DMAC
// channel 0 is event user 5, and a transfer is the witness that an event
// arrived. The event channel's own clock comes from generator 6.
constexpr uint8_t dma_ch = 0;
constexpr uint8_t user_dmac_ch0 = 5;
constexpr uint8_t ev_ch = 0;
constexpr uint8_t ev_gen = 6;
using Copy = DmaChannel<dma_ch>;
using EvGen = Gclk<ev_gen>;

constexpr uint16_t payload = 16;
volatile uint8_t src[payload];
volatile uint8_t dst[payload];

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

void spin(uint32_t turns) {
    volatile uint32_t sink = 0;
    for (uint32_t i = 0; i < turns; ++i) {
        sink = sink + 1u;
    }
}

void settle() { spin(20'000UL); }

/// Drive an analog input pad from PORT - the stimulus this whole suite
/// runs on. The pad stays under PORT (no PMUX): the comparator's input
/// is a direct pad connection and does not need the mux, while the mux
/// would take the output driver away (see docs/samc/port.md).
template <class P>
void drive(bool high) {
    P::output();
    if (high) {
        P::set();
    } else {
        P::clear();
    }
}

/// The precondition every measurement below rests on: does this pad
/// actually go where PORT drives it? Read back through PORT.IN, before
/// the AC sees the pin. This is a stronger question for THIS suite than
/// "is the pad free" (which asks the weak internal pull and, on this
/// board, PA04 and PA05 answer no - see docs/samc/ac.md): what an analog
/// input needs is a pad that reaches both rails.
template <class P>
bool pad_follows_port() {
    P::output();
    P::set();
    settle();
    const bool high = P::read();
    P::clear();
    settle();
    const bool low = P::read();
    return high && !low;
}

const char* window_state_name(AcWindowState s) {
    switch (s) {
    case AcWindowState::above: return "above";
    case AcWindowState::inside: return "inside";
    case AcWindowState::below: return "below";
    default: return "reserved";
    }
}

/// The EIC's stimulus is different in kind: the pad is handed to a
/// peripheral, so PORT's output driver is gone and only the internal
/// pull can move it (test_samc_eic established that).
template <class P>
bool pad_follows_pull() {
    P::input(PinPull::up);
    settle();
    const bool up = P::read();
    P::input(PinPull::down);
    settle();
    const bool down = P::read();
    P::configure({});
    return up && !down;
}

bool destination_matches(uint8_t seed) {
    for (uint16_t i = 0; i < payload; ++i) {
        if (dst[i] != static_cast<uint8_t>(seed + i)) {
            return false;
        }
    }
    return true;
}

bool destination_untouched() {
    for (uint16_t i = 0; i < payload; ++i) {
        if (dst[i] != 0u) {
            return false;
        }
    }
    return true;
}

bool arm_event_driven_copy(uint8_t seed) {
    for (uint16_t i = 0; i < payload; ++i) {
        src[i] = static_cast<uint8_t>(seed + i);
        dst[i] = 0;
    }
    if (!Copy::reset()) {
        return false;
    }
    const DmaChannelConfig cfg{
        .trigger = dma_trigger_none,
        .action = DmaTriggerAction::block,
        .event_action = DmaEventAction::trigger,
        .event_input = true,
    };
    if (!Copy::configure(cfg)) {
        return false;
    }
    const DmaTransfer t{
        .source = &src[0],
        .destination = &dst[0],
        .beats = payload,
        .beat = DmaBeat::byte,
    };
    if (!Copy::load(t)) {
        return false;
    }
    return Copy::enable(true);
}

/// Wait out t_STARTUP without pretending to know it: READY is the
/// chapter's own answer to "is the output valid yet" (40.8.8).
bool wait_ready(uint32_t spins = 200'000UL) {
    for (uint32_t i = 0; i < spins; ++i) {
        if (Comp0::ready()) {
            return true;
        }
    }
    return false;
}

// =============================================================================
// a - the block, its published vocabulary, and every refusal
// =============================================================================
void ta_block() {
    bench.verdict("four comparators paired into two windows",
                  Ac::comparator_count == 4u && Ac::window_count == 2u);

    // THE EVSYS VOCABULARY LIVES HERE, not in evsys.hpp - the fabric
    // driver owns channels and paths, a peripheral owns its own codes.
    print(serial, "  EVSYS codes: COMP0 gen ", Ac::comparator_generator(0),
          ", WIN0 gen ", Ac::window_generator(0), ", SOC0 user ",
          Ac::start_user(0), " (asynchronous path only)", crlf);
    bench.verdict("the comparator and window generator codes are the table's "
                  "own (COMP0 0x49, WIN0 0x4D)",
                  Ac::comparator_generator(0) == 0x49u &&
                      Ac::window_generator(0) == 0x4Du);
    bench.verdict("and the SOCn users are 34..37",
                  Ac::start_user(0) == 34u && Ac::start_user(3) == 37u);

    // PER-PACKAGE INPUT LEGALITY. The PAIR owns the pads: PIN2 means
    // AIN2 on COMP0 and AIN6 on COMP2, and AIN6 is a J-package pad.
    bench.verdict("the pair owns the pads: PIN2 is AIN2 on COMP0 and AIN6 on "
                  "COMP2",
                  ac_ain_of(0, 2) == 2u && ac_ain_of(2, 2) == 6u);
    bench.verdict("this package bonds AIN6 and AIN7, so COMP2 may use all four "
                  "(it is the J)",
                  ac_ain_exists(6) && ac_ain_exists(7) &&
                      AcComparator<2>::config_valid(
                          AcConfig{.positive = AcPositive::pin3}));

    // The two chapter rules that are not about pads.
    bench.verdict("hysteresis in single-shot mode is refused (40.6.6)",
                  !Comp0::config_valid(
                      AcConfig{.single_shot = true, .hysteresis = true}));
    bench.verdict("the end-of-comparison interrupt in continuous mode is "
                  "refused (40.8.12)",
                  !Comp0::config_valid(AcConfig{
                      .interrupt_on = AcInterrupt::end_of_comparison}));
    bench.verdict("and both are accepted where the chapter allows them",
                  Comp0::config_valid(AcConfig{.hysteresis = true}) &&
                      Comp0::config_valid(AcConfig{
                          .single_shot = true,
                          .interrupt_on = AcInterrupt::end_of_comparison}));

    bench.verdict("the block comes up", Ac::init(ac_gen));

    // EVCTRL IS ENABLE-PROTECTED (40.8.3), which init() leaves the block
    // in no position to write - so the refusal is the honest surface.
    bench.verdict("the block is enabled after init()", Ac::enabled());
    bench.verdict("so the event control is REFUSED - EVCTRL is "
                  "enable-protected",
                  !Ac::event_config(AcEventControl{.comparator_out = 0x1}));
    bench.verdict("with the block disabled it is accepted",
                  Ac::enable(false) &&
                      Ac::event_config(AcEventControl{.comparator_out = 0x5,
                                                      .window_out = 0x2,
                                                      .start_in = 0x3,
                                                      .invert_in = 0x1}));
    const AcEventControl back = Ac::event_config();
    bench.verdict("and reads back field for field",
                  back.comparator_out == 0x5u && back.window_out == 0x2u &&
                      back.start_in == 0x3u && back.invert_in == 0x1u);
    bench.verdict("inverting an event input nothing listens to is refused",
                  !Ac::event_config(AcEventControl{.invert_in = 0x1}));

    bench.verdict("every comparator reads not-ready while the block is down",
                  !Comp0::ready() && !Comp1::ready());
    Ac::release();
}

// =============================================================================
// b - one comparator against its own VDD scaler
// =============================================================================
void tb_comparator() {
    bench.verdict("PA04 reaches both rails under PORT - the precondition\n"
                  "                every analog measurement here rests on",
                  pad_follows_port<Ain0>());
    bench.verdict("so does PA12, the CMP0 pad", pad_follows_port<Cmp0Pad>());
    bench.verdict("the block comes up", Ac::init(ac_gen));

    // The scaler at 31 is (31+1)/64 of VDD, about half - comfortably
    // away from both rails, which is what makes a rail-driven pad an
    // unambiguous stimulus.
    Comp0::scaler(31);
    bench.verdict("COMP0 measures PA04 against its own VDD scaler, with its "
                  "output on the CMP0 pad",
                  Comp0::configure(AcConfig{.positive = AcPositive::pin0,
                                            .negative = AcNegative::vscale,
                                            .out = AcOut::synchronous}));
    Cmp0Pad::function(PinFunction::h, PinConfig{.input_enable = true});

    drive<Ain0>(true);
    bench.verdict("the comparator enables", Comp0::enable(true));
    bench.verdict("and reports itself READY", wait_ready());
    settle();
    const bool state_high = Comp0::state();
    const bool pad_high = Cmp0Pad::read();

    drive<Ain0>(false);
    settle();
    const bool state_low = Comp0::state();
    const bool pad_low = Cmp0Pad::read();

    print(serial, "  scaler 31 (~VDD/2): pad high -> STATE=", state_high ? "1" : "0",
          " CMP0=", pad_high ? "1" : "0", ", pad low -> STATE=",
          state_low ? "1" : "0", " CMP0=", pad_low ? "1" : "0", crlf);
    bench.verdict("STATE follows the pad, both ways", state_high && !state_low);
    bench.verdict("and the CMP0 pad carries the same answer out of the chip",
                  pad_high && !pad_low);

    // The scaler is a real 64-step divider, not a fixed threshold: with
    // the pad at VDD every step reads high, and with the pad at ground
    // every step reads low. Two ends and the middle, staying clear of
    // step 63, which sits ON the rail.
    bool high_all = true;
    bool low_all = true;
    for (const uint8_t v : {uint8_t{0}, uint8_t{31}, uint8_t{55}}) {
        Comp0::scaler(v);
        drive<Ain0>(true);
        settle();
        high_all = high_all && Comp0::state();
        drive<Ain0>(false);
        settle();
        low_all = low_all && !Comp0::state();
    }
    bench.verdict("a pad at VDD is above every scaler step tried (0, 31, 55)",
                  high_all);
    bench.verdict("and a pad at ground is below all of them", low_all);

    // The scaler as the POSITIVE input and ground as the negative: the
    // comparator's own reference beats a hard zero at every step, which
    // is the shape letter c's window needs.
    bench.verdict("the same comparator re-configures with the scaler as its "
                  "positive input",
                  Comp0::enable(false) &&
                      Comp0::configure(AcConfig{.positive = AcPositive::vscale,
                                                .negative = AcNegative::ground}));
    Comp0::scaler(31);
    bench.verdict("and enables", Comp0::enable(true) && wait_ready());
    settle();
    bench.verdict("the scaler is above ground, as 64 steps of VDD/64 must be",
                  Comp0::state());

    Cmp0Pad::release();
    Cmp0Pad::configure({});
    Ain0::configure({});
    Ac::release();
}

// =============================================================================
// c - window mode, and the three states
// =============================================================================
//
// THE PROBLEM THIS LETTER HAS TO SOLVE: a window needs a signal that can
// sit BETWEEN two limits, and on this board every pad can only be driven
// to a rail. So the roles are swapped - the SIGNAL is each comparator's
// VDD scaler (both set to the same step, so the pair really does see one
// level) and the two LIMITS are the two rail-driven pads. That reaches
// all three of WSTATE's values.
//
// The chapter's own recommended shape - "the same I/O pin must be chosen
// as positive input for each comparator" - is exercised afterwards for
// the two states it can reach here.
void tc_window() {
    bench.verdict("PA04 and PA05 both reach the rails under PORT",
                  pad_follows_port<Ain0>() && pad_follows_port<Ain1>());
    bench.verdict("the block comes up", Ac::init(ac_gen));

    // Signal = both scalers at the same step. Limits: COMP0's negative
    // is PA05, COMP1's is PA04.
    Comp0::scaler(31);
    Comp1::scaler(31);
    bench.verdict("COMP0 compares the scaler against PA05",
                  Comp0::configure(AcConfig{.positive = AcPositive::vscale,
                                            .negative = AcNegative::pin1}));
    bench.verdict("COMP1 compares the same scaler against PA04",
                  Comp1::configure(AcConfig{.positive = AcPositive::vscale,
                                            .negative = AcNegative::pin0}));
    bench.verdict("the window turns on - WINCTRL is write-synchronized but NOT "
                  "enable-protected, so this works under a running block",
                  Window0::configure(true, AcWindowInterrupt::inside));
    bench.verdict("and reads back", Window0::enabled() &&
                                        Window0::interrupt_on() ==
                                            AcWindowInterrupt::inside);
    bench.verdict("both comparators enable",
                  Comp0::enable(true) && Comp1::enable(true));
    bench.verdict("the pair is consistent - same measurement mode, same "
                  "positive input, which 40.6.4 requires and no register "
                  "enforces",
                  Window0::pair_consistent());
    bench.verdict("and the window is ready once both comparators are",
                  wait_ready() && Window0::ready());

    struct Case {
        bool pa05;
        bool pa04;
        AcWindowState expect;
        const char* what;
    };
    const Case cases[] = {
        {true, false, AcWindowState::inside, "signal between the two limits"},
        {false, false, AcWindowState::above, "both limits at ground"},
        {true, true, AcWindowState::below, "both limits at VDD"},
    };
    for (const auto& c : cases) {
        drive<Ain1>(c.pa05);
        drive<Ain0>(c.pa04);
        settle();
        const AcWindowState s = Window0::state();
        print(serial, "  ", c.what, ": WSTATE=", window_state_name(s),
              " (STATE0=", Comp0::state() ? "1" : "0", " STATE1=",
              Comp1::state() ? "1" : "0", ")", crlf);
        bench.verdict("the window reports the ", c.what, s == c.expect);
    }

    // The individual comparators keep working throughout - 40.6.4 says
    // so and it is cheap to hold it to that.
    drive<Ain1>(true);
    drive<Ain0>(false);
    settle();
    bench.verdict("and the comparators' own STATE bits still answer "
                  "independently under window mode",
                  !Comp0::state() && Comp1::state());

    // --- the chapter's own shape: one shared input PIN, scaler limits
    bench.verdict("the window turns off",
                  Window0::configure(false, AcWindowInterrupt::inside) &&
                      !Window0::enabled());
    bench.verdict("both comparators disable",
                  Comp0::enable(false) && Comp1::enable(false));
    Comp0::scaler(48);
    Comp1::scaler(16);
    bench.verdict("both now share PA04 as their positive input, with their "
                  "scalers as the two limits",
                  Comp0::configure(AcConfig{.positive = AcPositive::pin0,
                                            .negative = AcNegative::vscale}) &&
                      Comp1::configure(AcConfig{.positive = AcPositive::pin0,
                                                .negative = AcNegative::vscale}));
    bench.verdict("the window turns on again",
                  Window0::configure(true, AcWindowInterrupt::outside) &&
                      Comp0::enable(true) && Comp1::enable(true) && wait_ready());
    bench.verdict("and the pair is consistent in the chapter's own sense",
                  Window0::pair_consistent());

    drive<Ain0>(true);
    settle();
    const AcWindowState pin_high = Window0::state();
    drive<Ain0>(false);
    settle();
    const AcWindowState pin_low = Window0::state();
    print(serial, "  shared input pin, limits at 49/64 and 17/64 of VDD: "
          "pad high -> ", window_state_name(pin_high), ", pad low -> ",
          window_state_name(pin_low), crlf);
    bench.verdict("a rail-driven shared pin reads above the window at VDD",
                  pin_high == AcWindowState::above);
    bench.verdict("and below it at ground", pin_low == AcWindowState::below);
    print(serial, "  (INSIDE is unreachable in this shape on this board: it "
          "needs an analog level between the rails on a PIN, and neither the "
          "DAC nor the bandgap has a driver here)", crlf);

    (void)Window0::configure(false, AcWindowInterrupt::above);
    Ain0::configure({});
    Ain1::configure({});
    Ac::release();
}

// =============================================================================
// d - the four window interrupt selections
// =============================================================================
void td_window_interrupt() {
    bench.verdict("the block comes up", Ac::init(ac_gen));
    Comp0::scaler(31);
    Comp1::scaler(31);
    bench.verdict("the signal-as-scaler window is built again",
                  Comp0::configure(AcConfig{.positive = AcPositive::vscale,
                                            .negative = AcNegative::pin1}) &&
                      Comp1::configure(AcConfig{.positive = AcPositive::vscale,
                                                .negative = AcNegative::pin0}) &&
                      Comp0::enable(true) && Comp1::enable(true) && wait_ready());

    // Park OUTSIDE (above), so every case below is a real transition
    // into the state it names.
    drive<Ain1>(false);
    drive<Ain0>(false);
    settle();

    struct Case {
        AcWindowInterrupt sel;
        const char* name;
        bool pa05;
        bool pa04;
        AcWindowState reached;
    };
    const Case cases[] = {
        {AcWindowInterrupt::inside, "inside", true, false, AcWindowState::inside},
        {AcWindowInterrupt::below, "below", true, true, AcWindowState::below},
        {AcWindowInterrupt::above, "above", false, false, AcWindowState::above},
        {AcWindowInterrupt::outside, "outside", true, true, AcWindowState::below},
    };

    for (const auto& c : cases) {
        // Leave the target state first, so the move INTO it is an edge.
        drive<Ain1>(true);
        drive<Ain0>(false);
        settle();
        if (c.reached == AcWindowState::inside) {
            drive<Ain1>(false);
            drive<Ain0>(false);
            settle();
        }
        bench.verdict("the window takes its interrupt selection",
                      Window0::configure(true, c.sel));
        Window0::clear_flag();
        settle();
        const bool quiet_before = !Window0::flag_set();

        drive<Ain1>(c.pa05);
        drive<Ain0>(c.pa04);
        settle();
        const bool fired = Window0::flag_set();
        const AcWindowState s = Window0::state();

        print(serial, "  WINTSEL=", c.name, ": reached ", window_state_name(s),
              ", flag ", fired ? "1" : "0", " (quiet before: ",
              quiet_before ? "yes" : "no", ")", crlf);
        bench.verdict("the window flag rises on ", c.name, fired && quiet_before);
    }

    // And the negative: a selection whose condition is never met stays
    // silent through the same movements.
    bench.verdict("the selection changes to ABOVE",
                  Window0::configure(true, AcWindowInterrupt::above));
    drive<Ain1>(true);
    drive<Ain0>(true);
    settle();
    Window0::clear_flag();
    drive<Ain1>(true);
    drive<Ain0>(false);
    settle();
    const bool silent = !Window0::flag_set();
    print(serial, "  WINTSEL=above, moved below -> inside: flag ",
          silent ? "0" : "1", crlf);
    bench.verdict("and stays silent through transitions it does not select",
                  silent);

    (void)Window0::configure(false, AcWindowInterrupt::above);
    Ain0::configure({});
    Ain1::configure({});
    Ac::release();
}

// =============================================================================
// e - the AC as an EVSYS generator
// =============================================================================
void te_generator() {
    Evsys::bus_clock(true);
    Evsys::reset();
    bench.verdict("the event channel's own generic clock is routed",
                  EvGen::configure(GclkConfig{.source = GclkSource::osculp32k}) &&
                      GclkChannel::connect(Evsys::gclk_id(ev_ch), ev_gen));
    bench.verdict("the block comes up", Ac::init(ac_gen));
    bench.verdict("the block disables so EVCTRL can be written",
                  Ac::enable(false));
    bench.verdict("COMP0's and WIN0's event outputs are enabled",
                  Ac::event_config(AcEventControl{.comparator_out = 0x1,
                                                  .window_out = 0x1}));
    bench.verdict("and the block comes back up", Ac::enable(true));

    // --- the comparator's own output as a generator
    Comp0::scaler(31);
    bench.verdict("COMP0 measures PA04 against its scaler",
                  Comp0::configure(AcConfig{.positive = AcPositive::pin0,
                                            .negative = AcNegative::vscale}));
    drive<Ain0>(false);
    bench.verdict("the comparator enables low", Comp0::enable(true) &&
                                                    wait_ready() &&
                                                    !Comp0::state());
    // THE CHANNEL IS ROUTED BEFORE THE TRANSFER IS ARMED, and that
    // ordering is not cosmetic: writing CHANNELn attaches an edge
    // detector to a generator that already has a level, and the settle
    // that erratum 1.12.4 asks for is also where any event born of the
    // routing itself is spent. Arming afterwards makes "nothing has
    // moved yet" a statement about the STIMULUS and not about the setup.
    bench.verdict("the DMAC's channel-0 user listens to the channel carrying "
                  "COMP0",
                  Evsys::connect(user_dmac_ch0, ev_ch,
                                 EventChannelConfig{
                                     .generator = Comp0::event_generator,
                                     .path = EventPath::resynchronized,
                                     .edge = EventEdge::rising}));
    settle();   // erratum 1.12.4: the first channel-clock tick is blind
    bench.verdict("the DMA channel arms with no hardware trigger",
                  arm_event_driven_copy(0x21));
    settle();
    bench.verdict("and nothing has moved yet", destination_untouched());

    drive<Ain0>(true);
    settle();
    print(serial, "  after the comparator flipped: dst[0..3] = ", dst[0], " ",
          dst[1], " ", dst[2], " ", dst[3], crlf);
    bench.verdict("A COMPARATOR FLIP MOVED THE BYTES - pad to AC to EVSYS to "
                  "DMAC, with no CPU in the path",
                  destination_matches(0x21));

    // --- the window's inside/outside state as a generator
    //
    // 40.6.13: the window event is "a copy of the window inside/outside
    // status" and is generated regardless of the interrupt selection -
    // so this letter leaves WINTSEL where it is and only moves the
    // state.
    (void)Copy::enable(false);
    Evsys::disconnect(user_dmac_ch0);
    bench.verdict("the pair is rebuilt as a window",
                  Comp0::enable(false) &&
                      Comp0::configure(AcConfig{.positive = AcPositive::vscale,
                                                .negative = AcNegative::pin1}) &&
                      Comp1::configure(AcConfig{.positive = AcPositive::vscale,
                                                .negative = AcNegative::pin0}));
    Comp0::scaler(31);
    Comp1::scaler(31);
    bench.verdict("the window turns on and both comparators enable",
                  Window0::configure(true, AcWindowInterrupt::above) &&
                      Comp0::enable(true) && Comp1::enable(true) && wait_ready());
    // Park OUTSIDE first, then move INSIDE: the event copies the
    // inside/outside status, so that transition is its rising edge.
    drive<Ain1>(false);
    drive<Ain0>(false);
    settle();
    bench.verdict("the channel now carries WIN0",
                  Evsys::connect(user_dmac_ch0, ev_ch,
                                 EventChannelConfig{
                                     .generator = Window0::event_generator,
                                     .path = EventPath::resynchronized,
                                     .edge = EventEdge::both}));
    settle();
    // RE-POINTING A CHANNEL AT A NEW GENERATOR IS ITSELF AN EDGE for the
    // detector, and an event raised while its user is not ready is HELD
    // by the channel rather than dropped (29.2's USRRDY handshake) - so
    // the first arming after a re-route can consume one. Measured here
    // rather than assumed: the first block is reported, the SECOND
    // arming is the one the verdict rests on.
    bench.verdict("the DMA channel arms after the re-route",
                  arm_event_driven_copy(0x72));
    settle();
    const bool moved_on_reroute = !destination_untouched();
    print(serial, "  re-pointing the channel at WIN0 left an event standing: ",
          moved_on_reroute ? "yes" : "no", crlf);
    bench.verdict("the DMA channel arms once more", arm_event_driven_copy(0x72));
    settle();
    bench.verdict("and NOW nothing has moved - the channel is quiet with the "
                  "window parked outside",
                  destination_untouched());
    drive<Ain1>(true);
    settle();
    print(serial, "  after the window went inside: WSTATE=",
          window_state_name(Window0::state()), ", dst[0..3] = ", dst[0], " ",
          dst[1], " ", dst[2], " ", dst[3], crlf);
    bench.verdict("A WINDOW TRANSITION IS AN EVENT TOO, and it is generated "
                  "from the inside/outside state whatever WINTSEL says "
                  "(40.6.13)",
                  destination_matches(0x72));

    (void)Copy::enable(false);
    Evsys::disconnect(user_dmac_ch0);
    GclkChannel::disconnect(Evsys::gclk_id(ev_ch));
    (void)Window0::configure(false, AcWindowInterrupt::above);
    Ain0::configure({});
    Ain1::configure({});
    Ac::release();
}

// =============================================================================
// f - the AC as an EVSYS user: a pin edge starts a comparison
// =============================================================================
//
// SOC0 is user 34 and table 29-3 marks it ASYNCHRONOUS PATH ONLY, which
// is a constraint on the CHANNEL and not on this peripheral - and the
// asynchronous path is exactly the one test_samc_eic proved a HARDWARE
// generator can cross. So the stimulus is an EIC pin edge, moved by the
// pad's own internal pull, and the measurement is a single-shot
// comparison that started with no CPU in the path.
void tf_user() {
    bench.verdict("PA16 follows its own internal pull, which is what the EIC\n"
                  "                stimulus needs",
                  pad_follows_pull<EicPad>());
    Evsys::bus_clock(true);
    Evsys::reset();

    bench.verdict("the EIC comes up on CLK_ULP32K", Eic::init() &&
                                                        Eic::clock_select(
                                                            EicClock::ulp32k));
    EicPad::input(PinPull::down);
    EicLine::claim(PinPull::down);
    bench.verdict("EXTINT0 senses a rising edge and drives its event output",
                  Eic::configure_line(EicLine::line,
                                      EicLineConfig{.sense = EicSense::rising,
                                                    .asynchronous = true,
                                                    .event_out = true}) &&
                      Eic::enable(true));

    bench.verdict("the AC comes up", Ac::init(ac_gen));
    bench.verdict("the block disables so EVCTRL can be written",
                  Ac::enable(false));
    bench.verdict("COMP0's event INPUT is enabled - an event will start a "
                  "comparison",
                  Ac::event_config(AcEventControl{.start_in = 0x1}));
    bench.verdict("and the block comes back up", Ac::enable(true));

    Comp0::scaler(31);
    bench.verdict("COMP0 is SINGLE-SHOT, measuring PA04 against its scaler, "
                  "flagging at end of comparison",
                  Comp0::configure(AcConfig{
                      .positive = AcPositive::pin0,
                      .negative = AcNegative::vscale,
                      .single_shot = true,
                      .interrupt_on = AcInterrupt::end_of_comparison}));
    drive<Ain0>(true);
    bench.verdict("the comparator enables", Comp0::enable(true));
    settle();
    Comp0::clear_flag();
    settle();
    const bool ready_before = Comp0::ready();
    const bool flag_before = Comp0::flag_set();

    bench.verdict("SOC0 listens to the channel carrying EXTINT0, on the "
                  "ASYNCHRONOUS path table 29-3 restricts it to",
                  Evsys::connect(Ac::start_user(0), ev_ch,
                                 EventChannelConfig{
                                     .generator = EicLine::event_generator,
                                     .path = EventPath::asynchronous}));
    settle();

    // The pad's own pull is the edge - the technique test_samc_eic
    // established.
    EicPad::set();
    settle();

    const bool ready_after = Comp0::ready();
    const bool flag_after = Comp0::flag_set();
    const bool state_after = Comp0::state();
    print(serial, "  single-shot COMP0: before the edge READY=",
          ready_before ? "1" : "0", " EOC=", flag_before ? "1" : "0",
          ", after READY=", ready_after ? "1" : "0", " EOC=",
          flag_after ? "1" : "0", " STATE=", state_after ? "1" : "0", crlf);
    bench.verdict("A PIN EDGE STARTED THE COMPARISON - EIC to EVSYS to the "
                  "AC's SOC0 user, with no CPU in the path",
                  flag_after && !flag_before);
    bench.verdict("and the comparison it started answered correctly: the pad "
                  "is at VDD, above the scaler",
                  ready_after && state_after);

    // The negative: with the event input disabled, the same edge starts
    // nothing.
    bench.verdict("the blocks come down to rewrite EVCTRL",
                  Comp0::enable(false) && Ac::enable(false));
    bench.verdict("COMP0's event input is disabled",
                  Ac::event_config(AcEventControl{}));
    bench.verdict("and everything comes back up",
                  Ac::enable(true) && Comp0::enable(true));
    settle();
    Comp0::clear_flag();
    EicPad::clear();
    settle();
    Comp0::clear_flag();
    settle();
    EicPad::set();
    settle();
    bench.verdict("the same edge now starts nothing - EVCTRL.COMPEI is the "
                  "gate",
                  !Comp0::flag_set());

    Evsys::disconnect(Ac::start_user(0));
    (void)Eic::enable(false);
    Eic::release();
    EicPad::configure({});
    Ain0::configure({});
    Ac::release();
}

void banner() {
    print(serial, crlf,
          "test_samc_ac - SAMC21J18A AC (ch. 40): comparators, windows and "
          "both event directions, wireless, clk=",
          SysClock::hz, " Hz", crlf);
    bench.menu();
}

} // namespace

extern "C" void SysTick_Handler() { brio::Ticker::tick(); }
extern "C" void SERCOM5_Handler() { (void)Serial::isr(); }

extern "C" void DMAC_Handler() {
    while (const auto irq = brio::Dmac::take_pending()) {
        (void)irq;
    }
}

int main() {
    const bool clock_ok = SysClock::init();
    const bool serial_ok = Serial::init(clock, 115200);
    const bool tick_ok = brio::Ticker::init(clock);
    Led::output();

    const bool dma_ok = brio::Dmac::init();
    brio::Nvic::enable(brio::Dmac::irq());
    brio::enable_interrupts();

    bench.letter('a', "the block, its EVSYS codes and every refusal", ta_block);
    bench.letter('b', "one comparator against its own VDD scaler", tb_comparator);
    bench.letter('c', "window mode and the three WSTATE values", tc_window);
    bench.letter('d', "the four window interrupt selections", td_window_interrupt);
    bench.letter('e', "the AC as an EVSYS generator", te_generator);
    bench.letter('f', "the AC as an EVSYS user: a pin edge starts a comparison",
                 tf_user);

    if (serial_ok) {
        print(serial, crlf, "boot: clk=", clock_ok ? "OSC48M" : "FAILED",
              " tick=", tick_ok ? "SysTick" : "FAILED",
              " dmac=", dma_ok ? "up" : "FAILED", crlf);
        banner();
    }
    bench.prompt();

    for (;;) {
        uint8_t c = 0;
        if (!Serial::read_byte(c)) {
            continue;
        }
        if (c == '\r' || c == '\n') {
            continue;
        }
        print(serial, static_cast<char>(c), crlf);
        Led::toggle();
        if (c == '?') {
            banner();
        } else if (!bench.handle(static_cast<char>(c))) {
            print(serial, "unknown letter (? for the menu)", crlf);
        }
        bench.prompt();
    }
}

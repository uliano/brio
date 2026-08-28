// test_samc_eic - the reference bench suite for samc/eic.hpp.
//
// A test_<target>_<subject> suite is a menu of single-letter tests over
// the console, judged by tools/bench.py's "ALL: N pass, M fail" grammar
// (util/testbench.hpp owns that grammar). It is a REFERENCE test: it is
// meant to keep passing through every later restructuring of the driver
// under it.
//
// NOTHING TO WIRE - and for a chapter whose whole subject is EXTERNAL
// pins that takes some doing. The stimulus is the pad's own internal
// pull: with the pad handed to the EIC (peripheral function A) PINCFG
// still holds PULLEN, and the pull's DIRECTION is the PORT OUT bit
// (28.6.3.2), so writing OUT moves the pad between the rails through
// the pull resistor and the EIC sees a real edge on a real pin. Letter
// b is where that technique is established rather than assumed: it
// tries the obvious one first (drive the pad from PORT with the
// peripheral mux on) and reports what the silicon actually does with
// each.
//
// What is exercised, letter by letter:
//   a  the block: geometry, the pad-to-line map read out of the device
//      header, enable-protection as refusals, and the clock question
//   b  THE STIMULUS TECHNIQUE, measured: PORT-driven versus pull-driven
//      under PMUXEN, with the free-pad check that makes either credible
//   c  the five senses, and what an edge flag does that a level flag
//      does not
//   d  the EIC clock: a synchronous edge needs one and an asynchronous
//      edge does not - proven by taking the clock away - plus the
//      filter's minimum pulse width and CLK_ULP32K as the clock that
//      needs no GCLK channel at all
//   e  THE EVENT: an EIC edge through EVSYS into a DMA transfer, on the
//      clocked path AND on the asynchronous one - which is the question
//      docs/samc/evsys.md left open, and a line above 7, which is the
//      question 26.6.7's prose leaves open
//   f  the interrupt: one vector for sixteen lines, masked by INTENSET
//
// Outside z, because they need a human:
//   n  the NMI on PA08, edge-sensed and asynchronous (never a level: an
//      NMI cannot be masked, so a level on a pad held at that level is
//      an unbreakable loop)
//   u  the user button PB22 = EXTINT6, pressed by hand
//
// build: boards = c21j
// build: monitor_speed = 115200

#include <stdint.h>

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
// The pads this suite uses
//
// PA16 and PA17 are free on the bench board (the console is PB30/PB31,
// SWD is PA30/PA31, the crystal pads are PA14/PA15, LED PB23, button
// PB22). PA09 is the third one and it is here for one reason: it is
// EXTINT9, above the "EXTINT0-7" that 26.6.7's prose claims are the only
// event generators, while EVCTRL is sixteen bits wide and the EVSYS
// generator table lists all sixteen.
// ---------------------------------------------------------------------------
using PadA = Pin<'A', 16>;
using PadB = Pin<'A', 17>;
using PadHigh = Pin<'A', 9>;
using LineA = ExtInt<PadA>;       // EXTINT0
using LineB = ExtInt<PadB>;       // EXTINT1
using LineHigh = ExtInt<PadHigh>; // EXTINT9

using Button = Pin<'B', 22>;
using ButtonInt = ExtInt<Button>; // EXTINT6
using NmiPad = Pin<'A', 8>;
using Nmi = ExtNmi<NmiPad>;

// The EIC's own generic clock generator: OSCULP32K, so one EIC clock
// period is ~30 us and a CPU-length pulse is unambiguously shorter than
// the two periods 26.6.3 says are needed.
constexpr uint8_t eic_gen = 6;
using EicGen = Gclk<eic_gen>;

// The event fabric, as in test_samc_evsys: DMAC channel 0 is event user
// 5, and the transfer is the witness that an event arrived.
constexpr uint8_t dma_ch = 0;
constexpr uint8_t user_dmac_ch0 = 5;
constexpr uint8_t ev_ch = 0;
using Copy = DmaChannel<dma_ch>;

volatile uint32_t eic_isr_count = 0;
volatile uint32_t eic_isr_mask = 0;
volatile uint32_t nmi_count = 0;

// VOLATILE IN BOTH DIRECTIONS - the lesson the DMAC campaign paid for on
// this target: gcc cannot see the controller's reads either, and will
// sink a buffer's preparation past the thing that starts the transfer.
constexpr uint16_t payload = 16;
volatile uint8_t src[payload];
volatile uint8_t dst[payload];

// ---------------------------------------------------------------------------
// Small helpers
// ---------------------------------------------------------------------------

/// A short, calibrated-enough wait. One iteration is a handful of CPU
/// cycles at 48 MHz; 5000 of them is a few hundred microseconds, which
/// is many OSCULP32K periods and far longer than any pull needs to move
/// an unloaded pad.
void spin(uint32_t turns) {
    volatile uint32_t sink = 0;
    for (uint32_t i = 0; i < turns; ++i) {
        sink = sink + 1u;
    }
}

void settle() { spin(20'000UL); }

/// Move a pad that has been handed to the EIC, by flipping the
/// direction of its internal pull (PINCFG.PULLEN stays set; PORT.OUT is
/// what says up or down, 28.6.3.2).
template <class P>
void pull_to(bool high) {
    if (high) {
        P::set();
    } else {
        P::clear();
    }
}

/// Hand a pad to the EIC with its pull enabled and parked at `high`.
/// The pull is parked BEFORE the mux so that claiming a line never
/// produces an edge of its own.
template <class P>
void arm_pad(bool high) {
    P::input(high ? PinPull::up : PinPull::down);
    P::function(PinFunction::a,
                PinConfig{.input_enable = true,
                          .pull = high ? PinPull::up : PinPull::down});
}

/// Is this pad electrically free? A pad with nothing on it follows its
/// own weak pull; one tied to something on the board does not. Checked
/// through PORT, before the EIC ever sees the pin - every measurement
/// below rests on this.
template <class P>
bool pad_is_free() {
    P::input(PinPull::up);
    spin(2000);
    const bool up = P::read();
    P::input(PinPull::down);
    spin(2000);
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

/// Arm the DMA channel so that ONLY an event can move it: no hardware
/// trigger source at all, EVACT = trigger, EVIE set.
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

/// Bring the block up disabled, with GCLK_EIC from the slow generator.
bool eic_up_slow() {
    if (!Eic::init()) {
        return false;
    }
    if (!EicGen::configure(GclkConfig{.source = GclkSource::osculp32k})) {
        return false;
    }
    return Eic::clock(eic_gen) && Eic::clock_select(EicClock::gclk);
}

// =============================================================================
// a - the block, the map and the refusals
// =============================================================================
void ta_block() {
    bench.verdict("sixteen lines and two CONFIG registers, from the device "
                  "header",
                  Eic::line_count == 16u && Eic::config_regs == 2u);
    bench.verdict("a line past the last is refused", !Eic::configure_line(16, {}));

    // THE PAD-TO-LINE MAP IS NOT A FORMULA, and this prints the proof.
    print(serial, "  pad -> EXTINT: PA16=", LineA::line, " PA17=", LineB::line,
          " PA09=", LineHigh::line, " PB22=", ButtonInt::line,
          " PA24=", eic_extint_line('A', 24), " PA27=", eic_extint_line('A', 27),
          " PB30=", eic_extint_line('B', 30), crlf);
    bench.verdict("the map is irregular - PA24 is line 12 and PA27 is line 15, "
                  "so no arithmetic could stand in for the header's table",
                  eic_extint_line('A', 24) == 12 && eic_extint_line('A', 27) == 15);
    bench.verdict("PA08 carries the NMI and therefore no EXTINT line",
                  eic_extint_line('A', 8) < 0 && eic_nmi_pad('A', 8));

    // A clock from the start: this letter's subject is enable-protection,
    // and letter d is where what the clock is for gets measured. (It has
    // to be here at all because a line asking to be SAMPLED - this one
    // asks for the filter - cannot be enabled without one; that is
    // letter d's finding.)
    bench.verdict("the block comes up (APB clock, software reset, GCLK_EIC)",
                  eic_up_slow());
    bench.verdict("and reset leaves it DISABLED, which is what makes the "
                  "configuration registers writable",
                  !Eic::enabled());
    bench.verdict("every line reads back as sense NONE after the reset",
                  Eic::line_config(0).sense == EicSense::none &&
                      Eic::line_config(15).sense == EicSense::none);

    bench.verdict("a line configures while the block is disabled",
                  Eic::configure_line(0, EicLineConfig{.sense = EicSense::rising,
                                                       .filter = true,
                                                       .event_out = true}));
    const EicLineConfig back = Eic::line_config(0);
    bench.verdict("and reads back field for field out of three registers "
                  "(CONFIGn, ASYNCH, EVCTRL)",
                  back.sense == EicSense::rising && back.filter &&
                      !back.asynchronous && back.event_out);
    bench.verdict("the filter with asynchronous detection is refused "
                  "(26.8.10's own note)",
                  !Eic::configure_line(0, EicLineConfig{.sense = EicSense::rising,
                                                        .filter = true,
                                                        .asynchronous = true}));

    // ENABLE-PROTECTION, as refusals rather than as lost stores.
    bench.verdict("the block enables", Eic::enable(true));
    bench.verdict("and NOW a line configuration is refused - CONFIGn, ASYNCH "
                  "and EVCTRL are enable-protected (26.6.2.1)",
                  !Eic::configure_line(1, EicLineConfig{.sense = EicSense::low}));
    bench.verdict("so is the clock selection, which is enable-protected too",
                  !Eic::clock_select(EicClock::ulp32k));
    bench.verdict("line 0's configuration survived the attempt untouched",
                  Eic::line_config(0).sense == EicSense::rising);
    bench.verdict("the block disables again", Eic::enable(false));
    bench.verdict("and the same configuration is accepted now",
                  Eic::configure_line(1, EicLineConfig{.sense = EicSense::low}));

    // The clock question the chapter states in prose (26.5.3, 26.6.3).
    bench.verdict("a level with no filter needs no EIC clock",
                  !eic_needs_clock(EicLineConfig{.sense = EicSense::high}));
    bench.verdict("an asynchronous edge needs none either",
                  !eic_needs_clock(EicLineConfig{.sense = EicSense::rising,
                                                 .asynchronous = true}));
    bench.verdict("a synchronous edge does, and so does any filter",
                  eic_needs_clock(EicLineConfig{.sense = EicSense::rising}) &&
                      eic_needs_clock(EicLineConfig{.sense = EicSense::high,
                                                    .filter = true}));

    Eic::release();
}

// =============================================================================
// b - the stimulus technique, measured rather than assumed
// =============================================================================
//
// A chapter about EXTERNAL interrupts on a board with no wires needs the
// chip to move its own pad. There are two candidate ways and 28.6.1 does
// not settle either: "PMUXn ... will override the connection between the
// PORT and that I/O pin". Whether that override reaches the OUTPUT
// DRIVER, the PULL, or both is a silicon question, and everything after
// this letter depends on the answer.
void tb_stimulus() {
    bench.verdict("PA16 is electrically free (it follows its own pull)",
                  pad_is_free<PadA>());
    bench.verdict("so is PA17", pad_is_free<PadB>());
    bench.verdict("and PA09", pad_is_free<PadHigh>());

    bench.verdict("the block comes up on its slow generic clock", eic_up_slow());

    // A LEVEL sense is the right instrument for this question: it needs
    // no clock, no edge and no history - INTFLAG simply mirrors whether
    // the pad matches, so a flag is a statement about the pad's LEVEL
    // and nothing else.
    bench.verdict("line 0 senses a HIGH level",
                  Eic::configure_line(LineA::line,
                                      EicLineConfig{.sense = EicSense::high}));
    bench.verdict("the block enables", Eic::enable(true));

    // --- technique 1: drive the pad from PORT with the peripheral mux on
    PadA::output();
    PadA::clear();
    LineA::claim();
    spin(2000);
    Eic::clear_flags(0xFFFFu);
    spin(2000);
    const bool driven_low_flag = LineA::flag();
    PadA::set();
    spin(2000);
    const bool driven_high_flag = LineA::flag();
    print(serial, "  PORT-driven under PMUXEN: flag with OUT=0 -> ",
          driven_low_flag ? "1" : "0", ", with OUT=1 -> ",
          driven_high_flag ? "1" : "0", crlf);

    // --- technique 2: move the pad with its own internal pull
    LineA::release();
    arm_pad<PadA>(false);
    spin(4000);
    Eic::clear_flags(0xFFFFu);
    spin(2000);
    const bool pulled_low_flag = LineA::flag();
    pull_to<PadA>(true);
    spin(4000);
    const bool pulled_high_flag = LineA::flag();
    print(serial, "  pull-driven under PMUXEN: flag with the pull down -> ",
          pulled_low_flag ? "1" : "0", ", with the pull up -> ",
          pulled_high_flag ? "1" : "0", crlf);

    bench.verdict("THE INTERNAL PULL IS THE STIMULUS: with the pad handed to "
                  "the EIC, flipping the pull's direction moves the line "
                  "between the rails and the level sense follows it",
                  !pulled_low_flag && pulled_high_flag);
    bench.verdict("and it is the pad, not the register: pulled low the HIGH "
                  "sense stays silent",
                  !pulled_low_flag);

    // Whatever the first technique did, say so as a verdict rather than
    // only in a printed line - if a later silicon changes it, the suite
    // has to notice.
    bench.verdict("PMUXEN takes the pad away from PORT's OUTPUT DRIVER: "
                  "driving OUT with the mux on moves nothing",
                  driven_low_flag == driven_high_flag);

    (void)Eic::enable(false);
    Eic::release();
    PadA::configure({});
}

// =============================================================================
// c - the five senses, and the difference between an edge and a level
// =============================================================================
void tc_senses() {
    bench.verdict("the block comes up on its slow generic clock", eic_up_slow());

    // Edges are detected asynchronously here so that this letter says
    // nothing about the clock - that is letter d's subject.
    struct Case {
        EicSense sense;
        const char* name;
        bool flag_on_rise;
        bool flag_on_fall;
    };
    const Case cases[] = {
        {EicSense::rising, "rising", true, false},
        {EicSense::falling, "falling", false, true},
        {EicSense::both, "both edges", true, true},
    };

    for (const auto& c : cases) {
        arm_pad<PadA>(false);
        bench.verdict("the line takes the sense",
                      Eic::configure_line(LineA::line,
                                          EicLineConfig{.sense = c.sense,
                                                        .asynchronous = true}) &&
                          Eic::enable(true));
        Eic::clear_flags(0xFFFFu);

        pull_to<PadA>(true);
        settle();
        const bool on_rise = LineA::flag();
        LineA::clear_flag();
        pull_to<PadA>(false);
        settle();
        const bool on_fall = LineA::flag();
        LineA::clear_flag();

        print(serial, "  ", c.name, ": rise -> ", on_rise ? "1" : "0",
              ", fall -> ", on_fall ? "1" : "0", crlf);
        bench.verdict("the ", c.name, on_rise == c.flag_on_rise &&
                                          on_fall == c.flag_on_fall);
        (void)Eic::enable(false);
    }

    // A LEVEL is not an edge, and 26.6.3 says exactly how they differ:
    // "In level-sensitive mode, when interrupt has been cleared,
    // INTFLAG.EXTINT[x] will be set immediately if the EXTINTx pin still
    // matches the interrupt condition."
    arm_pad<PadA>(true);
    bench.verdict("line 0 senses a HIGH level",
                  Eic::configure_line(LineA::line,
                                      EicLineConfig{.sense = EicSense::high}) &&
                      Eic::enable(true));
    settle();
    LineA::clear_flag();
    spin(200);
    const bool level_returns = LineA::flag();

    (void)Eic::enable(false);
    arm_pad<PadA>(false);
    bench.verdict("line 0 senses a RISING edge instead",
                  Eic::configure_line(LineA::line,
                                      EicLineConfig{.sense = EicSense::rising,
                                                    .asynchronous = true}) &&
                      Eic::enable(true));
    settle();
    Eic::clear_flags(0xFFFFu);
    pull_to<PadA>(true);
    settle();
    const bool edge_set = LineA::flag();
    LineA::clear_flag();
    spin(200);
    const bool edge_returns = LineA::flag();

    print(serial, "  after clearing: level flag back -> ",
          level_returns ? "1" : "0", ", edge flag back -> ",
          edge_returns ? "1" : "0", crlf);
    bench.verdict("A CLEARED LEVEL FLAG COMES STRAIGHT BACK while the pin "
                  "still matches - a handler that only clears will spin",
                  level_returns);
    bench.verdict("a cleared EDGE flag stays cleared until the next edge",
                  edge_set && !edge_returns);

    // The two lines are independent, which is the thing a single shared
    // CONFIG register makes worth checking.
    (void)Eic::enable(false);
    arm_pad<PadB>(false);
    bench.verdict("a second line takes its own sense",
                  Eic::configure_line(LineB::line,
                                      EicLineConfig{.sense = EicSense::rising,
                                                    .asynchronous = true}) &&
                      Eic::enable(true));
    // Line 0's pad is left HIGH by the tests above, so it is parked low
    // again first: an edge sense answers to a TRANSITION, and re-writing
    // the level it already sits at is not one.
    pull_to<PadA>(false);
    settle();
    Eic::clear_flags(0xFFFFu);
    pull_to<PadB>(true);
    settle();
    bench.verdict("moving line 1 raises line 1's flag and only that one",
                  LineB::flag() && !LineA::flag());
    pull_to<PadA>(true);
    settle();
    bench.verdict("and moving line 0 raises line 0's",
                  LineA::flag() && LineB::flag());

    (void)Eic::enable(false);
    Eic::release();
    PadA::configure({});
    PadB::configure({});
}

// =============================================================================
// d - the clock: who needs it, who does not, and what the filter costs
// =============================================================================
void td_clock() {
    // --- the block with NO clock at all, twice: once for a line that
    //     does not need one and once for a line that does.
    //
    // 26.6.3 says the EIC "automatically requests GCLK_EIC or
    // CLK_ULP32K to operate" when filtering or synchronous edge
    // detection is enabled, and no chapter draws the consequence:
    // CTRLA.ENABLE is write-synchronized, and what it synchronizes
    // AGAINST is that requested clock. So whether the block can be
    // enabled at all depends on what its LINES were configured for -
    // which is not a sentence anywhere in ch. 26.
    bench.verdict("the block comes up with GCLK_EIC left DISCONNECTED",
                  Eic::init());
    GclkChannel::disconnect(Eic::gclk_id);

    arm_pad<PadA>(false);
    bench.verdict("an ASYNCHRONOUSLY detected rising edge is configured - a "
                  "line that requests no clock (26.6.4.2)",
                  Eic::configure_line(LineA::line,
                                      EicLineConfig{.sense = EicSense::rising,
                                                    .asynchronous = true}));
    const bool enable_async = Eic::enable(true);
    Eic::clear_flags(0xFFFFu);
    pull_to<PadA>(true);
    settle();
    const bool async_no_clock = LineA::flag();
    const bool disable_async = Eic::enable(false);

    arm_pad<PadA>(false);
    const bool config_sync =
        Eic::configure_line(LineA::line, EicLineConfig{.sense = EicSense::rising});
    const bool enable_sync = Eic::enable(true);
    const uint8_t ctrla = Eic::regs().EIC_CTRLA;
    const bool busy_sync =
        (Eic::regs().EIC_SYNCBUSY & EIC_SYNCBUSY_ENABLE_Msk) != 0u;
    Eic::clear_flags(0xFFFFu);
    pull_to<PadA>(true);
    settle();
    const bool sync_no_clock = LineA::flag();

    print(serial, "  with no EIC clock - asynchronous line: enable() -> ",
          enable_async ? "true" : "FALSE", ", edge seen -> ",
          async_no_clock ? "1" : "0", crlf);
    print(serial, "  with no EIC clock - synchronous line:  enable() -> ",
          enable_sync ? "true" : "FALSE", ", CTRLA=", hex(ctrla),
          ", SYNCBUSY.ENABLE=", busy_sync ? "1" : "0", ", edge seen -> ",
          sync_no_clock ? "1" : "0", crlf);

    bench.verdict("A LINE THAT REQUESTS NO CLOCK NEEDS NONE, right down to "
                  "the block's enable: an asynchronous edge is configured, "
                  "enabled and detected with GCLK_EIC disconnected",
                  enable_async && async_no_clock && disable_async);
    bench.verdict("A LINE THAT REQUESTS ONE CANNOT EVEN BE ENABLED WITHOUT "
                  "IT - CTRLA.ENABLE is written, reads back at once, and "
                  "SYNCBUSY.ENABLE never clears",
                  config_sync && !enable_sync && busy_sync &&
                      (ctrla & EIC_CTRLA_ENABLE_Msk) != 0u);
    bench.verdict("so nothing is detected either", !sync_no_clock);

    // --- give it a clock WITHOUT touching CTRLA
    //
    // The enable was PENDING, not lost: connecting the generic clock
    // channel is enough for the same write to complete, and the line
    // starts detecting with no second enable. That is what proves the
    // clock was the missing ingredient and not something else.
    bench.verdict("the slow generator is configured",
                  EicGen::configure(GclkConfig{.source = GclkSource::osculp32k}));
    bench.verdict("and routed to GCLK_EIC - with CTRLA untouched",
                  Eic::clock(eic_gen));
    const bool enable_completes = Eic::sync_wait(EIC_SYNCBUSY_ENABLE_Msk);
    bench.verdict("THE PENDING ENABLE COMPLETES THE MOMENT THE CLOCK ARRIVES",
                  enable_completes && Eic::enabled());
    pull_to<PadA>(false);
    settle();
    Eic::clear_flags(0xFFFFu);
    pull_to<PadA>(true);
    settle();
    bench.verdict("and the SYNCHRONOUS line now sees its edge, from the same "
                  "configuration written before the clock existed",
                  LineA::flag());

    // --- CLK_ULP32K: the clock that needs no GCLK channel
    (void)Eic::enable(false);
    GclkChannel::disconnect(Eic::gclk_id);
    bench.verdict("CTRLA.CKSEL selects CLK_ULP32K instead",
                  Eic::clock_select(EicClock::ulp32k) &&
                      Eic::clock_select() == EicClock::ulp32k);
    arm_pad<PadA>(false);
    bench.verdict("the synchronous edge is configured on it",
                  Eic::configure_line(LineA::line,
                                      EicLineConfig{.sense = EicSense::rising}) &&
                      Eic::enable(true));
    Eic::clear_flags(0xFFFFu);
    pull_to<PadA>(true);
    settle();
    bench.verdict("a synchronous edge is detected with NO generic clock "
                  "channel connected - CLK_ULP32K comes straight from "
                  "OSC32KCTRL (26.5.3)",
                  LineA::flag());

    // --- the filter's minimum pulse width
    //
    // 26.6.3: "the external pin is sampled at the EIC clock rate, thus
    // pulses with duration lower than two EIC clock periods may not be
    // properly detected". CLK_ULP32K is ~32 kHz, so one period is ~30 us
    // and a pull-driven pulse a few hundred CPU cycles long is far below
    // the threshold, while a settled one is far above it.
    (void)Eic::enable(false);
    arm_pad<PadA>(false);
    bench.verdict("a FILTERED both-edge line is configured",
                  Eic::configure_line(LineA::line,
                                      EicLineConfig{.sense = EicSense::both,
                                                    .filter = true}) &&
                      Eic::enable(true));
    Eic::clear_flags(0xFFFFu);

    // A pulse far shorter than one sampling period.
    pull_to<PadA>(true);
    spin(20);
    pull_to<PadA>(false);
    settle();
    const bool short_pulse = LineA::flag();
    Eic::clear_flags(0xFFFFu);

    // The same excursion, held long enough to be sampled.
    pull_to<PadA>(true);
    settle();
    pull_to<PadA>(false);
    settle();
    const bool long_pulse = LineA::flag();

    print(serial, "  filtered at ~32 kHz: short pulse -> ",
          short_pulse ? "1" : "0", ", held pulse -> ", long_pulse ? "1" : "0",
          crlf);
    bench.verdict("a held excursion passes the majority filter", long_pulse);
    bench.verdict("a pulse shorter than the sampling period does not - the "
                  "filter is a real low-pass, not a formality",
                  !short_pulse);

    (void)Eic::enable(false);
    Eic::release();
    PadA::configure({});
}

// =============================================================================
// e - the event: an EIC edge through EVSYS, into a DMA transfer
// =============================================================================
//
// THIS IS THE LETTER THE EVSYS CAMPAIGN COULD NOT WRITE. docs/samc/
// evsys.md declined to claim anything about a HARDWARE generator on the
// ASYNCHRONOUS path, because that suite had no generator to route - only
// the software event, which measurably does not cross an async channel.
// The EIC is that generator, and it settles the question both ways.
//
// It also settles a documentation dispute: 26.6.7 says the EIC's event
// outputs are "External event from pin (EXTINT0-7)", while its own
// EVCTRL register is sixteen bits wide and ch. 29's generator table
// lists EXTINT0..EXTINT15. EXTINT9 is the witness.
void te_event() {
    Evsys::bus_clock(true);
    Evsys::reset();
    bench.verdict("the block comes up on its slow generic clock", eic_up_slow());
    bench.verdict("the event channel's own generic clock is routed",
                  GclkChannel::connect(Evsys::gclk_id(ev_ch), eic_gen));

    // --- a clocked path first: the known-good shape
    arm_pad<PadA>(false);
    bench.verdict("line 0 senses a rising edge and drives its event output",
                  Eic::configure_line(LineA::line,
                                      EicLineConfig{.sense = EicSense::rising,
                                                    .asynchronous = true,
                                                    .event_out = true}) &&
                      Eic::enable(true));
    bench.verdict("the DMA channel arms with no hardware trigger",
                  arm_event_driven_copy(0x30));
    bench.verdict("and nothing has moved yet", destination_untouched());
    bench.verdict("the DMAC's channel-0 user connects to the channel carrying "
                  "EXTINT0",
                  Evsys::connect(user_dmac_ch0, ev_ch,
                                 EventChannelConfig{
                                     .generator = LineA::event_generator,
                                     .path = EventPath::resynchronized,
                                     .edge = EventEdge::rising}));
    settle();   // erratum 1.12.4: the first tick after configure is blind
    pull_to<PadA>(true);
    settle();

    print(serial, "  after one pad edge: dst[0..3] = ", dst[0], " ", dst[1], " ",
          dst[2], " ", dst[3], crlf);
    bench.verdict("A PIN EDGE MOVED THE BYTES - pad to EIC to EVSYS to DMAC, "
                  "with no CPU in the path",
                  destination_matches(0x30));

    // --- the same generator on the ASYNCHRONOUS path
    //
    // The asynchronous path has no clock and no edge detector; a software
    // event measurably does not cross it. A hardware generator, on the
    // other hand, holds its event line for as long as the condition
    // lasts, so there is something with width to propagate.
    (void)Copy::enable(false);
    Evsys::disconnect(user_dmac_ch0);
    (void)Eic::enable(false);
    arm_pad<PadA>(false);
    bench.verdict("line 0 is re-armed for the asynchronous path",
                  Eic::configure_line(LineA::line,
                                      EicLineConfig{.sense = EicSense::rising,
                                                    .asynchronous = true,
                                                    .event_out = true}) &&
                      Eic::enable(true));
    bench.verdict("the DMA channel arms again", arm_event_driven_copy(0x55));
    bench.verdict("routed through an ASYNCHRONOUS channel - no clock, no edge "
                  "detector, no status",
                  Evsys::connect(user_dmac_ch0, ev_ch,
                                 EventChannelConfig{
                                     .generator = LineA::event_generator,
                                     .path = EventPath::asynchronous}));
    settle();
    pull_to<PadA>(true);
    settle();
    const bool async_moved = destination_matches(0x55);
    print(serial, "  asynchronous path, hardware generator: dst[0..3] = ",
          dst[0], " ", dst[1], " ", dst[2], " ", dst[3], crlf);
    bench.verdict("A HARDWARE GENERATOR DOES CROSS AN ASYNCHRONOUS CHANNEL, "
                  "where a software event does not - the async path carries "
                  "what has width, and a register write has none",
                  async_moved);

    // --- a line above 7, which 26.6.7's prose says has no event
    (void)Copy::enable(false);
    Evsys::disconnect(user_dmac_ch0);
    (void)Eic::enable(false);
    arm_pad<PadHigh>(false);
    bench.verdict("EXTINT9 senses a rising edge and drives its event output",
                  Eic::configure_line(LineHigh::line,
                                      EicLineConfig{.sense = EicSense::rising,
                                                    .asynchronous = true,
                                                    .event_out = true}) &&
                      Eic::enable(true));
    bench.verdict("the DMA channel arms once more", arm_event_driven_copy(0x66));
    bench.verdict("and listens to the channel carrying EXTINT9",
                  Evsys::connect(user_dmac_ch0, ev_ch,
                                 EventChannelConfig{
                                     .generator = LineHigh::event_generator,
                                     .path = EventPath::resynchronized,
                                     .edge = EventEdge::rising}));
    settle();
    pull_to<PadHigh>(true);
    settle();
    print(serial, "  EXTINT9 (generator ", LineHigh::event_generator,
          "): dst[0..3] = ", dst[0], " ", dst[1], " ", dst[2], " ", dst[3], crlf);
    bench.verdict("EVERY LINE IS AN EVENT GENERATOR, not just EXTINT0-7: "
                  "26.6.7's prose is narrower than its own EVCTRL register "
                  "and than ch. 29's generator table",
                  destination_matches(0x66));

    // --- and with the event output disabled, nothing crosses
    (void)Copy::enable(false);
    (void)Eic::enable(false);
    arm_pad<PadHigh>(false);
    bench.verdict("EXTINT9 keeps its sense but drops its event output",
                  Eic::configure_line(LineHigh::line,
                                      EicLineConfig{.sense = EicSense::rising,
                                                    .asynchronous = true}) &&
                      Eic::enable(true));
    bench.verdict("the DMA channel arms", arm_event_driven_copy(0x88));
    settle();
    Eic::clear_flags(0xFFFFu);
    pull_to<PadHigh>(true);
    settle();
    bench.verdict("the line still flags the edge", LineHigh::flag());
    bench.verdict("but nothing crosses EVSYS - EVCTRL.EXTINTEO is the gate",
                  destination_untouched());

    (void)Copy::enable(false);
    Evsys::disconnect(user_dmac_ch0);
    GclkChannel::disconnect(Evsys::gclk_id(ev_ch));
    (void)Eic::enable(false);
    Eic::release();
    PadA::configure({});
    PadHigh::configure({});
}

// =============================================================================
// f - the interrupt: one vector, sixteen lines, masked by INTENSET
// =============================================================================
void tf_interrupt() {
    bench.verdict("the block comes up on its slow generic clock", eic_up_slow());
    arm_pad<PadA>(false);
    arm_pad<PadB>(false);
    bench.verdict("two lines sense a rising edge",
                  Eic::configure_line(LineA::line,
                                      EicLineConfig{.sense = EicSense::rising,
                                                    .asynchronous = true}) &&
                      Eic::configure_line(LineB::line,
                                          EicLineConfig{.sense = EicSense::rising,
                                                        .asynchronous = true}) &&
                      Eic::enable(true));
    Eic::clear_flags(0xFFFFu);
    Eic::disarm(0xFFFFu);
    eic_isr_count = 0;
    eic_isr_mask = 0;
    Nvic::enable(Eic::irq());

    // Unarmed first: the flag rises, the handler does not.
    pull_to<PadA>(true);
    settle();
    const bool flag_without_arm = LineA::flag();
    const uint32_t count_without_arm = eic_isr_count;
    LineA::clear_flag();

    bench.verdict("an unarmed line still raises its flag", flag_without_arm);
    bench.verdict("but no interrupt is taken - INTENSET is the gate, and the "
                  "ISR body masks with it because ONE vector serves sixteen "
                  "lines",
                  count_without_arm == 0u);

    // Now arm line 1 only, and move both.
    LineB::arm(true);
    eic_isr_count = 0;
    eic_isr_mask = 0;
    pull_to<PadA>(false);
    settle();
    pull_to<PadA>(true);
    settle();
    const uint32_t after_unarmed_line = eic_isr_count;
    pull_to<PadB>(true);
    settle();
    const uint32_t after_armed_line = eic_isr_count;
    const uint32_t mask = eic_isr_mask;

    print(serial, "  ISR entries: after moving the unarmed line ",
          after_unarmed_line, ", after moving the armed one ", after_armed_line,
          ", mask=", hex(mask), crlf);
    bench.verdict("moving the unarmed line takes no interrupt",
                  after_unarmed_line == 0u);
    bench.verdict("moving the armed one does", after_armed_line >= 1u);
    bench.verdict("and the handler is told WHICH line by the returned mask",
                  (mask & LineB::mask) != 0u && (mask & LineA::mask) == 0u);
    bench.verdict("the armed line's flag was cleared by the handler",
                  !LineB::flag());
    bench.verdict("while the unarmed line's flag still stands, untouched",
                  LineA::flag());

    Nvic::disable(Eic::irq());
    Eic::disarm(0xFFFFu);
    (void)Eic::enable(false);
    Eic::release();
    PadA::configure({});
    PadB::configure({});
}

// =============================================================================
// n - the NMI (outside z: it arms an interrupt nothing can mask)
// =============================================================================
//
// EDGE-SENSED AND ASYNCHRONOUS, DELIBERATELY. An NMI is always enabled,
// is taken at any priority and cannot be masked by PRIMASK, so a LEVEL
// sense on a pad sitting at that level is an unbreakable loop and the
// board would need a reflash. An edge cannot do that.
void tn_nmi() {
    bench.verdict("PA08 is electrically free", pad_is_free<NmiPad>());
    bench.verdict("the block comes up", Eic::init());

    // NMI detection does not need the block enabled at all (26.6.4.1) -
    // which is worth showing, since every other line here does.
    bench.verdict("the block is left DISABLED on purpose", !Eic::enabled());

    arm_pad<NmiPad>(false);
    nmi_count = 0;
    Eic::clear_nmi_flag();
    bench.verdict("the NMI takes a rising, asynchronously detected sense",
                  Nmi::configure(EicNmiConfig{.sense = EicSense::rising,
                                              .asynchronous = true}));
    const EicNmiConfig back = Eic::nmi_config();
    bench.verdict("and reads back out of NMICTRL",
                  back.sense == EicSense::rising && back.asynchronous &&
                      !back.filter);

    const uint32_t before = nmi_count;
    pull_to<NmiPad>(true);
    settle();
    const uint32_t after = nmi_count;
    print(serial, "  NMI exceptions taken: ", after, " (block ",
          Eic::enabled() ? "enabled" : "DISABLED", ")", crlf);
    bench.verdict("THE NMI FIRES WITH THE EIC DISABLED - NMISENSE alone "
                  "enables it (26.6.4.1)",
                  after > before);
    bench.verdict("and the handler cleared NMIFLAG", !Eic::nmi_flag());

    // Disarm before leaving: a sense of NONE is the only way to turn an
    // NMI off, since there is no enable to clear.
    bench.verdict("sense NONE is how an NMI is turned off",
                  Nmi::configure(EicNmiConfig{}) &&
                      Eic::nmi_config().sense == EicSense::none);
    pull_to<NmiPad>(false);
    settle();
    pull_to<NmiPad>(true);
    settle();
    bench.verdict("and it stays off across another edge",
                  Eic::nmi_flag() == false);

    Nmi::release();
    NmiPad::configure({});
    Eic::release();
}

// =============================================================================
// u - the user button (outside z: it needs a finger)
// =============================================================================
void tu_button() {
    bench.verdict("PB22 is EXTINT6, from the device header",
                  ButtonInt::line == 6u);

    // What the board does with this pad decides what can be VERDICTED
    // here: a pad left free follows its own internal pull and can be
    // exercised like every other line in this suite, while one held by
    // an external pull-up (the usual way a button to ground is fitted)
    // can only be moved by a finger. Both are printed; only the free
    // case is asserted, because a suite must not fail on a board's
    // wiring choice.
    const bool free = pad_is_free<Button>();
    Button::input(PinPull::up);
    spin(2000);
    const bool at_rest = Button::read();
    print(serial, "  PB22: follows its own pull -> ", free ? "yes" : "no",
          ", at rest with the pull up -> ", at_rest ? "high" : "LOW", crlf);

    bench.verdict("the block comes up on its slow generic clock", eic_up_slow());
    bench.verdict("EXTINT6 senses both edges, filtered - which is what a "
                  "mechanical contact needs",
                  Eic::configure_line(ButtonInt::line,
                                      EicLineConfig{.sense = EicSense::both,
                                                    .filter = true}));
    arm_pad<Button>(true);
    bench.verdict("the block enables", Eic::enable(true));

    if (free) {
        Eic::clear_flags(0xFFFFu);
        pull_to<Button>(false);
        settle();
        const bool fell = ButtonInt::flag();
        ButtonInt::clear_flag();
        pull_to<Button>(true);
        settle();
        bench.verdict("and with the pad free, its own pull drives both edges "
                      "through the filter",
                      fell && ButtonInt::flag());
    }

    // The human half. No verdict: nobody may be at the bench, and a
    // suite that fails for want of a finger is a suite that gets
    // ignored.
    ButtonInt::claim(PinPull::up);
    Eic::clear_flags(0xFFFFu);
    print(serial, "  press the button within 10 s (printed, not judged)...",
          crlf);
    const uint32_t deadline = Ticker::millis() + 10'000UL;
    uint32_t edges = 0;
    while (static_cast<int32_t>(Ticker::millis() - deadline) < 0) {
        if (ButtonInt::flag()) {
            ButtonInt::clear_flag();
            ++edges;
            Led::toggle();
        }
    }
    print(serial, "  edges seen: ", edges,
          edges >= 2u ? " - a real contact on a real pin" : " - nobody pressed it",
          crlf);

    (void)Eic::enable(false);
    ButtonInt::release();
    Button::configure({});
    Eic::release();
}

void banner() {
    print(serial, crlf,
          "test_samc_eic - SAMC21J18A EIC (ch. 26), wireless: the pads move "
          "themselves through their own pulls, clk=",
          SysClock::hz, " Hz", crlf);
    bench.menu();
}

} // namespace

extern "C" void SysTick_Handler() { brio::Ticker::tick(); }
extern "C" void SERCOM5_Handler() { (void)Serial::isr(); }

extern "C" void EIC_Handler() {
    const uint32_t m = brio::Eic::isr();
    if (m != 0u) {
        eic_isr_mask = eic_isr_mask | m;
        eic_isr_count = eic_isr_count + 1u;
    }
}

extern "C" void NonMaskableInt_Handler() {
    if (brio::Eic::take_nmi()) {
        nmi_count = nmi_count + 1u;
    }
}

/// Bound because a completed transfer raises the line, and an unbound
/// vector on this target is a silent death. Nothing here needs the
/// completion - the destination buffer is the witness.
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

    bench.letter('a', "the block, the pad map and the enable-protected "
                      "refusals", ta_block);
    bench.letter('b', "the stimulus technique, measured", tb_stimulus);
    bench.letter('c', "the five senses, and edge versus level", tc_senses);
    bench.letter('d', "the clock: who needs it, and what the filter costs",
                 td_clock);
    bench.letter('e', "an EIC edge through EVSYS into a DMA transfer", te_event);
    bench.letter('f', "one vector, sixteen lines, masked by INTENSET",
                 tf_interrupt);
    bench.letter('n', "THE NMI on PA08 (outside z)", tn_nmi, false);
    bench.letter('u', "the user button PB22, pressed by hand (outside z)",
                 tu_button, false);

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

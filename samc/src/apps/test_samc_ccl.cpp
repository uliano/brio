// test_samc_ccl - the reference bench suite for samc/ccl.hpp
// (SAM C21 Configurable Custom Logic, DS60001479M ch. 37).
//
// A test_<target>_<subject> suite is a menu of single-letter tests over
// the console, judged by tools/bench.py's "ALL: N pass, M fail" grammar
// (util/testbench.hpp owns that grammar). It is a REFERENCE test: it is
// meant to keep passing through every later restructuring of the driver
// under it.
//
// NOTHING TO WIRE. Three stimuli, all inside the chip:
//  - a free pad walked between the rails BY ITS OWN INTERNAL PULL, which
//    survives PMUXEN where the output driver does not (the technique
//    test_samc_eic established and docs/samc/port.md records) - that is
//    how a CCL *IO* input gets a real edge with no wire;
//  - TC0/TC1/TC4 and TCC0 waveforms, which the CCL takes internally;
//  - the analog comparator, whose positive input is a pad PORT is
//    driving as a plain GPIO (the analog mux keeps the output driver).
// The observers are the CCL's own OUT pads read back through PORT.IN,
// a DMA block that either moved or did not, and the SysTick cycle
// stopwatch.
//
// Letters (z = all):
//   a  the block: the reserve's geometry, the reset state, and THE
//      ENABLE-PROTECTION DISPUTE settled RAW - three documents give
//      three answers and erratum 1.7.3 gives a fourth
//   b  the truth table: all eight rows of a three-input LUT, the mask,
//      and what reconfiguring one LUT costs the others
//   c  the input multiplexer: LINK, TC, ALTTC, TCC and SERCOM - which
//      settles erratum 1.8.3's non-applicability by measurement and the
//      device header's MISSING TCC enumerator by experiment
//   d  the stages: a combinational LUT with NO generic clock at all, the
//      synchronizer, the filter, and the edge detector's one-GCLK strobe
//   e  the sequencers: DFF, JK, latch and RS against tables 37-2..37-5,
//      FEEDBACK, the asynchronous clear, and ERRATUM 1.7.2 with a
//      control on both sides
//   f  events both ways: LUTOUT into a DMA witness, and an event INTO a
//      LUT on the asynchronous path table 29-3 restricts it to
//   g  THE HEADLINE: what a CCL output costs in GCLK periods, measured
//      against the AC's own fraction + 2 - the question
//      docs/samc/ac.md left open
//
// build: boards = c21j
// build: monitor_speed = 115200

#include <stdint.h>

#include "samc/ac.hpp"
#include "samc/ccl.hpp"
#include "samc/clock.hpp"
#include "samc/dmac.hpp"
#include "samc/evsys.hpp"
#include "samc/nvic.hpp"
#include "samc/pin.hpp"
#include "samc/sercom.hpp"
#include "samc/tc.hpp"
#include "samc/tcc.hpp"
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

TestBench<Serial, 12> bench;

using brio::crlf;
using brio::print;

// ---------------------------------------------------------------------------
// The pads
//
// LUT0 takes PA16/PA17/PA18 (IN[0..2]) and comes out on PA19 (OUT[0]);
// LUT1 takes PA08 (IN[3]) and comes out on PA11 (OUT[1]). PA04 is the
// comparator's AIN[0] and stays a plain GPIO - it is ALSO CCL0/IN[0]'s
// other pad, which is why it is never muxed to function I here.
// ---------------------------------------------------------------------------
using PadA = Pin<'A', 16>;   // CCL0/IN[0]
using PadB = Pin<'A', 17>;   // CCL0/IN[1]
using PadC = Pin<'A', 18>;   // CCL0/IN[2]
using PadO0 = Pin<'A', 19>;  // CCL0/OUT[0]
using PadD = Pin<'A', 8>;    // CCL1/IN[3]
using PadO1 = Pin<'A', 11>;  // CCL1/OUT[1]
using AcStim = Pin<'A', 4>;  // AC AIN[0], driven as a plain GPIO
using CmpPad = Pin<'A', 12>; // AC CMP0, function H, read back through PORT.IN

using In0 = CclIn<PadA>;
using In1 = CclIn<PadB>;
using In2 = CclIn<PadC>;
using In3 = CclIn<PadD>;
using Out0 = CclOut<PadO0>;
using Out1 = CclOut<PadO1>;

using L0 = Lut<0>;
using L1 = Lut<1>;
using Comp = AcComparator<0>;

// SERCOM0 is LUT0's SERCOM input source, and 37.6.2.4 wants its TX
// signal on that SERCOM's PAD[0] - PA04 on this package. It lives at
// namespace scope because its interrupt vector has to be bound: an
// unserved transmit ring never drains, and write_byte() would wait for
// room that never comes.
constexpr UartPads sercom0_pads{
    .tx = SercomPad::pad0,
    .rx = SercomPad::pad1,
    .tx_pin = {'A', 4, PinFunction::c},
    .rx_pin = {'A', 5, PinFunction::c},
};
using Talker = Uart<0, sercom0_pads>;

// Generic clock generators this suite builds. Generator 1 is the only
// one with a sixteen-bit linear divider (table 16-3), so it is the one
// that can be slowed to 11.719 kHz = OSC48M / 4096 - and 4096 CPU
// cycles per period is what makes a software stopwatch able to resolve
// a fraction of one.
constexpr uint8_t gen_slow = 1;
constexpr uint32_t slow_period = 4096;   // CPU cycles per GCLK period
constexpr uint8_t gen_fast = 0;          // generator 0: OSC48M, always up

// ---------------------------------------------------------------------------
// The cycle stopwatch (the test_samc_dma / ac_sync_probe technique)
// ---------------------------------------------------------------------------
uint32_t cycles_now() {
    const uint32_t reload = SysTick->LOAD;
    for (;;) {
        const uint32_t t0 = Ticker::ticks();
        const uint32_t val = SysTick->VAL;
        const uint32_t t1 = Ticker::ticks();
        if (t0 == t1) {
            return t0 * (reload + 1u) + (reload - val);
        }
    }
}

void spin_until(uint32_t target) {
    while (static_cast<int32_t>(cycles_now() - target) < 0) {
    }
}
void settle(uint32_t cycles) { spin_until(cycles_now() + cycles); }

/// Long enough for any pull-driven pad to reach its rail and for any
/// combinational path to follow it.
void settle_pad() { settle(20'000u); }

constexpr uint32_t no_flip = 0xFFFFFFFFu;

template <typename F>
uint32_t time_until(F cond, uint32_t spins) {
    while (spins-- != 0u) {
        if (cond()) {
            return cycles_now();
        }
    }
    return no_flip;
}

uint32_t lcg_state = 0x51A3C7D9u;
uint32_t lcg(uint32_t modulus) {
    lcg_state = lcg_state * 1103515245u + 12345u;
    return (lcg_state >> 16) % modulus;
}

// ---------------------------------------------------------------------------
// Pads: the freedom proof, and driving one through its own pull
// ---------------------------------------------------------------------------

/// A pad is usable as a stimulus only if it FOLLOWS ITS OWN PULL with
/// nothing else on it. Proved before anything relies on it (the board's
/// own button pad, PB22, does not - docs/bench.md records that).
template <class P>
bool pad_is_free() {
    // Three attempts, because a pad this suite has just handed back can
    // still be carrying the charge the previous letter left on it and
    // the pull is a resistor, not a driver.
    for (uint8_t attempt = 0; attempt < 3u; ++attempt) {
        P::input(PinPull::up);
        settle_pad();
        const bool high = P::read();
        P::pull(PinPull::down);
        settle_pad();
        const bool low = !P::read();
        if (high && low) {
            P::configure({});
            return true;
        }
    }
    P::configure({});
    return false;
}

/// Drive a CCL input pad through its pull. PULLEN survives PMUXEN and
/// the pull's DIRECTION is still the PORT OUT bit (28.6.3.2), so this
/// is a real edge on a real pin with the pad handed to the CCL.
template <class Claimed>
void drive(bool high) {
    using P = typename Claimed::pin;
    if (high) {
        P::set();
    } else {
        P::clear();
    }
}

template <class Claimed>
void claim_input(bool initial_high) {
    using P = typename Claimed::pin;
    P::input(initial_high ? PinPull::up : PinPull::down);
    Claimed::claim(initial_high ? PinPull::up : PinPull::down);
}

/// Sample a pad many times and report whether it was ever high and ever
/// low - the way a signal too fast to poll deterministically is still
/// caught in the act.
template <class Claimed>
void sample_levels(uint32_t cycles, bool& saw_high, bool& saw_low) {
    saw_high = false;
    saw_low = false;
    const uint32_t deadline = cycles_now() + cycles;
    while (static_cast<int32_t>(cycles_now() - deadline) < 0) {
        if (Claimed::read()) {
            saw_high = true;
        } else {
            saw_low = true;
        }
    }
}

// ---------------------------------------------------------------------------
// Bringing the block up and down
// ---------------------------------------------------------------------------

/// Everything back to the state each letter starts from: no LUT
/// enabled, no sequencer, the block down, the pads back under PORT.
void teardown() {
    Ccl::enable(false);
    Ccl::reset();
    Evsys::disconnect(Ccl::input_user(0));
    Evsys::disconnect(Ccl::input_user(1));
    In0::release();
    In1::release();
    In2::release();
    In3::release();
    Out0::release();
    Out1::release();
    CmpPad::configure({});
    AcStim::configure({});
}

/// The slow generic clock: OSC48M / 4096.
bool slow_clock_up() {
    return Gclk<gen_slow>::configure(
        GclkConfig{.source = GclkSource::osc48m, .div = slow_period});
}

// =============================================================================
// a - the block, its geometry, and the enable-protection dispute
// =============================================================================
void ta_block() {
    Ccl::bus_clock(true);
    Ccl::reset();

    bench.verdict("four LUTs, two sequencers, twelve input lines",
                  Ccl::lut_count == 4u && Ccl::sequencer_count == 2u &&
                      Ccl::input_count == 12u);
    print(serial, "  GCLK_CCL is peripheral channel ", Ccl::gclk_id,
          ", the PAC id is ", Ccl::pac_id, " (bridge ", Ccl::pac_id / 32u,
          ", bit ", Ccl::pac_id % 32u, ")", crlf);
    bench.verdict("one generic clock channel for the whole block",
                  Ccl::gclk_id == 38u);
    bench.verdict("the EVSYS codes are the chapter's (LUTOUT0 = 0x52, "
                  "LUTIN0 = user 40)",
                  Ccl::output_generator(0) == 0x52u &&
                      Ccl::output_generator(3) == 0x55u &&
                      Ccl::input_user(0) == 40u && Ccl::input_user(3) == 43u);

    // The reset state, register by register.
    const uint8_t ctrl = CCL_REGS->CCL_CTRL;
    print(serial, "  after SWRST: CTRL=", hex(ctrl), " SEQCTRL0=",
          hex(CCL_REGS->CCL_SEQCTRL[0]), " LUTCTRL0=",
          hex(CCL_REGS->CCL_LUTCTRL[0]), crlf);
    bench.verdict("every register is at its reset value and the block is down",
                  ctrl == 0u && CCL_REGS->CCL_SEQCTRL[0] == 0u &&
                      CCL_REGS->CCL_SEQCTRL[1] == 0u &&
                      CCL_REGS->CCL_LUTCTRL[0] == 0u &&
                      CCL_REGS->CCL_LUTCTRL[3] == 0u && !Ccl::enabled());

    // CTRL's implemented bits: the header says 0x43 (SWRST, ENABLE,
    // RUNSTDBY). SWRST reads back as 0 once it has been taken.
    CCL_REGS->CCL_CTRL = 0xFFu;
    const uint8_t wide = CCL_REGS->CCL_CTRL;
    print(serial, "  CTRL after storing 0xFF reads ", hex(wide),
          " (header mask 0x43, SWRST self-clears)", crlf);
    bench.verdict("CTRL implements ENABLE and RUNSTDBY and nothing else",
                  (wide & ~static_cast<uint8_t>(CCL_CTRL_ENABLE_Msk |
                                                CCL_CTRL_RUNSTDBY_Msk)) == 0u);
    Ccl::reset();

    // ---- THE ENABLE-PROTECTION DISPUTE, measured raw -----------------------
    //
    // 37.6.2.1 protects LUTCTRLn by LUTCTRLn.ENABLE; 37.8.2's note
    // protects SEQCTRL by CTRL.ENABLE; erratum 1.7.3 says the silicon
    // protects BOTH registers by CTRL.ENABLE. Four cells settle it: a
    // TRUTH write attempted with each ENABLE in each state, and the
    // readback saying whether it landed.
    constexpr uint32_t base = CCL_LUTCTRL_TRUTH(0x0Fu) |
                              CCL_LUTCTRL_INSEL0(static_cast<uint32_t>(LutInput::io));
    auto truth_write_lands = [&](bool block_on, bool lut_on) {
        Ccl::enable(false);
        CCL_REGS->CCL_LUTCTRL[0] = base | (lut_on ? CCL_LUTCTRL_ENABLE_Msk : 0u);
        Ccl::enable(block_on);
        CCL_REGS->CCL_LUTCTRL[0] =
            (base & ~CCL_LUTCTRL_TRUTH_Msk) | CCL_LUTCTRL_TRUTH(0xA5u) |
            (lut_on ? CCL_LUTCTRL_ENABLE_Msk : 0u);
        const bool landed =
            ((CCL_REGS->CCL_LUTCTRL[0] & CCL_LUTCTRL_TRUTH_Msk) >>
             CCL_LUTCTRL_TRUTH_Pos) == 0xA5u;
        Ccl::enable(false);
        Ccl::reset();
        return landed;
    };
    const bool lut_off_off = truth_write_lands(false, false);
    const bool lut_off_on = truth_write_lands(false, true);
    const bool lut_on_off = truth_write_lands(true, false);
    const bool lut_on_on = truth_write_lands(true, true);
    print(serial, "  LUTCTRL0.TRUTH write lands?  CTRL.ENABLE/LUT.ENABLE = ",
          "0/0:", lut_off_off ? 1 : 0, " 0/1:", lut_off_on ? 1 : 0,
          " 1/0:", lut_on_off ? 1 : 0, " 1/1:", lut_on_on ? 1 : 0, crlf);
    bench.verdict("a LUTCTRL write lands with the block disabled, LUT disabled",
                  lut_off_off);
    bench.verdict("37.6.2.1's LUTCTRLn.ENABLE protection is real too: a "
                  "SECOND write to an already-enabled LUT is refused",
                  !lut_off_on);
    bench.verdict("ERRATUM 1.7.3 (every revision): CTRL.ENABLE is a gate as "
                  "well - a LUTCTRL write is refused with the block enabled "
                  "even though the LUT itself is disabled",
                  !lut_on_off);
    bench.verdict("and refused with both enabled", !lut_on_on);
    bench.verdict("SO THE TWO GATES ARE AN AND, not the swap the erratum's "
                  "sentence describes: the write lands only with BOTH ENABLE "
                  "bits clear",
                  lut_off_off && !lut_off_on && !lut_on_off && !lut_on_on);

    // 37.6.2.1's escape, which is what makes a ONE-STORE configuration
    // legal and which the driver's configure(cfg, true) depends on:
    // "enable-protected bits in the LUTCTRLx registers can be written at
    // the same time as LUTCTRLx.ENABLE is written to '1'". That is a
    // 0 -> 1 TRANSITION of ENABLE, not a store into an enabled LUT.
    Ccl::enable(false);
    CCL_REGS->CCL_LUTCTRL[0] = 0u;
    constexpr uint32_t escape_word = (base & ~CCL_LUTCTRL_TRUTH_Msk) |
                                     CCL_LUTCTRL_TRUTH(0x5Au) |
                                     CCL_LUTCTRL_ENABLE_Msk;
    CCL_REGS->CCL_LUTCTRL[0] = escape_word;
    const uint32_t escape_read = CCL_REGS->CCL_LUTCTRL[0];
    const bool escape_landed =
        ((escape_read & CCL_LUTCTRL_TRUTH_Msk) >> CCL_LUTCTRL_TRUTH_Pos) ==
            0x5Au &&
        (escape_read & CCL_LUTCTRL_ENABLE_Msk) != 0u;
    print(serial, "  one store of ", hex(escape_word), " reads back ",
          hex(escape_read), crlf);
    Ccl::reset();
    bench.verdict("37.6.2.1's ESCAPE works: configuration and ENABLE = 1 in "
                  "ONE store land together, which is what makes a one-store "
                  "configure() legal at all",
                  escape_landed);

    // The same four cells for SEQCTRL, whose two documents disagree in
    // the opposite direction (37.6.2.1 says the EVEN LUT, 37.8.2 says
    // CTRL).
    auto seq_write_lands = [&](bool block_on, bool lut_on) {
        Ccl::enable(false);
        CCL_REGS->CCL_SEQCTRL[0] = 0u;
        CCL_REGS->CCL_LUTCTRL[0] = base | (lut_on ? CCL_LUTCTRL_ENABLE_Msk : 0u);
        Ccl::enable(block_on);
        CCL_REGS->CCL_SEQCTRL[0] =
            static_cast<uint8_t>(CCL_SEQCTRL_SEQSEL_RS_Val);
        const bool landed = (CCL_REGS->CCL_SEQCTRL[0] & CCL_SEQCTRL_SEQSEL_Msk) ==
                            CCL_SEQCTRL_SEQSEL_RS_Val;
        Ccl::enable(false);
        Ccl::reset();
        return landed;
    };
    const bool seq_off_off = seq_write_lands(false, false);
    const bool seq_off_on = seq_write_lands(false, true);
    const bool seq_on_off = seq_write_lands(true, false);
    print(serial, "  SEQCTRL0 write lands?  CTRL/evenLUT = 0/0:",
          seq_off_off ? 1 : 0, " 0/1:", seq_off_on ? 1 : 0, " 1/0:",
          seq_on_off ? 1 : 0, crlf);
    bench.verdict("SEQCTRL takes a write with the block and the even LUT down",
                  seq_off_off);
    bench.verdict("and is refused with the block enabled (37.8.2 and erratum "
                  "1.7.3 agree here)",
                  !seq_on_off);
    print(serial, "  37.6.2.1 alone would predict 0/1 refused; measured ",
          seq_off_on ? "ACCEPTED" : "refused", crlf);

    // ---- RUNSTDBY: enable-protected AND write-before-enable ---------------
    bench.verdict("RUNSTDBY is written while the block is down",
                  Ccl::run_standby(true) && Ccl::run_standby());
    Ccl::enable(true);
    bench.verdict("and the driver refuses it while the block is up",
                  !Ccl::run_standby(false));
    CCL_REGS->CCL_CTRL = CCL_CTRL_ENABLE_Msk;   // raw: RUNSTDBY cleared
    const bool runstdby_stuck = Ccl::run_standby();
    print(serial, "  raw store clearing RUNSTDBY under a running block: bit is ",
          runstdby_stuck ? "UNCHANGED (enable-protected)" : "CLEARED", crlf);
    bench.verdict("37.8.1's 'this bit must be written before enabling the CCL' "
                  "is enforced by the silicon",
                  runstdby_stuck);
    Ccl::enable(false);
    Ccl::reset();
    bench.verdict("and the reset put it back", !Ccl::run_standby());

    // ---- ERRATUM 1.7.4: a software reset raises a PAC error ---------------
    PAC_REGS->PAC_INTFLAGC = PAC_REGS->PAC_INTFLAGC;
    const uint32_t before = PAC_REGS->PAC_INTFLAGC;
    Ccl::reset();
    const uint32_t after = PAC_REGS->PAC_INTFLAGC;
    const uint32_t ccl_bit = 1UL << (Ccl::pac_id % 32u);
    print(serial, "  PAC INTFLAGC before SWRST ", hex(before), ", after ",
          hex(after), " (CCL bit ", hex(ccl_bit), ")", crlf);
    bench.verdict("the software reset worked whatever the PAC thinks",
                  CCL_REGS->CCL_CTRL == 0u && !Ccl::enabled());
    print(serial, "  ERRATUM 1.7.4 (every revision): the PAC protection flag "
          "for the CCL is ", (after & ccl_bit) != 0u ? "RAISED" : "NOT raised",
          " by CTRL.SWRST", crlf);
    bench.verdict("either way the CPU walked past it - no bus fault, no hang",
                  true);
    PAC_REGS->PAC_INTFLAGC = after;

    // ---- the driver's own refusals ----------------------------------------
    Ccl::enable(true);
    bench.verdict("configure() refuses while the block is enabled "
                  "(erratum 1.7.3)",
                  !L0::configure(LutConfig{.truth = 0xAA}));
    bench.verdict("so does sequencer()",
                  !Ccl::sequencer(0, LutSequencer::d_flip_flop));
    bench.verdict("and truth()", !L0::truth(0x55));
    Ccl::enable(false);
    bench.verdict("a pair past the last sequencer is refused",
                  !Ccl::sequencer(2, LutSequencer::d_flip_flop));
    bench.verdict("a Reserved SEQSEL is refused",
                  !Ccl::sequencer(0, static_cast<LutSequencer>(5)));
    bench.verdict("the sequencer is refused while its EVEN LUT is enabled "
                  "(37.6.2.7)",
                  L0::configure(LutConfig{.truth = 0xAA}, true) &&
                      !Ccl::sequencer(0, LutSequencer::d_flip_flop));
    L0::enable(false);
    bench.verdict("and accepted once that LUT is down",
                  Ccl::sequencer(0, LutSequencer::d_flip_flop) &&
                      Ccl::sequencer(0) == LutSequencer::d_flip_flop);

    teardown();
}

// =============================================================================
// b - the truth table
// =============================================================================

/// Drive the three IO inputs to a pattern and read the LUT's output.
bool lut0_out_for(uint8_t pattern) {
    drive<In0>((pattern & 1u) != 0u);
    drive<In1>((pattern & 2u) != 0u);
    drive<In2>((pattern & 4u) != 0u);
    settle_pad();
    return Out0::read();
}

/// Bring LUT0 up as a three-IO-input table. The block goes down and up
/// around it because erratum 1.7.3 leaves no other way.
bool lut0_table(uint8_t truth) {
    Ccl::enable(false);
    const bool ok = L0::configure(LutConfig{.in0 = LutInput::io,
                                            .in1 = LutInput::io,
                                            .in2 = LutInput::io,
                                            .truth = truth},
                                  true);
    Ccl::enable(true);
    return ok;
}

void tb_truth() {
    bench.verdict("PA16 follows its own internal pull", pad_is_free<PadA>());
    bench.verdict("PA17 too", pad_is_free<PadB>());
    bench.verdict("PA18 too", pad_is_free<PadC>());
    bench.verdict("PA19 too - the output pad this suite reads back",
                  pad_is_free<PadO0>());

    bench.verdict("the block came up (no generic clock: a combinational LUT "
                  "does not need one, 37.5.3)",
                  Ccl::init());
    claim_input<In0>(false);
    claim_input<In1>(false);
    claim_input<In2>(false);
    Out0::claim();
    bench.verdict("the pad map put PA16/17/18 on LUT0's three inputs and "
                  "PA19 on its output",
                  In0::lut == 0u && In0::input == 0u && In1::input == 1u &&
                      In2::input == 2u && Out0::lut == 0u);

    // The full eight rows of a table with no symmetry to hide a wiring
    // mistake: 0x96 is three-input XOR, which changes on EVERY input.
    constexpr uint8_t xor3 = lut_truth([](bool a, bool b, bool c) {
        return (a != b) != c;
    });
    static_assert(xor3 == 0x96);
    bench.verdict("LUT0 configured as the three-input XOR", lut0_table(xor3));
    uint8_t observed = 0;
    for (uint8_t k = 0; k < 8u; ++k) {
        if (lut0_out_for(k)) {
            observed = static_cast<uint8_t>(observed | (1u << k));
        }
    }
    print(serial, "  TRUTH 0x96 (XOR3) read back off the pad as ", hex(observed),
          crlf);
    bench.verdict("all eight rows of the truth table are what was written",
                  observed == xor3);

    // A second table, asymmetric the other way, to show the first was
    // not a coincidence of the driving order.
    constexpr uint8_t majority = lut_truth([](bool a, bool b, bool c) {
        return (a && b) || (b && c) || (a && c);
    });
    static_assert(majority == 0xE8);
    bench.verdict("LUT0 reconfigured as the majority function",
                  lut0_table(majority));
    observed = 0;
    for (uint8_t k = 0; k < 8u; ++k) {
        if (lut0_out_for(k)) {
            observed = static_cast<uint8_t>(observed | (1u << k));
        }
    }
    print(serial, "  TRUTH 0xE8 (majority) read back as ", hex(observed), crlf);
    bench.verdict("and so are those - so a LUT can be RECONFIGURED, which "
                  "needs LUTCTRLn.ENABLE dropped for the store as well as "
                  "CTRL.ENABLE (the bug letter a's four cells explain)",
                  observed == majority);

    // The raw control for that: the same store without dropping
    // LUTCTRLn.ENABLE first is dropped IN SILENCE - no flag, no fault,
    // the old table still decoding.
    Ccl::enable(false);
    const uint32_t before_raw = L0::ctrl();
    L0::ctrl() = (before_raw & ~CCL_LUTCTRL_TRUTH_Msk) | CCL_LUTCTRL_TRUTH(0x3Cu);
    const bool raw_dropped = L0::truth() == majority;
    print(serial, "  a raw TRUTH store into an ENABLED LUT left it at ",
          hex(L0::truth()), " (0x3C was written)", crlf);
    bench.verdict("and a store into an enabled LUT is dropped in SILENCE - "
                  "no flag, no fault, the old table still decoding",
                  raw_dropped);
    Ccl::enable(true);

    // MASK: 37.6.2.4 ties the input to zero, so a table that passes IN2
    // through must read zero however the pad is driven.
    Ccl::enable(false);
    bench.verdict("LUT0 reconfigured to pass IN2 with IN2 MASKED",
                  L0::configure(LutConfig{.in0 = LutInput::masked,
                                          .in1 = LutInput::masked,
                                          .in2 = LutInput::masked,
                                          .truth = lut_truth_pass(2)},
                                true));
    Ccl::enable(true);
    drive<In2>(true);
    settle_pad();
    const bool masked_high = Out0::read();
    drive<In2>(false);
    settle_pad();
    const bool masked_low = Out0::read();
    bench.verdict("a masked input is tied LOW whatever its pad does",
                  !masked_high && !masked_low);

    // And the control: the same table with IN2 selected really follows.
    Ccl::enable(false);
    (void)L0::configure(LutConfig{.in2 = LutInput::io,
                                  .truth = lut_truth_pass(2)},
                        true);
    Ccl::enable(true);
    drive<In2>(true);
    settle_pad();
    const bool sel_high = Out0::read();
    drive<In2>(false);
    settle_pad();
    const bool sel_low = !Out0::read();
    bench.verdict("with IO selected the same pad reaches the same table",
                  sel_high && sel_low);

    // What erratum 1.7.3 costs: reconfiguring ONE LUT drops the others.
    // LUT1 is brought up as a constant HIGH on its own pad; LUT0 is then
    // reconfigured, which needs CTRL.ENABLE down - and LUT1's pad goes
    // with it.
    Ccl::enable(false);
    Out1::claim();
    bench.verdict("LUT1 up as a constant high on OUT[1]",
                  L1::configure(LutConfig{.truth = 0xFF}, true));
    Ccl::enable(true);
    settle_pad();
    const bool lut1_high = Out1::read();
    Ccl::enable(false);
    settle_pad();
    const bool lut1_dropped = !Out1::read();
    Ccl::enable(true);
    settle_pad();
    const bool lut1_back = Out1::read();
    bench.verdict("a LUT drives its pad high with the block enabled",
                  lut1_high);
    bench.verdict("THE PRICE OF ERRATUM 1.7.3: taking the block down to "
                  "reconfigure one LUT drops every other LUT's output",
                  lut1_dropped && lut1_back);

    // The output pad is DRIVEN by the peripheral, not by PORT: the pad
    // is high while PORT's own OUT bit for it is low.
    print(serial, "  PA19 OUT bit = ",
          (PORT_REGS->GROUP[0].PORT_OUT >> 19) & 1u, ", DIR bit = ",
          (PORT_REGS->GROUP[0].PORT_DIR >> 19) & 1u,
          " while the CCL drives the pad", crlf);

    teardown();
}

// =============================================================================
// c - the input multiplexer
// =============================================================================

using Timer0 = Tc<0>;
using Timer1 = Tc<1>;
using Timer4 = Tc<4>;
using Control0 = Tcc<0>;

/// A TC as a square wave on its WO[0], slow enough that a polling loop
/// sees both levels. 48 MHz / 256 / 256 = 733 Hz.
template <class T>
bool tc_square(uint8_t generator) {
    if (!T::init(generator)) {
        return false;
    }
    if (!T::configure(TcConfig{.mode = TcMode::count8,
                               .prescaler = TcPrescaler::div256,
                               .waveform = TcWaveform::normal_pwm})) {
        return false;
    }
    return T::set_period8(0xFFu) && T::set_cc8(0, 0x80u) && T::enable(true);
}

/// Stop a TC without touching its GCLK channel - TC0/TC1 and TC2/TC3
/// SHARE one, so release() would stop the sibling (tc.md's warning).
template <class T>
void tc_stop() {
    (void)T::enable(false);
    (void)T::reset();
}

/// Bring LUT0 up with one input source and the passthrough table for it.
bool lut0_source(LutInput src, uint8_t input_index) {
    Ccl::enable(false);
    LutConfig cfg{.truth = lut_truth_pass(input_index)};
    if (input_index == 0u) {
        cfg.in0 = src;
    } else if (input_index == 1u) {
        cfg.in1 = src;
    } else {
        cfg.in2 = src;
    }
    const bool ok = L0::configure(cfg, true);
    Ccl::enable(true);
    return ok;
}

/// Does the LUT's output move? A source that toggles shows both levels;
/// one that is not connected shows one.
bool lut0_toggles(uint32_t cycles = 400'000u) {
    bool hi = false, lo = false;
    sample_levels<Out0>(cycles, hi, lo);
    return hi && lo;
}

void tc_link() {
    // LINK: LUT0's input is LUT1's OUTPUT (37.6.2.4's figure 37-5).
    claim_input<In3>(false);
    Out0::claim();
    Ccl::enable(false);
    const bool cfg_ok =
        L1::configure(LutConfig{.in0 = LutInput::io, .truth = lut_truth_pass(0)},
                      true) &&
        L0::configure(LutConfig{.in0 = LutInput::link, .truth = lut_truth_pass(0)},
                      true);
    Ccl::enable(true);
    bench.verdict("LUT1 passes its pad, LUT0 takes LUT1 through LINK", cfg_ok);
    bench.verdict("LUT0's link_source names LUT1", L0::link_source == 1u);
    drive<In3>(true);
    settle_pad();
    const bool link_high = Out0::read();
    drive<In3>(false);
    settle_pad();
    const bool link_low = !Out0::read();
    bench.verdict("a pad edge crosses LUT1 and comes out of LUT0 - the link "
                  "is the NEXT LUT's output",
                  link_high && link_low);
}

void tc_timers() {
    // TC and ALTTC, and with them erratum 1.8.3 - "the default TC
    // selection as CCL input is not TC0, but TC4" - which the matrix
    // marks REVISION B ONLY. Three timers and three configurations
    // settle it by data rather than by reading a row.
    print(serial, "  LUT0's formulas: TC -> TC", L0::tc_source, ", ALTTC -> TC",
          L0::alt_tc_source, ", TCC -> TCC", L0::tcc_source, ", AC -> CMP",
          L0::ac_source, ", SERCOM -> SERCOM", L0::sercom_source, crlf);

    bench.verdict("TC0 runs a 733 Hz square wave on WO[0]",
                  tc_square<Timer0>(gen_fast));
    bench.verdict("LUT0 takes INSEL = TC", lut0_source(LutInput::tc, 0));
    const bool tc0_seen = lut0_toggles();
    bench.verdict("THE DEFAULT TC INPUT IS TC0 - erratum 1.8.3 is revision B "
                  "and this is rev F",
                  tc0_seen);

    // The control that makes it a measurement: stop TC0 and run TC4
    // instead. If the erratum applied, THIS is the one that would move.
    tc_stop<Timer0>();
    settle_pad();
    const bool tc0_stopped = !lut0_toggles(200'000u);
    bench.verdict("with TC0 stopped the LUT output stands still", tc0_stopped);
    bench.verdict("TC4 runs the same wave", tc_square<Timer4>(gen_fast));
    const bool tc4_seen = lut0_toggles(200'000u);
    bench.verdict("and TC4 does NOT reach LUT0's default TC input", !tc4_seen);
    tc_stop<Timer4>();

    // ALTTC = TC (n + 1) = TC1 for LUT0.
    bench.verdict("TC1 runs the wave", tc_square<Timer1>(gen_fast));
    bench.verdict("LUT0 takes INSEL = ALTTC", lut0_source(LutInput::alt_tc, 0));
    bench.verdict("the alternative TC input is TC1", lut0_toggles());
    // ... and the default input is not: TC0 is still stopped.
    bench.verdict("while the default one, with TC0 stopped, is still",
                  lut0_source(LutInput::tc, 0) && !lut0_toggles(200'000u));
    tc_stop<Timer1>();
}

void tc_tcc_and_sercom() {
    // TCC: INSEL 0x8. THE DEVICE HEADER HAS NO ENUMERATOR FOR IT on any
    // variant of this pack, although 37.8.3 lists it for all of them and
    // marks only ALT2TC and ASYNCEVENT as N-only. The bench decides -
    // and the control that lets it decide is TCC0's WO[0] read off its
    // OWN pad, so a silent TCC cannot be mistaken for a dead INSEL code.
    using Pwm = TccPwm<Control0, 0, 0xFFFFu>;
    const bool tcc_up = Control0::init(gen_fast) && Pwm::setup(TccPrescaler::div1);
    Pwm::duty(0x8000u);
    In3::release();
    PadD::function(PinFunction::e, {.input_enable = true});   // TCC0/WO[0]
    bool wo_hi = false, wo_lo = false;
    {
        const uint32_t deadline = cycles_now() + 400'000u;
        while (static_cast<int32_t>(cycles_now() - deadline) < 0) {
            if (PadD::read()) wo_hi = true; else wo_lo = true;
        }
    }
    bench.verdict("TCC0 runs a 732 Hz PWM and its WO[0] pad really moves",
                  tcc_up && wo_hi && wo_lo);
    PadD::configure({});
    bench.verdict("LUT0 takes INSEL = TCC (0x8, spelled from 37.8.3 because "
                  "the device header has no name for it)",
                  lut0_source(LutInput::tcc, 0));
    const bool tcc_seen = lut0_toggles();
    bench.verdict("AND THE CODE IS REAL: TCC0's WO[0] reaches LUT0's input 0",
                  tcc_seen);
    // The readback proves the field really holds 0x8 and is not masked
    // down to something the header does know.
    const uint32_t insel0 =
        (L0::ctrl() & CCL_LUTCTRL_INSEL0_Msk) >> CCL_LUTCTRL_INSEL0_Pos;
    print(serial, "  LUTCTRL0.INSEL0 reads back as ", insel0,
          " (0x8 = TCC; the header stops at 0x9 = SERCOM with 0x8 missing)",
          crlf);
    bench.verdict("INSEL0 holds the code that was written", insel0 == 8u);
    (void)Control0::enable(false);
    settle_pad();
    bench.verdict("and with TCC0 stopped the LUT stands still",
                  !lut0_toggles(200'000u));
    Control0::release();

    // SERCOM: "the SERCOM TX signal must be output on SERCOMn/pad[0],
    // which serves as input pad to the CCL". SERCOM0 is LUT0's, and its
    // PAD[0] is PA04 on this package.
    bench.verdict("SERCOM0 up as a transmitter with TxD on its PAD[0]",
                  Talker::init(clock, 9600));
    bench.verdict("LUT0 takes INSEL = SERCOM", lut0_source(LutInput::sercom, 0));
    settle_pad();
    // Idle TxD is high; a stream of 0x00 bytes at 9600 baud spends most
    // of its time low.
    const bool idle_high = Out0::read();
    for (uint8_t i = 0; i < 8u; ++i) {
        Talker::write_byte(0x00u);
    }
    bool hi = false, lo = false;
    sample_levels<Out0>(600'000u, hi, lo);
    bench.verdict("an idle SERCOM transmitter holds LUT0's input HIGH",
                  idle_high);
    bench.verdict("and a byte stream moves it - the SERCOM TX really is a "
                  "LUT input source",
                  hi && lo);
    Talker::release();
    PadA::configure({});
    Pin<'A', 5>::configure({});
}

void tc_inputs() {
    bench.verdict("the block came up", Ccl::init(gen_fast));
    tc_link();
    tc_timers();
    tc_tcc_and_sercom();
    teardown();
}

// =============================================================================
// d - the stages: no clock, the synchronizer, the filter, the edge detector
// =============================================================================
void td_stages() {
    bench.verdict("the block came up with NO generic clock connected",
                  Ccl::init());
    GclkChannel::disconnect(Ccl::gclk_id);
    claim_input<In0>(false);
    Out0::claim();

    Ccl::enable(false);
    bench.verdict("a purely combinational LUT is configured",
                  L0::configure(LutConfig{.in0 = LutInput::io,
                                          .truth = lut_truth_pass(0)},
                                true));
    Ccl::enable(true);
    drive<In0>(true);
    settle_pad();
    const bool comb_high = Out0::read();
    drive<In0>(false);
    settle_pad();
    const bool comb_low = !Out0::read();
    bench.verdict("37.5.3 IS EXACT: with GCLK_CCL disconnected the truth "
                  "table still decodes - the generic clock is needed for "
                  "events, filters, edges and sequencers and for nothing else",
                  comb_high && comb_low);

    // The synchronizer and the filter, however, are clocked - and with
    // no GCLK they have nothing to clock with.
    Ccl::enable(false);
    (void)L0::configure(LutConfig{.in0 = LutInput::io,
                                  .truth = lut_truth_pass(0),
                                  .filter = LutFilter::sync},
                        true);
    Ccl::enable(true);
    drive<In0>(true);
    settle_pad();
    const bool sync_dead = !Out0::read();
    print(serial, "  synchronizer with no GCLK_CCL: output ",
          sync_dead ? "never rises" : "rises anyway", crlf);
    bench.verdict("a synchronized LUT with no clock passes nothing",
                  sync_dead);

    // Give it the slow clock and it comes alive.
    bench.verdict("the slow generic clock is built (OSC48M / 4096 = 11719 Hz)",
                  slow_clock_up() && Ccl::clock(gen_slow));
    settle(8u * slow_period);
    bench.verdict("and the same LUT now follows its pad", Out0::read());
    drive<In0>(false);
    settle(8u * slow_period);
    bench.verdict("both ways", !Out0::read());

    // The filter option: same behaviour at DC, more delay - which is
    // letter g's measurement, not this one's.
    Ccl::enable(false);
    (void)L0::configure(LutConfig{.in0 = LutInput::io,
                                  .truth = lut_truth_pass(0),
                                  .filter = LutFilter::filter},
                        true);
    Ccl::enable(true);
    drive<In0>(true);
    settle(16u * slow_period);
    const bool filt_high = Out0::read();
    drive<In0>(false);
    settle(16u * slow_period);
    const bool filt_low = !Out0::read();
    bench.verdict("the filter option passes a steady level too",
                  filt_high && filt_low);

    // THE EDGE DETECTOR: a pulse of ONE GCLK_CCL cycle on a rising edge
    // (37.6.2.6). At 11.719 kHz that is 85 us - long enough for a
    // software stopwatch to measure.
    Ccl::enable(false);
    bench.verdict("the edge detector is configured (with the synchronizer, "
                  "which 37.6.2.6 requires)",
                  L0::configure(LutConfig{.in0 = LutInput::io,
                                          .truth = lut_truth_pass(0),
                                          .filter = LutFilter::sync,
                                          .edge_detect = true},
                                true));
    Ccl::enable(true);
    drive<In0>(false);
    settle(16u * slow_period);
    bench.verdict("the output is low with the input low", !Out0::read());

    uint32_t width_min = no_flip, width_max = 0;
    uint8_t caught = 0;
    for (uint8_t r = 0; r < 8u; ++r) {
        drive<In0>(false);
        settle(8u * slow_period);
        drive<In0>(true);
        const uint32_t rise = time_until([] { return Out0::read(); },
                                         40u * slow_period);
        if (rise == no_flip) {
            continue;
        }
        const uint32_t fall = time_until([] { return !Out0::read(); },
                                         40u * slow_period);
        if (fall == no_flip) {
            continue;
        }
        ++caught;
        const uint32_t w = fall - rise;
        if (w < width_min) width_min = w;
        if (w > width_max) width_max = w;
    }
    print(serial, "  edge strobe caught ", caught, "/8 times, width ",
          width_min, "..", width_max, " cycles (one GCLK_CCL period = ",
          slow_period, ")", crlf);
    bench.verdict("a rising edge produces a PULSE and not a level", caught >= 6u);
    bench.verdict("and the pulse is one GCLK_CCL period long (37.6.2.6)",
                  caught >= 6u && width_min > slow_period / 2u &&
                      width_max < 2u * slow_period);

    // Held high, the output stays low: it is an edge detector, not a
    // level pass.
    drive<In0>(true);
    settle(32u * slow_period);
    bench.verdict("with the input held high the output is back low",
                  !Out0::read());

    // 37.6.2.6: "after disabling a LUT, the corresponding internal Edge
    // Detector logic is cleared one APB clock cycle later" - so a
    // disabled LUT's pad is low whatever the input does.
    L0::enable(false);
    settle(4u * slow_period);
    bench.verdict("a disabled LUT drives nothing (its pad reads low)",
                  !Out0::read());

    GclkChannel::disconnect(Ccl::gclk_id);
    teardown();
}

// =============================================================================
// e - the sequencers
// =============================================================================

/// Bring the pair LUT0 (even, D/J/D/S) + LUT1 (odd, G/K/G/R) up, both
/// passing their own IO pad, with the named sequential module between
/// them. The module's output REPLACES LUT0's, so PA19 shows it.
bool pair_up(LutSequencer s) {
    Ccl::enable(false);
    L0::enable(false);
    L1::enable(false);
    if (!Ccl::sequencer(0, s)) {
        return false;
    }
    const bool ok =
        L0::configure(LutConfig{.in0 = LutInput::io, .truth = lut_truth_pass(0)},
                      true) &&
        L1::configure(LutConfig{.in0 = LutInput::io, .truth = lut_truth_pass(0)},
                      true);
    Ccl::enable(true);
    return ok;
}

/// Drive the even LUT's input (PA16) and the odd LUT's (PA08), let the
/// sequencer clock a few times, and read its output.
bool seq_out(bool even, bool odd) {
    drive<In0>(even);
    drive<In3>(odd);
    settle(8u * slow_period);
    return Out0::read();
}

void te_sequencers() {
    bench.verdict("the block came up on the slow clock, which every "
                  "sequential module needs (37.6.2.7)",
                  slow_clock_up() && Ccl::init(gen_slow));
    claim_input<In0>(false);
    claim_input<In3>(false);
    Out0::claim();
    Out1::claim();

    // ---- the gated D flip-flop, table 37-2 --------------------------------
    //
    // R is the even LUT's ENABLE (inverted); G is the odd LUT's output,
    // D the even LUT's, and the output is refreshed on the rising edge
    // of GCLK_CCL.
    bench.verdict("the pair is a gated D flip-flop",
                  pair_up(LutSequencer::d_flip_flop));
    const bool dff_set = seq_out(true, true);         // G=1 D=1 -> Set
    const bool dff_clear = !seq_out(false, true);     // G=1 D=0 -> Clear
    // THE GATE COMES DOWN FIRST, ALWAYS. Both stimuli are pads walking
    // between the rails through their own pulls, so moving D and G in
    // the same breath is a race the pads decide: close the gate, let it
    // settle, and only then move D.
    drive<In3>(false);                                // G = 0
    settle(8u * slow_period);
    drive<In0>(true);                                 // D back to 1
    settle(8u * slow_period);
    const bool dff_hold_low = !Out0::read();          // G=0 -> hold
    const bool dff_set_again = seq_out(true, true);
    drive<In3>(false);                                // G = 0
    settle(8u * slow_period);
    drive<In0>(false);                                // D to 0
    settle(8u * slow_period);
    const bool dff_hold_high = Out0::read();
    print(serial, "  DFF: set=", dff_set ? 1 : 0, " clear=", dff_clear ? 1 : 0,
          " hold-low=", dff_hold_low ? 1 : 0, " hold-high=",
          dff_hold_high ? 1 : 0, crlf);
    bench.verdict("table 37-2: G=1 D=1 sets, G=1 D=0 clears",
                  dff_set && dff_clear);
    bench.verdict("and G=0 holds, in both states", dff_hold_low &&
                                                   dff_set_again &&
                                                   dff_hold_high);

    // The ODD LUT's own output is still its own - the sequencer takes
    // over the EVEN one's pad and nothing else.
    drive<In3>(true);
    settle(8u * slow_period);
    const bool odd_high = Out1::read();
    drive<In3>(false);
    settle(8u * slow_period);
    const bool odd_low = !Out1::read();
    bench.verdict("the odd LUT's own output stays available on OUT[1]",
                  odd_high && odd_low);

    // ---- the JK flip-flop, table 37-3 -------------------------------------
    bench.verdict("the pair is a JK flip-flop", pair_up(LutSequencer::jk_flip_flop));
    const bool jk_set = seq_out(true, false);         // J=1 K=0 -> Set
    const bool jk_hold = seq_out(false, false);       // 0 0 -> hold (still 1)
    const bool jk_clear = !seq_out(false, true);      // J=0 K=1 -> Clear
    const bool jk_hold_low = !seq_out(false, false);
    print(serial, "  JK: set=", jk_set ? 1 : 0, " hold=", jk_hold ? 1 : 0,
          " clear=", jk_clear ? 1 : 0, " hold-low=", jk_hold_low ? 1 : 0, crlf);
    bench.verdict("table 37-3: 1/0 sets, 0/1 clears, 0/0 holds either way",
                  jk_set && jk_hold && jk_clear && jk_hold_low);

    // J=K=1 toggles - once per GCLK_CCL, i.e. a 5.9 kHz square wave off
    // an 11.7 kHz clock, which a polling loop catches in both states.
    drive<In0>(true);
    drive<In3>(true);
    settle(8u * slow_period);
    bool hi = false, lo = false;
    sample_levels<Out0>(200u * slow_period, hi, lo);
    bench.verdict("and 1/1 TOGGLES - a divide-by-two with no CPU in it",
                  hi && lo);

    // ---- FEEDBACK, and what it is for -------------------------------------
    //
    // The sequencer's output is fed back as an input, so K = Q turns the
    // same JK into "set, then toggle": held J high the output oscillates,
    // where a MASKED K would set it once and leave it. Two configurations,
    // one difference.
    Ccl::enable(false);
    L0::enable(false);
    L1::enable(false);
    (void)Ccl::sequencer(0, LutSequencer::jk_flip_flop);
    (void)L0::configure(LutConfig{.in0 = LutInput::io, .truth = lut_truth_pass(0)},
                        true);
    (void)L1::configure(LutConfig{.in1 = LutInput::feedback,
                                  .truth = lut_truth_pass(1)},
                        true);
    Ccl::enable(true);
    drive<In0>(true);
    settle(8u * slow_period);
    bool fb_hi = false, fb_lo = false;
    sample_levels<Out0>(200u * slow_period, fb_hi, fb_lo);
    bench.verdict("with K taken from FEEDBACK the flip-flop oscillates on a "
                  "held J", fb_hi && fb_lo);
    Ccl::enable(false);
    (void)L1::configure(LutConfig{.truth = 0x00}, true);   // K forced low
    Ccl::enable(true);
    drive<In0>(true);
    settle(16u * slow_period);
    bool mk_hi = false, mk_lo = false;
    sample_levels<Out0>(200u * slow_period, mk_hi, mk_lo);
    bench.verdict("and with K forced low it sets once and stays - so the "
                  "difference really was the feedback path",
                  mk_hi && !mk_lo);

    // ---- the gated D latch, table 37-4 ------------------------------------
    bench.verdict("the pair is a gated D latch", pair_up(LutSequencer::latch));
    const bool dl_set = seq_out(true, true);          // G=1 D=1 -> Set
    const bool dl_clear = !seq_out(false, true);      // G=1 D=0 -> Clear
    const bool dl_set2 = seq_out(true, true);
    drive<In3>(false);                                // G = 0 first
    settle(8u * slow_period);
    drive<In0>(false);                                // then D = 0
    settle(8u * slow_period);
    const bool dl_hold = Out0::read();
    print(serial, "  D-latch: set=", dl_set ? 1 : 0, " clear=", dl_clear ? 1 : 0,
          " hold=", dl_hold ? 1 : 0, crlf);
    bench.verdict("table 37-4: G=1 follows D, G=0 holds",
                  dl_set && dl_clear && dl_set2 && dl_hold);

    // ---- the RS latch, table 37-5, and erratum 1.7.1 ----------------------
    //
    // 1.7.1 says "the reset of the RS latch is not functional; the latch
    // can only be cleared by disabling the LUT" - and the matrix marks
    // it REVISION B ONLY. S=1/R=1 is the forbidden state and is never
    // asked for here.
    bench.verdict("the pair is an RS latch", pair_up(LutSequencer::rs_latch));
    const bool rs_set = seq_out(true, false);         // S=1 R=0 -> Set
    const bool rs_hold = seq_out(false, false);       // 0/0 -> hold
    const bool rs_clear = !seq_out(false, true);      // S=0 R=1 -> Clear
    const bool rs_hold_low = !seq_out(false, false);
    print(serial, "  RS: set=", rs_set ? 1 : 0, " hold=", rs_hold ? 1 : 0,
          " reset=", rs_clear ? 1 : 0, " hold-low=", rs_hold_low ? 1 : 0, crlf);
    bench.verdict("table 37-5: S sets and the latch HOLDS", rs_set && rs_hold);
    bench.verdict("ERRATUM 1.7.1 IS REVISION B: the RS reset works on this "
                  "silicon", rs_clear && rs_hold_low);

    // ---- the asynchronous clear, and ERRATUM 1.7.2 ------------------------
    //
    // Disabling the EVEN LUT clears the latch (37.6.2.7) - and 1.7.2,
    // live on EVERY revision, says the sequential logic then STAYS under
    // reset until CTRL.ENABLE is written again. Both halves, with the
    // control that makes the second one a measurement: the LUT is
    // re-enabled RAW (the driver's own enable() would restate CTRL for
    // us, which is exactly the workaround under test).
    (void)seq_out(true, false);                       // latch set
    const bool latched = Out0::read();
    L0::ctrl() = L0::ctrl() & ~CCL_LUTCTRL_ENABLE_Msk;
    settle(8u * slow_period);
    const bool cleared_by_disable = !Out0::read();
    bench.verdict("disabling the even LUT clears the latch asynchronously",
                  latched && cleared_by_disable);

    L0::ctrl() = L0::ctrl() | CCL_LUTCTRL_ENABLE_Msk;   // RAW, no restate
    drive<In0>(true);
    drive<In3>(false);
    settle(16u * slow_period);
    const bool stuck_under_reset = !Out0::read();
    Ccl::restate_enable();
    settle(16u * slow_period);
    const bool alive_again = Out0::read();
    print(serial, "  after a raw re-enable the latch is ",
          stuck_under_reset ? "STILL UNDER RESET" : "alive",
          "; after CTRL.ENABLE is restated it is ",
          alive_again ? "alive" : "STILL DEAD", crlf);
    bench.verdict("ERRATUM 1.7.2 (every revision) reproduced: a re-enabled "
                  "even LUT leaves its sequential logic under reset",
                  stuck_under_reset);
    bench.verdict("and the errata's own workaround - write CTRL.ENABLE again "
                  "- is what brings it back",
                  alive_again);

    teardown();
}

// =============================================================================
// f - events, both ways
// =============================================================================

// The DMA witness: a channel armed with NO hardware trigger, so only an
// event can move its bytes (the test_samc_evsys technique).
constexpr uint8_t dma_ch = 0;
constexpr uint8_t user_dmac_ch0 = 5;
constexpr uint8_t ev_ch = 2;
using Copy = DmaChannel<dma_ch>;

constexpr uint16_t payload = 16;
volatile uint8_t src[payload];
volatile uint8_t dst[payload];

bool arm_event_driven_copy(uint8_t seed) {
    for (uint16_t i = 0; i < payload; ++i) {
        src[i] = static_cast<uint8_t>(seed + i);
        dst[i] = 0;
    }
    if (!Copy::reset()) {
        return false;
    }
    if (!Copy::configure(DmaChannelConfig{.trigger = dma_trigger_none,
                                          .action = DmaTriggerAction::block,
                                          .event_action = DmaEventAction::trigger,
                                          .event_input = true})) {
        return false;
    }
    if (!Copy::load(DmaTransfer{.source = &src[0],
                                .destination = &dst[0],
                                .beats = payload,
                                .beat = DmaBeat::byte})) {
        return false;
    }
    return Copy::enable(true);
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

void tf_events() {
    bench.verdict("the block came up on the slow clock",
                  slow_clock_up() && Ccl::init(gen_slow));
    Evsys::bus_clock(true);
    claim_input<In0>(false);
    Out0::claim();

    // ---- OUT: LUTOUT0 as an EVSYS generator -------------------------------
    Ccl::enable(false);
    bench.verdict("LUT0 passes its pad and publishes its output as an event",
                  L0::configure(LutConfig{.in0 = LutInput::io,
                                          .truth = lut_truth_pass(0),
                                          .event_out = true},
                                true));
    Ccl::enable(true);
    bench.verdict("the DMA channel arms with NO hardware trigger",
                  arm_event_driven_copy(0x70));
    bench.verdict("and the channel is routed to LUTOUT0 on the asynchronous "
                  "path",
                  Evsys::connect(user_dmac_ch0, ev_ch,
                                 EventChannelConfig{
                                     .generator = L0::event_generator,
                                     .path = EventPath::asynchronous}));
    settle_pad();
    bench.verdict("nothing has moved yet", destination_untouched());
    drive<In0>(true);
    settle_pad();
    print(serial, "  after one LUT output edge: dst[0..3] = ", dst[0], " ",
          dst[1], " ", dst[2], " ", dst[3], crlf);
    bench.verdict("A LUT OUTPUT EDGE MOVED THE BYTES - pad to truth table to "
                  "EVSYS to DMAC, with no CPU in the path",
                  destination_matches(0x70));
    bench.verdict("and the channel completed rather than erroring",
                  !Copy::fetch_error());

    // The control: with LUTEO clear the same edge reaches nothing.
    Ccl::enable(false);
    (void)L0::configure(LutConfig{.in0 = LutInput::io,
                                  .truth = lut_truth_pass(0),
                                  .event_out = false},
                        true);
    Ccl::enable(true);
    drive<In0>(false);
    settle_pad();
    bench.verdict("the DMA channel re-arms", arm_event_driven_copy(0x90));
    drive<In0>(true);
    settle_pad();
    bench.verdict("with LUTEO clear the same edge moves nothing",
                  destination_untouched());
    Evsys::disconnect(user_dmac_ch0);
    (void)Copy::enable(false);

    // ---- IN: an event into a LUT ------------------------------------------
    //
    // Table 29-3 gives LUTIN0..3 the ASYNCHRONOUS PATH ONLY, and the
    // driver refuses anything else rather than writing a channel the
    // fabric will not honour.
    bench.verdict("a synchronous channel into a LUT's event input is refused",
                  !L0::listen(ev_ch, EventChannelConfig{
                                         .path = EventPath::synchronous,
                                         .edge = EventEdge::rising}));
    bench.verdict("and a resynchronized one too",
                  !L0::listen(ev_ch, EventChannelConfig{
                                         .path = EventPath::resynchronized,
                                         .edge = EventEdge::rising}));

    // The generator is a TC overflow: a HARDWARE generator, which is
    // what an asynchronous channel needs. (A SOFTWARE event cannot be
    // used here at all - test_samc_evsys measured that one does not
    // cross an asynchronous channel, and this user has no other path.)
    // EVCTRL is enable-protected with the rest, so the event output is
    // written BEFORE the timer is enabled and not after.
    constexpr TcConfig pacer{.mode = TcMode::count8,
                             .prescaler = TcPrescaler::div256,
                             .waveform = TcWaveform::normal_pwm};
    bench.verdict("TC0 overflows at about 733 Hz and publishes the overflow "
                  "as an event",
                  Timer0::init(gen_fast) && Timer0::configure(pacer) &&
                      Timer0::event_config(
                          pacer, TcEventConfig{.overflow_out = true}) &&
                      Timer0::set_period8(0xFFu) && Timer0::enable(true));
    Ccl::enable(false);
    bench.verdict("LUT0 takes its event line as input 0",
                  L0::configure(LutConfig{.in0 = LutInput::event,
                                          .truth = lut_truth_pass(0),
                                          .event_in = true},
                                true));
    Ccl::enable(true);
    bench.verdict("the channel is routed on the asynchronous path",
                  L0::listen(ev_ch, EventChannelConfig{
                                        .generator = Timer0::overflow_generator,
                                        .path = EventPath::asynchronous}));
    bool ev_hi = false, ev_lo = false;
    sample_levels<Out0>(400u * slow_period, ev_hi, ev_lo);
    print(serial, "  with the event arriving, the LUT output was seen ",
          ev_hi ? "high" : "never high", " and ", ev_lo ? "low" : "never low",
          crlf);
    bench.verdict("AN EVENT REACHES THE TRUTH TABLE - and it arrives as the "
                  "one-GCLK STROBE 37.6.2.4 describes, not as a level",
                  ev_hi && ev_lo);

    // The control: LUTEI clear and the same events reach nothing.
    Ccl::enable(false);
    (void)L0::configure(LutConfig{.truth = lut_truth_pass(0)}, true);
    Ccl::enable(true);
    bool off_hi = false, off_lo = false;
    sample_levels<Out0>(200u * slow_period, off_hi, off_lo);
    bench.verdict("with LUTEI clear the same events reach nothing",
                  !off_hi && off_lo);

    // And the fact that follows from the two: a SOFTWARE event cannot
    // drive this peripheral at all, because its user is asynchronous-only
    // and a software event does not cross an asynchronous channel.
    Ccl::enable(false);
    (void)L0::configure(LutConfig{.in0 = LutInput::event,
                                  .truth = lut_truth_pass(0),
                                  .event_in = true},
                        true);
    Ccl::enable(true);
    tc_stop<Timer0>();
    (void)L0::listen(ev_ch, EventChannelConfig{.path = EventPath::asynchronous});
    settle(8u * slow_period);

    auto ever_high = [](uint32_t cycles) {
        const uint32_t end = cycles_now() + cycles;
        while (static_cast<int32_t>(cycles_now() - end) < 0) {
            if (Out0::read()) {
                return true;
            }
        }
        return false;
    };

    // The control FIRST: with the timer stopped and nobody triggering,
    // the output must be quiet. Without this the next two numbers mean
    // nothing.
    const bool idle_high = ever_high(200u * slow_period);
    bench.verdict("with the generator stopped and nothing triggering, the "
                  "LUT output is quiet",
                  !idle_high);

    // SINGLE, SPACED software events: one trigger, then a window several
    // GCLK_CCL periods wide to catch the 85 us strobe it would produce.
    uint8_t caught_single = 0;
    for (uint8_t i = 0; i < 16u; ++i) {
        Evsys::trigger(ev_ch);
        if (ever_high(6u * slow_period)) {
            ++caught_single;
        }
        settle(4u * slow_period);
    }

    // And the same channel HAMMERED: a trigger every few hundred
    // nanoseconds for two hundred periods.
    const uint32_t hammer_end = cycles_now() + 200u * slow_period;
    bool hammered_high = false;
    while (static_cast<int32_t>(cycles_now() - hammer_end) < 0) {
        Evsys::trigger(ev_ch);
        if (Out0::read()) {
            hammered_high = true;
        }
    }
    // The second control: unhook the user and fire the same sixteen.
    L0::unlisten();
    settle(16u * slow_period);   // let the last strobe of the hammer expire
    uint8_t caught_unhooked = 0;
    for (uint8_t i = 0; i < 16u; ++i) {
        Evsys::trigger(ev_ch);
        if (ever_high(6u * slow_period)) {
            ++caught_unhooked;
        }
        settle(4u * slow_period);
    }
    (void)L0::listen(ev_ch, EventChannelConfig{.path = EventPath::asynchronous});

    print(serial, "  software events into a LUT: ", caught_single,
          "/16 single ones caught, ", caught_unhooked,
          "/16 with the user disconnected, hammered = ",
          hammered_high ? 1 : 0, crlf);
    bench.verdict("with the user disconnected nothing arrives, so the "
                  "sixteen above were really the events",
                  caught_unhooked == 0u);
    bench.verdict("A SOFTWARE EVENT *DOES* CROSS AN ASYNCHRONOUS CHANNEL - "
                  "every single one of sixteen reaches this LUT, where "
                  "test_samc_evsys letter d found none reaching a DMA "
                  "channel: what differs is the USER's input stage, not the "
                  "path",
                  caught_single == 16u);

    // The second witness, and a different kind of one: the whole chain
    // from a software event to memory - trigger -> asynchronous channel
    // -> the CCL's edge detector -> the truth table -> LUTOUT0 -> a
    // second channel -> the DMAC. A pad poll could in principle be
    // fooled; a block of bytes that moved cannot.
    constexpr uint8_t ev_ch_out = 3;
    Ccl::enable(false);
    (void)L0::configure(LutConfig{.in0 = LutInput::event,
                                  .truth = lut_truth_pass(0),
                                  .event_in = true,
                                  .event_out = true},
                        true);
    Ccl::enable(true);
    bench.verdict("the DMA channel re-arms behind LUTOUT0",
                  arm_event_driven_copy(0x20) &&
                      Evsys::connect(user_dmac_ch0, ev_ch_out,
                                     EventChannelConfig{
                                         .generator = L0::event_generator,
                                         .path = EventPath::asynchronous}));
    settle(8u * slow_period);
    bench.verdict("and nothing has moved", destination_untouched());
    Evsys::trigger(ev_ch);
    settle(16u * slow_period);
    print(serial, "  after ONE software event: dst[0..3] = ", dst[0], " ",
          dst[1], " ", dst[2], " ", dst[3], crlf);
    bench.verdict("ONE software event moved a block of memory THROUGH THE "
                  "CCL - the second witness agrees with the pad",
                  destination_matches(0x20));
    Evsys::disconnect(user_dmac_ch0);
    (void)Copy::enable(false);
    print(serial, "  hammering the channel raises the output too (",
          hammered_high ? 1 : 0, "), which is expected once one event does",
          crlf);

    L0::unlisten();
    teardown();
}

// =============================================================================
// g - THE HEADLINE: what a CCL output costs, against the AC's fraction + 2
// =============================================================================

/// COMP0 with the given output routing: positive input = AIN[0] = PA04
/// driven by PORT as a plain GPIO, negative input = the comparator's own
/// VDD scaler at mid-rail. The ac_sync_probe setup exactly, so the two
/// measurements are comparable number for number.
bool comp_up(AcOut out) {
    if (!Comp::configure({
            .positive = AcPositive::pin0,
            .negative = AcNegative::vscale,
            .speed = AcSpeed::high,
            .filter = AcFilter::off,
            .out = out,
        })) {
        return false;
    }
    Comp::scaler(31);
    if (!Comp::enable(true)) {
        return false;
    }
    const uint32_t deadline = cycles_now() + 32u * slow_period;
    while (!Comp::ready()) {
        if (static_cast<int32_t>(cycles_now() - deadline) >= 0) {
            return false;
        }
    }
    settle(4u * slow_period);
    return true;
}

/// One phase-anchored timed rising edge: prime the stimulus low, wait
/// for the observer to be low (that flip lands ON a GCLK edge, which is
/// what makes the staircase deterministic), spend `phase`, drive high,
/// time the flip.
template <typename F>
uint32_t timed_edge(F cond, uint32_t phase, uint32_t budget_periods) {
    AcStim::clear();
    const uint32_t low = time_until([&] { return !cond(); }, 40u * slow_period);
    if (low == no_flip) {
        return no_flip;
    }
    spin_until(low + 4u * slow_period + phase);
    const uint32_t t0 = cycles_now();
    AcStim::set();
    const uint32_t t1 = time_until(cond, budget_periods * slow_period);
    return (t1 == no_flip) ? no_flip : (t1 - t0);
}

/// A whole 32-step phase sweep, three shots per step, the MINIMUM kept
/// (the tick interrupt can stretch a poll, never shrink it). Reports the
/// latency in cycles; one GCLK period is `slow_period`.
template <typename F>
void sweep(const char* name, F cond, uint32_t& lo, uint32_t& hi) {
    lo = no_flip;
    hi = 0;
    uint8_t got = 0;
    for (uint8_t s = 0; s < 32u; ++s) {
        uint32_t best = no_flip;
        for (uint8_t r = 0; r < 3u; ++r) {
            const uint32_t dt = timed_edge(cond, static_cast<uint32_t>(s) * 128u, 12);
            if (dt < best) {
                best = dt;
            }
        }
        if (best == no_flip) {
            continue;
        }
        ++got;
        if (best < lo) lo = best;
        if (best > hi) hi = best;
    }
    // Twenty randomized shots on top, so a phase the staircase steps
    // over cannot hide a longer path.
    for (uint8_t r = 0; r < 20u; ++r) {
        const uint32_t dt = timed_edge(cond, lcg(slow_period), 12);
        if (dt == no_flip) {
            continue;
        }
        ++got;
        if (dt < lo) lo = dt;
        if (dt > hi) hi = dt;
    }
    if (lo == no_flip) {
        print(serial, "  ", name, ": NEVER FLIPPED", crlf);
        return;
    }
    print(serial, "  ", name, ": ", lo, "..", hi, " cycles = ",
          (lo * 100u) / slow_period, "..", (hi * 100u) / slow_period,
          " hundredths of a GCLK period (", got, " shots)", crlf);
}

bool out0_high() { return Out0::read(); }
bool cmp_pad_high() { return CmpPad::read(); }

/// LUT0 as a passthrough of the comparator on its input 1, with the
/// named output stage.
bool lut0_from_ac(LutFilter filter) {
    Ccl::enable(false);
    L0::enable(false);
    L1::enable(false);
    (void)Ccl::sequencer(0, LutSequencer::none);
    const bool ok = L0::configure(LutConfig{.in1 = LutInput::ac,
                                            .truth = lut_truth_pass(1),
                                            .filter = filter},
                                  true);
    Ccl::enable(true);
    return ok;
}

void tg_latency() {
    bench.verdict("PA04 follows its own pull, so it can be driven as the "
                  "comparator's input", pad_is_free<AcStim>());
    AcStim::output();
    AcStim::clear();

    bench.verdict("the slow generic clock is up on generator 1 (OSC48M / 4096 "
                  "= 11719 Hz, so one period is 4096 CPU cycles)",
                  slow_clock_up());
    bench.verdict("the AC block is on it", Ac::init(gen_slow));
    bench.verdict("and so is the CCL", Ccl::init(gen_slow));
    Out0::claim();
    CmpPad::function(PinFunction::h, {.input_enable = true});

    bench.verdict("LUT0's AC input is comparator 0", L0::ac_source == 0u);

    // ---- the reference: the AC's own synchronized output ------------------
    uint32_t lo = 0, hi = 0;
    bench.verdict("the comparator is up with OUT = SYNC", comp_up(AcOut::synchronous));
    sweep("AC SYNC on the CMP0 pad     ", cmp_pad_high, lo, hi);
    const uint32_t ac_sync_lo = lo, ac_sync_hi = hi;
    bench.verdict("the AC's synchronized output costs the fraction to the next "
                  "edge plus TWO whole periods (docs/samc/ac.md, re-measured "
                  "here as the reference this letter compares against)",
                  ac_sync_lo != no_flip && ac_sync_lo >= 2u * slow_period &&
                      ac_sync_hi < 4u * slow_period);

    // ---- and the AC's asynchronous output, for scale ----------------------
    bench.verdict("the comparator is up with OUT = ASYNC",
                  comp_up(AcOut::asynchronous));
    sweep("AC ASYNC on the CMP0 pad    ", cmp_pad_high, lo, hi);
    const uint32_t ac_async_lo = lo, ac_async_hi = hi;
    bench.verdict("the asynchronous output does not wait for a clock edge",
                  hi != no_flip && ac_async_hi < slow_period / 4u);

    // ---- THE ANSWER: the CCL on the AC's ASYNCHRONOUS flavour -------------
    //
    // 40.8.13's note - "for internal use of the comparison results by the
    // CCL, this bit must be 0x1 or 0x2" - says the CCL taps whichever
    // flavour COMPCTRL.OUT selects. With ASYNC selected, a combinational
    // LUT should dodge the comparator's sampler entirely.
    bench.verdict("LUT0 passes the comparator through, combinationally",
                  lut0_from_ac(LutFilter::none));
    sweep("AC ASYNC -> LUT, no filter  ", out0_high, lo, hi);
    const uint32_t comb_lo = lo, comb_hi = hi;
    bench.verdict("THE CCL SEES THE COMPARATOR AT ALL - 40.8.13's note is "
                  "about COMPCTRL.OUT and not about a pad",
                  comb_lo != no_flip);
    bench.verdict("AND A COMBINATIONAL LUT COSTS NO CLOCK EDGE: the whole "
                  "phase sweep lands inside a quarter of a GCLK period",
                  comb_lo != no_flip && comb_hi < slow_period / 4u);

    bench.verdict("the same LUT with the SYNCHRONIZER",
                  lut0_from_ac(LutFilter::sync));
    sweep("AC ASYNC -> LUT, SYNCH      ", out0_high, lo, hi);
    const uint32_t sync_lo = lo, sync_hi = hi;
    bench.verdict("THE SYNCHRONIZER COSTS THE FRACTION TO THE NEXT EDGE PLUS "
                  "EXACTLY ONE WHOLE PERIOD - half what the comparator's own "
                  "sampler charges, out of ONE LUT",
                  sync_lo != no_flip && sync_lo >= slow_period &&
                      sync_hi < 2u * slow_period + slow_period / 8u);

    bench.verdict("the same LUT with the FILTER", lut0_from_ac(LutFilter::filter));
    sweep("AC ASYNC -> LUT, FILTER     ", out0_high, lo, hi);
    const uint32_t filt_lo = lo, filt_hi = hi;
    bench.verdict("THE FILTER COSTS THE FRACTION PLUS EXACTLY THREE PERIODS "
                  "- so 37.6.2.5's 'two to five GCLK cycles' is a range over "
                  "the two OPTIONS and their phase, and each option's own "
                  "cost is exact",
                  filt_lo != no_flip && filt_lo >= 3u * slow_period &&
                      filt_hi < 4u * slow_period + slow_period / 8u);

    // ---- the LUT PAIR as a one-stage synchronizer -------------------------
    //
    // The lead docs/samc/ac.md left open: a DFF clocked by the wanted
    // clock is a ONE-stage synchronizer, where the AC's own sampler
    // costs two. D is the even LUT (the comparator), G is the odd one
    // (a constant high), and the module's output replaces LUT0's.
    Ccl::enable(false);
    L0::enable(false);
    L1::enable(false);
    const bool dff_ok =
        Ccl::sequencer(0, LutSequencer::d_flip_flop) &&
        L0::configure(LutConfig{.in1 = LutInput::ac, .truth = lut_truth_pass(1)},
                      true) &&
        L1::configure(LutConfig{.truth = 0xFF}, true);
    Ccl::enable(true);
    bench.verdict("the pair is a D flip-flop with the comparator as D and a "
                  "constant high as G", dff_ok);
    sweep("AC ASYNC -> LUT pair as DFF ", out0_high, lo, hi);
    const uint32_t dff_lo = lo, dff_hi = hi;
    bench.verdict("it flips", dff_lo != no_flip);
    bench.verdict("AND A DFF COSTS NO WHOLE PERIOD AT ALL - just the fraction "
                  "to the next GCLK_CCL edge, which is a TRUE one-stage "
                  "synchronizer and the cheapest path in this table",
                  dff_lo != no_flip && dff_lo < slow_period / 8u &&
                      dff_hi < slow_period + slow_period / 8u);

    // ---- the verdict the memory asked for ---------------------------------
    auto row = [](const char* name, uint32_t lo_c, uint32_t hi_c) {
        print(serial, "  ", name, " ", lo_c, "..", hi_c, " cycles = ",
              (lo_c * 100u) / slow_period, "..", (hi_c * 100u) / slow_period,
              " hundredths of a period", crlf);
    };
    print(serial, crlf, "  ONE GCLK PERIOD = ", slow_period,
          " CPU cycles; the measurement chain's own floor is the AC ASYNC row.",
          crlf);
    row("AC pad, OUT = ASYNC      ", ac_async_lo, ac_async_hi);
    row("LUT, combinational       ", comb_lo, comb_hi);
    row("LUT pair as a DFF        ", dff_lo, dff_hi);
    row("LUT, FILTSEL = SYNCH     ", sync_lo, sync_hi);
    row("AC pad, OUT = SYNC       ", ac_sync_lo, ac_sync_hi);
    row("LUT, FILTSEL = FILTER    ", filt_lo, filt_hi);
    bench.verdict("THE ANSWER docs/samc/ac.md ASKED FOR: the comparator's own "
                  "synchronized output is the SLOWEST clocked path here - one "
                  "LUT with the synchronizer costs one whole period less, and "
                  "a LUT PAIR as a DFF costs no whole period at all",
                  dff_lo != no_flip && sync_lo != no_flip &&
                      ac_sync_lo != no_flip && dff_lo < sync_lo &&
                      sync_lo < ac_sync_lo);
    bench.verdict("and the CCL's own combinational path dodges every sampler: "
                  "it tracks the comparator's ASYNCHRONOUS output to within a "
                  "few CPU cycles",
                  comb_lo != no_flip && ac_async_lo != no_flip &&
                      comb_hi < ac_async_lo + slow_period / 16u);

    Comp::enable(false);
    Ac::release();
    AcStim::input();          // give the stimulus pad back as an input
    teardown();
}

// =============================================================================
// the menu
// =============================================================================
void banner() {
    print(serial, crlf,
          "test_samc_ccl - SAMC21J18A configurable custom logic (ch. 37), clk=",
          SysClock::hz, " Hz", crlf);
    bench.menu();
}

} // namespace

extern "C" void SysTick_Handler() { brio::Ticker::tick(); }
extern "C" void SERCOM5_Handler() { (void)Serial::isr(); }
/// SERCOM0 is letter c's LUT input source; its vector is bound because
/// an unserved transmit ring never drains.
extern "C" void SERCOM0_Handler() { (void)Talker::isr(); }
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

    bench.letter('a', "the block, and the enable-protection dispute", ta_block);
    bench.letter('b', "the truth table, all eight rows", tb_truth);
    bench.letter('c', "the input multiplexer", tc_inputs);
    bench.letter('d', "the stages: no clock, synchronizer, filter, edge",
                 td_stages);
    bench.letter('e', "the sequencers", te_sequencers);
    bench.letter('f', "events, both ways", tf_events);
    bench.letter('g', "what a CCL output costs, against the AC's fraction + 2",
                 tg_latency);

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

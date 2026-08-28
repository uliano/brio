/*
 * ccl.hpp
 *
 * The SAM C21 Configurable Custom Logic (DS60001479M ch. 37): four
 * three-input look-up tables, each with an optional synchronizer or
 * filter and an optional edge detector, and two sequential sub-modules
 * that turn a LUT PAIR into a flip-flop or a latch. Glue logic with no
 * CPU in it, and - unlike almost everything else in this stratum -
 * with NO INTERRUPT AND NO DMA (37.5.4 and 37.5.5 are both "Not
 * applicable"), so the only ways out are a pad and an event.
 *
 * The shape is the AVR's (`avrdx/ccl.hpp`), because for once the two
 * families really do have the same peripheral:
 *
 *   Ccl        the block: one ENABLE, one software reset, one generic
 *              clock for every filter/edge/sequencer in it, the two
 *              sequencer selectors, and the EVSYS codes it publishes;
 *   Lut<n>     one look-up table: a config struct owning its three
 *              inputs, its truth table, its filter, its edge detector
 *              and its two event enables;
 *   CclIn<Pin> / CclOut<Pin>
 *              the pads, from the reserve's own map.
 *
 * WHAT DIFFERS FROM THE AVR, and it is not cosmetic:
 *  - there is ONE generic clock for the whole block, not a per-LUT
 *    clock selector: slowing a filter down slows every filter down;
 *  - a LUT has ONE event input line (LUTEI + INSEL = EVENT), where the
 *    AVR has two (EVENTA/EVENTB), and the CCL's own edge detector turns
 *    that event into a one-GCLK strobe;
 *  - the input menu is different and per-LUT rather than per-INPUT: on
 *    the AVR input k of a LUT sees peripheral instance k, here the
 *    whole LUT n sees instance n (AC gives CMP[n], TC gives TC[n], TCC
 *    gives TCC[n % 3], SERCOM gives SERCOM[n]) with the ONE exception
 *    that TCC hands WO[0], WO[1] and WO[2] to inputs 0, 1 and 2;
 *  - there is no interrupt at all.
 *
 * THE ENABLE-PROTECTION STORY, WHICH IS WHY EVERY CONFIGURING VERB HERE
 * REFUSES WHILE THE BLOCK IS ENABLED. Three documents give three
 * answers:
 *   37.6.2.1  SEQCTRLx.SEQSEL is protected by the EVEN LUT's ENABLE,
 *             and LUTCTRLx (bar its own ENABLE bit) by that LUT's;
 *   37.8.2    "SEQCTRL register is Enable Protected when
 *             CCL.CTRL.ENABLE = 1";
 *   errata 1.7.3, EVERY REVISION OF E/G/J: "the SEQCTRLx and LUCTRLx
 *             registers are enable-protected by the CTRL.ENABLE bit,
 *             whereas they must be enable-protected by the
 *             LUTCTRLx.ENABLE bits."
 * The erratum is the strictest and the one the silicon obeys
 * (measured - docs/samc/ccl.md), so the protocol this driver enforces
 * is the AVR's errata-2.4.1 protocol by another road:
 *
 *     Ccl::enable(false);
 *     Ccl::sequencer(pair, LutSequencer::d_flip_flop);   // before the even LUT
 *     Lut<0>::configure(cfg0, true);                     // whole register, ENABLE with it
 *     Lut<1>::configure(cfg1, true);
 *     Ccl::enable(true);
 *
 * Reconfiguring one LUT therefore drops every other LUT's output for
 * the duration. That is the silicon's design, not this driver's.
 *
 * ERRATA, silicon rev F (E/G/J row - never the N row):
 *  - 1.7.1 RS Latch Reset (the latch clears only by disabling the LUT):
 *    REVISION B ONLY. Not coded around, and the bench checks that the
 *    reset really works here.
 *  - 1.7.2 Sequential Logic, EVERY REVISION: after an even LUT has been
 *    disabled to clear its flip-flop and enabled again, the sequential
 *    logic STAYS UNDER RESET until CTRL.ENABLE is written again. The
 *    workaround is code here: Lut<n>::enable(true) on an EVEN LUT
 *    re-states CTRL.ENABLE, and `Ccl::restate_enable()` is the verb by
 *    name for a caller that drove the bit itself.
 *  - 1.7.3 Enable Protected Registers, EVERY REVISION: see above.
 *  - 1.7.4 PAC Protection Error, EVERY REVISION: writing CTRL.SWRST is
 *    said to trigger a PAC protection error. There is no workaround and
 *    no alternative to a software reset, so `reset()` writes it anyway
 *    and says so; `Ccl::pac_id` is the number a future PAC pass needs.
 *  - 1.8.3 TC Selection (the default TC input is TC4 and not TC0):
 *    REVISION B ONLY, and the bench confirms the documented mapping on
 *    this one rather than trusting the row.
 *
 * NOT ON THIS FAMILY (37.8.3): INSEL's ALT2TC (0xA) and ASYNCEVENT
 * (0xB) exist only on the C20/C21 N variants. The enum carries them so
 * the vocabulary is the chapter's whole option space, and
 * `ccl_lut_config_valid()` refuses them wherever the device header does
 * not declare them - the reserve's `ccl_has_alt2_tc()` /
 * `ccl_has_async_event()`.
 */

#pragma once

#include <stdint.h>

#include "sam.h"

#include "samc/clock.hpp"
#include "samc/device_tables.hpp"
#include "samc/evsys.hpp"
#include "samc/pin.hpp"

namespace brio {

// =============================================================================
// The vocabulary (37.8.3)
// =============================================================================

/// LUTCTRLn.INSELy: what one input of a LUT is connected to.
///
/// Every peripheral entry selects the instance from the LUT's OWN
/// NUMBER, not from the input index - which is the opposite of the AVR
/// and the reason `LutInput` is a plain menu here with no per-input
/// meaning. `Lut<n>::ac_source()` and its siblings below name what a
/// given LUT will actually get.
enum class LutInput : uint8_t {
    masked = CCL_LUTCTRL_INSEL0_MASK_Val,        ///< 0x0: the input is tied low
    feedback = CCL_LUTCTRL_INSEL0_FEEDBACK_Val,  ///< 0x1: this pair's sequencer output
    link = CCL_LUTCTRL_INSEL0_LINK_Val,          ///< 0x2: the NEXT LUT's output
    event = CCL_LUTCTRL_INSEL0_EVENT_Val,        ///< 0x3: this LUT's EVSYS input line
    io = CCL_LUTCTRL_INSEL0_IO_Val,              ///< 0x4: this input's own pad
    ac = CCL_LUTCTRL_INSEL0_AC_Val,              ///< 0x5: comparator n
    tc = CCL_LUTCTRL_INSEL0_TC_Val,              ///< 0x6: TC n, WO[0]
    alt_tc = CCL_LUTCTRL_INSEL0_ALTTC_Val,       ///< 0x7: TC n+1, WO[0]

    /// 0x8: TCC (n % 3), and the ONE entry that is per-input - input 0
    /// takes WO[0], input 1 WO[1], input 2 WO[2].
    ///
    /// THE DEVICE HEADER HAS NO ENUMERATOR FOR THIS CODE, on any
    /// variant of the pack, although 37.8.3's table lists it for every
    /// one of them and only marks ALT2TC and ASYNCEVENT as N-only. The
    /// house rule is that the header wins - but a MISSING name is not a
    /// disagreement about a name, and INSEL is four bits wide with 0x8
    /// inside `CCL_LUTCTRL_Msk`, so the code is spelled from the
    /// chapter and settled at the bench (docs/samc/ccl.md).
    tcc = 0x8,

    sercom = CCL_LUTCTRL_INSEL0_SERCOM_Val,      ///< 0x9: SERCOM n, TX on its PAD[0]

    /// 0xA: TC (n + 4). C20/C21 N VARIANTS ONLY - refused here.
    alt2_tc = 0xA,
    /// 0xB: the event input with the CCL's own edge detector DISABLED,
    /// so a level can be combined with any other source. C20/C21 N
    /// VARIANTS ONLY - refused here.
    async_event = 0xB,
};

/// LUTCTRLn.FILTSEL (37.6.2.5). Both options delay OUT by "two to five
/// GCLK cycles"; what each costs is measured in docs/samc/ccl.md.
enum class LutFilter : uint8_t {
    none = CCL_LUTCTRL_FILTSEL_DISABLE_Val,
    sync = CCL_LUTCTRL_FILTSEL_SYNCH_Val,     ///< the synchronizer alone
    filter = CCL_LUTCTRL_FILTSEL_FILTER_Val,  ///< the glitch filter
    // 0x3 is Reserved and has no name here on purpose.
};

/// SEQCTRLx.SEQSEL (37.8.2): what the sequential sub-module of a LUT
/// PAIR is. The EVEN LUT drives D / J / D / S and the ODD one drives
/// G / K / G / R; the module's output replaces the EVEN LUT's.
enum class LutSequencer : uint8_t {
    none = CCL_SEQCTRL_SEQSEL_DISABLE_Val,
    d_flip_flop = CCL_SEQCTRL_SEQSEL_DFF_Val,  ///< D = even, G = odd, clocked by GCLK_CCL
    jk_flip_flop = CCL_SEQCTRL_SEQSEL_JK_Val,  ///< J = even, K = odd (1/1 toggles)
    latch = CCL_SEQCTRL_SEQSEL_LATCH_Val,      ///< D = even, G = odd, transparent
    rs_latch = CCL_SEQCTRL_SEQSEL_RS_Val,      ///< S = even, R = odd (1/1 forbidden)
};

constexpr bool lut_sequencer_valid(LutSequencer s) {
    return static_cast<uint8_t>(s) <= static_cast<uint8_t>(LutSequencer::rs_latch);
}

/**
 * The truth table from a predicate of the three inputs:
 *
 *     lut_truth([](bool a, bool b, bool c) { return a && !b; })
 *
 * TRUTH[k] is the output for the input pattern k with IN[0] as the LSB
 * and IN[2] as the MSB (table 37-1) - the same convention the AVR uses,
 * so a table written for one family reads correctly on the other.
 */
template <typename F>
constexpr uint8_t lut_truth(F f) {
    uint8_t t = 0;
    for (uint8_t k = 0; k < 8; ++k) {
        if (f((k & 1u) != 0u, (k & 2u) != 0u, (k & 4u) != 0u)) {
            t = static_cast<uint8_t>(t | (1u << k));
        }
    }
    return t;
}

/// The truth table that passes input `i` through unchanged: 0xAA, 0xCC,
/// 0xF0. The commonest table there is - a LUT used as a wire, a probe
/// or the D input of a sequencer - so it gets a name.
constexpr uint8_t lut_truth_pass(uint8_t i) {
    return i == 0u ? 0xAAu : (i == 1u ? 0xCCu : 0xF0u);
}

/// ... and its complement, the inverter (37.6.2.6's way of making the
/// edge detector fire on a falling edge).
constexpr uint8_t lut_truth_invert(uint8_t i) {
    return static_cast<uint8_t>(~lut_truth_pass(i));
}

/**
 * One LUT's whole LUTCTRLn, bar the ENABLE bit - which is not here
 * because it is decided by WHICH verb runs (`configure(cfg, enable)` /
 * `enable(bool)`), never by a field a caller could set inconsistently
 * with what it asked for.
 */
struct LutConfig {
    LutInput in0 = LutInput::masked;
    LutInput in1 = LutInput::masked;
    LutInput in2 = LutInput::masked;

    /// TRUTH[7:0]: see lut_truth() and lut_truth_pass().
    uint8_t truth = 0;

    LutFilter filter = LutFilter::none;
    /// EDGESEL: a one-GCLK pulse on a rising edge of the LUT's output.
    /// 37.6.2.6 requires the filter or the synchronizer with it.
    bool edge_detect = false;

    /// LUTEI: this LUT's EVSYS input line reaches the input multiplexer.
    /// Without it an `event` input source has nothing behind it.
    bool event_in = false;
    /// INVEI: that incoming event is inverted first. Means nothing
    /// without LUTEI, and is refused on its own.
    bool invert_event_in = false;
    /// LUTEO: this LUT's output value is an EVSYS generator.
    bool event_out = false;
};

/// Which INSEL codes this device implements (37.8.3's last two rows are
/// N-variant only, and the reserve reads that off the header).
constexpr bool lut_input_available(LutInput in) {
    switch (in) {
        case LutInput::alt2_tc: return ccl_has_alt2_tc();
        case LutInput::async_event: return ccl_has_async_event();
        default: return static_cast<uint8_t>(in) <= static_cast<uint8_t>(LutInput::sercom);
    }
}

/// Whether an input source needs LUTCTRLn.LUTEI raised behind it.
constexpr bool lut_input_is_event(LutInput in) {
    return in == LutInput::event || in == LutInput::async_event;
}

/**
 * A LUT configuration's legality, and the four refusals it carries:
 *
 *  - an INSEL code this device does not implement (the N-only rows);
 *  - the edge detector with no filter and no synchronizer - 37.6.2.6
 *    calls the result "unpredictable behavior", which is a refusal;
 *  - an event INPUT SOURCE with LUTEI clear: the multiplexer would be
 *    pointed at a line nothing feeds;
 *  - INVEI with LUTEI clear: inverting an event nobody listens to (the
 *    same refusal `ac_event_control_valid()` makes in ac.hpp).
 *
 * `lut` is a parameter because a future member of this family could
 * bond the codes differently per LUT; nothing on the C21 does, and this
 * function says so by not using it beyond the count check.
 */
constexpr bool ccl_lut_config_valid(uint8_t lut, const LutConfig& c) {
    if (lut >= ccl_lut_count()) {
        return false;
    }
    if (!lut_input_available(c.in0) || !lut_input_available(c.in1) ||
        !lut_input_available(c.in2)) {
        return false;
    }
    if (c.edge_detect && c.filter == LutFilter::none) {
        return false;
    }
    if (!c.event_in &&
        (lut_input_is_event(c.in0) || lut_input_is_event(c.in1) ||
         lut_input_is_event(c.in2))) {
        return false;
    }
    return !(c.invert_event_in && !c.event_in);
}

constexpr uint32_t ccl_lutctrl_word(const LutConfig& c, bool enable) {
    return CCL_LUTCTRL_TRUTH(c.truth) |
           (c.event_out ? CCL_LUTCTRL_LUTEO_Msk : 0u) |
           (c.event_in ? CCL_LUTCTRL_LUTEI_Msk : 0u) |
           (c.invert_event_in ? CCL_LUTCTRL_INVEI_Msk : 0u) |
           CCL_LUTCTRL_INSEL2(static_cast<uint32_t>(c.in2)) |
           CCL_LUTCTRL_INSEL1(static_cast<uint32_t>(c.in1)) |
           CCL_LUTCTRL_INSEL0(static_cast<uint32_t>(c.in0)) |
           (c.edge_detect ? CCL_LUTCTRL_EDGESEL_Msk : 0u) |
           CCL_LUTCTRL_FILTSEL(static_cast<uint32_t>(c.filter)) |
           (enable ? CCL_LUTCTRL_ENABLE_Msk : 0u);
}

// =============================================================================
// The block
// =============================================================================

class Ccl {
public:
    Ccl() = delete;

    /// From the device header's own instance parameters, through the
    /// reserve: four LUTs, two sequencers, twelve input lines.
    static constexpr uint8_t lut_count = ccl_lut_count();
    static constexpr uint8_t sequencer_count = ccl_seq_count();
    static constexpr uint8_t input_count = ccl_io_count();

    /// ONE generic clock for the whole peripheral. 37.5.3: it is
    /// "optionally required" - needed for input events, a filter, an
    /// edge detector or a sequencer, and for nothing else, so a purely
    /// combinational LUT runs with this channel disconnected.
    static constexpr uint8_t gclk_id = ccl_gclk_id();

    /// PAC.WRCTRL.PERID (bridge = id / 32, STATUS bit = id % 32).
    /// Erratum 1.7.4 is about this number.
    static constexpr uint16_t pac_id = ccl_pac_id();

    static ccl_registers_t& regs() { return *CCL_REGS; }

    // ---- the EVSYS vocabulary this peripheral publishes ---------------------
    //
    // evsys.hpp owns the FABRIC and not the vocabulary, so ch. 29's CCL
    // rows live here - probed from the device header, not copied.

    /// Generator: LUT n's output value (LUTOUT0 is 0x52).
    static constexpr uint8_t output_generator(uint8_t lut) {
        return ccl_lutout_generator(lut);
    }
    /// User: LUT n's one event input line (LUTIN0 is user 40). TABLE
    /// 29-3 MARKS ALL FOUR ASYNCHRONOUS PATH ONLY - a fact about the
    /// fabric that the channel's configuration has to honour, which is
    /// why `Lut<n>::listen()` refuses anything else.
    static constexpr uint8_t input_user(uint8_t lut) {
        return ccl_lutin_user(lut);
    }

    // ---- claim and teardown ------------------------------------------------

    static void bus_clock(bool on) { Mclk::apb_c(MCLK_APBCMASK_CCL_Msk, on); }

    /// Point GCLK_CCL at a generator. Optional (see `gclk_id`), and one
    /// rate for every filter, edge detector and sequencer in the block.
    static bool clock(uint8_t generator, uint32_t spins = 0xFFFFu) {
        return GclkChannel::connect(gclk_id, generator, spins);
    }
    static void unclock() { GclkChannel::disconnect(gclk_id); }

    /**
     * CTRL.SWRST: every register back to reset, the block disabled.
     *
     * ERRATUM 1.7.4, EVERY REVISION OF E/G/J: this write "will trigger
     * a PAC protection error". There is no workaround offered and no
     * other way to reset the peripheral, so the write stands; what a
     * caller can do about it is know that a PAC error may be pending
     * afterwards. SWRST is not synchronized on this peripheral (no
     * SYNCBUSY register exists), and it self-clears.
     */
    static void reset() { regs().CCL_CTRL = CCL_CTRL_SWRST_Msk; }

    static bool resetting() {
        return (regs().CCL_CTRL & CCL_CTRL_SWRST_Msk) != 0u;
    }

    /// CTRL.ENABLE. Every configuring verb below refuses while this is
    /// set - erratum 1.7.3 makes the whole register file protected by
    /// it, whatever 37.6.2.1 says.
    static void enable(bool on) {
        const uint8_t v = static_cast<uint8_t>(regs().CCL_CTRL & CCL_CTRL_RUNSTDBY_Msk);
        regs().CCL_CTRL = static_cast<uint8_t>(on ? (v | CCL_CTRL_ENABLE_Msk) : v);
    }
    static bool enabled() { return (regs().CCL_CTRL & CCL_CTRL_ENABLE_Msk) != 0u; }

    /**
     * The erratum-1.7.2 workaround, by name: "write CTRL.ENABLE again
     * after LUT is enabled back". After an even LUT has been disabled
     * to clear its sequential module and enabled again, the module
     * stays under reset until this happens. `Lut<n>::enable(true)`
     * calls it for an even LUT; a caller that drove LUTCTRL.ENABLE by
     * hand calls it itself.
     */
    static void restate_enable() {
        if (enabled()) {
            regs().CCL_CTRL =
                static_cast<uint8_t>(regs().CCL_CTRL | CCL_CTRL_ENABLE_Msk);
        }
    }

    /**
     * CTRL.RUNSTDBY: keep GCLK_CCL alive in standby (37.6.4). Without
     * it a LUT using the filter, the edge detector or a sequencer has
     * its output FORCED TO ZERO in standby, where a purely
     * combinational one keeps working.
     *
     * Enable-protected, and 37.8.1 adds "this bit must be written
     * before enabling the CCL" - so this refuses while enabled rather
     * than storing where the silicon will not look.
     */
    static bool run_standby(bool on) {
        if (enabled()) {
            return false;
        }
        regs().CCL_CTRL = static_cast<uint8_t>(on ? CCL_CTRL_RUNSTDBY_Msk : 0u);
        return true;
    }
    static bool run_standby() {
        return (regs().CCL_CTRL & CCL_CTRL_RUNSTDBY_Msk) != 0u;
    }

    /// Bus clock, generic clock, software reset - the block left
    /// DISABLED, which is the state every LUT and sequencer has to be
    /// configured from (erratum 1.7.3).
    ///
    /// `generator` may be `no_clock` for a purely combinational
    /// configuration: 37.5.3 asks for GCLK_CCL only when an input
    /// event, a filter, an edge detector or a sequencer is in use.
    static constexpr uint8_t no_clock = 0xFFu;

    static bool init(uint8_t generator = no_clock, uint32_t spins = 0xFFFFu) {
        bus_clock(true);
        if (generator != no_clock && !clock(generator, spins)) {
            return false;
        }
        reset();
        return true;
    }

    static void release() {
        enable(false);
        reset();
        unclock();
        bus_clock(false);
    }

    // ---- the sequencers ----------------------------------------------------

    /**
     * SEQCTRLx.SEQSEL for LUT pair `pair` (LUTs 2p and 2p+1).
     *
     * TWO refusals, from the two documents that disagree about which
     * ENABLE protects this register: the block must be disabled (37.8.2
     * and erratum 1.7.3) and the EVEN LUT must be disabled (37.6.2.1
     * and 37.6.2.7's "while configuring the sequential logic, the even
     * LUT must be disabled"). Obeying both is free and obeying only one
     * is a write that may land nowhere - the AVR's own SEQCTRL lesson,
     * where a selector written after the even LUT's enable was silently
     * ignored.
     */
    static bool sequencer(uint8_t pair, LutSequencer s) {
        if (pair >= sequencer_count || !lut_sequencer_valid(s) || enabled()) {
            return false;
        }
        if ((regs().CCL_LUTCTRL[2u * pair] & CCL_LUTCTRL_ENABLE_Msk) != 0u) {
            return false;
        }
        regs().CCL_SEQCTRL[pair] =
            static_cast<uint8_t>(CCL_SEQCTRL_SEQSEL(static_cast<uint32_t>(s)));
        return true;
    }

    /// The same with the pair and the selector known at compile time: a
    /// Reserved SEQSEL value and a pair this device does not have are
    /// build failures rather than false returns.
    template <uint8_t pair, LutSequencer s>
    static bool sequencer() {
        static_assert(pair < sequencer_count,
                      "this device does not implement that LUT pair's sequencer");
        static_assert(lut_sequencer_valid(s),
                      "SEQCTRL.SEQSEL 0x5..0xF are Reserved (37.8.2)");
        return sequencer(pair, s);
    }

    static LutSequencer sequencer(uint8_t pair) {
        if (pair >= sequencer_count) {
            return LutSequencer::none;
        }
        return static_cast<LutSequencer>(regs().CCL_SEQCTRL[pair] &
                                         CCL_SEQCTRL_SEQSEL_Msk);
    }
};

// =============================================================================
// One look-up table
// =============================================================================

template <uint8_t n>
class Lut {
    static_assert(n < ccl_lut_count(), "this device does not implement that LUT");

public:
    Lut() = delete;

    static constexpr uint8_t index = n;
    /// LUTs are grouped in adjacent pairs and a pair shares a sequencer.
    static constexpr uint8_t pair = static_cast<uint8_t>(n / 2u);
    static constexpr bool is_even = (n % 2u) == 0u;

    /// This LUT's own EVSYS codes.
    static constexpr uint8_t event_generator = Ccl::output_generator(n);
    static constexpr uint8_t event_user = Ccl::input_user(n);

    /// Whether this package bonds any pad at all to this LUT. On the E
    /// and the G, LUT3 has NEITHER an input pad nor an output pad: the
    /// LUT exists and is perfectly usable through events, a link or a
    /// sequencer, but it has no pin of its own. A runtime `if` on a
    /// missing pad would kill the instance, so this is the compile-time
    /// question `if constexpr` asks.
    static constexpr bool has_input_pad = ccl_lut_has_input_pad(n);
    static constexpr bool has_output_pad = ccl_lut_has_output_pad(n);

    // ---- what this LUT's peripheral input codes actually select ------------
    //
    // 37.6.2.4's four formulas, evaluated for THIS LUT so a caller does
    // not have to re-derive them - and so a static_assert can name what
    // it is wiring. `0xFF` means the formula names an instance this
    // device does not have, which 37.6.2.4 says is tied to ground.

    /// INSEL = AC selects comparator (n % comparator_count).
    static constexpr uint8_t ac_source = static_cast<uint8_t>(n % 4u);
    /// INSEL = TC selects TC (n % tc_count()), WO[0].
    static constexpr uint8_t tc_source =
        tc_count() == 0u ? 0xFFu : static_cast<uint8_t>(n % tc_count());
    /// INSEL = ALTTC selects TC ((n + 1) % tc_count()), WO[0].
    static constexpr uint8_t alt_tc_source =
        tc_count() == 0u ? 0xFFu : static_cast<uint8_t>((n + 1u) % tc_count());
    /// INSEL = TCC selects TCC (n % tcc_count()); inputs 0/1/2 take
    /// WO[0]/WO[1]/WO[2]. TCC2 has only two outputs, and 37.6.2.4's own
    /// note says its WO[0] feeds inputs 0 AND 2.
    static constexpr uint8_t tcc_source =
        tcc_count() == 0u ? 0xFFu : static_cast<uint8_t>(n % tcc_count());
    /// INSEL = SERCOM selects SERCOM n's transmit output, which must be
    /// on that SERCOM's PAD[0] - "the SERCOM TX signal must be output
    /// on SERCOMn/pad[0], which serves as input pad to the CCL".
    static constexpr uint8_t sercom_source = n;
    /// INSEL = LINK selects the NEXT LUT's output, the last wrapping to
    /// LUT0 (37.6.2.4's figure 37-5 draws LUT2 as LUT1's input).
    static constexpr uint8_t link_source =
        static_cast<uint8_t>((n + 1u) % ccl_lut_count());

    static volatile uint32_t& ctrl() { return Ccl::regs().CCL_LUTCTRL[n]; }

    static constexpr bool config_valid(const LutConfig& c) {
        return ccl_lut_config_valid(n, c);
    }

    /**
     * Write the WHOLE LUTCTRLn in one store, optionally enabling the
     * LUT in the same write - which is what 37.6.2.1 explicitly allows
     * ("enable-protected bits ... can be written at the same time as
     * LUTCTRLx.ENABLE is written to '1', but not at the same time as it
     * is written to '0'") and what makes a one-store configuration
     * legal at all.
     *
     * REFUSED WHILE THE BLOCK IS ENABLED (erratum 1.7.3, every
     * revision), and a write that lands nowhere is worse than a false
     * return. The consequence is real and belongs to the silicon:
     * reconfiguring one LUT means dropping every other LUT's output for
     * as long as the block is down.
     *
     * AND THE TWO GATES ARE AN *AND*, not the swap erratum 1.7.3's
     * sentence describes - measured, four cells of a truth table
     * (test_samc_ccl letter a): a LUTCTRL write lands only with BOTH
     * CTRL.ENABLE and LUTCTRLn.ENABLE clear. So this verb CLEARS
     * LUTCTRLn.ENABLE first, in a store of its own carrying nothing
     * else, because 37.6.2.1 forbids writing the protected bits
     * together with ENABLE = 0. Without that first store a
     * reconfiguration of a running LUT is dropped IN SILENCE, which is
     * the bug this suite caught in the first version of this file.
     */
    static bool configure(const LutConfig& c, bool enable_now = true) {
        if (!config_valid(c) || Ccl::enabled()) {
            return false;
        }
        ctrl() = ctrl() & ~CCL_LUTCTRL_ENABLE_Msk;
        ctrl() = ccl_lutctrl_word(c, enable_now);
        return true;
    }

    /// The same with the configuration known at compile time, so an
    /// illegal one is a BUILD failure and not a false return - the
    /// `init<cfg>()` shape every driver in this stratum offers, and the
    /// form `test/family_samc/neg/` refuses.
    template <LutConfig c>
    static bool configure(bool enable_now = true) {
        static_assert(ccl_lut_config_valid(n, c),
                      "this LUT configuration cannot work on this device: see "
                      "ccl_lut_config_valid() for the four refusals");
        return configure(c, enable_now);
    }

    static LutConfig config() {
        const uint32_t v = ctrl();
        return LutConfig{
            .in0 = static_cast<LutInput>((v & CCL_LUTCTRL_INSEL0_Msk) >>
                                         CCL_LUTCTRL_INSEL0_Pos),
            .in1 = static_cast<LutInput>((v & CCL_LUTCTRL_INSEL1_Msk) >>
                                         CCL_LUTCTRL_INSEL1_Pos),
            .in2 = static_cast<LutInput>((v & CCL_LUTCTRL_INSEL2_Msk) >>
                                         CCL_LUTCTRL_INSEL2_Pos),
            .truth = static_cast<uint8_t>((v & CCL_LUTCTRL_TRUTH_Msk) >>
                                          CCL_LUTCTRL_TRUTH_Pos),
            .filter = static_cast<LutFilter>((v & CCL_LUTCTRL_FILTSEL_Msk) >>
                                             CCL_LUTCTRL_FILTSEL_Pos),
            .edge_detect = (v & CCL_LUTCTRL_EDGESEL_Msk) != 0u,
            .event_in = (v & CCL_LUTCTRL_LUTEI_Msk) != 0u,
            .invert_event_in = (v & CCL_LUTCTRL_INVEI_Msk) != 0u,
            .event_out = (v & CCL_LUTCTRL_LUTEO_Msk) != 0u,
        };
    }

    /**
     * LUTCTRLn.ENABLE alone - the one bit of this register that is not
     * enable-protected, so it works with the block running.
     *
     * ON AN EVEN LUT THIS ALSO RE-STATES CTRL.ENABLE (erratum 1.7.2,
     * every revision): disabling an even LUT is the documented way to
     * clear its flip-flop or latch, and without that second write the
     * sequential logic stays under reset when the LUT comes back. It
     * costs one byte store on a LUT that has no sequencer, and it can
     * never be wrong.
     */
    static void enable(bool on) {
        const uint32_t v = ctrl();
        ctrl() = on ? (v | CCL_LUTCTRL_ENABLE_Msk) : (v & ~CCL_LUTCTRL_ENABLE_Msk);
        if constexpr (is_even) {
            if (on) {
                Ccl::restate_enable();
            }
        }
    }
    static bool enabled() { return (ctrl() & CCL_LUTCTRL_ENABLE_Msk) != 0u; }

    /// The truth table alone, for a caller changing the function of a
    /// configured LUT without restating the rest. Enable-protected like
    /// the rest of the register, so it refuses while the BLOCK is
    /// enabled and drops LUTCTRLn.ENABLE for the store - the same
    /// two-gate discipline configure() follows, and for the same
    /// measured reason. The LUT comes back enabled if it was.
    static bool truth(uint8_t t) {
        if (Ccl::enabled()) {
            return false;
        }
        const uint32_t was = ctrl();
        ctrl() = was & ~CCL_LUTCTRL_ENABLE_Msk;
        ctrl() = (was & ~CCL_LUTCTRL_TRUTH_Msk) | CCL_LUTCTRL_TRUTH(t);
        return true;
    }
    static uint8_t truth() {
        return static_cast<uint8_t>((ctrl() & CCL_LUTCTRL_TRUTH_Msk) >>
                                    CCL_LUTCTRL_TRUTH_Pos);
    }

    // ---- events ------------------------------------------------------------

    /**
     * Route an EVSYS channel into this LUT's event input.
     *
     * TABLE 29-3 GRANTS THIS USER THE ASYNCHRONOUS PATH ONLY, so a
     * channel configuration asking for anything else is refused rather
     * than written - the same shape `dac.hpp` and `sdadc.hpp` give
     * their asynchronous-only START users. The edge detection that
     * turns the level into a strobe is the CCL's own (37.6.2.4's
     * "internal strobe ... one GCLK_CCL clock cycle"), which is why the
     * channel must not add one.
     *
     * LUTCTRLn.LUTEI still has to be set - it is part of the
     * configuration (`LutConfig::event_in`), because it is
     * enable-protected and this verb is not.
     */
    static bool listen(uint8_t channel, const EventChannelConfig& cfg) {
        if (cfg.path != EventPath::asynchronous) {
            return false;
        }
        return Evsys::connect(event_user, channel, cfg);
    }
    static void unlisten() { Evsys::disconnect(event_user); }
};

// =============================================================================
// The pads
// =============================================================================

/**
 * CclIn<Pin>: one LUT input line reached through a pad that carries it.
 *
 * A pad this device does not bond fails to compile, which is the
 * per-package gate with no hand-kept table behind it. Note that ONE
 * INPUT CAN HAVE TWO PADS - PA04 and PA16 are both CCL0/IN[0] - so
 * claiming both would be two drivers on one input line.
 *
 * The pad goes to peripheral function I with its input buffer on: the
 * mux does not raise INEN, and a CCL input that cannot see its pad
 * reads zero the way every disabled input buffer on this chip does.
 */
template <class P>
struct CclIn {
    CclIn() = delete;
    static_assert(ccl_in_exists<P::port_letter, P::pin_number>,
                  "this device does not bond that pad to a CCL input");

    using pin = P;
    /// IN[0..11] across the whole block.
    static constexpr uint8_t line =
        static_cast<uint8_t>(ccl_in_line(P::port_letter, P::pin_number));
    /// Which LUT, and which of its three inputs.
    static constexpr uint8_t lut = static_cast<uint8_t>(line / 3u);
    static constexpr uint8_t input = static_cast<uint8_t>(line % 3u);

    static constexpr PinFunction function = PinFunction::i;

    static void claim(PinPull pull = PinPull::none) {
        P::function(function, {.input_enable = true, .pull = pull});
    }
    static void release() { P::configure({}); }
};

/**
 * CclOut<Pin>: one LUT's output on a pad. `lut` IS the output number
 * (OUT[3:0] against LUT[3:0]).
 *
 * The input buffer is raised here too, deliberately: it costs nothing
 * and it makes the pad READABLE through PORT.IN, which is how every
 * verdict in `test_samc_ccl` observes a LUT at all.
 */
template <class P>
struct CclOut {
    CclOut() = delete;
    static_assert(ccl_out_exists<P::port_letter, P::pin_number>,
                  "this device does not bond that pad to a CCL output");

    using pin = P;
    static constexpr uint8_t lut =
        static_cast<uint8_t>(ccl_out_lut(P::port_letter, P::pin_number));

    static constexpr PinFunction function = PinFunction::i;

    static void claim() { P::function(function, {.input_enable = true}); }
    static void release() { P::configure({}); }
    /// What the pad is actually at.
    static bool read() { return P::read(); }
};

} // namespace brio

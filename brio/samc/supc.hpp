/*
 * supc.hpp
 *
 * The SAM C21 Supply Controller (DS60001479M ch. 22): the two brown-out
 * detectors, the core voltage regulator, and the bandgap reference every
 * analog block on this die eventually asks for.
 *
 *   Supc        the BLOCK - the status register, the interrupt surface
 *               behind the shared IRQ 0, and the ISR body.
 *   BodVdd      the VDD brown-out detector: level, action, hysteresis,
 *               continuous or sampled, and the synchronization its
 *               ENABLE bit needs. ITS RESET VALUE IS A FUSE.
 *   BodCore     the VDDCORE detector, READ-ONLY here on purpose.
 *   Vreg        the core regulator, which cannot be turned off.
 *   Vref        INTREF, the bandgap: three levels, the output enable
 *               that lets the ADC and the AC see it, and its sleep
 *               behaviour.
 *
 * WHAT MAKES THIS CHAPTER DIFFERENT FROM ITS NEIGHBOURS: three of these
 * registers come up with values NOBODY IN THE PROGRAM WROTE. The BODVDD
 * level, its enable and its action are loaded from the NVM user row at
 * every power-on or user reset (22.6.3.2), exactly as the watchdog's are
 * - so `samc/nvm.hpp`'s `NvmUserRow` and this header describe the same
 * four fields from opposite ends, and `BodVdd::matches_fuses()` exists
 * to say so out loud.
 *
 * FOUR RULES, each with code behind it.
 *
 * 1. BODVDD IS ENABLE-PROTECTED AND WRITE-SYNCHRONIZED AT ONCE, which
 *    is a combination no other register in this stratum has. Writes to
 *    the protected fields are DISCARDED while ENABLE is 1 and raise an
 *    APB error (22.6.3.1); and a write of any kind while
 *    STATUS.BVDDSRDY is 0 raises a PAC error (22.6.5). So every path
 *    here is: wait for BVDDSRDY, clear ENABLE, wait again, write the
 *    configuration, wait, set ENABLE ON ITS OWN, wait. `configure()`
 *    is that sequence and nothing else. THE LAST STEP IS NOT
 *    COSMETIC: a store carrying the configuration AND ENABLE = 1
 *    together sets the bit and leaves the protected fields where they
 *    were - the protection is judged on the value being WRITTEN, not
 *    on the one already in the register. Measured, at the cost of a
 *    restore that did not restore.
 *
 * 2. THE LEVEL IS A FIELD AND NOT A VOLTAGE. Table 45-18 gives three
 *    points - level 8 is 2.8 V, level 9 is 2.85 V, level 44 is 4.51 V -
 *    and a "step size" of 60 mV that those points do not support (they
 *    imply about 47.5 mV a step). Rather than invent a conversion this
 *    header cannot keep, the level is passed through as its field and
 *    the three anchors are stated here. The bench suite measures the
 *    step instead of trusting either number.
 *
 * 3. BODCORE IS NOT YOURS. 22.6.3.4: it is calibrated in production,
 *    its calibration lives in the user row, and "this configuration
 *    must not be changed to assure the correct behavior". The register
 *    is exposed READ-ONLY, and the header offers no verb that writes
 *    it. NOTE that the register EXISTS at all only according to the
 *    device header: chapter 22's register summary marks offset 0x14
 *    Reserved, and the same disagreement runs through STATUS and
 *    INTFLAG, where the header declares BODCORERDY / BODCOREDET /
 *    BCORESRDY at bits 3..5 that the chapter does not draw. The header
 *    wins in code, as always here, and the bench suite reads those bits
 *    to see whether anything is behind them.
 *
 * 4. THE MAIN REGULATOR CANNOT BE DISABLED. 22.8.6 says VREG.ENABLE
 *    "must never be changed from its reset value of one", so there is
 *    no enable verb - only RUNSTDBY, which is the one bit an
 *    application has business writing (and which erratum 1.8.14 makes
 *    the workaround for a standby bug in PM).
 *
 * ERRATA. 1.8.14 is LIVE on this silicon and belongs to the power pass,
 * not here: entering standby with PM.STDBYCFG.VREGSMOD in performance
 * mode wrongly switches to the low-power regulator and keeps requesting
 * GCLK0, and the workaround is SUPC.VREG.RUNSTDBY = 1 - which is why
 * that bit has a verb and a comment pointing at it. 1.8.11 (VREGSMOD
 * having no effect at all) is revision B only. Nothing else in the
 * errata document touches this chapter.
 *
 * NOT BUILT (docs/samc/supc.md carries the list): nothing here forces a
 * brown-out - the supply is not this program's to dip - so RESET and
 * INT actions are configured and read back but never fired; and standby
 * behaviour of all three blocks waits for the power pass.
 */

#pragma once

#include <stdint.h>

#include "sam.h"

#include "samc/clock.hpp"
#include "samc/nvm.hpp"

namespace brio {

/// INTFLAG / INTENSET / INTENCLR, and STATUS carries the same six bits
/// with the same meanings. The BODCORE half is the device header's;
/// chapter 22 draws only the BODVDD three.
struct SupcFlag {
    static constexpr uint32_t bodvdd_ready = SUPC_INTFLAG_BODVDDRDY_Msk;
    static constexpr uint32_t bodvdd_detect = SUPC_INTFLAG_BODVDDDET_Msk;
    static constexpr uint32_t bodvdd_sync_ready = SUPC_INTFLAG_BVDDSRDY_Msk;
    static constexpr uint32_t bodcore_ready = SUPC_INTFLAG_BODCORERDY_Msk;
    static constexpr uint32_t bodcore_detect = SUPC_INTFLAG_BODCOREDET_Msk;
    static constexpr uint32_t bodcore_sync_ready = SUPC_INTFLAG_BCORESRDY_Msk;
    static constexpr uint32_t all = SUPC_INTFLAG_Msk;
};

/// BODVDD.ACTION (22.8.5): what a detection DOES. `none` still sets
/// STATUS.BODVDDDET, which is what makes a threshold sweep safe.
enum class BodAction : uint8_t {
    none = SUPC_BODVDD_ACTION_NONE_Val,
    reset = SUPC_BODVDD_ACTION_RESET_Val,
    interrupt = SUPC_BODVDD_ACTION_INT_Val,
};

/// BODVDD.PSEL: the sampling clock is OSCULP32K's 1.024 kHz output
/// divided by 2^(PSEL+1) (22.6.3.6). The enumerator names the divisor,
/// as the device header does.
enum class BodPrescaler : uint8_t {
    div2 = SUPC_BODVDD_PSEL_DIV2_Val,
    div4 = SUPC_BODVDD_PSEL_DIV4_Val,
    div8 = SUPC_BODVDD_PSEL_DIV8_Val,
    div16 = SUPC_BODVDD_PSEL_DIV16_Val,
    div32 = SUPC_BODVDD_PSEL_DIV32_Val,
    div64 = SUPC_BODVDD_PSEL_DIV64_Val,
    div128 = SUPC_BODVDD_PSEL_DIV128_Val,
    div256 = SUPC_BODVDD_PSEL_DIV256_Val,
    div512 = SUPC_BODVDD_PSEL_DIV512_Val,
    div1024 = SUPC_BODVDD_PSEL_DIV1024_Val,
    div2048 = SUPC_BODVDD_PSEL_DIV2048_Val,
    div4096 = SUPC_BODVDD_PSEL_DIV4096_Val,
    div8192 = SUPC_BODVDD_PSEL_DIV8192_Val,
    div16384 = SUPC_BODVDD_PSEL_DIV16384_Val,
    div32768 = SUPC_BODVDD_PSEL_DIV32768_Val,
    div65536 = SUPC_BODVDD_PSEL_DIV65536_Val,
};

/// The sampling clock's rate for a prescaler code, in millihertz -
/// whole hertz would round 0.5 Hz to nothing at the slow end. The
/// source is the 1.024 kHz output of OSCULP32K (22.6.3.6), whose own
/// accuracy is per-cent class; this is the nominal.
constexpr uint32_t bod_sample_mhz(BodPrescaler p) {
    return 1'024'000UL >> (static_cast<uint32_t>(p) + 1u);
}

// =============================================================================
// The block
// =============================================================================

struct Supc {
    Supc() = delete;

    /// IRQ 0 is SHARED - MCLK, OSCCTRL, OSC32KCTRL, PAC and SUPC arrive
    /// on it together - so a handler asks each block in turn.
    static constexpr IRQn_Type irq() { return SUPC_IRQn; }

    /// The APB clock. It is ON out of reset (17.6.2.6) - a supply
    /// controller nobody could reach would be a poor idea - and the
    /// verb exists for symmetry and for a power pass that wants it off.
    static void bus_clock(bool on) { Mclk::apb_a(MCLK_APBAMASK_SUPC_Msk, on); }

    static uint32_t status() { return SUPC_REGS->SUPC_STATUS; }

    static uint32_t flags() { return SUPC_REGS->SUPC_INTFLAG; }
    static uint32_t armed() { return SUPC_REGS->SUPC_INTENSET; }
    static void clear_flags(uint32_t mask = SupcFlag::all) {
        SUPC_REGS->SUPC_INTFLAG = mask;
    }
    static void arm(uint32_t mask) { SUPC_REGS->SUPC_INTENSET = mask; }
    static void disarm(uint32_t mask) { SUPC_REGS->SUPC_INTENCLR = mask; }

    /// The ISR body; the app binds the handler. Answers only for this
    /// block - IRQ 0 is shared, see irq().
    [[gnu::always_inline]] static uint32_t isr() {
        const uint32_t p = flags() & armed();
        if (p != 0u) {
            clear_flags(p);
        }
        return p;
    }
};

// =============================================================================
// BODVDD - the detector whose reset value is a fuse
// =============================================================================

struct BodVddConfig {
    /// BODVDD.LEVEL, six bits. A FIELD, not a voltage - see rule 2 in
    /// this file's header for why no conversion is offered. The three
    /// anchors table 45-18 gives are `level_2v8`, `level_2v85` and
    /// `level_4v51` below.
    uint8_t level = 8;

    /// What a detection does. `none` is not a way of disabling the
    /// detector - STATUS.BODVDDDET still tracks - it is a way of
    /// watching without consequences.
    BodAction action = BodAction::reset;

    /// HYST: separate the falling and rising thresholds so supply
    /// ripple does not chatter the reset (22.6.3.7). Table 45-18 puts
    /// the separation at 40..75 mV.
    bool hysteresis = false;

    /// ACTCFG: sampled instead of continuous while the CPU is awake.
    /// Sampling costs microamps instead of tens of them (table 45-19)
    /// and buys latency; STATUS.BODVDDRDY is NEVER set in sampling mode
    /// (22.8.4), which is a trap worth knowing before waiting on it.
    bool sampled = false;

    /// RUNSTDBY and STDBYCFG: whether the detector runs in standby at
    /// all, and whether it samples there. Untested by this stratum -
    /// standby belongs to the power pass.
    bool run_standby = false;
    bool sampled_in_standby = false;

    BodPrescaler prescaler = BodPrescaler::div2;
};

/**
 * The VDD brown-out detector.
 *
 * TWO THINGS TO KNOW BEFORE USING IT. Its reset value comes from the
 * NVM user row, so a program that never touches it is still running
 * under whatever the fuses say - `matches_fuses()` is how to check that
 * the two agree. And every write is fenced by BVDDSRDY: `configure()`
 * spends the whole enable-protection dance so a caller does not have to
 * remember it, and refuses rather than writing into a busy register.
 */
struct BodVdd {
    BodVdd() = delete;

    static constexpr uint8_t level_max = 0x3F;

    /// The three points table 45-18 actually gives, as field values.
    /// Everything between them is an interpolation this header does not
    /// make.
    static constexpr uint8_t level_2v8 = 8;    ///< 2.71 / 2.80 / 2.89 V
    static constexpr uint8_t level_2v85 = 9;   ///< 2.75 / 2.85 / 2.95 V
    static constexpr uint8_t level_4v51 = 44;  ///< 4.37 / 4.51 / 4.66 V

    static constexpr bool config_valid(const BodVddConfig& c) {
        return c.level <= level_max;
    }

    static uint32_t reg() { return SUPC_REGS->SUPC_BODVDD; }
    static bool enabled() { return (reg() & SUPC_BODVDD_ENABLE_Msk) != 0u; }
    static uint8_t level() {
        return static_cast<uint8_t>((reg() & SUPC_BODVDD_LEVEL_Msk) >>
                                    SUPC_BODVDD_LEVEL_Pos);
    }
    static BodAction action() {
        return static_cast<BodAction>((reg() & SUPC_BODVDD_ACTION_Msk) >>
                                      SUPC_BODVDD_ACTION_Pos);
    }
    static bool hysteresis() { return (reg() & SUPC_BODVDD_HYST_Msk) != 0u; }

    /// STATUS.BODVDDRDY. NEVER set in sampling mode (22.8.4), so a wait
    /// on it is only meaningful for a continuous detector.
    static bool ready() { return (Supc::status() & SupcFlag::bodvdd_ready) != 0u; }

    /// STATUS.BODVDDDET: the supply is BELOW the threshold right now.
    /// Set whatever ACTION says, which is what makes a threshold sweep
    /// with ACTION = none both safe and informative.
    static bool detected() {
        return (Supc::status() & SupcFlag::bodvdd_detect) != 0u;
    }

    /// STATUS.BVDDSRDY: the ENABLE write has crossed into the sampling
    /// clock's domain. Zero means a write would be a PAC error (22.6.5).
    static bool sync_ready() {
        return (Supc::status() & SupcFlag::bodvdd_sync_ready) != 0u;
    }
    static bool wait_sync(uint32_t spins = 0xFFFFu) {
        while (!sync_ready() && spins-- != 0u) {
        }
        return sync_ready();
    }

    static constexpr uint32_t word(const BodVddConfig& c, bool enable) {
        return SUPC_BODVDD_LEVEL(c.level) |
               SUPC_BODVDD_ACTION(static_cast<uint32_t>(c.action)) |
               SUPC_BODVDD_PSEL(static_cast<uint32_t>(c.prescaler)) |
               (c.hysteresis ? SUPC_BODVDD_HYST_Msk : 0u) |
               (c.sampled ? SUPC_BODVDD_ACTCFG_Msk : 0u) |
               (c.run_standby ? SUPC_BODVDD_RUNSTDBY_Msk : 0u) |
               (c.sampled_in_standby ? SUPC_BODVDD_STDBYCFG_Msk : 0u) |
               (enable ? SUPC_BODVDD_ENABLE_Msk : 0u);
    }

    /**
     * The whole enable-protection dance, in one verb.
     *
     * Disable, wait, write the configuration with ENABLE clear, wait,
     * set ENABLE, wait. Every one of those waits is BVDDSRDY, and each
     * one is what stops the next write from being a PAC error. False -
     * and the detector left disabled - for an impossible level or a
     * synchronization that never completed.
     *
     * `enable = false` configures without starting, which is what a
     * caller wanting to change the level of a detector it will start
     * later should ask for.
     */
    static bool configure(const BodVddConfig& cfg, bool enable = true,
                          uint32_t spins = 0xFFFFu) {
        if (!config_valid(cfg)) {
            return false;
        }
        if (!wait_sync(spins)) {
            return false;
        }
        // ENABLE is NOT enable-protected, so this store lands even
        // though it also rewrites the protected fields (which are
        // discarded, harmlessly - they are being rewritten with what
        // they already hold).
        SUPC_REGS->SUPC_BODVDD = reg() & ~static_cast<uint32_t>(SUPC_BODVDD_ENABLE_Msk);
        if (!wait_sync(spins)) {
            return false;
        }
        SUPC_REGS->SUPC_BODVDD = word(cfg, false);
        if (!wait_sync(spins)) {
            return false;
        }
        if (!enable) {
            return true;
        }
        // ENABLE ON ITS OWN, and this is measured rather than deduced: a
        // single store carrying both the configuration and ENABLE = 1
        // leaves the ENABLE bit set and the protected fields UNCHANGED
        // (test_samc_supc letter c caught it restoring a saved
        // register). The protection is evaluated against the value
        // being written, not against the one already there.
        return BodVdd::enable(true, spins);
    }

    /// Start or stop an already configured detector. The same
    /// synchronization fence, without touching anything protected.
    static bool enable(bool on, uint32_t spins = 0xFFFFu) {
        if (!wait_sync(spins)) {
            return false;
        }
        SUPC_REGS->SUPC_BODVDD =
            on ? (reg() | SUPC_BODVDD_ENABLE_Msk)
               : (reg() & ~static_cast<uint32_t>(SUPC_BODVDD_ENABLE_Msk));
        return wait_sync(spins);
    }

    /**
     * Does the detector in force match the fuses that were supposed to
     * set it?
     *
     * This is the same cross-check `samc/reset.hpp` makes for the
     * watchdog, from the other end: `samc/nvm.hpp` reads the user row,
     * this reads the register, and the three fields the row claims to
     * load (22.6.3.2) either agree or something has written the
     * register since reset.
     */
    static bool matches_fuses() {
        const NvmUserRow row = NvmUserRow::read();
        return level() == row.bodvdd_level() &&
               enabled() != row.bodvdd_disabled() &&
               static_cast<uint8_t>(action()) == row.bodvdd_action() &&
               hysteresis() == row.bodvdd_hysteresis();
    }
};

// =============================================================================
// BODCORE - read-only on purpose
// =============================================================================

/**
 * The VDDCORE brown-out detector.
 *
 * DELIBERATELY WITHOUT A SINGLE WRITING VERB. 22.6.3.4 says its
 * calibration is a production value living in the user row and "must
 * not be changed to assure the correct behavior of the BODCORE", and
 * the user row's own table 9-4 marks those bits DO NOT CHANGE. A driver
 * that offered a setter would be offering a way to brick a board.
 *
 * The register is not in chapter 22's summary at all - offset 0x14 is
 * drawn Reserved - and exists here on the device header's authority,
 * along with the three STATUS/INTFLAG bits at 3..5 that the chapter
 * also omits. Reading is how one finds out whether they are real.
 */
struct BodCore {
    BodCore() = delete;

    static uint32_t reg() { return SUPC_REGS->SUPC_BODCORE; }
    static bool enabled() { return (reg() & SUPC_BODCORE_ENABLE_Msk) != 0u; }
    static BodAction action() {
        return static_cast<BodAction>((reg() & SUPC_BODCORE_ACTION_Msk) >>
                                      SUPC_BODCORE_ACTION_Pos);
    }
    static bool hysteresis() { return (reg() & SUPC_BODCORE_HYST_Msk) != 0u; }

    /// The three status bits the device header declares and chapter 22
    /// does not draw.
    static bool ready() { return (Supc::status() & SupcFlag::bodcore_ready) != 0u; }
    static bool detected() {
        return (Supc::status() & SupcFlag::bodcore_detect) != 0u;
    }
    static bool sync_ready() {
        return (Supc::status() & SupcFlag::bodcore_sync_ready) != 0u;
    }
};

// =============================================================================
// VREG - the regulator that cannot be turned off
// =============================================================================

/**
 * The core voltage regulator.
 *
 * There is NO enable verb: 22.8.6 says the bit "must never be changed
 * from its reset value of one", and a header that offered the change
 * would be offering to stop the core's supply. What is left is
 * RUNSTDBY, and that one bit matters more than its size suggests -
 * erratum 1.8.14, live on this silicon, makes it the workaround for a
 * standby entry that would otherwise switch to the low-power regulator
 * and keep requesting GCLK0.
 */
struct Vreg {
    Vreg() = delete;

    static uint32_t reg() { return SUPC_REGS->SUPC_VREG; }

    /// True on every healthy device; false would mean somebody wrote
    /// the bit 22.8.6 forbids.
    static bool enabled() { return (reg() & SUPC_VREG_ENABLE_Msk) != 0u; }

    /// RUNSTDBY: keep the MAIN regulator supplying VDDCORE in standby
    /// instead of handing over to the low-power one (22.6.1.3).
    static void run_standby(bool on) {
        SUPC_REGS->SUPC_VREG =
            on ? (reg() | SUPC_VREG_RUNSTDBY_Msk)
               : (reg() & ~static_cast<uint32_t>(SUPC_VREG_RUNSTDBY_Msk));
    }
    static bool run_standby() { return (reg() & SUPC_VREG_RUNSTDBY_Msk) != 0u; }
};

// =============================================================================
// VREF - the bandgap the analog blocks ask for
// =============================================================================

/// VREF.SEL (22.8.7). Only three of the sixteen codes are implemented;
/// the others are Reserved and `Vref::config_valid()` refuses them.
enum class VrefLevel : uint8_t {
    v1_024 = 0,
    v2_048 = 2,
    v4_096 = 3,
};

/// The nominal output of a level, in millivolts. Nominal is the
/// chapter's own word ("typical value"); what a die really produces is
/// a measurement, and the AC's own scaler is one way to make it.
constexpr uint16_t vref_mv(VrefLevel v) {
    switch (v) {
    case VrefLevel::v1_024: return 1024;
    case VrefLevel::v2_048: return 2048;
    case VrefLevel::v4_096: return 4096;
    }
    return 0;
}

struct VrefConfig {
    VrefLevel level = VrefLevel::v1_024;

    /// VREFOE. 22.8.7 words it as routing the reference "to an ADC
    /// input channel", which undersells it: the ANALOG COMPARATOR's
    /// MUXNEG bandgap selection needs it too, and without this bit that
    /// input is a floating promise. Its absence is what
    /// docs/samc/ac.md's gap list has been waiting for.
    bool output_enable = false;

    /// ONDEMAND: the reference runs only while a peripheral asks
    /// (table 22-1). Clear is what a measurement wants.
    bool on_demand = false;
    bool run_standby = false;
};

struct Vref {
    Vref() = delete;

    static constexpr bool config_valid(const VrefConfig& c) {
        return c.level == VrefLevel::v1_024 || c.level == VrefLevel::v2_048 ||
               c.level == VrefLevel::v4_096;
    }

    static uint32_t reg() { return SUPC_REGS->SUPC_VREF; }

    static VrefLevel level() {
        return static_cast<VrefLevel>((reg() & SUPC_VREF_SEL_Msk) >>
                                      SUPC_VREF_SEL_Pos);
    }
    static bool output_enabled() { return (reg() & SUPC_VREF_VREFOE_Msk) != 0u; }

    static constexpr uint32_t word(const VrefConfig& c) {
        return SUPC_VREF_SEL(static_cast<uint32_t>(c.level)) |
               (c.output_enable ? SUPC_VREF_VREFOE_Msk : 0u) |
               (c.on_demand ? SUPC_VREF_ONDEMAND_Msk : 0u) |
               (c.run_standby ? SUPC_VREF_RUNSTDBY_Msk : 0u);
    }

    /// One store, no synchronization - VREF has none. False and nothing
    /// written for a Reserved level code.
    static bool configure(const VrefConfig& cfg) {
        if (!config_valid(cfg)) {
            return false;
        }
        SUPC_REGS->SUPC_VREF = word(cfg);
        return true;
    }

    /// The compile-time twin, so a Reserved level is a compile error.
    template <VrefConfig cfg>
    static bool configure() {
        static_assert(config_valid(cfg),
                      "brio Vref: only the 1.024 V, 2.048 V and 4.096 V codes "
                      "are implemented (22.8.7); the rest of SEL is Reserved");
        return configure(cfg);
    }

    /// Turn the bandgap output on or off without disturbing the level.
    static void output_enable(bool on) {
        SUPC_REGS->SUPC_VREF =
            on ? (reg() | SUPC_VREF_VREFOE_Msk)
               : (reg() & ~static_cast<uint32_t>(SUPC_VREF_VREFOE_Msk));
    }
};

} // namespace brio

/*
 * pac.hpp
 *
 * The SAM C21 Peripheral Access Controller (DS60001479M ch. 11): the
 * block that can WRITE-PROTECT any peripheral on any of the APB bridges,
 * and that reports every access violation - a protected write, a write
 * to an unimplemented register, a write while a synchronization is
 * running, an access to an address no client answers.
 *
 * THE POSITION THIS HEADER TAKES: MECHANISM ONLY, NO CONCEPT.
 *
 * There is no RAII guard here, no policy, no util/ contract, and no
 * "protect everything at boot" ceremony. The reasons are three and they
 * are worth stating, because a PAC driver invites all of them:
 *
 *  1. THE FRAMEWORK HAS NO USER YET. Nothing in brio turns protection
 *     on, and a guard type designed before its first caller would be
 *     designed against an imagined one. The shape a guard should take -
 *     scoped, reference-counted, per-peripheral, or a boot-time policy
 *     table - is a question the first real safety-minded application
 *     gets to answer.
 *  2. THE DOUBLE-WRITE RULE MAKES NESTING A DESIGN DECISION, NOT A
 *     DETAIL. 11.5.2.6: setting protection on an already-protected
 *     peripheral is an ERROR, and so is clearing it twice. A naive
 *     scoped guard nested twice therefore raises a PAC error on the way
 *     in and again on the way out. Getting that right needs either a
 *     count or a read of STATUS, and which of those is right depends on
 *     whether interrupts share the peripheral - the chapter says so
 *     itself.
 *  3. PROTECTION IS NOT PROTECTION FROM EVERYTHING. Erratum 1.13.3
 *     (live on this silicon) lets IOBUS writes straight past a
 *     protected PORT; erratum 1.23.1 lets MCLK.CTRLA be written with no
 *     error at all. A util/ concept that promised "protected" would be
 *     promising something this silicon does not deliver uniformly, and
 *     the house rule is never to state what is not enforced.
 *
 * So: a monostate that can set, clear and lock a peripheral's write
 * protection by the identifier the peripheral itself publishes, read
 * back the per-bridge status, and read and clear the four interrupt-flag
 * banks. Each driver publishes its own `pac_id` (samc/tsens.hpp and
 * samc/ccl.hpp did so before this file existed, for exactly the errata
 * below); this header owns the fabric and not the census.
 *
 * WHAT THE SILICON DOES.
 *
 * WRCTRL IS A KEYED, WORD-WISE STORE. One 32-bit write carries the
 * peripheral identifier in the low sixteen bits and the operation in
 * KEY: 1 = clear, 2 = set, 3 = set-and-lock. 11.5.2.6 is explicit that
 * ONLY a word-wise write takes effect and that any other access is
 * itself an error, flagged in INTFLAGA.PAC. There is no read-back of a
 * request: STATUSn is where the outcome is read.
 *
 * PERID = 32 x bridge + index. Bridge A is 0, B is 1, C is 2; the index
 * is the peripheral's position in that bridge's list (ch. 12's "PAC,
 * Index" column, which is also the bit position in INTFLAGn and STATUSn).
 * The device header states the whole table as `ID_<PERIPHERAL>` macros,
 * so no driver has to compute it.
 *
 * THE LOCK IS ONE-WAY UNTIL A RESET. Key 3 sets protection and locks
 * the setting; 11.5.2.5 says it "will only be cleared by a hardware
 * reset", and 11.5.2.2 that "only a hardware reset will reset the PAC
 * module". Which resets count as hardware is not spelled out anywhere in
 * the chapter, and it matters: see docs/samc/pac.md, where the bench
 * answers it.
 *
 * THE FLAGS ARE FOUR BANKS. INTFLAGAHB reports client-bus errors (an
 * access to an address no bridge or peripheral answers), INTFLAGA/B/C
 * one bit per peripheral of that bridge. All are write-one-to-clear and
 * all are OUTSIDE PAC write protection themselves (11.4.8), together
 * with WRCTRL - so a fully protected system can still clear its own
 * error flags.
 *
 * ONE INTERRUPT, AND IT IS THE SHARED LINE. PAC_IRQn is line 0, shared
 * with PM, MCLK, OSCCTRL, OSC32KCTRL and SUPC on this family - the
 * established discipline in this stratum (samc/clock.hpp, samc/supc.hpp,
 * samc/osc32kctrl.hpp): the app binds one handler and calls each block's
 * isr() body, each of which acknowledges only what it armed.
 *
 * THE ERRATA. There is NO errata section for the PAC itself in
 * DS80000740S - not one item in 1.1..1.25 names this chapter. What
 * exists instead is a set of items in OTHER chapters, all live at
 * revision F, each of which says the PAC does not behave as ch. 11
 * promises for one particular peripheral:
 *
 *  - 1.7.4 (CCL): writing CCL.CTRL.SWRST triggers a PAC protection
 *    error - i.e. a flag with no protection set at all.
 *  - 1.13.2 (PORT): reads and writes of unimplemented PORT registers do
 *    NOT raise the protection error 11.5.2.4 promises for an illegal
 *    access.
 *  - 1.13.3 (PORT): IOBUS writes are not prevented when PORT is
 *    protected - the protection has a back door.
 *  - 1.19.1 (TSENS): a write to a protected TSENS.CTRLB is dropped, and
 *    the bench found it dropped IN SILENCE, with no flag raised.
 *  - 1.23.1 (MCLK): writes to MCLK.CTRLA raise no error even when MCLK
 *    is protected.
 *  - 1.24.1 (FREQM): READING FREQM.CTRLB raises a protection error, with
 *    no protection involved and no workaround.
 *
 * Taken together those say the useful thing about this block: a raised
 * flag is good evidence, an ABSENT flag is not. docs/samc/pac.md carries
 * the measured map of which peripherals flag and which do not.
 *
 * NOT BUILT (docs/samc/pac.md carries the list): no guard type and no
 * policy, for the reasons above; the ERR interrupt is exposed as flags,
 * an enable and an ISR body but nothing here waits on it; the fourth
 * bridge (D) exists on the C21N alone and neither its header nor a board
 * is here.
 */

#pragma once

#include <stdint.h>

#include "sam.h"

#include "samc/clock.hpp"
#include "samc/device_tables.hpp"

namespace brio {

/// WRCTRL.KEY: what a write control request asks for (11.7.1).
enum class PacKey : uint8_t {
    /// No action. Written by nobody on purpose - it is the reset value
    /// and it is here so the enumeration is the register's whole one.
    off = PAC_WRCTRL_KEY_OFF_Val,
    clear = PAC_WRCTRL_KEY_CLR_Val,
    set = PAC_WRCTRL_KEY_SET_Val,
    /// Set AND lock: cleared only by a reset (11.5.2.5).
    lock = PAC_WRCTRL_KEY_SETLCK_Val,
};

/// INTFLAGAHB (11.7.5): one bit per AHB client that can report an
/// access to an address nothing answers. Named from the device header.
struct PacAhbFlag {
    static constexpr uint32_t flash = PAC_INTFLAGAHB_FLASH_Msk;
    static constexpr uint32_t sram_cm0p = PAC_INTFLAGAHB_HSRAMCM0P_Msk;
    static constexpr uint32_t sram_dsu = PAC_INTFLAGAHB_HSRAMDSU_Msk;
    static constexpr uint32_t bridge_a = PAC_INTFLAGAHB_HPB0_Msk;
    static constexpr uint32_t bridge_b = PAC_INTFLAGAHB_HPB1_Msk;
    static constexpr uint32_t bridge_c = PAC_INTFLAGAHB_HPB2_Msk;
    static constexpr uint32_t lpram_dmac = PAC_INTFLAGAHB_LPRAMDMAC_Msk;
    /// THE DIVIDER'S OWN BIT, and the one cross-chapter link this block
    /// has that is not an erratum: 14.5.8 says writing DIVAS.CTRLA,
    /// DIVIDEND, DIVISOR or SQRNUM while the engine is busy "will result
    /// in an error", and this is where such an error would be reported.
    static constexpr uint32_t divas = PAC_INTFLAGAHB_DIVAS_Msk;
    static constexpr uint32_t all = PAC_INTFLAGAHB_Msk;
};

/**
 * The Peripheral Access Controller as a monostate resource.
 *
 * Every verb takes a PERID - the number a peripheral publishes as its
 * own `pac_id`, or the device header's `ID_<PERIPHERAL>` - and derives
 * the bridge and the bit from it, because that is exactly the
 * arithmetic 11.7.1 defines.
 */
struct Pac {
    Pac() = delete;

    /// This block's own identifier: it can protect itself, which is why
    /// 11.4.8 has to except WRCTRL and the flag banks by name.
    static constexpr uint16_t pac_id = pac_pac_id();      // 0

    /// How many peripheral bridges this device's PAC covers. Three here
    /// (A, B, C); the C21N adds a fourth and neither its device header
    /// nor a board is present, so a request past this is refused rather
    /// than silently aimed at the wrong register.
    static constexpr uint8_t bridge_count = pac_bridge_count();

    /// EVSYS generator: an access error, from the header's own table
    /// (EVENT_ID_GEN_PAC_ACCERR). evsys.hpp owns the fabric; this is the
    /// vocabulary this peripheral publishes into it.
    static constexpr uint8_t ev_gen_error = pac_accerr_generator();

    /// INTENSET/INTENCLR: the block has exactly one interrupt source.
    static constexpr uint8_t int_error = PAC_INTENSET_ERR_Msk;

    /// The line is SHARED (table 10-4): PM, MCLK, OSCCTRL, OSC32KCTRL,
    /// SUPC and PAC all arrive on IRQ 0.
    static constexpr IRQn_Type irq() { return PAC_IRQn; }

    // ---- the identifier arithmetic (11.7.1) --------------------------------

    static constexpr uint8_t bridge_of(uint16_t perid) {
        return static_cast<uint8_t>(perid / 32u);
    }
    static constexpr uint32_t bit_of(uint16_t perid) {
        return 1UL << (perid % 32u);
    }
    static constexpr uint16_t perid_of(uint8_t bridge, uint8_t index) {
        return static_cast<uint16_t>(32u * bridge + index);
    }
    static constexpr bool id_valid(uint16_t perid) {
        return bridge_of(perid) < bridge_count;
    }

    // ---- clocks ------------------------------------------------------------

    /// The PAC's APB clock is ON out of reset (table 12-3) and there is
    /// no reason for an application to turn it off; the verb exists so
    /// the option space is complete and so a suite can prove the block
    /// is where it says it is.
    static void bus_clock(bool on) { Mclk::apb_a(MCLK_APBAMASK_PAC_Msk, on); }

    // ---- write protection --------------------------------------------------

    /**
     * The one request verb: a single WORD-WISE store of KEY and PERID.
     *
     * 11.5.2.6 is unusually specific - "Only word-wise writes to the
     * WRCTRL register will effectively change the access protection.
     * Other type of accesses will have no effect and will cause a PAC
     * write access error" - so this is a `uint32_t` store and never a
     * byte or halfword one, and callers never get a narrower path.
     *
     * False only for an identifier this device has no bridge for; the
     * silicon's own answer (did it work? was it a double-set?) is read
     * from `is_protected()` and `flagged()`, which is the block's actual
     * interface and not something a return value can stand in for.
     */
    static bool write_control(uint16_t perid, PacKey key) {
        if (!id_valid(perid)) {
            return false;
        }
        PAC_REGS->PAC_WRCTRL =
            PAC_WRCTRL_PERID(perid) |
            PAC_WRCTRL_KEY(static_cast<uint32_t>(key));
        return true;
    }

    /// Set write protection. NOTE 11.5.2.6: doing this to an ALREADY
    /// protected peripheral is an error and raises INTFLAGA.PAC - the
    /// chapter's deliberate "the program flow must be balanced" rule.
    static bool protect(uint16_t perid) { return write_control(perid, PacKey::set); }

    /// Clear write protection - and the same balance rule applies to a
    /// double clear.
    static bool unprotect(uint16_t perid) {
        return write_control(perid, PacKey::clear);
    }

    /**
     * Set write protection and LOCK it. 11.5.2.5: the protection "will
     * only be cleared by a hardware reset", and a later clear (or a
     * later lock) of a locked peripheral is itself an error.
     *
     * There is no unlock verb because there is no unlock.
     */
    static bool lock(uint16_t perid) { return write_control(perid, PacKey::lock); }

    /// STATUSA/B/C for one bridge (11.7.10..12); zero for a bridge this
    /// device does not have.
    static uint32_t status(uint8_t bridge) {
        switch (bridge) {
            case 0: return PAC_REGS->PAC_STATUSA;
            case 1: return PAC_REGS->PAC_STATUSB;
            case 2: return PAC_REGS->PAC_STATUSC;
            default: return 0;
        }
    }

    /// Whether one peripheral is write protected right now. NOTE that
    /// STATUS cannot distinguish "set" from "set and locked": the lock
    /// is invisible except by attempting a clear and watching the flag.
    static bool is_protected(uint16_t perid) {
        return id_valid(perid) && (status(bridge_of(perid)) & bit_of(perid)) != 0u;
    }

    // ---- the flag banks ----------------------------------------------------

    /// INTFLAGA/B/C for one bridge; zero for a bridge this device does
    /// not have.
    static uint32_t flags(uint8_t bridge) {
        switch (bridge) {
            case 0: return PAC_REGS->PAC_INTFLAGA;
            case 1: return PAC_REGS->PAC_INTFLAGB;
            case 2: return PAC_REGS->PAC_INTFLAGC;
            default: return 0;
        }
    }

    /// Write-one-to-clear, per bridge.
    static void clear_flags(uint8_t bridge, uint32_t mask) {
        switch (bridge) {
            case 0: PAC_REGS->PAC_INTFLAGA = mask; break;
            case 1: PAC_REGS->PAC_INTFLAGB = mask; break;
            case 2: PAC_REGS->PAC_INTFLAGC = mask; break;
            default: break;
        }
    }

    /// Whether one peripheral has an access error standing against it.
    static bool flagged(uint16_t perid) {
        return id_valid(perid) && (flags(bridge_of(perid)) & bit_of(perid)) != 0u;
    }
    static void clear_flag(uint16_t perid) {
        if (id_valid(perid)) {
            clear_flags(bridge_of(perid), bit_of(perid));
        }
    }

    static uint32_t ahb_flags() { return PAC_REGS->PAC_INTFLAGAHB; }
    static void clear_ahb_flags(uint32_t mask = PacAhbFlag::all) {
        PAC_REGS->PAC_INTFLAGAHB = mask;
    }

    /// Every bank at once - the boot-time and between-tests broom.
    static void clear_all_flags() {
        clear_ahb_flags();
        for (uint8_t b = 0; b < bridge_count; ++b) {
            clear_flags(b, 0xFFFFFFFFUL);
        }
    }

    /// True when anything at all is flagged, across every bank.
    static bool any_error() {
        if (ahb_flags() != 0u) {
            return true;
        }
        for (uint8_t b = 0; b < bridge_count; ++b) {
            if (flags(b) != 0u) {
                return true;
            }
        }
        return false;
    }

    // ---- interrupt and event -----------------------------------------------

    static uint8_t armed() { return PAC_REGS->PAC_INTENSET; }
    static void arm(bool on) {
        if (on) {
            PAC_REGS->PAC_INTENSET = int_error;
        } else {
            PAC_REGS->PAC_INTENCLR = int_error;
        }
    }

    /// EVCTRL.ERREO: publish an event whenever any flag bank sets a bit.
    static void event_output(bool on) {
        PAC_REGS->PAC_EVCTRL = on ? PAC_EVCTRL_ERREO_Msk : static_cast<uint8_t>(0);
    }
    static bool event_output() {
        return (PAC_REGS->PAC_EVCTRL & PAC_EVCTRL_ERREO_Msk) != 0u;
    }

    /**
     * The ISR body; the app binds the shared handler and calls this
     * alongside the other IRQ-0 blocks.
     *
     * There is ONE enable bit for every bank, so an interrupt says only
     * "something violated something". This returns the four banks the
     * handler needs to tell them apart, AND CLEARS THEM - leaving them
     * standing would re-enter the handler forever, since the flags are
     * the interrupt's only level.
     */
    struct Report {
        uint32_t ahb;
        uint32_t bridge[3];
        bool any() const {
            return ahb != 0u || bridge[0] != 0u || bridge[1] != 0u ||
                   bridge[2] != 0u;
        }
    };

    [[gnu::always_inline]] static Report isr() {
        Report r{};
        r.ahb = ahb_flags();
        if (r.ahb != 0u) {
            clear_ahb_flags(r.ahb);
        }
        for (uint8_t b = 0; b < bridge_count; ++b) {
            r.bridge[b] = flags(b);
            if (r.bridge[b] != 0u) {
                clear_flags(b, r.bridge[b]);
            }
        }
        return r;
    }
};

} // namespace brio

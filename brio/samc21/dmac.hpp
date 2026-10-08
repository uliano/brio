/*
 * dmac.hpp
 *
 * The SAM C21 Direct Memory Access Controller (DS60001479M ch. 25), driven
 * for ONE use and refused for every other: a single channel draining a
 * SERCOM's transmit ring - `DmaTxEngine`, the engine samc21/sercom.hpp's
 * Uart takes in its one optional slot.
 *
 *  Dmac         the block's one public face: its NVIC line. The rest of
 *               the block (the AHB clock, the reset, the two descriptor
 *               sections, the arbitration) is DmaTxEngine's to configure.
 *
 *  DmaTxEngine  channel 0, byte beats, one trigger a beat, one block a
 *               run, its completion the end of the run - claimed by ONE
 *               owner at a time.
 *
 * WHY ONE. Erratum 1.10.4 (DS80000740S, "Concurrent channels triggers",
 * E/G/J at revisions E, F and H - this silicon): "When using concurrent
 * channels triggers, the DMAC write-back descriptors may get corrupted."
 * The write-back section is not a report: for an ongoing block the
 * descriptor is FETCHED from it (25.6.2.6), so a corrupted write-back is a
 * corrupted transfer - measured here as a transmit channel running another
 * channel's descriptor into the SERCOM's own registers, an SPI pair that
 * hangs or completes with its bytes missing beside any third active
 * channel, and a write-only SPI channel hung by two others
 * (docs/samc21/dmac.md). The erratum's one workaround - "Multiple
 * transfers must only be sequenced using linked descriptors on a single
 * channel" - is concurrency forbidden. So this stratum drives one channel
 * and nothing that could start a second: no channel type, no other
 * engine, no trigger vocabulary, no register reachable from outside this
 * file but the NVIC line.
 *
 * THE ONE-USER RULE, AND HOW IT IS ENFORCED. Structurally first: there is
 * one channel (channel 0) and this file is the only code that touches the
 * controller, so two channels cannot run at once in any image. A program
 * may still name the engine in two Uarts - two owners of ONE channel,
 * which would reprogram each other's live block - so the engine is CLAIMED
 * at arm() by its owner (the Uart's SERCOM number) and a second owner's
 * arm() while the first holds the claim is refused with a PANIC, the
 * breadcrumb written (PanicCode::assert_failed, context
 * dma_claim_refused_context(owner)): deterministically, at the init that
 * would make the second user, never silently. A claim is given back by
 * release(), so two transports that take the engine IN TURN (a suite
 * moving its engine from one SERCOM to another) are legal: one user at a
 * time. A build-time refusal was the preferred form and is not built: the
 * image is one translation unit, but "two DIFFERENT instantiations name
 * this type" is nothing C++ can count without stateful metaprogramming,
 * and it would also refuse the legal in-turn shape.
 *
 * TWO MOMENTS. arm() configures the binding ONCE - the block reset and
 * enabled with its descriptor sections registered, the channel reset and
 * configured (the trigger, TRIGACT beat), TCMPL and TERR armed, the NVIC
 * line opened, and the slot's constant half written: the fixed
 * destination (the SERCOM's DATA), no next descriptor. A block start then
 * writes what changes - BTCTRL (one literal), BTCNT and SRCADDR, three
 * stores into SRAM - and enables the channel with one store.
 *
 * CHID IS WRITTEN ONCE. The channel registers sit behind a selector
 * (25.8.17: CHCTRLA, CHCTRLB, CHINTEN*, CHINTFLAG and CHSTATUS talk to
 * whichever channel CHID names), which is a select-then-use pair with an
 * interrupt between - unless the selector never moves. With one channel
 * it never does: the block's reset leaves CHID at 0, arm() writes 0, and
 * nothing else writes it, so every channel access below is ONE access and
 * needs no critical section.
 *
 * THE END ADDRESS. For an incrementing side SRCADDR holds the address one
 * beat PAST the last one - start plus the run's length in bytes
 * (25.6.2.7; the register descriptions 25.10.3/25.10.4 print a stray
 * "+ 1" that their own definition of BEATSIZE makes meaningless, and the
 * bench decided it by moving bytes, docs/samc21/dmac.md). The engine
 * computes it; a caller hands in a run.
 *
 * DISABLING IS NOT INSTANTANEOUS, and the edges behind it are silent: a
 * '0' written to CHCTRLA.ENABLE does not clear until the internal buffer
 * drains (25.8.18, 25.6.3.6), and until it does CHCTRLB is still
 * enable-protected and CHCTRLA.SWRST is IGNORED - the bit never reads back
 * set, so a wait for it to clear succeeds having reset nothing. Every
 * disable here waits, bounded, and the channel reset refuses when it did
 * not complete.
 *
 * BLOCKACT = INT and TCMPL armed: the completion is the run's end. (On this
 * die BLOCKACT = NOACT does not silence TCMPL, contrary to 25.8.22's
 * clause - measured, dmac.md - so the interrupt enable, not BLOCKACT, is
 * what decides whether a block interrupts.) A single block disables its
 * channel at its end (25.6.2.6), which is what makes a block start onto an
 * idle channel safe with no wait. A BUS ERROR disables the channel and
 * sets TERR with the count written back (25.6.2.8); the engine answers it
 * as a block lost (abandon()), and a channel that took one loses the
 * first beat of its next block (measured, dmac.md) - which no run from a
 * ring in SRAM provokes.
 *
 * THE STANDING REQUEST. A SERCOM's DRE is a level and the DMAC latches a
 * trigger on its RISE (25.8.8) - but a rise while the channel is disabled
 * with its trigger selected is latched and served on the next enable, and
 * so is the selection of a trigger onto a request already standing
 * (measured, dmac.md), so the enable of a block fires its first beat by
 * itself and nothing kicks: a software trigger after that beat started is
 * a SECOND beat into a full DATA (measured, docs/samc21/sercom.md).
 *
 * Errata 1.10.1 (CRCDATAIN, rev B), 1.10.2 and 1.10.3 (linked descriptors,
 * E/G/J revisions B..D - the N family's row carries the marks under E and
 * F: read the row, not the column) are not this silicon; the CRC engine
 * and linked descriptors are not built.
 *
 * ZERO WHEN ABSENT. The descriptor sections are static members, present
 * only in an image that reaches them (-fdata-sections, --gc-sections):
 * a Uart that names no engine never includes this header at all.
 */

#pragma once

#include <stdint.h>

#include <span>

#include "sam.h"

#include "kernel/panic.hpp"
#include "samc21/clock.hpp"
#include "samc21/nvic.hpp"
#include "samc21/platform.hpp"

namespace brio {

/// The panic context a refused claim writes: 0xD0 with the refused owner
/// in the low nibble (a Uart's owner is its SERCOM number plus one, so
/// 0xD1 .. 0xD6 name SERCOM0 .. SERCOM5).
constexpr uint8_t dma_claim_refused_context(uint8_t owner) {
    return static_cast<uint8_t>(0xD0u | (owner & 0x0Fu));
}

/// The block's one public face: the NVIC line an app binds DMAC_Handler
/// to. Everything else of the block is the engine's (the file header).
struct Dmac {
    Dmac() = delete;
    static constexpr IRQn_Type irq() { return DMAC_IRQn; }
};

/**
 * DmaTxEngine - the one DMA user this stratum admits: drain a run of bytes
 * into a peripheral's DATA register on channel 0, one beat per trigger.
 *
 * THE CLAIM AND THE PROGRAMMING ARE TWO STEPS (reserve(), launch()), so an
 * owner that starts blocks from two contexts - the Uart's producer in main
 * context and its completion in the handler - masks the claim alone, a
 * test-and-set of busy_, and programs a reserved channel unmasked: no
 * completion can land under the programming, because no block is in
 * flight. start() is both, for an owner that starts from one context.
 *
 * NO WRITE-BACK IS EVER READ. The engine knows the run it programmed and
 * learns of its end from TCMPL; there is no third fact to want.
 */
class DmaTxEngine {
public:
    DmaTxEngine() = delete;

    /// The tag samc21/sercom.hpp's Uart tests with `if constexpr`.
    static constexpr bool present = true;
    /// The one channel (the file header).
    static constexpr uint8_t channel = 0;
    using element = uint8_t;

    /**
     * Claim the engine for `owner` (non-zero; a Uart passes its SERCOM
     * number plus one). True when it was free or already `owner`'s; false
     * when another owner holds it - nothing is touched then. The decision
     * is a test-and-set under the mask and nothing else.
     */
    static bool claim(uint8_t owner) {
        typename SamPlatform::CriticalSection cs;
        if (owner_ != 0u && owner_ != owner) {
            return false;
        }
        owner_ = owner;
        return true;
    }

    /// Who holds the claim; 0 when nobody does.
    static uint8_t owner() { return owner_; }

    /**
     * Claim for `owner` and configure the binding: `data` the address of
     * the peripheral's transmit data register, `trigger` its TX trigger
     * code (Sercom<n>::dma_tx_trigger()).
     *
     * A CLAIM HELD BY ANOTHER OWNER IS A PANIC - the one-user rule
     * (the file header): the breadcrumb is written with PanicCode::
     * assert_failed and dma_claim_refused_context(owner), and the program
     * stops there. False, with the claim given back, when the block or the
     * channel would not reset (a bounded wait ran out).
     */
    static bool arm(uint8_t owner, volatile void* data, uint8_t trigger) {
        if (!claim(owner)) {
            panic<SamPlatform>(PanicCode::assert_failed, dma_claim_refused_context(owner));
        }
        data_ = data;
        trigger_ = trigger;
        armed_ = true;
        if (!block_up() || !take_channel()) {
            give_back();
            return false;
        }
        Nvic::enable(Dmac::irq());
        return true;
    }

    /**
     * Stop, and give the claim back if `owner` holds it: the channel
     * disabled (bounded), its interrupts disarmed, the NVIC line closed,
     * the block disabled and its clock off - for an owner that armed the
     * engine; a claim that never armed touches no register (the block's
     * clock may be off). A block in flight is abandoned without counting
     * - what it had not moved is the owner's.
     */
    static void release(uint8_t owner) {
        if (owner_ != owner) {
            return;
        }
        give_back();
    }

    /**
     * THE CLAIM ON THE CHANNEL: false when a block is already in flight,
     * otherwise the engine is the caller's until launch() or cancel().
     */
    [[gnu::always_inline]] static bool reserve() {
        if (busy_) {
            return false;
        }
        busy_ = true;
        return true;
    }

    /// Give a reservation back unused (the owner found nothing to send).
    static void cancel() { busy_ = false; }

    /**
     * Send one contiguous run on a RESERVED engine: three stores into the
     * slot and the enable. False - and the reservation given back - when
     * the run is empty or longer than BTCNT counts (65535 beats); the
     * caller keeps the bytes and offers them again.
     */
    static bool launch(std::span<const uint8_t> run) {
        const uint32_t count = static_cast<uint32_t>(run.size());
        if (count == 0u || count > 0xFFFFu) {
            busy_ = false;
            return false;
        }
        slot_.DMAC_BTCTRL = block_control;
        slot_.DMAC_BTCNT = static_cast<uint16_t>(count);
        slot_.DMAC_SRCADDR = reinterpret_cast<uint32_t>(run.data()) + count;
        in_flight_ = static_cast<uint16_t>(count);
        regs().DMAC_CHCTRLA = static_cast<uint8_t>(DMAC_CHCTRLA_ENABLE_Msk);
        return true;
    }

    /// reserve() and launch() in one call, for an owner that starts blocks
    /// from ONE context. False when a block is in flight or the run is
    /// refused.
    static bool start(std::span<const uint8_t> run) { return reserve() && launch(run); }

    /// Bytes of the block in flight (0 when idle).
    static uint16_t in_flight() { return busy_ ? in_flight_ : 0u; }
    static bool busy() { return busy_; }

    /// The channel is ENABLED with no trigger pending and no beat moving
    /// (CHCTRLA.ENABLE set, CHSTATUS.PEND and BUSY clear). Beside a
    /// trigger that STANDS - the owner's proof - a channel that cannot
    /// move: samc21/sercom.hpp's dead-block predicate asks it.
    static bool waiting() {
        return (regs().DMAC_CHCTRLA & DMAC_CHCTRLA_ENABLE_Msk) != 0u &&
               (regs().DMAC_CHSTATUS & (DMAC_CHSTATUS_PEND_Msk | DMAC_CHSTATUS_BUSY_Msk)) == 0u;
    }

    /// The block's interrupt flags, taken and cleared in one INTPEND read
    /// and store (25.8.10: a write of {flags, id} clears them for that id).
    /// Zero when nothing is pending. The DMAC_Handler body, through the
    /// owner's dma_isr().
    [[gnu::always_inline]] static uint8_t take_interrupt() {
        const uint16_t word = regs().DMAC_INTPEND;
        const uint8_t flags =
            static_cast<uint8_t>((word >> DMAC_INTPEND_TERR_Pos) & flag_all);
        if (flags != 0u) {
            regs().DMAC_INTPEND = static_cast<uint16_t>(
                DMAC_INTPEND_ID(channel) |
                (static_cast<uint16_t>(flags) << DMAC_INTPEND_TERR_Pos));
        }
        return flags;
    }
    static constexpr uint8_t flag_error = DMAC_CHINTFLAG_TERR_Msk;
    static constexpr uint8_t flag_complete = DMAC_CHINTFLAG_TCMPL_Msk;

    /**
     * The block ended (TCMPL): how many bytes it carried, so the owner
     * consumes exactly that much of its ring.
     */
    static uint16_t complete() {
        const uint16_t moved = in_flight_;
        in_flight_ = 0;
        busy_ = false;
        return moved;
    }

    /**
     * Throw away a block the silicon is not finishing, and hand the
     * channel back ready for the next one: reset, reconfigured, re-armed.
     * THE CALLER DECIDES the block is dead - only the peripheral's owner
     * can read the flags that make it a fact - or a bus error did (TERR,
     * the channel disabled by the silicon). What is lost is an unknown
     * tail of the block; every abandonment is counted in faults().
     * True when a block was abandoned.
     */
    static bool abandon() {
        if (!busy_) {
            return false;
        }
        ++faults_;
        (void)take_channel();
        in_flight_ = 0;
        busy_ = false;
        return true;
    }

    static uint32_t faults() { return faults_; }
    static void clear_faults() { faults_ = 0; }

private:
    /// BTCTRL of every block: VALID, the source walking, byte beats, the
    /// block's end an interrupt (and the channel disabled, 25.6.2.6).
    static constexpr uint16_t block_control = static_cast<uint16_t>(
        DMAC_BTCTRL_VALID_Msk | DMAC_BTCTRL_SRCINC_Msk |
        DMAC_BTCTRL_BEATSIZE(DMAC_BTCTRL_BEATSIZE_BYTE_Val) |
        DMAC_BTCTRL_BLOCKACT(DMAC_BTCTRL_BLOCKACT_INT_Val));
    static constexpr uint8_t flag_all = static_cast<uint8_t>(
        DMAC_CHINTFLAG_TERR_Msk | DMAC_CHINTFLAG_TCMPL_Msk | DMAC_CHINTFLAG_SUSP_Msk);
    /// The bound on every wait below.
    static constexpr uint32_t spins = 0xFFFFu;

    static dmac_registers_t& regs() { return *DMAC_REGS; }

    /**
     * The block from reset: its clock (CLK_DMAC_AHB is its only one,
     * 25.5.3 - no GCLK channel), CTRL disabled and waited out, SWRST
     * (refused by the silicon with either engine enabled, 25.8.1), the
     * two sections registered while it is stopped (BASEADDR and WRBADDR
     * are enable-protected: a write under DMAENABLE is discarded), CHID
     * at 0 for good, level 0 alone enabled, the block on.
     *
     * The level is the group macro DMAC_CTRL_LVLEN(): the per-level
     * DMAC_CTRL_LVLEN0() masks to one bit, and a channel whose level is
     * not enabled is INVISIBLE to the arbiter (25.8.1, measured).
     */
    static bool block_up() {
        Nvic::disable(Dmac::irq());
        Mclk::ahb(MCLK_AHBMASK_DMAC_Msk, true);
        if (!block_off()) {
            return false;
        }
        regs().DMAC_CTRL = DMAC_CTRL_SWRST_Msk;
        uint32_t left = spins;
        while ((regs().DMAC_CTRL & DMAC_CTRL_SWRST_Msk) != 0u) {
            if (left-- == 0u) {
                return false;
            }
        }
        clear(slot_);
        clear(write_back_);
        regs().DMAC_BASEADDR = reinterpret_cast<uint32_t>(&slot_);
        regs().DMAC_WRBADDR = reinterpret_cast<uint32_t>(&write_back_);
        regs().DMAC_CHID = static_cast<uint8_t>(DMAC_CHID_ID(channel));
        regs().DMAC_CTRL = static_cast<uint16_t>(DMAC_CTRL_LVLEN(0x1u) | DMAC_CTRL_DMAENABLE_Msk);
        return true;
    }

    /// CTRL.DMAENABLE cleared and waited out (25.8.1: it stays set until
    /// the internal buffer has drained), with the CRC engine off too.
    static bool block_off() {
        regs().DMAC_CTRL = static_cast<uint16_t>(
            regs().DMAC_CTRL & ~(DMAC_CTRL_DMAENABLE_Msk | DMAC_CTRL_CRCENABLE_Msk));
        uint32_t left = spins;
        while ((regs().DMAC_CTRL & DMAC_CTRL_DMAENABLE_Msk) != 0u) {
            if (left-- == 0u) {
                return false;
            }
        }
        return true;
    }

    /// CHCTRLA.ENABLE cleared and waited out (25.8.18).
    static bool channel_off() {
        regs().DMAC_CHCTRLA = static_cast<uint8_t>(
            regs().DMAC_CHCTRLA & static_cast<uint8_t>(~DMAC_CHCTRLA_ENABLE_Msk));
        uint32_t left = spins;
        while ((regs().DMAC_CHCTRLA & DMAC_CHCTRLA_ENABLE_Msk) != 0u) {
            if (left-- == 0u) {
                return false;
            }
        }
        return true;
    }

    /// The channel from whatever state it is in: disabled (bounded - an
    /// SWRST under ENABLE is ignored silently), reset, the slot and its
    /// write-back cleared, CHCTRLB written (the trigger, TRIGACT beat,
    /// level 0), the flags cleared, TCMPL and TERR armed, and the slot's
    /// constant half: the destination, no next descriptor. arm() and
    /// abandon() are the same act for a different reason.
    static bool take_channel() {
        if (!channel_off()) {
            return false;
        }
        regs().DMAC_CHCTRLA = DMAC_CHCTRLA_SWRST_Msk;
        uint32_t left = spins;
        while ((regs().DMAC_CHCTRLA & DMAC_CHCTRLA_SWRST_Msk) != 0u) {
            if (left-- == 0u) {
                return false;
            }
        }
        clear(slot_);
        clear(write_back_);
        regs().DMAC_CHCTRLB = DMAC_CHCTRLB_TRIGACT(DMAC_CHCTRLB_TRIGACT_BEAT_Val) |
                              DMAC_CHCTRLB_TRIGSRC(trigger_) | DMAC_CHCTRLB_LVL(0u);
        regs().DMAC_CHINTENCLR = flag_all;
        regs().DMAC_CHINTFLAG = flag_all;
        regs().DMAC_CHINTENSET = static_cast<uint8_t>(DMAC_CHINTENSET_TCMPL_Msk |
                                                      DMAC_CHINTENSET_TERR_Msk);
        slot_.DMAC_DSTADDR = reinterpret_cast<uint32_t>(data_);
        slot_.DMAC_DESCADDR = 0u;
        return true;
    }

    /// A section entry back to zeros, field by field (VALID first: a
    /// fetch between the stores must not find a valid half-old entry).
    static void clear(volatile dmac_descriptor_registers_t& d) {
        d.DMAC_BTCTRL = 0u;
        d.DMAC_BTCNT = 0u;
        d.DMAC_SRCADDR = 0u;
        d.DMAC_DSTADDR = 0u;
        d.DMAC_DESCADDR = 0u;
    }

    static void give_back() {
        if (armed_) {
            Nvic::disable(Dmac::irq());
            (void)channel_off();
            regs().DMAC_CHINTENCLR = flag_all;
            (void)block_off();
            Mclk::ahb(MCLK_AHBMASK_DMAC_Msk, false);
            armed_ = false;
        }
        in_flight_ = 0;
        busy_ = false;
        typename SamPlatform::CriticalSection cs;
        owner_ = 0;
    }

    /// Channel 0's first descriptor (BASEADDR) and its write-back
    /// (WRBADDR): the sections are arrays indexed by channel, and channel 0
    /// is the first entry of each, so one entry is the whole table. 64-bit
    /// alignment is required (25.8.15, 25.8.16); 16 asked.
    alignas(16) static inline volatile dmac_descriptor_registers_t slot_;
    alignas(16) static inline volatile dmac_descriptor_registers_t write_back_;

    static inline volatile void* data_ = nullptr;
    static inline uint8_t trigger_ = 0;
    static inline uint8_t owner_ = 0;
    /// arm() ran for the current claim: the block is clocked and set up.
    static inline bool armed_ = false;
    static inline uint16_t in_flight_ = 0;
    static inline uint32_t faults_ = 0;
    static inline bool busy_ = false;
};

} // namespace brio

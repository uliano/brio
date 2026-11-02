/*
 * dmac.hpp
 *
 * The SAM C21 Direct Memory Access Controller (DS60001479M ch. 25), driven
 * within a measured budget: AT MOST TWO CHANNELS HELD IN AN IMAGE, of any
 * direction and any owner. A program does not call the engines: it names
 * them in a transport's template arguments - samc21/sercom.hpp's Uart,
 * samc21/spi.hpp's SpiHost - and binds the DMAC's one vector to those
 * transports' bodies, and the transport then moves its data in blocks with
 * no interrupt per byte (docs/samc21/dmac.md):
 *
 *     #include "samc21/dmac.hpp"
 *     using Serial = brio::Uart<5, console_pads, 64, 256, brio::DmaTxEngine<0>>;
 *     using SpiHw = brio::SpiHost<1, spi_pads, 0, brio::DmaTxEngine<1>>;
 *     extern "C" void DMAC_Handler() {
 *         Serial::dma_isr();
 *         if (SpiHw::dma_isr()) { brio::post<SpiBus>(brio::TransferDone{SpiHw::status()}); }
 *     }
 *
 * Who calls what. A transport's init() calls arm() (main context, a setup
 * verb), which claims the channel, brings the block up if it is the first
 * holder, configures the channel with bounded waits and opens the DMAC's
 * NVIC line; its release() calls release(). Blocks are started from the
 * transport's verbs in main context and from its completion in the DMAC
 * handler. Every access to a channel's registers runs through one selector
 * (below) and is masked for those two stores or loads and no longer; the
 * programming of a block is three stores into SRAM, unmasked. take_interrupt(),
 * complete() and abandon() are a completion body's, reached through the
 * transport's dma_isr() in the program's DMAC_Handler; a DMAC_Handler left
 * unbound is the crt's weak alias of its silent spin, where the program
 * stops at the first block's end. EVERY OWNER IN THE IMAGE IS CALLED FROM THE
 * ONE VECTOR: the line stays up while any armed flag stands, and each owner
 * clears its own channels' flags and nobody else's. The buffers a block
 * carries are lent to the channel until the owner hears its end.
 *
 *  Dmac            the block's public face: its NVIC line and the budget's
 *                  two questions, who holds a channel and how many are held.
 *
 *  DmaTxEngine<ch> a channel draining a run of bytes into a peripheral's
 *                  DATA register, one beat per trigger, ch being 0 or 1.
 *
 *  DmaRxEngine<ch> a channel filling a run of bytes from a peripheral's
 *                  DATA register, the same way.
 *
 * Why two. Erratum 1.10.4 (DS80000740S, "Concurrent channels triggers",
 * E/G/J at revisions E, F and H - this silicon): "When using concurrent
 * channels triggers, the DMAC write-back descriptors may get corrupted."
 * The write-back section is not a report: for an ongoing block the
 * descriptor is fetched from it (25.6.2.6), so a corrupted write-back is a
 * corrupted transfer. Measured on one ATSAMC21J18A at revision F
 * (docs/samc21/dmac.md, "What the rule rests on"): two active channels never
 * faulted in any shape; three or more faulted in every shape, whoever owned
 * them, under every arbitration and quality-of-service setting the
 * controller offers.
 *
 * The budget, and how it is enforced. There are two channels to name: an
 * engine's number is 0 or 1 and any other does not compile, so the section
 * tables hold two entries and no image can make a third channel run. Two
 * transports may still name one channel, so a channel is claimed at arm() by
 * its owner (the SERCOM number plus one, whichever personality the instance
 * has) and the claim that finds it held by another owner PANICS - the third
 * claim of an image, or the second on one number - with the breadcrumb written
 * (PanicCode::assert_failed, context dma_claim_refused_context(owner)): at the
 * first boot that tries it, never silently. A claim is given back by release(),
 * so transports that take a channel in turn are legal. A build-time refusal is
 * not built: "two different instantiations name this type" is nothing C++ can
 * count without stateful metaprogramming, and it would also refuse that legal
 * shape.
 *
 * What is not offered, and what a driver here never does. No channel is
 * suspended: a software suspend of a running channel corrupts the OTHER
 * channel's live descriptor within a second (measured, dmac.md), so progress
 * is read with no suspend (DmaRxEngine::take_landed()) or at the block's end.
 * No linked descriptor, no event-triggered channel, no memory copy, no
 * converter or timer stream: dmac.md's "Not covered yet" says why each.
 *
 * The level is the rule's: receive 3, transmit 2 (docs/design/dma.md, "The
 * priority rule"). Each channel takes one of four levels (CHCTRLB.LVL), a
 * higher number winning while every level is enabled, and a level is enabled
 * by CTRL.LVLENx (25.6.2.4) - a channel at a level not enabled is invisible to
 * the arbiter (25.8.1, measured). Within a level the lower channel number wins,
 * which is the one thing the number of an engine says to the program. On this
 * chip the level protects nothing against the erratum (measured worthless for
 * two channels, harmful for three, dmac.md); it ranks a receive above a transmit
 * because an overrun loses data and a late transmit does not. The bus's own
 * QOSCTRL (25.8.7, the DMAC as a host of the SRAM, 10.4.3) is another knob and
 * stays at its reset value.
 *
 * Two moments. arm() configures the binding once - the channel reset and
 * configured (the trigger, TRIGACT beat, the level), its interrupts armed,
 * and the slot's constant half written: the fixed side's address, no next
 * descriptor. A block start then writes what changes - BTCTRL (one literal),
 * BTCNT and the walking side's address, three stores into SRAM - and enables
 * the channel.
 *
 * CHID is a selector with an interrupt between. CHCTRLA, CHCTRLB, CHINTEN*,
 * CHINTFLAG and CHSTATUS talk to whichever channel CHID names (25.8.17), a
 * select-then-use pair the other owner's completion can land inside. A handler
 * here runs to completion, so a vector's select and use need nothing and main
 * context's take the platform's critical section around the pair alone
 * (Dmac::with_channel()); CHID is left wherever the last user put it, and no
 * path assumes otherwise. take_interrupt() asks INTSTATUS first (25.8.11, one
 * bit a channel, set only for an armed flag), so an owner with nothing pending
 * pays one load and moves no selector.
 *
 * The end address. For an incrementing side the descriptor holds the address
 * one beat past the last one - start plus the run's length in bytes (25.6.2.7;
 * the register descriptions 25.10.3/25.10.4 print a stray "+ 1" that their own
 * definition of BEATSIZE makes meaningless, and the bench decided it by moving
 * bytes, docs/samc21/dmac.md). The engine computes it; a caller hands in a run.
 *
 * Disabling is not instantaneous, and the edges behind it are silent: a '0'
 * written to CHCTRLA.ENABLE does not clear until the internal buffer drains
 * (25.8.18, 25.6.3.6), and until it does CHCTRLB is still enable-protected and
 * CHCTRLA.SWRST is ignored - the bit never reads back set, so a wait for it to
 * clear succeeds having reset nothing. Every disable here waits, bounded, and
 * the channel reset refuses when it did not complete.
 *
 * BLOCKACT = INT, and what silences a block is the interrupt enable: on this
 * die BLOCKACT = NOACT does not silence TCMPL, contrary to 25.8.22's clause
 * (measured, dmac.md), so one literal serves both completions and DmaCompletion
 * decides what is armed. A single block disables its channel at its end
 * (25.6.2.6), which is what makes a block start onto an idle channel safe with
 * no wait. A BUS ERROR disables the channel and sets TERR with the count written
 * back (25.6.2.8); an owner answers it as a block lost (abandon()), and a channel
 * that took one loses the first beat of its next block (measured, dmac.md).
 *
 * The standing request. A SERCOM's transmit request is a level - "set when the
 * transmit buffer is empty", "cleared when DATA is written" (31.6.4.1) - and the
 * channel takes it as it finds it: a request that stands at the channel's
 * enable, or when its trigger is selected onto it, is served at once, so the
 * enable of a block fires its first beat by itself and nothing kicks - a
 * software trigger after that beat started is a second beat into a full DATA
 * (measured, docs/samc21/sercom.md). What the channel does not do is keep a
 * request across its own disable (25.6.2.4: a disabled channel leaves the queue
 * of pending ones, PEND cleared, 25.8.23): a DRE that rose and fell again while
 * the channel was disabled fires nothing at the next enable (measured,
 * docs/samc21/dmac.md, "A released requester").
 *
 * A release leaves the next owner nothing. The channel keeps no request (above),
 * and arm() resets the channel before it selects the next owner's trigger. The
 * requester's own line is its owner's to withdraw: a SERCOM gated with ENABLE set
 * holds DRE up and hands whoever selects its trigger next one stray beat, so a
 * transport resets its SERCOM in release() before the gate (measured, the same
 * section).
 *
 * Errata 1.10.1 (CRCDATAIN, rev B), 1.10.2 and 1.10.3 (linked descriptors, E/G/J
 * revisions B..D - the N family's row carries the marks under E and F: read the
 * row, not the column) are not this silicon; the CRC engine and linked
 * descriptors are not built.
 *
 * Zero when absent. The descriptor sections are static members, present only in
 * an image that reaches them (-fdata-sections, --gc-sections): a transport that
 * names no engine never includes this header at all.
 */

#pragma once

#include <stdint.h>

#include <optional>
#include <span>

#include "sam.h"

#include "kernel/panic.hpp"
#include "samc21/clock.hpp"
#include "samc21/nvic.hpp"
#include "samc21/platform.hpp"

namespace brio {

/// The panic context a refused claim writes: 0xD0 with the refused owner
/// in the low nibble (an owner is its SERCOM number plus one, so 0xD1 ..
/// 0xD6 name SERCOM0 .. SERCOM5).
constexpr uint8_t dma_claim_refused_context(uint8_t owner) {
    return static_cast<uint8_t>(0xD0u | (owner & 0x0Fu));
}

/// What a block's end does, chosen once at arm().
///  - interrupt: TCMPL armed, the owner's next step hangs on the block's end
///    (the Uart consumes what it carried and starts the next run);
///  - silent: only TERR armed, the owner proves the end elsewhere (the SPI
///    host, on the SERCOM's TXC).
enum class DmaCompletion : uint8_t { interrupt, silent };

template <uint8_t ch>
class DmaChannel;
template <uint8_t ch>
class DmaTxEngine;
template <uint8_t ch>
class DmaRxEngine;

/**
 * The block's public face. Everything else of the block (the AHB clock, the
 * reset, the two descriptor sections, the arbitration) is the engines' to
 * configure, through the private half here: the first holder brings it up,
 * the last one takes it down, and a holder between them touches its channel
 * alone.
 */
class Dmac {
public:
    Dmac() = delete;

    static constexpr IRQn_Type irq() { return DMAC_IRQn; }

    /// The budget: the channels an image may hold, and so the numbers an
    /// engine may name (0 .. budget - 1). A channel's number is its index in
    /// the two section tables.
    static constexpr uint8_t budget = 2;

    /// Who holds this channel; 0 when nobody does.
    static uint8_t owner(uint8_t channel) { return owners_[channel]; }

    /// The channels held now, 0 .. budget.
    static uint8_t held() {
        return static_cast<uint8_t>((owners_[0] != 0u ? 1u : 0u) + (owners_[1] != 0u ? 1u : 0u));
    }

private:
    template <uint8_t>
    friend class DmaChannel;

    static dmac_registers_t& regs() { return *DMAC_REGS; }

    /// The one door to a channel's registers from main context: select under the
    /// critical section, use under the same one, release. A pair and nothing
    /// else: no wait and no copy runs inside it (the file header).
    template <typename Op>
    [[gnu::always_inline]] static decltype(auto) with_channel(uint8_t id, Op&& op) {
        typename SamPlatform::CriticalSection cs;
        regs().DMAC_CHID = static_cast<uint8_t>(DMAC_CHID_ID(id));
        return op(regs());
    }

    /// The bound on every wait of this file.
    static constexpr uint32_t spins = 0xFFFFu;

    /// The decision of a claim, a test-and-set under the mask and nothing
    /// else: true when the channel was free or already `owner`'s.
    static bool claim(uint8_t channel, uint8_t owner) {
        typename SamPlatform::CriticalSection cs;
        if (owners_[channel] != 0u && owners_[channel] != owner) {
            return false;
        }
        owners_[channel] = owner;
        return true;
    }

    /// One more holder of the block; the first brings it up.
    static bool hold() {
        if (armed_ == 0u && !block_up()) {
            return false;
        }
        armed_ = static_cast<uint8_t>(armed_ + 1u);
        Nvic::enable(irq());
        return true;
    }

    /// One holder fewer; the last takes the block down, its clock with it.
    static void let_go() {
        if (armed_ == 0u) {
            return;
        }
        armed_ = static_cast<uint8_t>(armed_ - 1u);
        if (armed_ == 0u) {
            Nvic::disable(irq());
            (void)block_off();
            Mclk::ahb(MCLK_AHBMASK_DMAC_Msk, false);
        }
    }

    /// The block from reset: its clock (CLK_DMAC_AHB is its only one, 25.5.3 -
    /// no GCLK channel), CTRL disabled and waited out, SWRST (refused by the
    /// silicon with either engine enabled, 25.8.1), the two sections
    /// registered while it is stopped (BASEADDR and WRBADDR are
    /// enable-protected: a write under DMAENABLE is discarded), the two levels
    /// the engines use enabled, the block on.
    ///
    /// The levels are the group macro DMAC_CTRL_LVLEN(), one bit a level: the
    /// per-level DMAC_CTRL_LVLEN0() masks to one bit, and a channel whose level
    /// is not enabled is INVISIBLE to the arbiter (25.8.1, measured).
    static bool block_up() {
        Nvic::disable(irq());
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
        for (uint8_t k = 0; k < budget; ++k) {
            clear(slots_[k]);
            clear(write_backs_[k]);
        }
        regs().DMAC_BASEADDR = reinterpret_cast<uint32_t>(&slots_[0]);
        regs().DMAC_WRBADDR = reinterpret_cast<uint32_t>(&write_backs_[0]);
        regs().DMAC_CTRL = static_cast<uint16_t>(
            DMAC_CTRL_LVLEN((1u << transmit_level) | (1u << receive_level)) |
            DMAC_CTRL_DMAENABLE_Msk);
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

    /// A section entry back to zeros, field by field (VALID first: a
    /// fetch between the stores must not find a valid half-old entry).
    static void clear(volatile dmac_descriptor_registers_t& d) {
        d.DMAC_BTCTRL = 0u;
        d.DMAC_BTCNT = 0u;
        d.DMAC_SRCADDR = 0u;
        d.DMAC_DSTADDR = 0u;
        d.DMAC_DESCADDR = 0u;
    }

    static constexpr uint8_t transmit_level = 2;
    static constexpr uint8_t receive_level = 3;

    /// The first-descriptor section (BASEADDR) and the write-back section
    /// (WRBADDR): arrays indexed by channel, 16 bytes an entry (25.6.2.3), two
    /// entries the whole table. 64-bit alignment is required (25.8.15, 25.8.16);
    /// 16 asked.
    alignas(16) static inline volatile dmac_descriptor_registers_t slots_[budget];
    alignas(16) static inline volatile dmac_descriptor_registers_t write_backs_[budget];
    static inline uint8_t owners_[budget] = {0u, 0u};
    /// Holders that armed: the block is clocked and up while this is not zero.
    static inline uint8_t armed_ = 0;
};

/**
 * One channel, what both engines do on it: the claim, the selector, the reset,
 * the configuration, the enable, the state's questions and the interrupt's
 * take. The engines add the run's direction.
 */
template <uint8_t ch>
class DmaChannel {
    static_assert(ch < Dmac::budget,
                  "brio DMA: an engine's channel is 0 or 1 - an image holds two channels at "
                  "most, the budget docs/samc21/dmac.md gives its measurements for");

public:
    DmaChannel() = delete;

    /// The flags take_interrupt() answers with, CHINTFLAG's layout.
    static constexpr uint8_t flag_error = DMAC_CHINTFLAG_TERR_Msk;
    static constexpr uint8_t flag_complete = DMAC_CHINTFLAG_TCMPL_Msk;

    /**
     * Claim the channel for `owner` (non-zero). True when it was free or already
     * `owner`'s; false when another owner holds it - nothing is touched then.
     */
    static bool claim(uint8_t owner) { return Dmac::claim(ch, owner); }

    /// Who holds the claim; 0 when nobody does.
    static uint8_t owner() { return Dmac::owners_[ch]; }

    /**
     * Claim for `owner` and configure the binding: `fixed` the address of the
     * peripheral's data register, `trigger` its trigger code, `level` the
     * arbitration level, `completion` what a block's end does. A setup verb,
     * main context.
     *
     * A claim held by another owner is a panic - the budget (the file header):
     * the breadcrumb is written with PanicCode::assert_failed and
     * dma_claim_refused_context(owner), and the program stops there. False, with
     * the claim given back, when the block or the channel would not reset (a
     * bounded wait ran out).
     */
    static bool arm(uint8_t owner, uint8_t trigger, uint8_t level, DmaCompletion completion) {
        if (!claim(owner)) {
            panic<SamPlatform>(PanicCode::assert_failed, dma_claim_refused_context(owner));
        }
        trigger_ = trigger;
        level_ = level;
        irqs_ = completion == DmaCompletion::silent
                    ? flag_error
                    : static_cast<uint8_t>(flag_error | flag_complete);
        if (!armed_) {
            if (!Dmac::hold()) {
                give_back();
                return false;
            }
            armed_ = true;
        }
        if (!take()) {
            give_back();
            return false;
        }
        return true;
    }

    /**
     * Stop, and give the claim back if `owner` holds it: the channel disabled
     * (bounded), its interrupts disarmed; the last holder of the block takes the
     * block down too. A claim that never armed touches no register (the block's
     * clock may be off). The caller has waited for a block in flight, or knows
     * no other channel runs: this disable is the one docs/samc21/dmac.md's rule
     * speaks of.
     */
    static void release(uint8_t owner) {
        if (Dmac::owners_[ch] == owner) {
            give_back();
        }
    }

    /// The channel's own reset and configuration, whatever state it is in:
    /// disabled (bounded - an SWRST under ENABLE is ignored silently), reset, the
    /// slot and its write-back cleared, CHCTRLB written (the trigger, TRIGACT
    /// beat, the level), the flags cleared, its interrupts armed. arm() and
    /// abandon() are the same act for a different reason. The owner writes the
    /// slot's constant half after it.
    static bool take() {
        if (!off()) {
            return false;
        }
        Dmac::with_channel(ch, [](dmac_registers_t& r) { r.DMAC_CHCTRLA = DMAC_CHCTRLA_SWRST_Msk; });
        uint32_t left = Dmac::spins;
        while (Dmac::with_channel(ch, [](dmac_registers_t& r) {
            return (r.DMAC_CHCTRLA & DMAC_CHCTRLA_SWRST_Msk) != 0u;
        })) {
            if (left-- == 0u) {
                return false;
            }
        }
        Dmac::clear(Dmac::slots_[ch]);
        Dmac::clear(Dmac::write_backs_[ch]);
        Dmac::with_channel(ch, [](dmac_registers_t& r) {
            r.DMAC_CHCTRLB = DMAC_CHCTRLB_TRIGACT(DMAC_CHCTRLB_TRIGACT_BEAT_Val) |
                             DMAC_CHCTRLB_TRIGSRC(trigger_) | DMAC_CHCTRLB_LVL(level_);
            r.DMAC_CHINTENCLR = flag_all;
            r.DMAC_CHINTFLAG = flag_all;
            r.DMAC_CHINTENSET = irqs_;
        });
        return true;
    }

    /// CHCTRLA.ENABLE cleared and waited out (25.8.18). Each look is its own
    /// masked pair, so the other owner's completion runs between two looks.
    static bool off() {
        Dmac::with_channel(ch, [](dmac_registers_t& r) {
            r.DMAC_CHCTRLA = static_cast<uint8_t>(r.DMAC_CHCTRLA &
                                                  static_cast<uint8_t>(~DMAC_CHCTRLA_ENABLE_Msk));
        });
        uint32_t left = Dmac::spins;
        while (enabled()) {
            if (left-- == 0u) {
                return false;
            }
        }
        return true;
    }

    /// The enable of a block whose slot is written: the masked pair, one select
    /// and one store. The first beat is fired by the standing request if one
    /// stands (the file header).
    [[gnu::always_inline]] static void enable() {
        Dmac::with_channel(ch, [](dmac_registers_t& r) {
            r.DMAC_CHCTRLA = static_cast<uint8_t>(DMAC_CHCTRLA_ENABLE_Msk);
        });
    }

    /// The channel is enabled: it has beats left, or has not yet noticed.
    [[gnu::always_inline]] static bool enabled() {
        return Dmac::with_channel(ch, [](dmac_registers_t& r) {
            return (r.DMAC_CHCTRLA & DMAC_CHCTRLA_ENABLE_Msk) != 0u;
        });
    }

    /// The channel is enabled with no trigger pending and no beat moving
    /// (CHCTRLA.ENABLE set, CHSTATUS.PEND and BUSY clear). Beside a trigger that
    /// stands - the owner's proof - a channel that cannot move: the transports'
    /// dead-block predicates ask it.
    static bool waiting() {
        return Dmac::with_channel(ch, [](dmac_registers_t& r) {
            return (r.DMAC_CHCTRLA & DMAC_CHCTRLA_ENABLE_Msk) != 0u &&
                   (r.DMAC_CHSTATUS & (DMAC_CHSTATUS_PEND_Msk | DMAC_CHSTATUS_BUSY_Msk)) == 0u;
        });
    }

    /// The level the armed channel reads back (CHCTRLB.LVL), when CTRL enables
    /// that level - nothing when it does not, the channel then invisible to the
    /// arbiter (25.6.2.4). For an owner that holds the engine armed: the block's
    /// clock is off otherwise.
    static std::optional<uint8_t> priority() {
        const uint8_t lvl = Dmac::with_channel(ch, [](dmac_registers_t& r) {
            return static_cast<uint8_t>((r.DMAC_CHCTRLB & DMAC_CHCTRLB_LVL_Msk) >>
                                        DMAC_CHCTRLB_LVL_Pos);
        });
        if ((Dmac::regs().DMAC_CTRL & DMAC_CTRL_LVLEN(1u << lvl)) == 0u) {
            return std::nullopt;
        }
        return lvl;
    }

    /// This channel's interrupt flags, taken and cleared. Zero, for one load,
    /// when nothing armed stands on it (INTSTATUS, 25.8.11). A completion body's
    /// verb: the vector's select and use need no mask, main context's every pair
    /// has one (the file header). A TCMPL standing unarmed on a silent channel is
    /// cleared with an armed flag and means nothing.
    [[gnu::always_inline]] static uint8_t take_interrupt() {
        dmac_registers_t& r = Dmac::regs();
        if ((r.DMAC_INTSTATUS & (1u << ch)) == 0u) {
            return 0u;
        }
        r.DMAC_CHID = static_cast<uint8_t>(DMAC_CHID_ID(ch));
        const uint8_t flags = static_cast<uint8_t>(r.DMAC_CHINTFLAG & flag_all);
        r.DMAC_CHINTFLAG = flags;
        return flags;
    }

    /// This channel's first descriptor, the owner's to write.
    static volatile dmac_descriptor_registers_t& slot() { return Dmac::slots_[ch]; }

    /**
     * Beats still to move in the block the slot describes, read WITHOUT stopping
     * it. ACTIVE (25.8.14) holds the active channel's BTCNT while ABUSY stands; a
     * channel paced one beat per trigger keeps its descriptor in the active
     * registers between beats until a different channel is granted (25.6.2.5), and
     * then the write-back holds its count. So the reading is ACTIVE's when ABUSY
     * stands with this channel's ID, and otherwise the write-back's. Both run
     * BEHIND the beats at worst, never ahead.
     *
     * A VALID write-back is judged against the slot - everything but the count and
     * VALID is invariant - and nothing is answered when it disagrees (erratum
     * 1.10.4's fingerprint, counted in violations()). A write-back with VALID clear
     * is a finished descriptor or the zeros take() left, and the count is the
     * slot's: a block's end is its completion's to report. Nothing is masked here:
     * the caller holds the block, so no other context rewrites the slot under it.
     */
    static std::optional<uint16_t> observe() {
        const volatile dmac_descriptor_registers_t& s = Dmac::slots_[ch];
        const uint16_t length = s.DMAC_BTCNT;
        uint16_t remaining = length;
        const uint32_t a = Dmac::regs().DMAC_ACTIVE;
        if ((a & DMAC_ACTIVE_ABUSY_Msk) != 0u && (a & DMAC_ACTIVE_ID_Msk) == DMAC_ACTIVE_ID(ch)) {
            remaining = static_cast<uint16_t>(a >> DMAC_ACTIVE_BTCNT_Pos);
        } else {
            const volatile dmac_descriptor_registers_t& w = Dmac::write_backs_[ch];
            const uint16_t control = w.DMAC_BTCTRL;
            if ((control & DMAC_BTCTRL_VALID_Msk) != 0u) {
                if (control != s.DMAC_BTCTRL || w.DMAC_SRCADDR != s.DMAC_SRCADDR ||
                    w.DMAC_DSTADDR != s.DMAC_DSTADDR || w.DMAC_DESCADDR != s.DMAC_DESCADDR) {
                    violations_ = violations_ + 1u;
                    return std::nullopt;
                }
                remaining = w.DMAC_BTCNT;
            }
        }
        if (remaining > length) {
            violations_ = violations_ + 1u;
            return std::nullopt;
        }
        return remaining;
    }

    /// Readings observe() refused.
    static uint32_t violations() { return violations_; }

private:
    static constexpr uint8_t flag_all = static_cast<uint8_t>(
        DMAC_CHINTFLAG_TERR_Msk | DMAC_CHINTFLAG_TCMPL_Msk | DMAC_CHINTFLAG_SUSP_Msk);

    static void give_back() {
        if (armed_) {
            (void)off();
            Dmac::with_channel(ch, [](dmac_registers_t& r) { r.DMAC_CHINTENCLR = flag_all; });
            armed_ = false;
            Dmac::let_go();
        }
        typename SamPlatform::CriticalSection cs;
        Dmac::owners_[ch] = 0u;
    }

    static inline uint8_t trigger_ = 0;
    static inline uint8_t level_ = 0;
    static inline uint8_t irqs_ = 0;
    /// arm() ran for the current claim: this channel counts among the block's
    /// holders.
    static inline bool armed_ = false;
    static inline volatile uint32_t violations_ = 0;
};

/**
 * DmaTxEngine<ch> - drain a run of bytes into a peripheral's DATA register on
 * channel `ch`, one beat per trigger.
 *
 * The claim and the programming are two steps (reserve(), launch()), so an owner
 * that starts blocks from two contexts - the Uart's producer in main context and
 * its completion in the handler - masks the claim alone, a test-and-set of busy_,
 * and programs a reserved channel unmasked: no completion can land under the
 * programming, because no block is in flight. start() is both, for an owner that
 * starts from one context.
 *
 * No write-back is ever read. The engine knows the run it programmed and learns of
 * its end from TCMPL, or its owner does from its peripheral; there is no third
 * fact to want.
 */
template <uint8_t ch>
class DmaTxEngine {
    // Stated here as well as in DmaChannel: an engine is named inside a
    // transport's template arguments, where the number is one somebody typed,
    // and a member that never reaches the channel must not let it pass.
    static_assert(ch < Dmac::budget,
                  "brio DMA: an engine's channel is 0 or 1 - an image holds two channels at "
                  "most, the budget docs/samc21/dmac.md gives its measurements for");
    using Channel = DmaChannel<ch>;

public:
    DmaTxEngine() = delete;

    /// The tag the transports test with `if constexpr`.
    static constexpr bool present = true;
    static constexpr uint8_t channel = ch;
    /// The arbitration level the channel arms at (the file header).
    static constexpr uint8_t level = 2;
    using element = uint8_t;
    /// arm()'s vocabulary named through the engine, for an owner that does not
    /// include this header.
    using Completion = DmaCompletion;

    static constexpr uint8_t flag_error = Channel::flag_error;
    static constexpr uint8_t flag_complete = Channel::flag_complete;

    /// DmaChannel::claim(), owner() and release(), for this channel.
    static bool claim(uint8_t owner) { return Channel::claim(owner); }
    static uint8_t owner() { return Channel::owner(); }

    /**
     * Claim for `owner` and configure the binding (DmaChannel::arm(): a panic on
     * a claim held by another): `data` the address of the peripheral's transmit
     * data register, `trigger` its TX trigger code. A setup verb, main context.
     */
    static bool arm(uint8_t owner, volatile void* data, uint8_t trigger,
                    DmaCompletion completion = DmaCompletion::interrupt) {
        data_ = data;
        if (!Channel::arm(owner, trigger, level, completion)) {
            return false;
        }
        constant_half();
        in_flight_ = 0;
        busy_ = false;
        return true;
    }

    /// DmaChannel::release(): a block in flight is abandoned without counting.
    static void release(uint8_t owner) {
        if (Channel::owner() == owner) {
            in_flight_ = 0;
            busy_ = false;
        }
        Channel::release(owner);
    }

    /**
     * The claim on the channel: false when a block is already in flight, otherwise
     * the engine is the caller's until launch() or cancel(). An owner that starts
     * blocks from two contexts calls it under the platform's critical section (it
     * is a test-and-set of busy_ and masks nothing itself).
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
     * Send one contiguous run on a reserved engine: three stores into the slot and
     * the enable. False - and the reservation given back - when the run is empty or
     * longer than BTCNT counts (65535 beats); the caller keeps the bytes and offers
     * them again.
     */
    static bool launch(std::span<const uint8_t> run) {
        const uint32_t count = static_cast<uint32_t>(run.size());
        if (count == 0u || count > 0xFFFFu) {
            busy_ = false;
            return false;
        }
        volatile dmac_descriptor_registers_t& s = Channel::slot();
        s.DMAC_BTCTRL = walking_control;
        s.DMAC_BTCNT = static_cast<uint16_t>(count);
        s.DMAC_SRCADDR = reinterpret_cast<uint32_t>(run.data()) + count;
        in_flight_ = static_cast<uint16_t>(count);
        Channel::enable();
        return true;
    }

    /// reserve() and launch() in one call, for an owner that starts blocks from one
    /// context. False when a block is in flight or the run is refused.
    static bool start(std::span<const uint8_t> run) { return reserve() && launch(run); }

    /**
     * Send a run of `count` (1 .. 65535) bytes, for an owner that starts from one
     * context and has checked `count`: nothing is refused here, so no request hangs
     * on a refusal nobody heard.
     */
    static void start_run(const uint8_t* run, uint16_t count) {
        busy_ = true;
        volatile dmac_descriptor_registers_t& s = Channel::slot();
        s.DMAC_BTCTRL = walking_control;
        s.DMAC_BTCNT = count;
        s.DMAC_SRCADDR = reinterpret_cast<uint32_t>(run) + count;
        in_flight_ = count;
        Channel::enable();
    }

    /**
     * The same with the source held still: an SPI host's dummies.
     */
    static void start_fixed(const uint8_t* one, uint16_t count) {
        busy_ = true;
        volatile dmac_descriptor_registers_t& s = Channel::slot();
        s.DMAC_BTCTRL = fixed_control;
        s.DMAC_BTCNT = count;
        s.DMAC_SRCADDR = reinterpret_cast<uint32_t>(one);
        in_flight_ = count;
        Channel::enable();
    }

    /// Bytes of the block in flight (0 when idle).
    static uint16_t in_flight() { return busy_ ? in_flight_ : 0u; }
    static bool busy() { return busy_; }

    /// The channel is enabled: the block's last beat has not been written, or the
    /// channel has not yet noticed. A single block disables it at its end.
    [[gnu::always_inline]] static bool running() { return Channel::enabled(); }

    /// DmaChannel::waiting(), for the owner's dead-block predicate.
    static bool waiting() { return Channel::waiting(); }

    /// DmaChannel::priority().
    static std::optional<uint8_t> priority() { return Channel::priority(); }

    /// DmaChannel::take_interrupt(): the completion body's first look.
    [[gnu::always_inline]] static uint8_t take_interrupt() { return Channel::take_interrupt(); }

    /**
     * The block ended (TCMPL, or the owner's proof on a silent channel): how many
     * bytes it carried, so the owner consumes exactly that much of its ring.
     */
    [[gnu::always_inline]] static uint16_t complete() {
        const uint16_t moved = in_flight_;
        in_flight_ = 0;
        busy_ = false;
        return moved;
    }

    /**
     * Throw away a block the silicon is not finishing, and hand the channel back
     * ready for the next one: reset, reconfigured, re-armed. The caller decides the
     * block is dead - only the peripheral's owner can read the flags that make it a
     * fact - or a bus error did (TERR, the channel disabled by the silicon). What is
     * lost is an unknown tail of the block; every abandonment is counted in faults().
     * True when a block was abandoned. Out of line and cold: the rare path, so an
     * owner's completion flattened into its vector, or placed in SRAM, carries none
     * of it. A channel still enabled here is disabled mid-block: docs/samc21/dmac.md's
     * rule says what that can do to another running channel.
     */
    [[gnu::noinline, gnu::cold]] static bool abandon() {
        if (!busy_) {
            return false;
        }
        faults_ = faults_ + 1u;
        (void)Channel::take();
        constant_half();
        in_flight_ = 0;
        busy_ = false;
        return true;
    }

    static uint32_t faults() { return faults_; }
    static void clear_faults() { faults_ = 0; }

private:
    /// BTCTRL of every block: VALID, byte beats, the block's end an interrupt where
    /// armed (and the channel disabled, 25.6.2.6); the source walking, or held.
    static constexpr uint16_t fixed_control =
        static_cast<uint16_t>(DMAC_BTCTRL_VALID_Msk |
                              DMAC_BTCTRL_BEATSIZE(DMAC_BTCTRL_BEATSIZE_BYTE_Val) |
                              DMAC_BTCTRL_BLOCKACT(DMAC_BTCTRL_BLOCKACT_INT_Val));
    static constexpr uint16_t walking_control =
        static_cast<uint16_t>(fixed_control | DMAC_BTCTRL_SRCINC_Msk);

    /// The slot's constant half: the fixed destination, no next descriptor.
    static void constant_half() {
        volatile dmac_descriptor_registers_t& s = Channel::slot();
        s.DMAC_DSTADDR = reinterpret_cast<uint32_t>(data_);
        s.DMAC_DESCADDR = 0u;
    }

    static inline volatile void* data_ = nullptr;
    // Written by the owner's completion vector (complete(), abandon() on a transfer
    // error) and read by the thread - busy() in the transport's refused-byte repair,
    // faults() and in_flight() by whoever asks - so volatile: a thread loop must load
    // them, not keep a stale copy.
    static inline volatile uint16_t in_flight_ = 0;
    static inline volatile uint32_t faults_ = 0;
    static inline volatile bool busy_ = false;
};

/**
 * DmaRxEngine<ch> - fill a run of bytes from a peripheral's DATA register on channel
 * `ch`, one beat per trigger. The transmit engine's mirror, with one asymmetry: the
 * arrival of data is not an event anyone is told about. A receive block completes
 * only when its run is full, which on an idle line may be never, so what has landed
 * so far is read by asking (take_landed(), DmaChannel::observe(): no suspend).
 *
 * A block starts on an idle channel: its previous block completed (a single block
 * disables its channel at its end, 25.6.2.6) or it never ran, so the start writes the
 * slot and enables and waits for nothing. An owner whose claim must cover the enable
 * and nothing before it splits the start: prepare() is the work, launch() the enable.
 */
template <uint8_t ch>
class DmaRxEngine {
    // Stated here as well as in DmaChannel: an engine is named inside a
    // transport's template arguments, where the number is one somebody typed,
    // and a member that never reaches the channel must not let it pass.
    static_assert(ch < Dmac::budget,
                  "brio DMA: an engine's channel is 0 or 1 - an image holds two channels at "
                  "most, the budget docs/samc21/dmac.md gives its measurements for");
    using Channel = DmaChannel<ch>;

public:
    DmaRxEngine() = delete;

    static constexpr bool present = true;
    static constexpr uint8_t channel = ch;
    /// The arbitration level the channel arms at (the file header).
    static constexpr uint8_t level = 3;
    using element = uint8_t;
    using Completion = DmaCompletion;

    static constexpr uint8_t flag_error = Channel::flag_error;
    static constexpr uint8_t flag_complete = Channel::flag_complete;

    /// DmaChannel::claim(), owner() and release(), for this channel.
    static bool claim(uint8_t owner) { return Channel::claim(owner); }
    static uint8_t owner() { return Channel::owner(); }

    /// As DmaTxEngine::arm(), `data` the peripheral's receive data register and
    /// `trigger` its RX trigger code.
    static bool arm(uint8_t owner, volatile void* data, uint8_t trigger,
                    DmaCompletion completion = DmaCompletion::interrupt) {
        data_ = data;
        if (!Channel::arm(owner, trigger, level, completion)) {
            return false;
        }
        constant_half();
        capacity_ = 0;
        taken_ = 0;
        return true;
    }

    static void release(uint8_t owner) {
        if (Channel::owner() == owner) {
            capacity_ = 0;
            taken_ = 0;
        }
        Channel::release(owner);
    }

    /// The slot written for a run of `count` (1 .. 65535) bytes on an idle channel;
    /// the owner has checked the count. launch() is the enable.
    [[gnu::always_inline]] static void prepare(uint8_t* run, uint16_t count) {
        volatile dmac_descriptor_registers_t& s = Channel::slot();
        s.DMAC_BTCTRL = walking_control;
        s.DMAC_BTCNT = count;
        s.DMAC_DSTADDR = reinterpret_cast<uint32_t>(run) + count;
        capacity_ = count;
        taken_ = 0;
    }
    [[gnu::always_inline]] static void launch() { Channel::enable(); }

    /// prepare() and launch() in one call.
    static void start(uint8_t* run, uint16_t count) {
        prepare(run, count);
        launch();
    }

    /// The channel is not running a block: it finished, or was never started.
    [[gnu::always_inline]] static bool idle() { return !Channel::enabled(); }

    /// DmaChannel::waiting(), for the owner's dead-block predicate.
    static bool waiting() { return Channel::waiting(); }

    /// DmaChannel::priority().
    static std::optional<uint8_t> priority() { return Channel::priority(); }

    /// DmaChannel::take_interrupt(): the completion body's first look.
    [[gnu::always_inline]] static uint8_t take_interrupt() { return Channel::take_interrupt(); }

    /**
     * The beats landed since the last take, read WITHOUT suspending the channel
     * (DmaChannel::observe()), and taken. Zero when none landed or the reading was
     * refused. The caller holds the block: no completion may retire it and start the
     * next one between the reading and the take.
     */
    static uint16_t take_landed() {
        const auto remaining = Channel::observe();
        if (!remaining) {
            return 0u;
        }
        const uint16_t done = static_cast<uint16_t>(capacity_ - *remaining);
        if (done <= taken_) {
            return 0u;
        }
        const uint16_t fresh = static_cast<uint16_t>(done - taken_);
        taken_ = done;
        return fresh;
    }

    /**
     * The block ended: every beat of the run has landed, so the count is the run's and
     * not the write-back's. Returns the beats not taken yet and takes them.
     */
    [[gnu::always_inline]] static uint16_t complete() {
        const uint16_t fresh = static_cast<uint16_t>(capacity_ - taken_);
        taken_ = capacity_;
        return fresh;
    }

    /// As DmaTxEngine::abandon(), with nothing to ask: the block's bytes are the
    /// owner's to count before it calls this.
    [[gnu::noinline, gnu::cold]] static void abandon() {
        faults_ = faults_ + 1u;
        (void)Channel::take();
        constant_half();
        capacity_ = 0;
        taken_ = 0;
    }

    static uint32_t faults() { return faults_; }
    static void clear_faults() { faults_ = 0; }

private:
    /// BTCTRL of every block: VALID, byte beats, the destination walking, the block's
    /// end an interrupt where armed.
    static constexpr uint16_t walking_control =
        static_cast<uint16_t>(DMAC_BTCTRL_VALID_Msk | DMAC_BTCTRL_DSTINC_Msk |
                              DMAC_BTCTRL_BEATSIZE(DMAC_BTCTRL_BEATSIZE_BYTE_Val) |
                              DMAC_BTCTRL_BLOCKACT(DMAC_BTCTRL_BLOCKACT_INT_Val));

    /// The slot's constant half: the fixed source, no next descriptor.
    static void constant_half() {
        volatile dmac_descriptor_registers_t& s = Channel::slot();
        s.DMAC_SRCADDR = reinterpret_cast<uint32_t>(data_);
        s.DMAC_DESCADDR = 0u;
    }

    static inline volatile void* data_ = nullptr;
    static inline uint16_t capacity_ = 0;
    static inline uint16_t taken_ = 0;
    static inline volatile uint32_t faults_ = 0;
};

} // namespace brio

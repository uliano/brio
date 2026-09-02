/*
 * dma.hpp
 *
 * The DMA controller (RM0444 ch. 10) and the request multiplexer that
 * feeds it (ch. 11), in the two strata every brio driver has:
 *
 *   Dma<n>                  the BLOCK: its bus clock, its reset, the two
 *                           flag registers every channel reports into,
 *                           and how many channels it has;
 *   DmaChannel<n, ch>       the RESOURCE: one channel's four registers
 *                           (CCR/CNDTR/CPAR/CMAR), the enable discipline
 *                           the chapter states, the flag surface and one
 *                           ISR body;
 *   DmaMux / DmaMuxGenerator<x>
 *                           the FABRIC that decides which peripheral's
 *                           request reaches which channel, with the
 *                           synchronization block and the request
 *                           generator;
 *   DmaTxEngine / DmaRxEngine
 *                           the two TASKS a byte transport wants: drain a
 *                           run into a peripheral, fill a run from one;
 *   DmaLoopEngine           the TASK a waveform wants - one caller-owned
 *                           table played for ever (util/block_stream.hpp's
 *                           BlockPlayer on this silicon);
 *   DmaPingPongEngine       the TASK a sampled stream wants - fill one
 *                           caller-owned buffer while the caller drains
 *                           the other (util/block_stream.hpp's
 *                           BlockSource on this silicon).
 *
 * THE DRIVER OWNS THE FABRIC AND NOT THE REQUEST VOCABULARY. Table 55
 * numbers 77 request lines by peripheral, and no device header of this
 * pack declares one of those numbers (the DMAMUX_REQ_* spellings live in
 * ST's HAL/LL, which this project does not vendor). Reproducing the table
 * here would be a list somebody has to keep, so the samc EVSYS ruling
 * applies unchanged: a peripheral publishes ITS OWN request codes
 * (Usart<n>::dma_rx_request(), Tim<n>::dma_update_request() and their
 * kin) and this file takes a plain `uint8_t` request id. `dma_request_none`
 * is the null the chapter gives DMAREQ_ID = 0.
 *
 * WHAT THIS CONTROLLER DOES THAT THE SAM C21'S DOES NOT, and both facts
 * change the engines above:
 *
 *  - A REQUEST IS A LEVEL SERVED ON ENABLE, NOT AN EDGE LATCHED ON THE
 *    RISE. 10.4.3 spells the handshake out: the peripheral drives its
 *    request, the controller acknowledges, the peripheral releases. A
 *    channel enabled while its peripheral's request is ALREADY standing
 *    therefore serves it at once - there is no software-trigger register
 *    on this controller because there is nothing for one to do. The SAM's
 *    kick() has no twin here, and its absence is measured, not assumed
 *    (docs/stm32g0/dma.md).
 *
 *  - THERE IS A HARDWARE CIRCULAR MODE (CCR.CIRC). At the end of a block
 *    CNDTR and both current address registers reload themselves and the
 *    channel keeps serving requests, with no CPU in the path at all. That
 *    is what DmaLoopEngine is built on, and it is why this target's
 *    player has no per-lap re-arm window: the interrupt at the wrap
 *    COUNTS laps, it does not create them.
 *
 * WHY DmaPingPongEngine IS *NOT* CIRCULAR, which is this file's one real
 * design position. Circular mode plus the half-transfer flag looks like a
 * ping-pong for free: two halves of one buffer, HT and TC as the two
 * edges. It is not one, because a circular channel NEVER STOPS. The
 * BlockSource contract (docs/design/block-stream.md) is that a source
 * which has no free buffer SKIPS the lap rather than write into the block
 * the caller is reading - integrity traded for samples, so that everything
 * handed over is untorn. Under CIRC that decision can only be taken AFTER
 * the edge, in the handler, and by then the controller is already writing
 * the next half: the tear happens while the software is deciding not to
 * allow it. That race is measured in test_stm32_dma (letter g), and the
 * answer is the non-circular channel this engine uses - it stops itself at
 * the end of every block, exactly as the SAM's does, and the swap into the
 * other buffer happens in the handler with the channel idle. The concept
 * needed no change; the natural-looking implementation did.
 *
 * ERRATA (ES0548 rev 3, silicon revision Z - all five DMA/DMAMUX items
 * apply to this part):
 *  - 2.4.1, DMA disable failure and error flag omission upon simultaneous
 *    transfer error and global flag clear: a CGIFx write landing in the
 *    same cycle as a transfer error loses the automatic disable AND the
 *    TEIFx flag. THIS FILE NEVER WRITES CGIFx AT ALL - clear() masks the
 *    global bit out and clears only CTCIFx/CHTIFx/CTEIFx, which is the
 *    erratum's own workaround made structural (there is no state to get
 *    wrong, and a caller cannot opt into the bug).
 *  - 2.5.4, wrong input DMA request routed upon a DMAMUX_CxCR write
 *    coinciding with a synchronization event: the workaround is that SPOL
 *    must be 00 whenever SE is 0, and must be set non-zero in the SAME
 *    write that sets SE. DmaMux::synchronize() writes one word and
 *    enforces both halves; there is no verb that can leave the illegal
 *    combination behind.
 *  - 2.5.1 and 2.5.3, SOFx/OFx not asserted when the software writes the
 *    matching clear register: no workaround exists. Stated as an
 *    obligation on the two clear verbs - do not run two synchronized
 *    channels, or two request generators, that can both overrun.
 *  - 2.5.2, OFx not asserted for a trigger event coinciding with the last
 *    generated request (and the next trigger then generating ONE request
 *    instead of GNBREQ + 1), for GNBREQ > 1: an obligation on the caller
 *    of DmaMuxGenerator::configure(), since only the application knows its
 *    trigger period.
 */

#pragma once

#include <stdint.h>
#include <optional>

#include "stm32g0xx.h"

#include "stm32g0/clock.hpp"
#include "stm32g0/device_tables.hpp"
#include "stm32g0/nvic.hpp"
#include "stm32g0/platform_stm32.hpp"

namespace brio {

// ---- vocabulary ---------------------------------------------------------------

/// CCR.PSIZE / CCR.MSIZE (10.6.3): the width of one bus access. The code
/// is not the width - `dma_width_bytes()` is.
enum class DmaWidth : uint8_t {
    byte = 0,
    half = 1,
    word = 2,
};

constexpr uint8_t dma_width_bytes(DmaWidth w) {
    switch (w) {
        case DmaWidth::byte: return 1;
        case DmaWidth::half: return 2;
        default: return 4;
    }
}

/**
 * THE ELEMENT TYPE IS THE ACCESS WIDTH. One `sizeof` decides PSIZE, MSIZE
 * and the address arithmetic together, so they cannot disagree - the SAM
 * campaign's dma_beat_of() rule, kept because it was right.
 *
 * Only the three widths the silicon has are element types: anything else
 * is a compile error at the engine that named it, which is where a
 * programmer can read it.
 */
template <typename Elem>
constexpr DmaWidth dma_width_of() {
    static_assert(sizeof(Elem) == 1 || sizeof(Elem) == 2 || sizeof(Elem) == 4,
                  "a DMA element is one bus access wide: 1, 2 or 4 bytes "
                  "(CCR.PSIZE/MSIZE have no other code)");
    return sizeof(Elem) == 1   ? DmaWidth::byte
           : sizeof(Elem) == 2 ? DmaWidth::half
                               : DmaWidth::word;
}

/// CCR.PL (10.6.3): the software half of the arbitration. The hardware
/// half is the channel INDEX, and it cannot be configured: on equal
/// levels the lower-numbered channel wins (10.4.4).
enum class DmaPriority : uint8_t {
    low = 0,
    medium = 1,
    high = 2,
    very_high = 3,
};

/**
 * CCR.DIR (10.6.3), and the register naming trap it carries. DIR does not
 * say "peripheral" or "memory": it says WHICH SIDE IS THE SOURCE. With
 * DIR = 0 the CPAR side is read and the CMAR side written; with DIR = 1 it
 * is the other way round. In memory-to-memory mode both sides are memory
 * and the names mean nothing at all - which is why the enumerators here
 * are spelled from the SOURCE, and the transfer struct calls its two ends
 * `peripheral` and `memory` only because that is what the registers are
 * called.
 */
enum class DmaDirection : uint8_t {
    peripheral_to_memory = 0,   ///< DIR = 0: CPAR is read, CMAR is written
    memory_to_peripheral = 1,   ///< DIR = 1: CMAR is read, CPAR is written
};

/// The four bits one channel owns in DMA_ISR and DMA_IFCR (10.6.1,
/// 10.6.2). A channel's group sits at 4 * (ch - 1).
struct DmaFlag {
    static constexpr uint32_t global = 1u << 0;     ///< GIFx - READ ONLY here, see below
    static constexpr uint32_t complete = 1u << 1;   ///< TCIFx
    static constexpr uint32_t half = 1u << 2;       ///< HTIFx
    static constexpr uint32_t error = 1u << 3;      ///< TEIFx
    /// Everything a caller may clear. GIFx is deliberately NOT in it:
    /// ES0548 2.4.1 makes a CGIFx write that coincides with a transfer
    /// error lose both the error flag and the automatic channel disable,
    /// and the erratum's workaround is simply never to use the bit.
    static constexpr uint32_t all = complete | half | error;
};

/// Everything about a channel that is not an address or a length. Every
/// field here is read-only while the channel is enabled (10.6.3), so
/// configure() refuses rather than storing into a register the silicon
/// ignores.
struct DmaChannelConfig {
    DmaDirection direction = DmaDirection::peripheral_to_memory;
    bool circular = false;              ///< CIRC: reload and keep going
    bool memory_to_memory = false;      ///< MEM2MEM: no request, run on enable
    bool peripheral_increment = false;  ///< PINC
    bool memory_increment = true;       ///< MINC
    DmaWidth peripheral_width = DmaWidth::byte;   ///< PSIZE
    DmaWidth memory_width = DmaWidth::byte;       ///< MSIZE
    DmaPriority priority = DmaPriority::low;      ///< PL
};

/// 10.4.5's own rule, as a predicate: "The states of MEM2MEM and CIRC
/// bits must not be both high at the same time" - said twice more in the
/// circular and memory-to-memory paragraphs, and enforced here so it
/// cannot be got wrong at run time either.
constexpr bool dma_channel_config_valid(const DmaChannelConfig& c) {
    return !(c.circular && c.memory_to_memory);
}

/// One block transfer: both ends and how many data items. The names are
/// the registers' (CPAR, CMAR); which one is the source is `direction`'s
/// to say.
struct DmaTransfer {
    volatile void* peripheral = nullptr;   ///< CPAR
    volatile void* memory = nullptr;       ///< CMAR
    uint16_t count = 0;                    ///< CNDTR, in data items
    DmaChannelConfig config{};
};

constexpr bool dma_transfer_valid(const DmaTransfer& t) {
    return t.peripheral != nullptr && t.memory != nullptr && t.count != 0u &&
           dma_channel_config_valid(t.config);
}

/// How far a running channel has got - and unlike the SAM's, this
/// reading costs nothing and is never refused: CNDTR is a live register
/// the controller decrements, readable at any time (10.6.4), where the
/// SAM had to SUSPEND a channel and validate a write-back against an
/// erratum. There is no harvest ceremony on this silicon.
struct DmaProgress {
    uint16_t remaining = 0;   ///< data items still to move
    uint16_t done = 0;        ///< data items moved in the current block
};

/// DMAREQ_ID = 0 (11.4.4): "no DMA request line selected". A channel left
/// on it moves nothing unless MEM2MEM runs it.
constexpr uint8_t dma_request_none = 0;

// ---- the block ----------------------------------------------------------------

/**
 * Dma<n>: one controller. Two registers of its own (ISR and IFCR) and a
 * clock; everything else belongs to a channel.
 *
 * THE RESET IS SHARED WITH THE DMAMUX and, on a part with two
 * controllers, resetting either one resets the multiplexer for BOTH
 * (RCC_AHBRSTR's own bit descriptions). reset() therefore says so and
 * exists for the bring-up case; a program with two live controllers uses
 * per-channel reset() instead.
 */
template <uint8_t n>
struct Dma {
    static_assert(dma_present(n),
                  "brio Dma: this device has no such DMA controller (the device "
                  "header declares no DMAn_BASE for it - DMA2 exists on the G0B1 "
                  "class alone)");

    Dma() = delete;

    static constexpr uint8_t instance = n;
    static constexpr uint8_t channels = dma_channels(n);

    static DMA_TypeDef& regs() { return *reinterpret_cast<DMA_TypeDef*>(dma_base(n)); }

    /// The AHB clock of the controller - and of the DMAMUX, which 17.4.2
    /// keeps alive as long as at least one DMA is clocked.
    static void bus_clock(bool on) { Rcc::ahb_clock(dma_clock_mask(n), on); }
    static bool bus_clock() { return (RCC->AHBENR & dma_clock_mask(n)) != 0u; }

    /// Every register back to its reset value - THE DMAMUX WITH IT.
    static void reset() {
        RCC->AHBRSTR = RCC->AHBRSTR | dma_reset_mask(n);
        RCC->AHBRSTR = RCC->AHBRSTR & ~dma_reset_mask(n);
    }

    /// The whole interrupt status register: four bits per channel, the
    /// group of channel `ch` at 4 * (ch - 1).
    static uint32_t flags() { return regs().ISR; }

    static constexpr uint32_t flag_shift(uint8_t ch) {
        return static_cast<uint32_t>(4u * (ch - 1u));
    }

    /// The NVIC line channel `ch` reports on. Three vectors serve twelve
    /// channels here, so a handler answers for a SET (device_tables.hpp).
    static constexpr IRQn_Type irq(uint8_t ch) { return dma_channel_irq(n, ch); }
};

// ---- the channel --------------------------------------------------------------

/**
 * DmaChannel<n, ch>: one channel of controller n, numbered as the
 * silicon numbers it (1-based; there is no channel 0).
 *
 * THE ENABLE DISCIPLINE IS THE CHAPTER'S, and it is not the SAM's.
 * 10.4.5's "channel state and disabling a channel" says plainly that
 * suspend-and-resume is NOT SUPPORTED: a channel disabled mid-block and
 * re-enabled without reprogramming is not guaranteed to finish correctly.
 * The supported sequence is abort-and-restart - disable, reconfigure,
 * enable, in SEPARATE writes to CCR - and load() is exactly that
 * sequence. What this driver never offers is the unsupported one.
 *
 * A TRANSFER ERROR DISABLES THE CHANNEL IN HARDWARE (10.4.7) and EN
 * cannot be set again until TEIFx is cleared. enable(true) therefore
 * fails, visibly, on a channel whose error flag still stands - which is
 * the one place this driver's `false` means "the silicon refused", not
 * "the driver refused".
 */
template <uint8_t n, uint8_t ch>
class DmaChannel {
    static_assert(dma_present(n),
                  "brio DmaChannel: this device has no such DMA controller");
    static_assert(dma_channel_present(n, ch),
                  "brio DmaChannel: this controller has no such channel - they are "
                  "numbered from ONE (seven on DMA1 of the G0B1/G071 class, five on "
                  "DMA1 of the G031 class, five on DMA2)");

public:
    DmaChannel() = delete;

    static constexpr uint8_t controller = n;
    static constexpr uint8_t index = ch;
    /// The DMAMUX channel wired to this DMA channel (11.3.2).
    static constexpr uint8_t mux_channel = dmamux_channel_of(n, ch);

    static DMA_Channel_TypeDef& regs() {
        return *reinterpret_cast<DMA_Channel_TypeDef*>(dma_channel_base(n, ch));
    }

    static constexpr IRQn_Type irq() { return dma_channel_irq(n, ch); }

    // -- enable ------------------------------------------------------------

    static bool enabled() { return (regs().CCR & DMA_CCR_EN) != 0u; }

    /**
     * Set or clear CCR.EN in a write of its own (10.4.5: the disable and
     * the reconfiguration must be separate accesses).
     *
     * @return for `true`, whether the channel really came up: the silicon
     * refuses the bit while TEIFx stands (10.4.7). For `false`, always
     * true - a disable cannot fail, though what a mid-block one leaves in
     * CNDTR is not to be trusted.
     */
    static bool enable(bool on) {
        if (!on) {
            regs().CCR = regs().CCR & ~DMA_CCR_EN;
            return true;
        }
        regs().CCR = regs().CCR | DMA_CCR_EN;
        return enabled();
    }

    // -- configuration -----------------------------------------------------

    /// Write CCR's configuration fields. REFUSED while the channel is
    /// enabled: those fields are read-only then (10.6.3), so a store
    /// would be silently dropped. The interrupt enables are NOT touched
    /// here - arm() owns them, so a re-configuration cannot disarm a
    /// handler by omission.
    static bool configure(const DmaChannelConfig& c) {
        if (enabled() || !dma_channel_config_valid(c)) {
            return false;
        }
        const uint32_t armed = regs().CCR & (DMA_CCR_TCIE | DMA_CCR_HTIE | DMA_CCR_TEIE);
        uint32_t v = armed;
        if (c.direction == DmaDirection::memory_to_peripheral) {
            v |= DMA_CCR_DIR;
        }
        if (c.circular) {
            v |= DMA_CCR_CIRC;
        }
        if (c.memory_to_memory) {
            v |= DMA_CCR_MEM2MEM;
        }
        if (c.peripheral_increment) {
            v |= DMA_CCR_PINC;
        }
        if (c.memory_increment) {
            v |= DMA_CCR_MINC;
        }
        v |= static_cast<uint32_t>(c.peripheral_width) << DMA_CCR_PSIZE_Pos;
        v |= static_cast<uint32_t>(c.memory_width) << DMA_CCR_MSIZE_Pos;
        v |= static_cast<uint32_t>(c.priority) << DMA_CCR_PL_Pos;
        regs().CCR = v;
        return true;
    }

    static uint32_t control() { return regs().CCR; }

    /// Read back the two mode bits a caller may want to ASK about rather
    /// than remember - which is also how a test says "this stream really
    /// is not circular" without spelling a register bit.
    static bool circular() { return (regs().CCR & DMA_CCR_CIRC) != 0u; }
    static bool memory_to_memory() { return (regs().CCR & DMA_CCR_MEM2MEM) != 0u; }

    /// CNDTR. Writable only with the channel disabled (10.6.4), and a
    /// zero here serves nothing whatever the enable says.
    static bool set_count(uint16_t count) {
        if (enabled()) {
            return false;
        }
        regs().CNDTR = count;
        return true;
    }
    static uint16_t count() { return static_cast<uint16_t>(regs().CNDTR); }

    static void set_peripheral(volatile void* address) {
        regs().CPAR = reinterpret_cast<uint32_t>(address);
    }
    static void set_memory(volatile void* address) {
        regs().CMAR = reinterpret_cast<uint32_t>(address);
    }

    /**
     * The whole abort-and-restart sequence of 10.4.5, in the order that
     * chapter gives: disable, program, enable. Leaves the channel RUNNING.
     *
     * A standing transfer error is cleared first - not to hide it, but
     * because EN cannot be set while TEIFx stands and the caller has
     * already decided to start a new block. Whoever wants to know a block
     * died reads flags() before calling this, or counts faults() on an
     * engine.
     */
    static bool load(const DmaTransfer& t) {
        if (!dma_transfer_valid(t)) {
            return false;
        }
        (void)enable(false);
        clear(DmaFlag::all);
        if (!configure(t.config)) {
            return false;
        }
        set_peripheral(t.peripheral);
        set_memory(t.memory);
        if (!set_count(t.count)) {
            return false;
        }
        return enable(true);
    }

    /// Program a block without starting it - the DMAMUX's own channel
    /// configuration procedure (11.4.3) wants the DMA channel completely
    /// set up and NOT enabled before the multiplexer is written.
    static bool prepare(const DmaTransfer& t) {
        if (!dma_transfer_valid(t)) {
            return false;
        }
        (void)enable(false);
        clear(DmaFlag::all);
        if (!configure(t.config)) {
            return false;
        }
        set_peripheral(t.peripheral);
        set_memory(t.memory);
        return set_count(t.count);
    }

    // -- flags and interrupts ----------------------------------------------

    /// This channel's four bits, shifted down to DmaFlag's positions.
    static uint32_t flags() {
        return (Dma<n>::flags() >> Dma<n>::flag_shift(ch)) & 0xFu;
    }
    static bool flag(uint32_t mask) { return (flags() & mask) != 0u; }

    /**
     * Clear this channel's flags. THE GLOBAL BIT IS MASKED OUT and there
     * is no way to ask for it: ES0548 2.4.1 loses both a transfer error
     * and the automatic disable when a CGIFx write coincides with the
     * error, and the erratum's workaround is to clear the specific flags
     * instead - which this makes structural rather than advisory. Clearing
     * all three specific bits clears GIFx anyway (10.6.2).
     */
    static void clear(uint32_t mask) {
        Dma<n>::regs().IFCR = (mask & DmaFlag::all) << Dma<n>::flag_shift(ch);
    }

    /// CCR's three interrupt enables. Read-modify-write under a guard:
    /// the handler touches the same register.
    static void arm(uint32_t mask, bool on) {
        uint32_t bits = 0;
        if ((mask & DmaFlag::complete) != 0u) {
            bits |= DMA_CCR_TCIE;
        }
        if ((mask & DmaFlag::half) != 0u) {
            bits |= DMA_CCR_HTIE;
        }
        if ((mask & DmaFlag::error) != 0u) {
            bits |= DMA_CCR_TEIE;
        }
        InterruptGuard guard;
        regs().CCR = on ? (regs().CCR | bits) : (regs().CCR & ~bits);
    }

    static uint32_t armed() {
        const uint32_t c = regs().CCR;
        uint32_t mask = 0;
        if ((c & DMA_CCR_TCIE) != 0u) {
            mask |= DmaFlag::complete;
        }
        if ((c & DMA_CCR_HTIE) != 0u) {
            mask |= DmaFlag::half;
        }
        if ((c & DMA_CCR_TEIE) != 0u) {
            mask |= DmaFlag::error;
        }
        return mask;
    }

    /**
     * The channel's ISR BODY: read this channel's flags, clear exactly
     * those, hand them back. The app's handler for a shared vector calls
     * one of these per channel it owns - there is no "which channel
     * interrupted" register on this controller, only the flag word, so
     * asking each owner IS the dispatch (the samc's take_pending() has no
     * twin here).
     *
     * Only ARMED flags are reported and cleared: HT is set by hardware
     * whether or not HTIE is on, and a body that swallowed it would
     * consume a flag its owner is polling for.
     */
    [[gnu::always_inline]] static uint32_t isr() {
        const uint32_t pending = flags() & armed();
        if (pending != 0u) {
            clear(pending);
        }
        return pending;
    }

    // -- progress ----------------------------------------------------------

    /// Where the current block has got to. `done` is meaningful only
    /// against the length the caller programmed, which is why the engines
    /// keep it and this returns both numbers.
    static DmaProgress progress(uint16_t programmed) {
        const uint16_t remaining = count();
        return DmaProgress{remaining,
                           static_cast<uint16_t>(remaining > programmed
                                                     ? 0u
                                                     : programmed - remaining)};
    }

    /// Stop the channel and put its flags and interrupt enables back.
    /// What a mid-block disable leaves in CNDTR is not to be trusted
    /// (10.4.5) - which is why nothing here reports it.
    static void stop() {
        (void)enable(false);
        arm(DmaFlag::all, false);
        clear(DmaFlag::all);
    }
};

// ---- the multiplexer ----------------------------------------------------------

/// DMAMUX_CxCR.SPOL and RGxCR.GPOL (11.6.1, 11.6.4) - one enum, because
/// the two fields are the same three bits with the same meanings. `none`
/// is the reset value and detects nothing.
enum class DmaMuxEdge : uint8_t {
    none = 0,
    rising = 1,
    falling = 2,
    both = 3,
};

/// A synchronized multiplexer channel: hold the peripheral's request back
/// until an edge arrives on `input`, then let `requests` of them through
/// (11.4.4 - the register field is the count MINUS ONE, and this struct
/// is the count itself so nobody has to remember which).
struct DmaMuxSync {
    uint8_t input = 0;                     ///< SYNC_ID, table 57
    DmaMuxEdge edge = DmaMuxEdge::rising;  ///< SPOL
    uint8_t requests = 1;                  ///< NBREQ + 1
    bool generate_event = false;           ///< EGE
    bool overrun_interrupt = false;        ///< SOIE
};

constexpr bool dmamux_sync_valid(const DmaMuxSync& s) {
    // NBREQ is five bits, so the count runs 1..32; and a synchronization
    // that detects no edge would hold the request line for ever, which is
    // a configuration error and not a way to switch the feature off
    // (leaving `sync` unset is).
    return s.requests >= 1u && s.requests <= 32u && s.edge != DmaMuxEdge::none &&
           s.input < 24u;
}

/**
 * DmaMux: the request multiplexer, addressed by DMAMUX channel (0-based
 * as 11.6.1 numbers them; DmaChannel<n, ch>::mux_channel is the map).
 *
 * There is no enable and no clock of its own: 17.4.2 clocks the DMAMUX
 * with the DMA controllers, and a channel is "off" when its DMAREQ_ID is
 * zero.
 */
struct DmaMux {
    DmaMux() = delete;

    static constexpr uint8_t channels = dmamux_channels();
    static constexpr uint8_t generators = dmamux_generators();

    static volatile uint32_t& ccr(uint8_t x) {
        return reinterpret_cast<DMAMUX_Channel_TypeDef*>(dmamux_channel_base(x))->CCR;
    }

    static DMAMUX_ChannelStatus_TypeDef& status() {
        return *reinterpret_cast<DMAMUX_ChannelStatus_TypeDef*>(
            dmamux_channel_status_base());
    }

    /**
     * Point multiplexer channel `x` at request line `id` (table 55, as
     * the peripheral publishes it), with no synchronization at all.
     *
     * ONE WRITE, AND SPOL LEFT AT ZERO: ES0548 2.5.4 is about a CxCR
     * write that turns synchronization on while SPOL already holds an
     * edge, and its workaround is the invariant "SPOL is 00 whenever SE
     * is 0". This verb is what maintains that half of it.
     *
     * 11.4.4's caution is the caller's: the same non-null request id must
     * not be live on two channels at once.
     */
    static bool request(uint8_t x, uint8_t id) {
        if (x >= channels || id > 0x7Fu) {
            return false;
        }
        ccr(x) = static_cast<uint32_t>(id);
        return true;
    }

    /// Point channel `x` at `id` AND synchronize it. The whole word is
    /// written once, with SE and a non-zero SPOL together - ES0548
    /// 2.5.4's workaround, and the reason this is not a `synchronize()`
    /// that could be called after a `request()`.
    static bool request_synchronized(uint8_t x, uint8_t id, const DmaMuxSync& s) {
        if (x >= channels || id > 0x7Fu || !dmamux_sync_valid(s)) {
            return false;
        }
        uint32_t v = static_cast<uint32_t>(id);
        v |= static_cast<uint32_t>(s.input) << DMAMUX_CxCR_SYNC_ID_Pos;
        v |= static_cast<uint32_t>(s.requests - 1u) << DMAMUX_CxCR_NBREQ_Pos;
        v |= static_cast<uint32_t>(s.edge) << DMAMUX_CxCR_SPOL_Pos;
        v |= DMAMUX_CxCR_SE;
        if (s.generate_event) {
            v |= DMAMUX_CxCR_EGE;
        }
        if (s.overrun_interrupt) {
            v |= DMAMUX_CxCR_SOIE;
        }
        ccr(x) = v;
        return true;
    }

    /**
     * Count requests and emit a channel event every `requests` of them,
     * WITHOUT synchronization (11.4.4's second figure: EGE with SE clear).
     * The event is a one-AHB-cycle pulse on dmamux_evtx, which table 56
     * offers back as trigger inputs 16..19 - so this is how one channel
     * paces another with no peripheral in between.
     */
    static bool request_counted(uint8_t x, uint8_t id, uint8_t requests,
                                bool overrun_interrupt = false) {
        if (x >= channels || id > 0x7Fu || requests < 1u || requests > 32u) {
            return false;
        }
        uint32_t v = static_cast<uint32_t>(id);
        v |= static_cast<uint32_t>(requests - 1u) << DMAMUX_CxCR_NBREQ_Pos;
        v |= DMAMUX_CxCR_EGE;
        if (overrun_interrupt) {
            v |= DMAMUX_CxCR_SOIE;
        }
        ccr(x) = v;
        return true;
    }

    static uint8_t request(uint8_t x) {
        return x < channels ? static_cast<uint8_t>(ccr(x) & DMAMUX_CxCR_DMAREQ_ID_Msk)
                            : dma_request_none;
    }

    /// Put a channel back: no request, no synchronization, SPOL at zero.
    static bool release(uint8_t x) {
        if (x >= channels) {
            return false;
        }
        ccr(x) = 0;
        return true;
    }

    /// SOFx, one bit per multiplexer channel (11.6.2): a synchronization
    /// event arrived while the previous one's requests were still being
    /// served.
    static uint32_t overruns() { return status().CSR; }
    static bool overrun(uint8_t x) {
        return x < channels && (status().CSR & (1u << x)) != 0u;
    }
    /// ES0548 2.5.1: a SOFx raised by ANOTHER channel during this very
    /// write is lost, with no workaround. Two synchronized channels that
    /// can both overrun are therefore a configuration to avoid, not a
    /// case this verb can make safe.
    static void clear_overrun(uint32_t mask) { status().CFR = mask; }
};

/**
 * DmaMuxGenerator<x>: one of the four request-generator channels
 * (11.4.5). It turns an EDGE on a trigger input - table 56: EXTI lines
 * 0..15, the four DMAMUX events, the low-power timers' outputs, TIM14's
 * output compare - into `requests` DMA requests, which a multiplexer
 * channel then routes like any peripheral's.
 *
 * Its output is request line x + 1 (table 55's first four rows), which is
 * what `request_id` is for: `DmaMux::request(mux, Gen::request_id)`.
 */
template <uint8_t x>
struct DmaMuxGenerator {
    static_assert(x < dmamux_generators(),
                  "brio DmaMuxGenerator: this device has four request-generator "
                  "channels, numbered 0..3");

    DmaMuxGenerator() = delete;

    static constexpr uint8_t index = x;
    /// Table 55: multiplexer inputs 1..4 are dmamux_req_gen0..3.
    static constexpr uint8_t request_id = static_cast<uint8_t>(x + 1u);

    static volatile uint32_t& rgcr() {
        return reinterpret_cast<DMAMUX_RequestGen_TypeDef*>(dmamux_generator_base(x))
            ->RGCR;
    }

    static DMAMUX_RequestGenStatus_TypeDef& status() {
        return *reinterpret_cast<DMAMUX_RequestGenStatus_TypeDef*>(
            dmamux_generator_status_base());
    }

    /**
     * Select the trigger and how many requests one edge is worth.
     * REFUSED while the generator is enabled: 11.4.5 says GNBREQ may be
     * written only with GE clear, and warns in the same breath that there
     * is NO HARDWARE WRITE PROTECTION - so the refusal has to be the
     * driver's, or the field takes a value the silicon then half-uses.
     *
     * ES0548 2.5.2 is the caller's obligation here: with more than two
     * requests per trigger, a trigger arriving at the very end of the
     * previous batch loses its overrun flag AND makes the next batch one
     * single request. Only the application knows its trigger period.
     */
    static bool configure(uint8_t trigger, DmaMuxEdge edge, uint8_t requests = 1) {
        if (enabled() || trigger >= 24u || requests < 1u || requests > 32u ||
            edge == DmaMuxEdge::none) {
            return false;
        }
        uint32_t v = static_cast<uint32_t>(trigger);
        v |= static_cast<uint32_t>(requests - 1u) << DMAMUX_RGxCR_GNBREQ_Pos;
        v |= static_cast<uint32_t>(edge) << DMAMUX_RGxCR_GPOL_Pos;
        rgcr() = v;
        return true;
    }

    static void enable(bool on) {
        rgcr() = on ? (rgcr() | DMAMUX_RGxCR_GE) : (rgcr() & ~DMAMUX_RGxCR_GE);
    }
    static bool enabled() { return (rgcr() & DMAMUX_RGxCR_GE) != 0u; }

    static void overrun_interrupt(bool on) {
        rgcr() = on ? (rgcr() | DMAMUX_RGxCR_OIE) : (rgcr() & ~DMAMUX_RGxCR_OIE);
    }

    /// OFx (11.6.5): a trigger arrived before the previous batch was
    /// served.
    static bool overrun() { return (status().RGSR & (1u << x)) != 0u; }
    /// ES0548 2.5.3, the twin of 2.5.1 on this side: an OFx raised by
    /// another generator during this write is lost. No workaround.
    static void clear_overrun() { status().RGCFR = 1u << x; }

    static void release() {
        enable(false);
        rgcr() = 0;
    }
};

/// Table 56: the DMAMUX's trigger inputs 0..15 ARE the EXTI's lines 0..15,
/// one for one. Spelled as a verb rather than left to arithmetic because
/// the identity is a table's and not a law's - and because the EXTI's own
/// SWIER makes trigger input `line` reachable from software, which is what
/// lets a request generator be exercised with no wire (test_stm32_dma
/// letter f).
constexpr uint8_t dmamux_trigger_exti(uint8_t line) { return line; }

/// Table 56 again: trigger inputs 16..19 are the four multiplexer channel
/// events (dmamux_evt0..3), i.e. what DmaMux::request_counted() emits.
constexpr uint8_t dmamux_trigger_event(uint8_t event) {
    return static_cast<uint8_t>(16u + event);
}

// ---- the byte-transport engines ------------------------------------------------

/**
 * DmaTxEngine<n, ch, Elem> - "drain a run of memory into a peripheral's
 * data register".
 *
 * The engine owns a channel and nothing else: the peripheral's data
 * address and its DMAMUX request id are handed in at arm() time by
 * whoever owns the peripheral, so this type knows nothing about USARTs
 * and would serve an SPI or a DAC unchanged.
 *
 * `Elem` IS THE ACCESS WIDTH (dma_width_of): the default byte engine is
 * what a USART wants; a converter's data register wants uint16_t and
 * nothing else in the engine changes.
 *
 * THERE IS NO KICK ON THIS CONTROLLER and the absence is a fact, not an
 * omission. The SAM's DMAC latches a trigger on the RISE of a request
 * level, so a channel armed while the peripheral's request was ALREADY
 * standing waited for an edge that had been and gone - hence kick(). Here
 * 10.4.3's handshake is level-driven: the controller looks at the request
 * line, not at its edge, so enabling the channel with TXE already set
 * moves the first byte immediately. Measured (test_stm32_dma letter h),
 * because it is exactly the kind of claim that costs a dead transmitter
 * when it is wrong.
 */
template <uint8_t n, uint8_t ch, typename Elem = uint8_t>
class DmaTxEngine {
    using Channel = DmaChannel<n, ch>;

public:
    DmaTxEngine() = delete;

    static constexpr bool present = true;
    static constexpr uint8_t controller = n;
    static constexpr uint8_t channel = ch;
    static constexpr DmaWidth width = dma_width_of<Elem>();
    using element = Elem;

    /// The bits service() reports, published BY THE ENGINE so a driver
    /// that owns one never has to name DmaFlag - or include this file's
    /// channel types - to read the answer.
    static constexpr uint8_t flag_complete = DmaFlag::complete;
    static constexpr uint8_t flag_half = DmaFlag::half;
    static constexpr uint8_t flag_error = DmaFlag::error;

    /**
     * The channel's ISR BODY folded into the engine: read this channel's
     * own armed flags, clear exactly those, hand them back. Call it from
     * whichever vector the channel reports on (three vectors serve twelve
     * channels here) - it answers for its own channel and returns 0
     * otherwise, so it is safe on a shared line.
     *
     * IT ACTS ON NOTHING. What a completion or an error MEANS is the
     * owner's to decide - complete(), abandon(), fail() are the verbs -
     * because only the owner can see its peripheral's state.
     */
    [[gnu::always_inline]] static uint8_t service() {
        return static_cast<uint8_t>(Channel::isr());
    }

    /// Claim the channel for this peripheral: `data` is the register the
    /// run is poured into, `request` the peripheral's own DMAMUX request
    /// id.
    static void arm(volatile void* data, uint8_t request,
                    DmaPriority priority = DmaPriority::low) {
        data_ = data;
        request_ = request;
        priority_ = priority;
        claim();
        Nvic::enable(Channel::irq());
    }

    /// Start moving `length` elements from `buffer`. The buffer is the
    /// CALLER'S and must stay put until complete() reports the block.
    static bool start(const Elem* buffer, uint16_t length) {
        if (busy_ || buffer == nullptr || length == 0u) {
            return false;
        }
        in_flight_ = length;
        busy_ = true;
        if (!Channel::load(DmaTransfer{
                .peripheral = data_,
                .memory = const_cast<Elem*>(buffer),
                .count = length,
                .config = {.direction = DmaDirection::memory_to_peripheral,
                           .circular = false,
                           .memory_to_memory = false,
                           .peripheral_increment = false,
                           .memory_increment = true,
                           .peripheral_width = width,
                           .memory_width = width,
                           .priority = priority_},
            })) {
            busy_ = false;
            in_flight_ = 0;
            return false;
        }
        return true;
    }

    /// The block ended - called from the channel's handler when its
    /// completion flag is up.
    /// @return the elements the block carried, so the owner can release
    /// exactly that much of its ring.
    static uint16_t complete() {
        if (!busy_) {
            return 0;
        }
        busy_ = false;
        const uint16_t moved = in_flight_;
        in_flight_ = 0;
        return moved;
    }

    static bool busy() { return busy_; }
    static uint16_t in_flight() { return busy_ ? in_flight_ : 0u; }
    static DmaProgress progress() { return Channel::progress(in_flight_); }

    /**
     * Throw away a block the silicon has stopped running and free the
     * channel - the SAM's caller-decides doctrine, kept for the same
     * reason: only the peripheral's owner can read the flags that make
     * "dead" a fact rather than a timeout. What is lost is the untransmitted
     * tail, and it is counted rather than papered over.
     */
    static bool abandon() {
        if (!busy_) {
            return false;
        }
        ++faults_;
        busy_ = false;
        in_flight_ = 0;
        claim();
        return true;
    }

    static uint32_t faults() { return faults_; }
    static void clear_faults() { faults_ = 0; }

    static void stop() {
        Channel::stop();
        busy_ = false;
        in_flight_ = 0;
    }

private:
    /// Take the channel from whatever state it is in: stop it, point the
    /// multiplexer at this peripheral, arm the two flags that matter.
    /// arm() and abandon() are the same act with a different reason.
    static void claim() {
        Channel::stop();
        (void)DmaMux::request(Channel::mux_channel, request_);
        Channel::arm(DmaFlag::complete | DmaFlag::error, true);
    }

    static inline volatile void* data_ = nullptr;
    static inline uint8_t request_ = dma_request_none;
    static inline DmaPriority priority_ = DmaPriority::low;
    static inline uint16_t in_flight_ = 0;
    static inline uint32_t faults_ = 0;
    static inline volatile bool busy_ = false;
};

/**
 * DmaRxEngine<n, ch, Elem> - "fill a run of memory from a peripheral's
 * data register".
 *
 * The asymmetry that shapes it: a receive block completes only when the
 * buffer FILLS, which on an idle line may be never. So the owner does not
 * wait for a completion - it ASKS, with take(), which reports what has
 * arrived since the last question.
 *
 * AND ON THIS SILICON ASKING IS FREE. CNDTR is a live counter the
 * controller decrements and software may read at any time (10.6.4), so
 * take() is one register read and a subtraction. The SAM had to SUSPEND
 * the channel, read a write-back, and validate it against an erratum that
 * could corrupt it - a whole ceremony this controller simply does not
 * need. The PACING is still the owner's (a kernel TimeEvent every few
 * ticks), because the latency of asking late is the owner's to choose;
 * what is gone is the cost of asking.
 */
template <uint8_t n, uint8_t ch, typename Elem = uint8_t>
class DmaRxEngine {
    using Channel = DmaChannel<n, ch>;

public:
    DmaRxEngine() = delete;

    static constexpr bool present = true;
    static constexpr uint8_t controller = n;
    static constexpr uint8_t channel = ch;
    static constexpr DmaWidth width = dma_width_of<Elem>();
    using element = Elem;

    /// The bits service() reports, published BY THE ENGINE so a driver
    /// that owns one never has to name DmaFlag - or include this file's
    /// channel types - to read the answer.
    static constexpr uint8_t flag_complete = DmaFlag::complete;
    static constexpr uint8_t flag_half = DmaFlag::half;
    static constexpr uint8_t flag_error = DmaFlag::error;

    /**
     * The channel's ISR BODY folded into the engine: read this channel's
     * own armed flags, clear exactly those, hand them back. Call it from
     * whichever vector the channel reports on (three vectors serve twelve
     * channels here) - it answers for its own channel and returns 0
     * otherwise, so it is safe on a shared line.
     *
     * IT ACTS ON NOTHING. What a completion or an error MEANS is the
     * owner's to decide - complete(), abandon(), fail() are the verbs -
     * because only the owner can see its peripheral's state.
     */
    [[gnu::always_inline]] static uint8_t service() {
        return static_cast<uint8_t>(Channel::isr());
    }

    static void arm(volatile void* data, uint8_t request,
                    DmaPriority priority = DmaPriority::low) {
        data_ = data;
        request_ = request;
        priority_ = priority;
        claim();
        Nvic::enable(Channel::irq());
    }

    /// True while the channel is not running a block at all - it filled
    /// up, it errored, or it was never started. The owner hands it a new
    /// run then, whatever the arithmetic says.
    static bool idle() { return !Channel::enabled(); }

    /// Point the channel at a run of free memory and start filling it.
    static bool start(Elem* buffer, uint16_t length) {
        if (buffer == nullptr || length == 0u) {
            return false;
        }
        capacity_ = length;
        taken_ = 0;
        return Channel::load(DmaTransfer{
            .peripheral = data_,
            .memory = buffer,
            .count = length,
            .config = {.direction = DmaDirection::peripheral_to_memory,
                       .circular = false,
                       .memory_to_memory = false,
                       .peripheral_increment = false,
                       .memory_increment = true,
                       .peripheral_width = width,
                       .memory_width = width,
                       .priority = priority_},
        });
    }

    /**
     * How many elements have arrived since the last take(). One CNDTR
     * read; nothing is suspended and nothing can be refused, so the
     * return is a plain number and not the SAM's optional.
     */
    static uint16_t take() {
        if (capacity_ == 0u) {
            return 0;
        }
        const uint16_t remaining = Channel::count();
        const uint16_t filled = remaining > capacity_
                                    ? capacity_
                                    : static_cast<uint16_t>(capacity_ - remaining);
        if (filled <= taken_) {
            return 0;
        }
        const uint16_t fresh = static_cast<uint16_t>(filled - taken_);
        taken_ = filled;
        return fresh;
    }

    /// True once the run is full: the owner must hand over a new one or
    /// the peripheral piles its own losses up.
    static bool full() { return capacity_ != 0u && taken_ >= capacity_; }
    static uint16_t capacity() { return capacity_; }
    static uint16_t taken() { return taken_; }

    static bool abandon() {
        ++faults_;
        capacity_ = 0;
        taken_ = 0;
        claim();
        return true;
    }

    static uint32_t faults() { return faults_; }
    static void clear_faults() { faults_ = 0; }

    static void stop() {
        Channel::stop();
        capacity_ = 0;
        taken_ = 0;
    }

private:
    static void claim() {
        Channel::stop();
        (void)DmaMux::request(Channel::mux_channel, request_);
        Channel::arm(DmaFlag::complete | DmaFlag::error, true);
    }

    static inline volatile void* data_ = nullptr;
    static inline uint8_t request_ = dma_request_none;
    static inline DmaPriority priority_ = DmaPriority::low;
    static inline uint16_t capacity_ = 0;
    static inline uint16_t taken_ = 0;
    static inline uint32_t faults_ = 0;
};

// ---- the block-stream engines --------------------------------------------------

/**
 * DmaLoopEngine<n, ch, Elem> - "play one table into a peripheral, for
 * ever". util/block_stream.hpp's BlockPlayer on this silicon.
 *
 * THIS IS WHERE THE HARDWARE CIRCULAR MODE BELONGS. CCR.CIRC makes the
 * controller reload CNDTR and both current address registers at the end of
 * every block (10.4.5), so the table repeats with NO CPU in the path at
 * all. The SAM's engine had to re-arm from the completion interrupt - one
 * interrupt per lap, and a window at every lap boundary in which the
 * peripheral was unserved. Neither exists here: the interrupt at the wrap
 * only COUNTS the lap, and a program that never enables it still plays the
 * table for ever.
 *
 * Which is also why nothing about a lap can be lost: laps() is a count
 * kept by software and an interrupt that is late (or masked) delays the
 * COUNT, never the stream. `faults()` counts transfer errors and the
 * abandons that follow them, and on this controller a transfer error also
 * disables the channel in hardware (10.4.7) - so a fault really is the end
 * of the stream, which running() then reports honestly.
 *
 * THE FIRST BEAT NEEDS NO KICK: 10.4.3's request is a level, so a table
 * armed while the peripheral is already asking is served at once.
 */
template <uint8_t n, uint8_t ch, typename Elem = uint8_t>
class DmaLoopEngine {
    using Channel = DmaChannel<n, ch>;

public:
    DmaLoopEngine() = delete;

    static constexpr bool present = true;
    static constexpr uint8_t controller = n;
    static constexpr uint8_t channel = ch;
    static constexpr DmaWidth width = dma_width_of<Elem>();
    using element = Elem;

    /// The bits service() reports, published BY THE ENGINE so a driver
    /// that owns one never has to name DmaFlag - or include this file's
    /// channel types - to read the answer.
    static constexpr uint8_t flag_complete = DmaFlag::complete;
    static constexpr uint8_t flag_half = DmaFlag::half;
    static constexpr uint8_t flag_error = DmaFlag::error;

    /**
     * The channel's ISR BODY folded into the engine: read this channel's
     * own armed flags, clear exactly those, hand them back. Call it from
     * whichever vector the channel reports on (three vectors serve twelve
     * channels here) - it answers for its own channel and returns 0
     * otherwise, so it is safe on a shared line.
     *
     * IT ACTS ON NOTHING. What a completion or an error MEANS is the
     * owner's to decide - complete(), abandon(), fail() are the verbs -
     * because only the owner can see its peripheral's state.
     */
    [[gnu::always_inline]] static uint8_t service() {
        return static_cast<uint8_t>(Channel::isr());
    }

    /// Claim the channel: `data` is the register the table is played
    /// into, `request` the peripheral's own DMAMUX request id.
    static void arm(volatile void* data, uint8_t request,
                    DmaPriority priority = DmaPriority::low) {
        data_ = data;
        request_ = request;
        priority_ = priority;
        claim();
        Nvic::enable(Channel::irq());
    }

    /**
     * Begin playing `table`, and keep playing it until stop().
     *
     * The table is the CALLER'S and must outlive the stream; nothing is
     * copied. THE POINTER IS `const volatile` ON PURPOSE - the controller
     * reads this memory and the compiler cannot see it happen, so a table
     * the program fills and then hands over is exactly the shape gcc has
     * been caught optimizing (a zeroing store sunk past a transfer, the
     * SAM campaign's own lesson). A plain array converts for free.
     */
    static bool start(const volatile Elem* table, uint16_t length) {
        if (table == nullptr || length == 0u) {
            return false;
        }
        table_ = table;
        length_ = length;
        laps_ = 0;
        return launch();
    }

    /**
     * The table wrapped - called from the channel's handler on the
     * completion flag. Counts the lap. IT DOES NOT RE-ARM ANYTHING: the
     * controller already reloaded itself, and that is the whole point of
     * this engine.
     *
     * @return the elements the finished lap carried, so an owner that
     * wants to know the stream is alive has a number rather than a
     * promise.
     */
    static uint16_t complete() {
        if (!running_) {
            return 0;
        }
        laps_ = laps_ + 1u;
        return length_;
    }

    /// A transfer error - called from the handler on the error flag. The
    /// silicon has already disabled the channel (10.4.7); this records it
    /// and tells the truth about running().
    static void fail() {
        faults_ = faults_ + 1u;
        running_ = false;
    }

    static uint32_t laps() { return laps_; }
    static uint32_t faults() { return faults_; }
    static void clear_faults() { faults_ = 0; }
    static bool running() { return running_ && Channel::enabled(); }
    static uint16_t length() { return length_; }

    /// How far into the CURRENT lap the controller has got - one register
    /// read, never refused.
    static DmaProgress progress() { return Channel::progress(length_); }

    /// Throw away a lap the silicon has stopped running and start a fresh
    /// one from the table's beginning, so the stream restarts IN PHASE and
    /// the loss shows as a gap rather than a permanent offset.
    static bool abandon() {
        if (table_ == nullptr) {
            return false;
        }
        faults_ = faults_ + 1u;
        claim();
        return launch();
    }

    /// Stop at the end of nothing - immediately. What the current lap had
    /// already moved is in the peripheral; the rest is not sent.
    static void stop() {
        Channel::stop();
        running_ = false;
    }

private:
    static bool launch() {
        if (table_ == nullptr || length_ == 0u) {
            return false;
        }
        running_ = Channel::load(DmaTransfer{
            .peripheral = data_,
            .memory = const_cast<Elem*>(table_),
            .count = length_,
            .config = {.direction = DmaDirection::memory_to_peripheral,
                       .circular = true,
                       .memory_to_memory = false,
                       .peripheral_increment = false,
                       .memory_increment = true,
                       .peripheral_width = width,
                       .memory_width = width,
                       .priority = priority_},
        });
        return running_;
    }

    static void claim() {
        Channel::stop();
        (void)DmaMux::request(Channel::mux_channel, request_);
        Channel::arm(DmaFlag::complete | DmaFlag::error, true);
    }

    static inline volatile void* data_ = nullptr;
    static inline const volatile Elem* table_ = nullptr;
    static inline uint8_t request_ = dma_request_none;
    static inline DmaPriority priority_ = DmaPriority::low;
    static inline uint16_t length_ = 0;
    // laps_ is written in the handler and READ FROM THREAD CONTEXT,
    // typically in a polling loop - the ticker's lesson (gcc -Os deleted a
    // bare polling loop once), so the load must be a volatile one.
    // faults_ is thread-written wherever abandon() is the owner's verb,
    // and handler-written on fail(); it is volatile for the same reason.
    static inline volatile uint32_t laps_ = 0;
    static inline volatile uint32_t faults_ = 0;
    static inline volatile bool running_ = false;
};

/**
 * DmaPingPongEngine<n, ch, Elem> - "fill one caller-owned buffer while
 * the caller drains the other". util/block_stream.hpp's BlockSource on
 * this silicon.
 *
 * THE ACCOUNTING IS THE API, and it is the contract's, unchanged:
 *
 *   laps()      buffers filled and handed over, since start()
 *   overruns()  times the engine had NO free buffer to start the next
 *               block in, i.e. the caller still held both
 *   stalled()   whether it is in that state right now
 *
 * ON AN OVERRUN THE ENGINE SKIPS THE LAP - it starts no block at all -
 * rather than write into the buffer the caller is reading. That trades
 * SAMPLES for INTEGRITY: everything handed over is a complete, untorn
 * block. release() restarts it. What is NOT counted, said plainly, is HOW
 * MANY samples the stall cost: a stopped channel moves nothing, and the
 * arrivals that went unserved are the PERIPHERAL's to report (a USART's
 * ORE, a converter's overrun), never this engine's to invent.
 *
 * WHY THIS ENGINE IS NOT CIRCULAR, which is the campaign's one real
 * finding about the contract. Circular mode with the half-transfer flag
 * looks like a free ping-pong: one buffer, two halves, HT and TC as the
 * two edges, no re-arm at all. It cannot honour the paragraph above. A
 * circular channel never stops, so when the caller still holds the half
 * the controller is about to write, the only moment software could
 * intervene is the interrupt AFTER the edge - and by then the controller
 * has already begun writing that half. The tear happens while the
 * software is deciding not to allow it, and it is invisible: nothing in
 * the block says which of its elements are this lap's and which are the
 * next one's. test_stm32_dma letter g measures exactly that race (how many
 * elements land in the held half before a handler can disable the
 * channel), which is the evidence behind this class using a NON-circular
 * channel: it stops itself at the end of every block, so the swap into the
 * other buffer happens with the channel idle and the guarantee is
 * structural rather than a hope about interrupt latency.
 *
 * WHAT THE NON-CIRCULAR CHOICE COSTS is the re-arm window - between the
 * completion and the handler's next start(), the peripheral is unserved.
 * On this controller that is much softer than it sounds: 10.4.3's request
 * is a LEVEL the peripheral holds until it is acknowledged, so a request
 * raised in the window is served late rather than lost. What can still be
 * lost is a SECOND arrival inside one window, and that loss is the
 * peripheral's own overrun flag to report - which is exactly where the
 * contract says it belongs.
 *
 * BUFFER OWNERSHIP is strictly alternating: three counters (which buffer
 * the engine fills next, which the caller drains next, how many are
 * pending) make "the buffer the engine needs is the one the caller holds"
 * a fact of the arithmetic rather than a comparison of pointers.
 */
template <uint8_t n, uint8_t ch, typename Elem = uint8_t>
class DmaPingPongEngine {
    using Channel = DmaChannel<n, ch>;

public:
    DmaPingPongEngine() = delete;

    static constexpr bool present = true;
    static constexpr uint8_t controller = n;
    static constexpr uint8_t channel = ch;
    static constexpr DmaWidth width = dma_width_of<Elem>();
    using element = Elem;

    /// The bits service() reports, published BY THE ENGINE so a driver
    /// that owns one never has to name DmaFlag - or include this file's
    /// channel types - to read the answer.
    static constexpr uint8_t flag_complete = DmaFlag::complete;
    static constexpr uint8_t flag_half = DmaFlag::half;
    static constexpr uint8_t flag_error = DmaFlag::error;

    /**
     * The channel's ISR BODY folded into the engine: read this channel's
     * own armed flags, clear exactly those, hand them back. Call it from
     * whichever vector the channel reports on (three vectors serve twelve
     * channels here) - it answers for its own channel and returns 0
     * otherwise, so it is safe on a shared line.
     *
     * IT ACTS ON NOTHING. What a completion or an error MEANS is the
     * owner's to decide - complete(), abandon(), fail() are the verbs -
     * because only the owner can see its peripheral's state.
     */
    [[gnu::always_inline]] static uint8_t service() {
        return static_cast<uint8_t>(Channel::isr());
    }

    /// Claim the channel: `data` is the peripheral's data register,
    /// `request` its own DMAMUX request id.
    static void arm(volatile void* data, uint8_t request,
                    DmaPriority priority = DmaPriority::low) {
        data_ = data;
        request_ = request;
        priority_ = priority;
        claim();
        Nvic::enable(Channel::irq());
    }

    /**
     * Begin streaming into `first`, with `second` as the buffer the next
     * block will use. Both are the CALLER'S, both must hold `length`
     * elements, and both must outlive the stream.
     *
     * `volatile` because the controller WRITES these buffers and the
     * compiler sees nothing: a caller reading a drained buffer through a
     * plain pointer would be reading what gcc thinks is there. ready()
     * hands back a volatile pointer for the same reason.
     */
    static bool start(volatile Elem* first, volatile Elem* second, uint16_t length) {
        if (first == nullptr || second == nullptr || first == second || length == 0u) {
            return false;
        }
        buffer_[0] = first;
        buffer_[1] = second;
        length_ = length;
        fill_ = 0;
        drain_ = 0;
        pending_ = 0;
        laps_ = 0;
        overruns_ = 0;
        stalled_ = false;
        return launch();
    }

    /**
     * The block filled - called from the channel's handler on the
     * completion flag. Hands the buffer to the caller and starts the next
     * block in the other one, or counts an overrun and stalls.
     *
     * @return the elements the finished block carried, or zero when
     * nothing was running.
     */
    static uint16_t complete() {
        if (!running_) {
            return 0;
        }
        laps_ = laps_ + 1u;
        pending_ = static_cast<uint8_t>(pending_ + 1u);
        fill_ = static_cast<uint8_t>(fill_ ^ 1u);
        if (pending_ >= 2u) {
            // The buffer the engine needs next is the one the caller has
            // not released. Skip the lap rather than write into it.
            overruns_ = overruns_ + 1u;
            stalled_ = true;
            running_ = false;
            return length_;
        }
        if (!launch()) {
            running_ = false;
        }
        return length_;
    }

    /// A transfer error - called from the handler on the error flag.
    static void fail() {
        faults_ = faults_ + 1u;
        running_ = false;
    }

    /// The buffer that is full and waiting for the caller, or nullptr.
    /// Valid until release() is called for it and not one element longer.
    static volatile Elem* ready() {
        return pending_ != 0u ? buffer_[drain_] : nullptr;
    }
    /// How many elements the ready buffer holds - always the whole block,
    /// because a buffer is handed over only when it is full.
    static uint16_t ready_length() { return pending_ != 0u ? length_ : 0u; }

    /**
     * Hand the ready buffer back. Restarts a stalled stream, which is the
     * one place this verb does more than bookkeeping - and why it holds a
     * critical section: complete() runs in the DMA handler and touches the
     * same three counters.
     */
    static bool release() {
        typename Stm32Platform::CriticalSection cs;
        if (pending_ == 0u) {
            return false;
        }
        pending_ = static_cast<uint8_t>(pending_ - 1u);
        drain_ = static_cast<uint8_t>(drain_ ^ 1u);
        if (stalled_) {
            stalled_ = false;
            if (!launch()) {
                return false;
            }
        }
        return true;
    }

    static uint32_t laps() { return laps_; }
    /// Times the engine found both buffers held by the caller. See the
    /// class comment on what this does and does NOT count.
    static uint32_t overruns() { return overruns_; }
    static bool stalled() { return stalled_; }
    static bool running() { return running_; }
    static uint16_t length() { return length_; }
    /// Buffers filled and not yet released: 0, 1, or 2 (2 = stalled).
    static uint8_t pending() { return pending_; }

    /// How far into the CURRENT block the controller has got - one
    /// register read, never refused.
    static DmaProgress progress() { return Channel::progress(length_); }

    /**
     * Throw away a block the silicon has stopped running and start a
     * fresh one in the same buffer. The partly filled buffer is NOT handed
     * over: a torn block is what this engine exists not to produce, so the
     * whole thing is discarded and counted.
     *
     * A STALLED STREAM IS NOT A DEAD ONE and this refuses it without
     * counting anything: while the engine waits for a release there is no
     * block in flight, so a fault counted there would be one that never
     * happened. release() is the verb for that state.
     */
    static bool abandon() {
        if (stalled_ || !running_) {
            return false;
        }
        faults_ = faults_ + 1u;
        claim();
        return launch();
    }

    static uint32_t faults() { return faults_; }
    static void clear_faults() { faults_ = 0; }

    static void stop() {
        Channel::stop();
        running_ = false;
        stalled_ = false;
        pending_ = 0;
        length_ = 0;
    }

private:
    static bool launch() {
        if (buffer_[fill_] == nullptr || length_ == 0u) {
            return false;
        }
        running_ = Channel::load(DmaTransfer{
            .peripheral = data_,
            .memory = const_cast<Elem*>(buffer_[fill_]),
            .count = length_,
            .config = {.direction = DmaDirection::peripheral_to_memory,
                       .circular = false,
                       .memory_to_memory = false,
                       .peripheral_increment = false,
                       .memory_increment = true,
                       .peripheral_width = width,
                       .memory_width = width,
                       .priority = priority_},
        });
        return running_;
    }

    static void claim() {
        Channel::stop();
        (void)DmaMux::request(Channel::mux_channel, request_);
        Channel::arm(DmaFlag::complete | DmaFlag::error, true);
    }

    static inline volatile void* data_ = nullptr;
    static inline volatile Elem* buffer_[2] = {nullptr, nullptr};
    static inline uint8_t request_ = dma_request_none;
    static inline DmaPriority priority_ = DmaPriority::low;
    static inline uint16_t length_ = 0;
    // laps_ and overruns_ are handler-written and thread-read, typically
    // in a polling loop: volatile for the ticker's reason.
    static inline volatile uint32_t laps_ = 0;
    static inline volatile uint32_t overruns_ = 0;
    static inline volatile uint32_t faults_ = 0;
    static inline volatile uint8_t fill_ = 0;
    static inline volatile uint8_t drain_ = 0;
    static inline volatile uint8_t pending_ = 0;
    static inline volatile bool stalled_ = false;
    static inline volatile bool running_ = false;
};

} // namespace brio

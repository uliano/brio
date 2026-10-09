/*
 * dma.hpp
 *
 * The RP2040's DMA controller (datasheet 2.5): twelve channels over one
 * read master and one write master, each channel four live registers
 * - READ_ADDR, WRITE_ADDR, TRANS_COUNT, CTRL - aliased four times so
 * that any one of them can be the TRIGGER (2.5.2.1), a credit-based
 * data request per channel that keeps as many transfers in flight as
 * the peripheral has room for (2.5.3.2), two system interrupt lines
 * each with its own enable and status bank (2.5.4), chaining between
 * channels, address wrapping, four pacing timers, a passive checksum
 * sniffer, and an abort register with erratum E13 attached.
 *
 *  Dma            the block: out of the reset controller once, the raw
 *                 interrupt status, the channel count, a trigger or an
 *                 abort over several channels
 *  DmaChannel<ch> one channel, 0..11: a transfer prepared or loaded
 *                 (loaded = triggered), the live count, the bus errors,
 *                 the route to one of the two lines, the abort with
 *                 E13's workaround, the debug registers
 *  DmaLine<n>     one of the two interrupt lines: its NVIC number, the
 *                 masked status, the force bits - a line belongs to the
 *                 CORE whose kernel serves it (rule 2 of the two-kernel
 *                 model), line 0 to core 0 by convention
 *  DmaTimer<n>    a pacing timer (2.5.5.1): X/Y of clk_sys as a request
 *  DmaSniffer     the checksum sniffer (2.5.5.2) over one channel's data
 *  DmaTxEngine<ch, Elem, line> / DmaRxEngine<ch, Elem, line>
 *                 the two engines util-level transports name in their
 *                 slots, the other strata's surface: a transmit engine
 *                 pours a caller-owned run into one peripheral register
 *                 and reports what the block carried when it completes;
 *                 a receive engine fills a caller-owned run and answers
 *                 how many items have arrived since it was last asked -
 *                 here from TRANS_COUNT, which is erratum E12's rule.
 *                 Configured ONCE at arm(), two stores a block, the beat
 *                 per block from the span it is handed (the engines'
 *                 section says why each of those is this chapter's)
 *  DmaCopyEngine<ch, line>
 *                 memory to memory on one channel: a copy, and a fill
 *                 from one cell, the beat the element type
 *
 * WHAT DIFFERS FROM THE OTHER FAMILIES' CONTROLLERS, stated once:
 *  - the request is a FIELD, not a channel: any channel takes any DREQ
 *    (table 119), so an engine is named by a free channel and told its
 *    peripheral's request in arm();
 *  - the trigger is a WRITE: a channel starts when its trigger alias is
 *    written non-zero with EN set, or when another channel chains to it,
 *    or through MULTI_CHAN_TRIGGER; a trigger at a running channel is
 *    ignored, and a zero written to a trigger register (a null trigger)
 *    starts nothing and, with IRQ_QUIET, is the one thing that raises
 *    the interrupt;
 *  - a completed channel stays ENABLED and goes not-BUSY: load() refuses
 *    a BUSY channel, never an enabled one, and clearing EN merely pauses
 *    a channel - the way out of a stalled one is the abort;
 *  - THE ABORT, erratum E13: a channel aborted with transfers in flight
 *    reports a completion when they land. So abort() unroutes the
 *    channel from both lines, aborts, waits for BUSY to fall (bounded),
 *    clears the raw and masked status it may have left, and restores
 *    the routes - the datasheet's own workaround, as one verb;
 *  - PROGRESS IS TRANS_COUNT, erratum E12: READ_ADDR and WRITE_ADDR read
 *    wrong while a non-incrementing or wrapping sequence is in progress,
 *    so no verb here derives progress from an address;
 *  - a bus error halts the channel, marks CTRL (AHB_ERROR with READ_ERROR
 *    or WRITE_ERROR, cleared by writing one) and RAISES THE CHANNEL'S
 *    INTERRUPT: the engines read the error before the completion;
 *  - one FIFO per DMA must not be touched by the processor while a
 *    channel serves it (2.5.3.2): a transport with an engine on a
 *    direction leaves that direction's data register to the engine.
 *
 * TWO CORES. The controller is one; a channel belongs to the core whose
 * kernel hosts the transport that armed it, and it reports on the line
 * that core enabled - line 0 for core 0, line 1 for core 1 - which is
 * why every engine and every channel verb that routes takes the line
 * as a parameter and enables it in the CALLING core's NVIC.
 */

#pragma once

#include <stddef.h>
#include <stdint.h>

#include <atomic>
#include <span>

#include "rp2040/device.hpp"

#include "rp2040/dma_engine.hpp"
#include "rp2040/nvic.hpp"
#include "rp2040/resets.hpp"

namespace brio {

// ---- the vocabulary ------------------------------------------------------------

inline constexpr uint8_t dma_channel_count = 12;

/// CTRL.DATA_SIZE: reads and writes are the same width.
enum class DmaSize : uint8_t { byte = 0, half = 1, word = 2 };

template <typename Elem>
constexpr DmaSize dma_size_of() {
    static_assert(sizeof(Elem) == 1 || sizeof(Elem) == 2 || sizeof(Elem) == 4,
                  "a DMA element is one bus access wide: 1, 2 or 4 bytes");
    return sizeof(Elem) == 1 ? DmaSize::byte : sizeof(Elem) == 2 ? DmaSize::half : DmaSize::word;
}

constexpr uint8_t dma_size_bytes(DmaSize s) { return static_cast<uint8_t>(1u << static_cast<uint8_t>(s)); }

/// The two system interrupt lines.
inline constexpr uint8_t dma_line_count = 2;

/// "Chain to nobody": CHAIN_TO set to the channel's own index (2.5.2.2).
inline constexpr uint8_t dma_no_chain = 0xFFu;

/// Everything of CTRL but EN and the status bits.
struct DmaChannelConfig {
    DmaSize size = DmaSize::byte;
    bool incr_read = true;           ///< INCR_READ: the source is a buffer, not a register
    bool incr_write = true;          ///< INCR_WRITE
    uint8_t ring_bits = 0;           ///< RING_SIZE: 0 = no wrap, n = wrap at 2^n bytes (1..15)
    bool ring_on_write = false;      ///< RING_SEL: the wrap applies to the write side
    Dreq treq = Dreq::permanent;
    uint8_t chain_to = dma_no_chain; ///< another channel to trigger at completion
    bool high_priority = false;      ///< HIGH_PRIORITY: served before normal channels
    bool byte_swap = false;          ///< BSWAP: reverse the bytes of each half-word or word
    bool sniff = false;              ///< SNIFF_EN: the sniffer sees this channel's data
    bool irq_quiet = false;          ///< IRQ_QUIET: an interrupt on a null trigger only
};

constexpr bool dma_channel_config_valid(const DmaChannelConfig& c, uint8_t ch) {
    if (c.ring_bits > 15u) {
        return false;
    }
    if (c.chain_to != dma_no_chain && (c.chain_to >= dma_channel_count || c.chain_to == ch)) {
        return false;
    }
    return true;
}

/// CTRL's word for a channel, EN as asked.
constexpr uint32_t dma_ctrl_word(const DmaChannelConfig& c, uint8_t ch, bool enable) {
    const uint8_t chain = c.chain_to == dma_no_chain ? ch : c.chain_to;
    uint32_t v = static_cast<uint32_t>(c.size) << DMA_CH0_CTRL_TRIG_DATA_SIZE_LSB;
    if (c.incr_read) { v |= DMA_CH0_CTRL_TRIG_INCR_READ_BITS; }
    if (c.incr_write) { v |= DMA_CH0_CTRL_TRIG_INCR_WRITE_BITS; }
    v |= static_cast<uint32_t>(c.ring_bits) << DMA_CH0_CTRL_TRIG_RING_SIZE_LSB;
    if (c.ring_on_write) { v |= DMA_CH0_CTRL_TRIG_RING_SEL_BITS; }
    v |= static_cast<uint32_t>(chain) << DMA_CH0_CTRL_TRIG_CHAIN_TO_LSB;
    v |= static_cast<uint32_t>(c.treq) << DMA_CH0_CTRL_TRIG_TREQ_SEL_LSB;
    if (c.high_priority) { v |= DMA_CH0_CTRL_TRIG_HIGH_PRIORITY_BITS; }
    if (c.byte_swap) { v |= DMA_CH0_CTRL_TRIG_BSWAP_BITS; }
    if (c.sniff) { v |= DMA_CH0_CTRL_TRIG_SNIFF_EN_BITS; }
    if (c.irq_quiet) { v |= DMA_CH0_CTRL_TRIG_IRQ_QUIET_BITS; }
    if (enable) { v |= DMA_CH0_CTRL_TRIG_EN_BITS; }
    return v;
}

/// One transfer sequence: both ends, the count in ITEMS, the config.
struct DmaTransfer {
    const volatile void* read = nullptr;
    volatile void* write = nullptr;
    uint32_t count = 0;
    DmaChannelConfig config{};
};

constexpr bool dma_aligned(const volatile void* p, DmaSize s) {
    return (reinterpret_cast<uintptr_t>(p) & (dma_size_bytes(s) - 1u)) == 0u;
}

/// Both ends present and aligned to the size (2.5.1.1's caution), a
/// non-zero count, a legal config.
constexpr bool dma_transfer_valid(const DmaTransfer& t, uint8_t ch) {
    return t.read != nullptr && t.write != nullptr && t.count != 0u &&
           dma_aligned(t.read, t.config.size) && dma_aligned(t.write, t.config.size) &&
           dma_channel_config_valid(t.config, ch);
}

/// CTRL's error bits.
struct DmaError {
    static constexpr uint32_t ahb = DMA_CH0_CTRL_TRIG_AHB_ERROR_BITS;
    static constexpr uint32_t read = DMA_CH0_CTRL_TRIG_READ_ERROR_BITS;
    static constexpr uint32_t write = DMA_CH0_CTRL_TRIG_WRITE_ERROR_BITS;
    static constexpr uint32_t all = ahb | read | write;
};

struct DmaProgress {
    uint32_t remaining = 0;
    uint32_t done = 0;
};

// ---- the block -------------------------------------------------------------------

static_assert(offsetof(DMA_Type, CH1_READ_ADDR) == offsetof(DMA_Type, CH0_READ_ADDR) + 0x40u,
              "a channel's four registers and their aliases are sixteen words");
static_assert(offsetof(DMA_Type, CH0_AL1_CTRL) == offsetof(DMA_Type, CH0_READ_ADDR) + 0x10u);

struct Dma {
    Dma() = delete;

    /// The block out of the reset controller: every channel at its reset
    /// state, every enable clear. Once per program, before any channel.
    static bool init() { return Resets::cycle(ResetBlock::dma); }
    static bool released() { return Resets::released(ResetBlock::dma); }

    /// How many channels the silicon says it has (N_CHANNELS).
    static uint8_t channels() { return static_cast<uint8_t>(DMA->N_CHANNELS); }

    /// The raw interrupt status, one bit per channel: set at every
    /// completion (or null trigger under IRQ_QUIET), whatever the lines'
    /// enables; cleared by writing one.
    static uint32_t raw() { return DMA->INTR; }
    static void clear_raw(uint32_t mask) { DMA->INTR = mask; }

    /// Start several channels at once (MULTI_CHAN_TRIGGER): the ones
    /// prepared, enabled and not running.
    static void trigger(uint32_t mask) { DMA->MULTI_CHAN_TRIGGER = mask; }

    /// The raw abort of several channels (CHAN_ABORT), E13 NOT handled:
    /// DmaChannel::abort() is the verb with the workaround.
    static void abort_raw(uint32_t mask) { DMA->CHAN_ABORT = mask; }
};

/// One of the two system interrupt lines.
template <uint8_t line>
struct DmaLine {
    static_assert(line < dma_line_count, "the RP2040 DMA has two interrupt lines, 0 and 1");
    DmaLine() = delete;

    static constexpr IRQn_Type irq() { return line == 0 ? DMA_IRQ_0_IRQn : DMA_IRQ_1_IRQn; }

    /// The channels routed to this line (INTEn).
    static uint32_t routed() { return line == 0 ? DMA->INTE0 : DMA->INTE1; }
    /// The masked status (INTSn): routed AND raised; cleared by writing one.
    static uint32_t pending() { return line == 0 ? DMA->INTS0 : DMA->INTS1; }
    static void clear(uint32_t mask) {
        if constexpr (line == 0) { DMA->INTS0 = mask; } else { DMA->INTS1 = mask; }
    }
    /// The force bits (INTFn): a channel's interrupt asserted by software.
    static void force(uint32_t mask, bool on) {
        if constexpr (line == 0) {
            if (on) { hw_set(DMA->INTF0, mask); } else { hw_clear(DMA->INTF0, mask); }
        } else {
            if (on) { hw_set(DMA->INTF1, mask); } else { hw_clear(DMA->INTF1, mask); }
        }
    }

    /// The line in the CALLING core's NVIC.
    static void enable() { Nvic::enable(irq()); }
    static void disable() { Nvic::disable(irq()); }
};

/// One channel, 0..11.
template <uint8_t ch>
class DmaChannel {
    static_assert(ch < dma_channel_count, "the RP2040 DMA has channels 0..11");

public:
    DmaChannel() = delete;

    static constexpr uint8_t number = ch;
    static constexpr uint32_t bit = 1u << ch;

    // The sixteen words: alias 0 at +0 (READ, WRITE, COUNT, CTRL_TRIG),
    // alias 1 at +0x10 (CTRL, READ, WRITE, COUNT_TRIG), alias 2 at +0x20
    // (CTRL, COUNT, READ, WRITE_TRIG), alias 3 at +0x30 (CTRL, WRITE,
    // COUNT, READ_TRIG).
    static volatile uint32_t& read_addr() { return word(0); }
    static volatile uint32_t& write_addr() { return word(1); }
    static volatile uint32_t& trans_count() { return word(2); }
    static volatile uint32_t& ctrl_trig() { return word(3); }
    static volatile uint32_t& ctrl() { return word(4); }              ///< alias 1: no trigger
    static volatile uint32_t& trans_count_trig() { return word(7); }
    static volatile uint32_t& write_addr_trig() { return word(11); }
    static volatile uint32_t& read_addr_trig() { return word(15); }

    static bool enabled() { return (ctrl() & DMA_CH0_CTRL_TRIG_EN_BITS) != 0u; }
    /// Transfers in progress or in flight (CTRL.BUSY).
    static bool busy() { return (ctrl() & DMA_CH0_CTRL_TRIG_BUSY_BITS) != 0u; }

    /// EN on or off through the non-trigger alias: off PAUSES a running
    /// channel (2.5.5.3), it does not end it.
    static void enable(bool on) {
        if (on) { hw_set(ctrl(), DMA_CH0_CTRL_TRIG_EN_BITS); }
        else { hw_clear(ctrl(), DMA_CH0_CTRL_TRIG_EN_BITS); }
    }

    /// CTRL's fields but EN, which is kept as it stands. Refused while
    /// busy, and for a config the chapter forbids.
    static bool configure(const DmaChannelConfig& c) {
        if (busy() || !dma_channel_config_valid(c, ch)) {
            return false;
        }
        ctrl() = dma_ctrl_word(c, ch, enabled());
        return true;
    }

    static bool set_read(const volatile void* address) {
        if (busy()) { return false; }
        read_addr() = static_cast<uint32_t>(reinterpret_cast<uintptr_t>(address));
        return true;
    }
    static bool set_write(volatile void* address) {
        if (busy()) { return false; }
        write_addr() = static_cast<uint32_t>(reinterpret_cast<uintptr_t>(address));
        return true;
    }
    /// The count of the NEXT sequence (2.5.1.2).
    static bool set_count(uint32_t count) {
        if (busy() || count == 0u) { return false; }
        trans_count() = count;
        return true;
    }

    /// Program the whole transfer WITHOUT starting it: the four
    /// registers through non-trigger aliases, EN set, the errors and
    /// the raw status cleared. trigger() or a chain starts it.
    static bool prepare(const DmaTransfer& t) {
        if (busy() || !dma_transfer_valid(t, ch)) {
            return false;
        }
        clear_errors();
        Dma::clear_raw(bit);
        read_addr() = static_cast<uint32_t>(reinterpret_cast<uintptr_t>(t.read));
        write_addr() = static_cast<uint32_t>(reinterpret_cast<uintptr_t>(t.write));
        trans_count() = t.count;
        ctrl() = dma_ctrl_word(t.config, ch, true);
        return true;
    }

    /// Program the whole transfer and START it: the three registers,
    /// then CTRL through its trigger alias.
    static bool load(const DmaTransfer& t) {
        if (busy() || !dma_transfer_valid(t, ch)) {
            return false;
        }
        clear_errors();
        Dma::clear_raw(bit);
        read_addr() = static_cast<uint32_t>(reinterpret_cast<uintptr_t>(t.read));
        write_addr() = static_cast<uint32_t>(reinterpret_cast<uintptr_t>(t.write));
        trans_count() = t.count;
        ctrl_trig() = dma_ctrl_word(t.config, ch, true);
        return true;
    }

    /// Start a prepared channel (MULTI_CHAN_TRIGGER's bit).
    static void trigger() { Dma::trigger(bit); }

    /// A NULL TRIGGER: zero into the trigger alias starts nothing and,
    /// under IRQ_QUIET, is what raises the channel's interrupt - the end
    /// of a control-block chain (2.5.2.3).
    static void null_trigger() { ctrl_trig() = 0; }

    /// Transfers remaining in the current sequence, live - the one
    /// progress reading erratum E12 leaves honest.
    static uint32_t count() { return trans_count(); }

    static DmaProgress progress(uint32_t programmed) {
        const uint32_t remaining = count();
        return DmaProgress{remaining, remaining > programmed ? 0u : programmed - remaining};
    }

    /// CTRL's error bits: AHB_ERROR (sticky OR of the two) with
    /// READ_ERROR and WRITE_ERROR; a bus error halts the channel and
    /// raises its interrupt.
    static uint32_t errors() { return ctrl() & DmaError::all; }
    static void clear_errors() { hw_set(ctrl(), DmaError::read | DmaError::write); }

    /// The raw status bit (INTR) and its clear.
    static bool raised() { return (Dma::raw() & bit) != 0u; }
    static void clear_raised() { Dma::clear_raw(bit); }

    /// Route the channel's interrupt to `line`, or take it off it.
    static void route(uint8_t line, bool on) {
        if (line == 0u) {
            if (on) { hw_set(DMA->INTE0, bit); } else { hw_clear(DMA->INTE0, bit); }
        } else {
            if (on) { hw_set(DMA->INTE1, bit); } else { hw_clear(DMA->INTE1, bit); }
        }
    }
    static bool routed(uint8_t line) {
        return ((line == 0u ? DMA->INTE0 : DMA->INTE1) & bit) != 0u;
    }
    /// Pending on `line`: routed there and raised; cleared by writing one
    /// (which clears the raw bit too).
    static bool pending(uint8_t line) {
        return ((line == 0u ? DMA->INTS0 : DMA->INTS1) & bit) != 0u;
    }
    static void clear_pending(uint8_t line) {
        if (line == 0u) { DMA->INTS0 = bit; } else { DMA->INTS1 = bit; }
    }

    /**
     * End the sequence early, erratum E13's way: the routes to both
     * lines lifted, CHAN_ABORT written, BUSY awaited (bounded - the
     * transfers in flight land in a few cycles), the completion the
     * landing may have raised cleared raw and on both lines, the routes
     * restored. False when BUSY never fell.
     */
    static bool abort() {
        const bool on0 = routed(0);
        const bool on1 = routed(1);
        route(0, false);
        route(1, false);
        Dma::abort_raw(bit);
        bool settled = false;
        for (uint32_t spins = 100'000u; spins != 0u; --spins) {
            if (!busy()) {
                settled = true;
                break;
            }
        }
        Dma::clear_raw(bit);
        DMA->INTS0 = bit;
        DMA->INTS1 = bit;
        route(0, on0);
        route(1, on1);
        return settled;
    }

    /// Everything of the channel's back to nothing: aborted, disabled,
    /// off both lines, the errors and the status cleared.
    static void stop() {
        (void)abort();
        route(0, false);
        route(1, false);
        ctrl() = 0;
        clear_errors();
        Dma::clear_raw(bit);
    }

    /// The request credits the channel holds (DBG_CTDREQ), and their
    /// clear: THE CREDITS OUTLIVE A SEQUENCE. A peripheral keeps pulsing
    /// while the channel sits completed, and a full FIFO that overflowed
    /// meanwhile has pulsed more than it holds - the next sequence would
    /// spend those credits reading an empty FIFO (measured: zeros at the
    /// head of a receive run after an overrun). A level request like
    /// the PL011's re-pulses for every byte still waiting, so clearing
    /// the count before a new sequence loses nothing.
    static uint32_t debug_credits() {
        return reg_at(DMA_BASE, DMA_CH0_DBG_CTDREQ_OFFSET + 0x40u * ch) & 0x3Fu;
    }
    static void clear_credits() { reg_at(DMA_BASE, DMA_CH0_DBG_CTDREQ_OFFSET + 0x40u * ch) = 0; }
    static uint32_t debug_reload() { return reg_at(DMA_BASE, DMA_CH0_DBG_TCR_OFFSET + 0x40u * ch); }

private:
    static volatile uint32_t& word(uint8_t i) { return *(&DMA->CH0_READ_ADDR + 16u * ch + i); }
};

// ---- the pacing timers (2.5.5.1) --------------------------------------------------

/// A request every Y / X cycles of clk_sys, at most one a cycle; X = 0
/// requests nothing.
template <uint8_t n>
struct DmaTimer {
    static_assert(n < 4u, "the RP2040 DMA has four pacing timers");
    DmaTimer() = delete;

    static constexpr Dreq dreq() {
        return n == 0 ? Dreq::timer0 : n == 1 ? Dreq::timer1 : n == 2 ? Dreq::timer2 : Dreq::timer3;
    }
    static void set(uint16_t x, uint16_t y) {
        reg() = (static_cast<uint32_t>(x) << DMA_TIMER0_X_LSB) | y;
    }
    static void stop() { reg() = 0; }

private:
    static volatile uint32_t& reg() { return *(&DMA->TIMER0 + n); }
};

// ---- the sniffer (2.5.5.2) ----------------------------------------------------------

/// What the sniffer computes over the data of one channel.
enum class DmaSniffCalc : uint8_t {
    crc32 = DMA_SNIFF_CTRL_CALC_VALUE_CRC32,               ///< 0x04C11DB7, most significant bit first
    crc32_reversed = DMA_SNIFF_CTRL_CALC_VALUE_CRC32R,     ///< the same, bit-reversed data
    crc16 = DMA_SNIFF_CTRL_CALC_VALUE_CRC16,               ///< CRC-16-CCITT (0x1021), MSB first
    crc16_reversed = DMA_SNIFF_CTRL_CALC_VALUE_CRC16R,
    even_parity = DMA_SNIFF_CTRL_CALC_VALUE_EVEN,
    sum = DMA_SNIFF_CTRL_CALC_VALUE_SUM,                   ///< a 32-bit accumulator
};

struct DmaSniffConfig {
    DmaSniffCalc calc = DmaSniffCalc::sum;
    bool byte_swap = false;      ///< BSWAP: the result's bytes swapped in SNIFF_DATA
    bool out_reverse = false;    ///< OUT_REV: the result bit-reversed
    bool out_invert = false;     ///< OUT_INV: the result inverted
};

/// The passive checksum over one channel's data: the channel's config
/// must say `sniff`, and only one channel is sniffed at a time.
struct DmaSniffer {
    DmaSniffer() = delete;

    static void start(uint8_t channel, const DmaSniffConfig& c, uint32_t seed = 0) {
        uint32_t v = DMA_SNIFF_CTRL_EN_BITS |
                     (static_cast<uint32_t>(channel) << DMA_SNIFF_CTRL_DMACH_LSB) |
                     (static_cast<uint32_t>(c.calc) << DMA_SNIFF_CTRL_CALC_LSB);
        if (c.byte_swap) { v |= DMA_SNIFF_CTRL_BSWAP_BITS; }
        if (c.out_reverse) { v |= DMA_SNIFF_CTRL_OUT_REV_BITS; }
        if (c.out_invert) { v |= DMA_SNIFF_CTRL_OUT_INV_BITS; }
        DMA->SNIFF_DATA = seed;
        DMA->SNIFF_CTRL = v;
    }
    static uint32_t result() { return DMA->SNIFF_DATA; }
    static void stop() { DMA->SNIFF_CTRL = 0; }
};

// ---- the engines ---------------------------------------------------------------------
//
// TWO MOMENTS, written from this controller's own offer (2.5.1, 2.5.2.1).
// An engine binds its channel ONCE, at arm(): the channel stopped (E13's
// abort), the side that does not move written - the peripheral's data
// register, which 2.5.1.1 lets a channel keep from one sequence to the
// next ("if the address does not increment ... there is no need to write
// to the register again") - CTRL written through its non-trigger alias
// with everything constant for the binding (the request, the priority,
// CHAIN_TO at the channel itself, what the channel reports, EN), the route
// to the line set and the line enabled. A block then writes what changes
// and nothing else: the count and the memory address, the last of them
// through the alias whose final word is a TRIGGER (2.5.2.1) - for a
// transmit TRANS_COUNT then READ_ADDR_TRIG (alias 3: the datasheet's
// "(TRANS_COUNT, READ_ADDR_TRIG) for peripheral gather operations"), for a
// receive WRITE_ADDR then TRANS_COUNT_TRIG (alias 1: "(WRITE_ADDR,
// TRANS_COUNT_TRIG) for peripheral scatter operations"). Two stores a
// block. CTRL is a third only when the block's beat or step differs from
// the word the channel holds, which the engine keeps (DmaBinding), so the
// question is a compare against RAM and not a read of the bus. No flag is
// cleared per block: an engine's completion and its bus-error flags are
// cleared by its service() when served, and by abandon() and stop(), so a
// block starts on a channel nothing stands on.
//
// THE BEAT IS THE TYPE. start() takes a run of uint8_t, uint16_t or
// uint32_t - a span, or a pointer and a count - and DATA_SIZE follows the
// element (one field of CTRL, reads and writes the same width, 2.5).
// `Elem`, the template parameter, is the WIDEST beat the binding allows
// (the data register's width); a narrower one is always legal; a run
// wider than a byte whose address is not aligned to its width is REFUSED
// at run time (2.5.1.1: the initial addresses' alignment "is up to
// software"), so the caller falls back to its pump. Narrow writes are
// byte-lane replicated (2.5), so a byte beat into a half-word data
// register writes the frame's low byte where the peripheral reads it.
//
// THE CLAIM. A transmit engine's `busy_` is its claim on the channel, and
// the claim is the one step a completion on the line can race: claim()
// is that test-and-set, for a transport to take under its mask; launch()
// programs a claimed channel with no mask held, because a claimed channel
// cannot complete under the loader's feet; unclaim() gives back a claim
// with nothing to send. start() is claim() then launch(), for a caller
// with one context.
//
// THE LEVEL IS THE PERIPHERAL'S (docs/design/dma.md), and this controller
// has one bit of it, CTRL.HIGH_PRIORITY (2.5.7): "in each scheduling
// round, all high priority channels are considered first, and then only a
// single low priority channel". A normal channel therefore waits one round
// of the high ones at most and is never starved outright, so here the
// level decides no correctness and no transport offers it as an option
// (docs/rp2040/dma.md measures a receive ring at normal beside two
// HIGH_PRIORITY copies). The defaults are the rule's all the same, one bit
// wide: a receive engine arms HIGH, its peripheral being the one that
// overruns when starved; a transmit engine and the copy engine normal; and
// a transport names its engines' levels at the call.
//
// THE NAMES ARE THE IP STRATA'S. brio/pl011/, brio/pl022/ and
// brio/dw_apb_i2c/ drive this chip's engines through DmaReport (reached as
// the engine's own `Report`), the binding, the claim and the typed runs,
// so an IP file names `TxEngine::Report::errors` without knowing the
// family. What is this chip's below is a count of 32 bits with no mode,
// the two address steps (incrementing or not, INCR_READ / INCR_WRITE),
// E12 (progress from TRANS_COUNT), E13 (the abort that may raise a
// completion, and leaves CTRL as it stood), and two lines.

/// CTRL's DATA_SIZE field for a beat of T.
template <typename T>
constexpr uint32_t dma_beat_bits() {
    return static_cast<uint32_t>(dma_size_of<T>()) << DMA_CH0_CTRL_TRIG_DATA_SIZE_LSB;
}

/// A run's address aligned to its beat (2.5.1.1's caution); a byte is.
template <typename T>
[[gnu::always_inline]] inline bool dma_beat_aligned(const volatile void* p) {
    if constexpr (sizeof(T) == 1u) {
        return true;
    } else {
        return (reinterpret_cast<uintptr_t>(p) & (sizeof(T) - 1u)) == 0u;
    }
}

/// The checks a block's arguments can fail: a run, a non-zero count (a
/// zero written to a trigger register is a NULL TRIGGER, 2.5.2.3: it
/// starts nothing), an address aligned to its beat. TRANS_COUNT is 32
/// bits on this chip, so no count a uint32_t holds is too long.
template <typename T>
[[gnu::always_inline]] inline bool dma_run_valid(const volatile void* p, uint32_t count) {
    return p != nullptr && count != 0u && dma_beat_aligned<T>(p);
}

[[gnu::always_inline]] inline uint32_t dma_address(const volatile void* p) {
    return static_cast<uint32_t>(reinterpret_cast<uintptr_t>(p));
}

/// A fence the compiler may not move a memory access across, emitted as
/// nothing: a block's bookkeeping and its data are stored BEFORE the
/// trigger store that hands them to the controller and to the completion
/// handler. The core issues its bus accesses in program order, so no
/// barrier instruction is owed.
[[gnu::always_inline]] inline void dma_handoff_fence() { std::atomic_signal_fence(std::memory_order_seq_cst); }

/// What an engine's channel reports on its line, chosen at arm(): every
/// block (`blocks`), or a bus error alone (`errors`). `errors` is
/// IRQ_QUIET (2.5.2.3): no interrupt at the end of a block - only a null
/// trigger raises one, and no engine writes a null trigger - while a bus
/// error still raises the channel's flag (CTRL.AHB_ERROR, 2.5.7: the
/// channel "always raises its channel IRQ flag"). It is how two channels
/// whose second completion proves the first's - the SPI host's transmit
/// beside its receive - cost ONE interrupt a transaction.
enum class DmaReport : uint8_t { blocks, errors };

/// The part of CTRL constant for a binding: EN, the request, the
/// priority, CHAIN_TO at the channel itself (no chain), the report.
constexpr uint32_t dma_binding_word(uint8_t ch, Dreq treq, bool high_priority, DmaReport report) {
    uint32_t v = DMA_CH0_CTRL_TRIG_EN_BITS;
    v |= static_cast<uint32_t>(ch) << DMA_CH0_CTRL_TRIG_CHAIN_TO_LSB;
    v |= static_cast<uint32_t>(treq) << DMA_CH0_CTRL_TRIG_TREQ_SEL_LSB;
    if (high_priority) { v |= DMA_CH0_CTRL_TRIG_HIGH_PRIORITY_BITS; }
    if (report == DmaReport::errors) { v |= DMA_CH0_CTRL_TRIG_IRQ_QUIET_BITS; }
    return v;
}

/**
 * What every engine keeps of the channel it binds - ONE PER CHANNEL, so
 * that two engine types naming one channel (a byte engine and a word
 * engine on one PIO FIFO, armed in turn) agree on what CTRL holds. The
 * channel is the engine's from arm() to stop(): a channel verb that
 * rewrites CTRL behind it (DmaChannel::stop, configure, load) breaks the
 * binding until the next arm().
 */
template <uint8_t ch>
struct DmaBinding {
    DmaBinding() = delete;
    using Channel = DmaChannel<ch>;

    /// The CTRL word the channel holds, as an engine last wrote it.
    static inline uint32_t ctrl = 0;

    /// The block's CTRL, written only when it differs from the held word.
    [[gnu::always_inline]] static void steer(uint32_t want) {
        if (want != ctrl) {
            ctrl = want;
            Channel::ctrl() = want;
        }
    }

    /// The channel stopped and bound: CTRL with EN through the
    /// non-trigger alias, and the route to `line` when the engine's
    /// completions or errors are served there.
    static void bind(uint32_t word, uint8_t line, bool routed) {
        Channel::stop();
        ctrl = word;
        Channel::ctrl() = word;
        if (routed) {
            Channel::route(line, true);
        }
    }

    /// Back to nothing, the held word with it.
    static void release() {
        Channel::stop();
        ctrl = 0;
    }

    /// The ISR body every engine folds in: this channel's status on its
    /// line, cleared and handed back as flags - the error read first,
    /// since a bus error raises the completion's flag too.
    template <uint8_t line>
    [[gnu::always_inline]] static uint8_t service() {
        uint8_t f = 0;
        if (Channel::pending(line)) {
            Channel::clear_pending(line);
            f |= 1u << 0;
            if (Channel::errors() != 0u) {
                Channel::clear_errors();
                f |= 1u << 1;
            }
        }
        return f;
    }
};

/**
 * A transmit engine: one caller-owned run poured into one peripheral
 * register, paced by that peripheral's request - or by a DmaTimer's,
 * which is the PACED shape (a table into one cell at a known rate).
 * `line` is the system interrupt line the completion reports on - the
 * owner core's.
 */
template <uint8_t ch, typename Elem = uint8_t, uint8_t line = 0>
class DmaTxEngine {
    static_assert(ch < dma_channel_count, "no such DMA channel for this engine");
    static_assert(line < dma_line_count, "the RP2040 DMA has two interrupt lines, 0 and 1");
    using Channel = DmaChannel<ch>;
    using Binding = DmaBinding<ch>;

public:
    DmaTxEngine() = delete;

    static constexpr bool present = true;
    static constexpr uint8_t channel = ch;
    static constexpr uint8_t irq_line = line;
    /// The WIDEST beat the binding allows; a run may be narrower.
    static constexpr DmaSize size = dma_size_of<Elem>();
    using element = Elem;
    /// DmaReport by a name the IP strata reach through the engine type.
    using Report = DmaReport;

    static constexpr uint8_t flag_complete = 1u << 0;
    static constexpr uint8_t flag_half = 0;   ///< this controller has no half-transfer event
    static constexpr uint8_t flag_error = 1u << 1;

    /// The ISR body folded into the engine (DmaBinding::service).
    [[gnu::always_inline]] static uint8_t service() { return Binding::template service<line>(); }

    /// Bind the channel: `data` is the register the runs are poured into,
    /// `dreq` the request that paces them, `report` what reaches the line
    /// (DmaReport). The line is enabled in the calling core's NVIC.
    /// NORMAL priority by default, the rule's level for a transmit or a
    /// paced output (the engines' section): a starved transmit leaves the
    /// line idle a while, a paced output holds its last value a period.
    static void arm(volatile void* data, Dreq dreq, bool high_priority = false,
                    DmaReport report = DmaReport::blocks) {
        data_ = dma_address(data);
        base_ = dma_binding_word(ch, dreq, high_priority, report);
        busy_ = false;
        in_flight_ = 0;
        bind();
        DmaLine<line>::enable();
    }

    /// THE CLAIM: busy_ tested and set, the one step to take under the
    /// caller's mask. True = the channel is the caller's until
    /// complete(), abandon() or unclaim().
    [[gnu::always_inline]] static bool claim() {
        if (busy_) {
            return false;
        }
        busy_ = true;
        return true;
    }
    /// A claim given back with nothing started.
    [[gnu::always_inline]] static void unclaim() { busy_ = false; }

    /// Program a CLAIMED channel and start it: `length` elements from
    /// `buffer` (the caller's, and it stays put until complete() reports
    /// the block). False, and the claim given back, for an empty run or
    /// a misaligned one.
    template <typename T>
    static bool launch(const T* buffer, uint32_t length) {
        return program(buffer, length, DMA_CH0_CTRL_TRIG_INCR_READ_BITS);
    }
    template <typename T>
    static bool launch(std::span<T> run) {
        return launch(run.data(), static_cast<uint32_t>(run.size()));
    }
    /// `length` copies of ONE element on a claimed channel: the read
    /// address does not move. What a full-duplex bus needs to clock a read.
    template <typename T>
    static bool launch_fixed(const T* cell, uint32_t length) {
        return program(cell, length, 0u);
    }

    /// claim() and launch(), for a caller with one context.
    template <typename T>
    static bool start(const T* buffer, uint32_t length) {
        return claim() && launch(buffer, length);
    }
    template <typename T>
    static bool start(std::span<T> run) {
        return claim() && launch(run);
    }
    template <typename T>
    static bool start_fixed(const T* cell, uint32_t length) {
        return claim() && launch_fixed(cell, length);
    }

    /// The block ended: how many elements it carried, so the owner can
    /// release exactly that much of its ring.
    static uint32_t complete() {
        const uint32_t n = in_flight_;
        busy_ = false;
        in_flight_ = 0;
        return n;
    }

    static bool busy() { return busy_; }
    static uint32_t in_flight() { return busy_ ? in_flight_ : 0u; }
    static DmaProgress progress() { return Channel::progress(in_flight_); }

    /// Throw the running block away (E13's abort) and hand the channel
    /// back bound and ready.
    static bool abandon() {
        ++faults_;
        busy_ = false;
        in_flight_ = 0;
        bind();
        return true;
    }
    static uint32_t faults() { return faults_; }
    static void clear_faults() { faults_ = 0; }

    static void stop() {
        Binding::release();
        busy_ = false;
        in_flight_ = 0;
    }

private:
    template <typename T>
    static bool program(const T* p, uint32_t length, uint32_t step) {
        static_assert(sizeof(T) == 1u || sizeof(T) == 2u || sizeof(T) == 4u,
                      "a DMA beat is one bus access wide: 1, 2 or 4 bytes");
        static_assert(sizeof(T) <= sizeof(Elem),
                      "a run's beat is at most the engine's Elem - the widest the binding allows");
        if (!dma_run_valid<T>(p, length)) {
            busy_ = false;
            return false;
        }
        in_flight_ = length;
        Binding::steer(base_ | dma_beat_bits<T>() | step);
        Channel::trans_count() = length;
        dma_handoff_fence();
        Channel::read_addr_trig() = dma_address(p);
        return true;
    }
    /// Bound: the data register written ONCE, here - the write side
    /// does not move, so 2.5.1.1 lets every block find it in place.
    static void bind() {
        Binding::bind(base_ | dma_beat_bits<Elem>() | DMA_CH0_CTRL_TRIG_INCR_READ_BITS, line, true);
        Channel::write_addr() = data_;
    }

    static inline uint32_t data_ = 0;
    static inline uint32_t base_ = 0;
    static inline uint32_t in_flight_ = 0;
    static inline uint32_t faults_ = 0;
    static inline bool busy_ = false;
};

/**
 * A receive engine: one caller-owned run filled from one peripheral
 * register, and asked how much has arrived - from TRANS_COUNT, one
 * read, nothing suspended (erratum E12: never from WRITE_ADDR). The
 * count is in the elements of the run started, whatever its beat.
 */
template <uint8_t ch, typename Elem = uint8_t, uint8_t line = 0>
class DmaRxEngine {
    static_assert(ch < dma_channel_count, "no such DMA channel for this engine");
    static_assert(line < dma_line_count, "the RP2040 DMA has two interrupt lines, 0 and 1");
    using Channel = DmaChannel<ch>;
    using Binding = DmaBinding<ch>;

public:
    DmaRxEngine() = delete;

    static constexpr bool present = true;
    static constexpr uint8_t channel = ch;
    static constexpr uint8_t irq_line = line;
    /// The WIDEST beat the binding allows; a run may be narrower.
    static constexpr DmaSize size = dma_size_of<Elem>();
    using element = Elem;
    using Report = DmaReport;

    static constexpr uint8_t flag_complete = 1u << 0;
    static constexpr uint8_t flag_half = 0;
    static constexpr uint8_t flag_error = 1u << 1;

    [[gnu::always_inline]] static uint8_t service() { return Binding::template service<line>(); }

    /// Bind the channel: `data` is the register the runs are filled
    /// from, `dreq` the request that paces them. HIGH_PRIORITY by
    /// default, the rule's level for a receive (the engines' section): a
    /// peripheral a receive channel empties overruns when the channel is
    /// starved.
    static void arm(volatile void* data, Dreq dreq, bool high_priority = true,
                    DmaReport report = DmaReport::blocks) {
        data_ = dma_address(data);
        base_ = dma_binding_word(ch, dreq, high_priority, report);
        capacity_ = 0;
        taken_ = 0;
        bind();
        DmaLine<line>::enable();
    }

    /// Nothing in progress: the run filled, or none started.
    static bool idle() { return !Channel::busy(); }

    /// Fill `length` elements of `buffer`. A run still in progress is
    /// aborted first (E13's way), and the channel's request credits are
    /// cleared - a stale credit is a read of nothing (DmaChannel).
    template <typename T>
    static bool start(T* buffer, uint32_t length) {
        return begin(buffer, length, DMA_CH0_CTRL_TRIG_INCR_WRITE_BITS);
    }
    template <typename T>
    static bool start(std::span<T> room) {
        return start(room.data(), static_cast<uint32_t>(room.size()));
    }
    /// `length` elements into ONE cell, thrown away (the receive side of
    /// a bus write).
    template <typename T>
    static bool start_discard(T* cell, uint32_t length) {
        return begin(cell, length, 0u);
    }

    /// How many elements have arrived since the last take().
    static uint32_t take() {
        if (capacity_ == 0u) {
            return 0;
        }
        const uint32_t remaining = Channel::count();
        const uint32_t filled = remaining > capacity_ ? capacity_ : capacity_ - remaining;
        if (filled <= taken_) {
            return 0;
        }
        const uint32_t fresh = filled - taken_;
        taken_ = filled;
        return fresh;
    }
    static bool full() { return capacity_ != 0u && taken_ >= capacity_; }
    static uint32_t capacity() { return capacity_; }
    static uint32_t taken() { return taken_; }

    static bool abandon() {
        ++faults_;
        capacity_ = 0;
        taken_ = 0;
        bind();
        return true;
    }
    static uint32_t faults() { return faults_; }
    static void clear_faults() { faults_ = 0; }

    static void stop() {
        Binding::release();
        capacity_ = 0;
        taken_ = 0;
    }

private:
    template <typename T>
    static bool begin(T* p, uint32_t length, uint32_t step) {
        static_assert(sizeof(T) == 1u || sizeof(T) == 2u || sizeof(T) == 4u,
                      "a DMA beat is one bus access wide: 1, 2 or 4 bytes");
        static_assert(sizeof(T) <= sizeof(Elem),
                      "a run's beat is at most the engine's Elem - the widest the binding allows");
        if (!dma_run_valid<T>(p, length)) {
            return false;
        }
        if (Channel::busy()) {
            // Never on a transport's path - a run is re-armed when it has
            // ended - but a caller that restarts a running receive gets
            // E13's abort, which leaves CTRL as it stood on this chip.
            (void)Channel::abort();
        }
        Channel::clear_credits();   // a stale credit is a read of nothing (DmaChannel)
        capacity_ = length;
        taken_ = 0;
        Binding::steer(base_ | dma_beat_bits<T>() | step);
        Channel::write_addr() = dma_address(p);
        dma_handoff_fence();
        Channel::trans_count_trig() = length;
        return true;
    }
    /// Bound: the data register written ONCE, here (the read side does
    /// not move).
    static void bind() {
        Binding::bind(base_ | dma_beat_bits<Elem>() | DMA_CH0_CTRL_TRIG_INCR_WRITE_BITS, line, true);
        Channel::read_addr() = data_;
    }

    static inline uint32_t data_ = 0;
    static inline uint32_t base_ = 0;
    static inline uint32_t capacity_ = 0;
    static inline uint32_t taken_ = 0;
    static inline uint32_t faults_ = 0;
};

/**
 * COPY AND FILL as an engine: one channel in memory-to-memory mode, the
 * controller's native case (2.5: "the DMA transfers data between two
 * buffers in RAM, as fast as possible" - TREQ PERMANENT, one read and one
 * write a clock). copy() moves `n` elements from `src` to `dst`; fill()
 * writes `n` copies of the ONE cell `cell` - the read address held, the
 * write address moving. The beat is the element type, a word being the
 * fastest the bus has (four bytes a clock). A block is one read of BUSY
 * (a block over a running one is refused) and three stores, alias 1's
 * tuple READ_ADDR, WRITE_ADDR, TRANS_COUNT_TRIG - four when the beat or
 * the read step changed. Completion is the line's interrupt
 * (`DmaReport::blocks`) or busy() polled (`DmaReport::errors`). A PACED
 * transfer - a table into one cell at a timer's rate - is not this
 * engine's: it is a DmaTxEngine whose request is a DmaTimer's.
 */
template <uint8_t ch, uint8_t line = 0>
class DmaCopyEngine {
    static_assert(ch < dma_channel_count, "no such DMA channel for this engine");
    static_assert(line < dma_line_count, "the RP2040 DMA has two interrupt lines, 0 and 1");
    using Channel = DmaChannel<ch>;
    using Binding = DmaBinding<ch>;

public:
    DmaCopyEngine() = delete;

    static constexpr bool present = true;
    static constexpr uint8_t channel = ch;
    static constexpr uint8_t irq_line = line;

    static constexpr uint8_t flag_complete = 1u << 0;
    static constexpr uint8_t flag_half = 0;
    static constexpr uint8_t flag_error = 1u << 1;

    [[gnu::always_inline]] static uint8_t service() { return Binding::template service<line>(); }

    /// Bind the channel, unpaced (TREQ PERMANENT). With
    /// `DmaReport::blocks` every block's end reaches `line`, enabled in
    /// the calling core's NVIC; with `DmaReport::errors` the channel is
    /// left off every line and the caller polls busy() - a bus error then
    /// stands in errors() and stops the block. NORMAL priority by
    /// default, the rule's level for a copy: no request paces it, and a
    /// channel it outranked would wait behind it.
    static void arm(DmaReport report = DmaReport::blocks, bool high_priority = false) {
        const bool routed = report == DmaReport::blocks;
        base_ = dma_binding_word(ch, Dreq::permanent, high_priority, report);
        Binding::bind(base_ | dma_beat_bits<uint32_t>() | both_, line, routed);
        if (routed) {
            DmaLine<line>::enable();
        }
    }

    /// `n` elements from `src` to `dst`. False while a block runs, for an
    /// empty or a misaligned one.
    template <typename T>
    static bool copy(T* dst, const T* src, uint32_t n) {
        return program(dst, src, n, both_);
    }
    /// `n` copies of the element at `cell` into `dst`.
    template <typename T>
    static bool fill(T* dst, const T* cell, uint32_t n) {
        return program(dst, cell, n, DMA_CH0_CTRL_TRIG_INCR_WRITE_BITS);
    }

    /// A block in progress, as the channel says (CTRL.BUSY).
    static bool busy() { return Channel::busy(); }
    /// A bus error the last block met (DmaError's bits), and its clear.
    static uint32_t errors() { return Channel::errors(); }
    static void clear_errors() { Channel::clear_errors(); }
    /// Elements the running block has still to move (TRANS_COUNT, E12).
    static uint32_t remaining() { return Channel::count(); }

    static void stop() { Binding::release(); }

private:
    static constexpr uint32_t both_ = DMA_CH0_CTRL_TRIG_INCR_READ_BITS | DMA_CH0_CTRL_TRIG_INCR_WRITE_BITS;

    template <typename T>
    static bool program(T* dst, const T* src, uint32_t n, uint32_t step) {
        static_assert(sizeof(T) == 1u || sizeof(T) == 2u || sizeof(T) == 4u,
                      "a DMA element is one bus access wide: 1, 2 or 4 bytes");
        if (!dma_run_valid<T>(dst, n) || !dma_beat_aligned<T>(src) || src == nullptr || Channel::busy()) {
            return false;
        }
        Binding::steer(base_ | dma_beat_bits<T>() | step);
        Channel::read_addr() = dma_address(src);
        Channel::write_addr() = dma_address(dst);
        dma_handoff_fence();
        Channel::trans_count_trig() = n;
        return true;
    }

    static inline uint32_t base_ = 0;
};

} // namespace brio

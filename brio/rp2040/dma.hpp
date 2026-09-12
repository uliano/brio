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
 *                 here from TRANS_COUNT, which is erratum E12's rule
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

/**
 * A transmit engine: one caller-owned run poured into one peripheral
 * register, paced by that peripheral's request. `line` is the system
 * interrupt line the completion reports on - the owner core's.
 */
template <uint8_t ch, typename Elem = uint8_t, uint8_t line = 0>
class DmaTxEngine {
    static_assert(ch < dma_channel_count, "no such DMA channel for this engine");
    static_assert(line < dma_line_count, "the RP2040 DMA has two interrupt lines, 0 and 1");
    using Channel = DmaChannel<ch>;

public:
    DmaTxEngine() = delete;

    static constexpr bool present = true;
    static constexpr uint8_t channel = ch;
    static constexpr uint8_t irq_line = line;
    static constexpr DmaSize size = dma_size_of<Elem>();
    using element = Elem;

    static constexpr uint8_t flag_complete = 1u << 0;
    static constexpr uint8_t flag_half = 0;   ///< this controller has no half-transfer event
    static constexpr uint8_t flag_error = 1u << 1;

    /// The ISR body folded into the engine: this channel's status on
    /// its line, cleared and handed back as flags - the error read
    /// first, since a bus error raises the completion too.
    [[gnu::always_inline]] static uint8_t service() {
        uint8_t f = 0;
        if (Channel::pending(line)) {
            Channel::clear_pending(line);
            f |= flag_complete;
            if (Channel::errors() != 0u) {
                Channel::clear_errors();
                f |= flag_error;
            }
        }
        return f;
    }

    /// Claim the channel: `data` is the register the run is poured
    /// into, `dreq` the peripheral's transmit request. The channel is
    /// routed to `line` and the line enabled in the calling core's NVIC.
    static void arm(volatile void* data, Dreq dreq, bool high_priority = false) {
        data_ = data;
        dreq_ = dreq;
        high_ = high_priority;
        claim();
        DmaLine<line>::enable();
    }

    /// Start moving `length` elements from `buffer` (the caller's, and
    /// it stays put until complete() reports the block).
    static bool start(const Elem* buffer, uint32_t length) {
        return begin(buffer, length, true);
    }
    /// `length` copies of ONE element: the read pointer does not
    /// increment. What a full-duplex bus needs to clock a read.
    static bool start_fixed(const Elem* cell, uint32_t length) {
        return begin(cell, length, false);
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
    /// back ready.
    static bool abandon() {
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
    static bool begin(const Elem* p, uint32_t length, bool incr) {
        if (busy_ || p == nullptr || length == 0u) {
            return false;
        }
        in_flight_ = length;
        busy_ = Channel::load(DmaTransfer{
            .read = p,
            .write = data_,
            .count = length,
            .config = {.size = size,
                       .incr_read = incr,
                       .incr_write = false,
                       .treq = dreq_,
                       .high_priority = high_},
        });
        if (!busy_) {
            in_flight_ = 0;
        }
        return busy_;
    }
    static void claim() {
        Channel::stop();
        Channel::route(line, true);
    }

    static inline volatile void* data_ = nullptr;
    static inline Dreq dreq_ = Dreq::permanent;
    static inline bool high_ = false;
    static inline uint32_t in_flight_ = 0;
    static inline uint32_t faults_ = 0;
    static inline bool busy_ = false;
};

/**
 * A receive engine: one caller-owned run filled from one peripheral
 * register, and asked how much has arrived - from TRANS_COUNT, one
 * read, nothing suspended (erratum E12: never from WRITE_ADDR).
 */
template <uint8_t ch, typename Elem = uint8_t, uint8_t line = 0>
class DmaRxEngine {
    static_assert(ch < dma_channel_count, "no such DMA channel for this engine");
    static_assert(line < dma_line_count, "the RP2040 DMA has two interrupt lines, 0 and 1");
    using Channel = DmaChannel<ch>;

public:
    DmaRxEngine() = delete;

    static constexpr bool present = true;
    static constexpr uint8_t channel = ch;
    static constexpr uint8_t irq_line = line;
    static constexpr DmaSize size = dma_size_of<Elem>();
    using element = Elem;

    static constexpr uint8_t flag_complete = 1u << 0;
    static constexpr uint8_t flag_half = 0;
    static constexpr uint8_t flag_error = 1u << 1;

    [[gnu::always_inline]] static uint8_t service() {
        uint8_t f = 0;
        if (Channel::pending(line)) {
            Channel::clear_pending(line);
            f |= flag_complete;
            if (Channel::errors() != 0u) {
                Channel::clear_errors();
                f |= flag_error;
            }
        }
        return f;
    }

    static void arm(volatile void* data, Dreq dreq, bool high_priority = false) {
        data_ = data;
        dreq_ = dreq;
        high_ = high_priority;
        claim();
        DmaLine<line>::enable();
    }

    /// Nothing in progress: the run filled, or none started.
    static bool idle() { return !Channel::busy(); }

    static bool start(Elem* buffer, uint32_t length) { return begin(buffer, length, true); }
    /// `length` elements into ONE cell, thrown away (the receive side of
    /// a bus write).
    static bool start_discard(Elem* cell, uint32_t length) { return begin(cell, length, false); }

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
    static bool begin(Elem* p, uint32_t length, bool incr) {
        if (p == nullptr || length == 0u) {
            return false;
        }
        if (Channel::busy()) {
            (void)Channel::abort();
        }
        Channel::clear_credits();   // a stale credit is a read of nothing (DmaChannel)
        capacity_ = length;
        taken_ = 0;
        const bool ok = Channel::load(DmaTransfer{
            .read = data_,
            .write = p,
            .count = length,
            .config = {.size = size,
                       .incr_read = false,
                       .incr_write = incr,
                       .treq = dreq_,
                       .high_priority = high_},
        });
        if (!ok) {
            capacity_ = 0;
        }
        return ok;
    }
    static void claim() {
        Channel::stop();
        Channel::route(line, true);
    }

    static inline volatile void* data_ = nullptr;
    static inline Dreq dreq_ = Dreq::permanent;
    static inline bool high_ = false;
    static inline uint32_t capacity_ = 0;
    static inline uint32_t taken_ = 0;
    static inline uint32_t faults_ = 0;
};

} // namespace brio

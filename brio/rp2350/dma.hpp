/*
 * dma.hpp
 *
 * The RP2350's DMA controller (datasheet 12.6): SIXTEEN channels over one
 * read master and one write master, each channel four live registers -
 * READ_ADDR, WRITE_ADDR, TRANS_COUNT, CTRL - aliased four times so that
 * any one of them can be the TRIGGER (12.6.3.1), a credit-based data
 * request per channel that keeps as many transfers in flight as the
 * peripheral has room for (12.6.4.2), FOUR system interrupt lines each
 * with its own enable, force and status bank (12.6.5), chaining, address
 * wrapping, four pacing timers, a passive checksum sniffer, an abort
 * register, and - new on this chip - a security level per channel, per
 * line, per timer and for the sniffer, plus a memory protection unit of
 * eight regions in front of both masters.
 *
 *  Dma            the block: out of the reset controller once, the raw
 *                 interrupt status, the channel count, a trigger or an
 *                 abort over several channels, the debug FIFO levels,
 *                 and the security assignment READ BACK
 *  DmaChannel<ch> one channel, 0..15: a transfer prepared or loaded
 *                 (loaded = triggered), the live count and its MODE, the
 *                 bus errors, the route to any of the four lines, the
 *                 abort with erratum RP2350-E5's workaround, the debug
 *                 registers
 *  DmaLine<n>     one of the FOUR interrupt lines: its interrupt number,
 *                 the masked status, the force bits - a line belongs to
 *                 the CORE whose kernel serves it (rule 2 of the
 *                 two-kernel model)
 *  DmaTimer<n>    a pacing timer (12.6.8.1): X/Y of clk_sys as a request
 *  DmaSniffer     the checksum sniffer (12.6.8.2) over one channel's data
 *  DmaMpu         the memory protection unit (12.6.6.3), READ-ONLY here
 *  DmaTxEngine<ch, Elem, line> / DmaRxEngine<ch, Elem, line>
 *                 the two engines util-level transports name in their
 *                 slots, the other strata's surface: a transmit engine
 *                 pours a caller-owned run into one peripheral register
 *                 and reports what the block carried when it completes;
 *                 a receive engine fills a caller-owned run and answers
 *                 how many items have arrived since it was last asked
 *
 * WHAT DIFFERS FROM THE OTHER FAMILIES' CONTROLLERS, stated once:
 *  - the request is a FIELD, not a channel: any channel takes any DREQ
 *    (12.6.4.1), so an engine is named by a free channel and told its
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
 *  - a bus error halts the channel, marks CTRL (AHB_ERROR with READ_ERROR
 *    or WRITE_ERROR, cleared by writing one) and RAISES THE CHANNEL'S
 *    INTERRUPT; the channel refuses to restart until the flags are
 *    cleared, and 12.6.7.2 wants BUSY read down first;
 *  - one FIFO served by a channel must not be touched by the processor
 *    (12.6.4.2): a transport with an engine on a direction leaves that
 *    direction's data register to the engine.
 *
 * WHAT DIFFERS FROM THE RP2040'S SAME BLOCK (12.6.1), which is the
 * reason this file is an adapted copy and not a shared one:
 *  - SIXTEEN channels against twelve, and FOUR interrupt lines against
 *    two - so CHAIN_TO is four bits wide and every bit map here is
 *    sixteen bits;
 *  - THE TOP FOUR BITS OF TRANS_COUNT ARE A MODE. 0x0 counts down and
 *    halts (the RP2040's only behaviour), 0x1 re-triggers the channel at
 *    zero (TRIGGER_SELF: the count reloads and the addresses carry on -
 *    the interrupt and the CHAIN_TO still happen), 0xf never decrements
 *    at all (ENDLESS: the register description is explicit that such a
 *    channel triggers no other channel and raises no interrupt, and only
 *    an abort ends it). A count is therefore 28 bits, and a READ of
 *    TRANS_COUNT carries the mode in its top nibble - every verb here
 *    that reads progress MASKS it off;
 *  - AN ADDRESS HAS FOUR STEPS, not two. INCR_READ/INCR_WRITE gained a
 *    REV partner, and the four combinations are: the same address every
 *    time, forward by the transfer size, BACKWARD by it, or forward by
 *    TWICE it (skipping alternate cells). `DmaStep` is that field pair
 *    as one enumeration, so the illegal reading of two bools cannot be
 *    written;
 *  - the CTRL fields above INCR_READ have all MOVED to make room for
 *    them (BUSY is bit 26 here, 24 there): every constant below is the
 *    device header's own name, and not one is retyped;
 *  - RP2040-E12 IS GONE: the read-back adjustment that made READ_ADDR and
 *    WRITE_ADDR lie during a wrapping or non-incrementing sequence is
 *    disabled for exactly those transfers now. Progress is still read
 *    from TRANS_COUNT here, because a count is what a run's accounting is
 *    in - one read, and no arithmetic against a base address;
 *  - RP2040-E13 IS GONE TOO, and replaced: the ABORT register may now be
 *    POLLED to completion (12.6.8.3), which is what abort() below does
 *    instead of waiting on BUSY alone. But RP2350-E5 arrives with it -
 *    see the abort verb;
 *  - the DREQ table has not one row in the RP2040's place (rp2350/
 *    dma_engine.hpp holds it).
 *
 * THE TWO ERRATA OF THIS CHAPTER, both live on stepping A2:
 *  - RP2350-E5, aborting against CHAIN_TO: an aborted channel may fire
 *    its CHAIN_TO, and may itself be re-triggered on the last cycle of
 *    the abort and then complete immediately - raising its interrupt and
 *    its chain again. The datasheet's own sequence is the workaround and
 *    it is what Dma::abort() does: EN cleared AND CHAIN_TO pointed at the
 *    channel itself (which is how this chapter spells "chain to nobody")
 *    on every channel of the abort, THEN the CHAN_ABORT write, THEN the
 *    poll. DmaChannel<ch>::abort() adds what a single channel knows: its
 *    routes come off all four lines first and go back afterwards, so
 *    nothing the abort raises reaches a handler.
 *  - RP2350-E8, CHAIN_TO against a zero-length transfer: a channel
 *    triggered with a count of zero completes with no bus access, and its
 *    CHAIN_TO then fires only by accident. THERE IS NO WAY TO REACH IT
 *    FROM THIS FILE: a zero count is refused by dma_transfer_valid, by
 *    set_count and by every verb that starts a channel, so no sequence
 *    this driver can start has length zero.
 *
 * SECURITY, AND WHY THERE IS NO WRITE VERB FOR IT (12.6.6). Channels,
 * lines, the timers and the sniffer each carry a security level, and the
 * MPU filters both masters' addresses against eight regions. At reset
 * every channel, line, timer and the sniffer are SP - secure and
 * privileged - and the MPU's global level is the lowest, so a program
 * that runs entirely Secure and Privileged (which is what brio runs on
 * this chip: the bootrom hands over in that state and no stratum leaves
 * it) is never refused by any of it. What this file offers is therefore
 * the READ side alone - what the levels are, and whether a channel's
 * assignment has locked - because a level written low cannot be written
 * back except through the block's reset, and because nothing here has a
 * second security domain to hand a channel to. Note one automatic
 * effect a program should know about: a successful write to a channel's
 * control registers LOCKS that channel's SECCFG register (12.6.6.1), so
 * after the first prepare() or load() the assignment is read-only until
 * Dma::init() cycles the block.
 *
 * TWO CORES. The controller is one; a channel belongs to the core whose
 * kernel hosts the transport that armed it, and it reports on the line
 * that core enabled. THE CONVENTION OF THIS STRATUM, with four lines to
 * spend: line 0 is core 0's and line 1 is core 1's, as on the RP2040,
 * and lines 2 and 3 are SPARE - a second line a core may take for a
 * channel whose completion it wants served apart from the rest. Which is
 * why every engine and every channel verb that routes takes the line as
 * a parameter and enables it in the CALLING core's interrupt controller.
 */

#pragma once

#include <stddef.h>
#include <stdint.h>

#include "rp2350/device.hpp"

#include "rp2350/core.hpp"
#include "rp2350/dma_engine.hpp"
#include "rp2350/resets.hpp"

namespace brio {

// ---- the vocabulary ------------------------------------------------------------

inline constexpr uint8_t dma_channel_count = 16;

/// The four system interrupt lines (12.6.5).
inline constexpr uint8_t dma_line_count = 4;

/// The four pacing timers (12.6.8.1).
inline constexpr uint8_t dma_timer_count = 4;

/// CTRL.DATA_SIZE: reads and writes are the same width.
enum class DmaSize : uint8_t { byte = 0, half = 1, word = 2 };

template <typename Elem>
constexpr DmaSize dma_size_of() {
    static_assert(sizeof(Elem) == 1 || sizeof(Elem) == 2 || sizeof(Elem) == 4,
                  "a DMA element is one bus access wide: 1, 2 or 4 bytes");
    return sizeof(Elem) == 1 ? DmaSize::byte : sizeof(Elem) == 2 ? DmaSize::half : DmaSize::word;
}

constexpr uint8_t dma_size_bytes(DmaSize s) { return static_cast<uint8_t>(1u << static_cast<uint8_t>(s)); }

/**
 * How an address moves after each transfer: the INCR and INCR_REV pair of
 * one side as ONE value, because the four combinations are four distinct
 * behaviours and none of them is "both bits happen to be set".
 */
enum class DmaStep : uint8_t {
    fixed = 0,           ///< INCR 0, REV 0: every access at the same address (a peripheral FIFO)
    forward = 1,         ///< INCR 1, REV 0: + the transfer size
    backward = 2,        ///< INCR 1, REV 1: - the transfer size
    forward_by_two = 3,  ///< INCR 0, REV 1: + twice the transfer size, skipping alternate cells
};

constexpr bool dma_step_increments(DmaStep s) {
    return s == DmaStep::forward || s == DmaStep::backward;
}
constexpr bool dma_step_reversed(DmaStep s) {
    return s == DmaStep::backward || s == DmaStep::forward_by_two;
}

/// TRANS_COUNT.MODE, the top four bits of the count register (12.6.2.2.1).
/// Every other value of the field is reserved.
enum class DmaCountMode : uint8_t {
    normal = DMA_CH0_TRANS_COUNT_MODE_VALUE_NORMAL,
    /// The channel re-triggers itself at zero: the count reloads and the
    /// addresses carry on. The completion interrupt and the CHAIN_TO still
    /// happen, which is what makes a ring buffer with periodic interrupts.
    trigger_self = DMA_CH0_TRANS_COUNT_MODE_VALUE_TRIGGER_SELF,
    /// The count never decrements: an endless sequence that triggers no
    /// other channel and raises no interrupt, ended by an abort alone.
    endless = DMA_CH0_TRANS_COUNT_MODE_VALUE_ENDLESS,
};

constexpr bool dma_count_mode_valid(DmaCountMode m) {
    return m == DmaCountMode::normal || m == DmaCountMode::trigger_self ||
           m == DmaCountMode::endless;
}

/// The longest sequence this chip counts: 2^28 - 1, the four most
/// significant bits of TRANS_COUNT being the mode.
inline constexpr uint32_t dma_count_max = DMA_CH0_TRANS_COUNT_COUNT_BITS;

/// "Chain to nobody": CHAIN_TO set to the channel's own index (12.6.3.2).
inline constexpr uint8_t dma_no_chain = 0xFFu;

/// Everything of CTRL but EN and the status bits.
struct DmaChannelConfig {
    DmaSize size = DmaSize::byte;
    DmaStep read_step = DmaStep::forward;   ///< the source is a buffer by default
    DmaStep write_step = DmaStep::forward;
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
    if (dma_step_increments(c.read_step)) { v |= DMA_CH0_CTRL_TRIG_INCR_READ_BITS; }
    if (dma_step_reversed(c.read_step)) { v |= DMA_CH0_CTRL_TRIG_INCR_READ_REV_BITS; }
    if (dma_step_increments(c.write_step)) { v |= DMA_CH0_CTRL_TRIG_INCR_WRITE_BITS; }
    if (dma_step_reversed(c.write_step)) { v |= DMA_CH0_CTRL_TRIG_INCR_WRITE_REV_BITS; }
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

/// TRANS_COUNT's word: the mode in the top nibble, the count below it.
constexpr uint32_t dma_count_word(uint32_t count, DmaCountMode mode) {
    return (static_cast<uint32_t>(mode) << DMA_CH0_TRANS_COUNT_MODE_LSB) |
           (count & DMA_CH0_TRANS_COUNT_COUNT_BITS);
}

/// One transfer sequence: both ends, the count in ITEMS, the count mode,
/// the config.
struct DmaTransfer {
    const volatile void* read = nullptr;
    volatile void* write = nullptr;
    uint32_t count = 0;
    DmaCountMode mode = DmaCountMode::normal;
    DmaChannelConfig config{};
};

constexpr bool dma_aligned(const volatile void* p, DmaSize s) {
    return (reinterpret_cast<uintptr_t>(p) & (dma_size_bytes(s) - 1u)) == 0u;
}

/**
 * Both ends present and aligned to the size (12.6.2.1.1: the DMA does not
 * enforce alignment and the result of an unaligned access is
 * unspecified), a count within the 28 bits the mode leaves and NOT ZERO -
 * which is also erratum RP2350-E8 answered, since a zero-length sequence
 * is the one case whose CHAIN_TO the silicon gets wrong - and a legal
 * config and mode.
 */
constexpr bool dma_transfer_valid(const DmaTransfer& t, uint8_t ch) {
    return t.read != nullptr && t.write != nullptr && t.count != 0u && t.count <= dma_count_max &&
           dma_count_mode_valid(t.mode) && dma_aligned(t.read, t.config.size) &&
           dma_aligned(t.write, t.config.size) && dma_channel_config_valid(t.config, ch);
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

/// The four security levels of 12.6.6, ordered sp > su > nsp > nsu, as
/// the S and P bits spell them.
enum class DmaSecurity : uint8_t {
    nsu = 0,  ///< non-secure, unprivileged
    nsp = 1,  ///< non-secure, privileged
    su = 2,   ///< secure, unprivileged
    sp = 3,   ///< secure and privileged - what every resource is at reset
};

/// The three debug fill levels of 12.6.8.4.
struct DmaFifoLevels {
    uint8_t read_address = 0;
    uint8_t write_address = 0;
    uint8_t transfer_data = 0;
};

// ---- the block -------------------------------------------------------------------

static_assert(offsetof(DMA_Type, CH1_READ_ADDR) == offsetof(DMA_Type, CH0_READ_ADDR) + 0x40u,
              "a channel's four registers and their aliases are sixteen words");
static_assert(offsetof(DMA_Type, CH0_AL1_CTRL) == offsetof(DMA_Type, CH0_READ_ADDR) + 0x10u);
static_assert(DMA_INTE1_OFFSET - DMA_INTE0_OFFSET == 0x10u &&
                  DMA_INTE3_OFFSET - DMA_INTE0_OFFSET == 0x30u,
              "the four interrupt banks are four words apart with one reserved between them");

struct Dma {
    Dma() = delete;

    /// The block out of the reset controller: every channel at its reset
    /// state, every enable clear, every security assignment back at SP
    /// with its lock dropped. Once per program, before any channel.
    static bool init() { return Resets::cycle(ResetBlock::dma); }
    static bool released() { return Resets::released(ResetBlock::dma); }

    /// How many channels the silicon says it has (N_CHANNELS).
    static uint8_t channels() {
        return static_cast<uint8_t>(DMA->N_CHANNELS & DMA_N_CHANNELS_BITS);
    }

    /// The raw interrupt status, one bit per channel: set at every
    /// completion (or null trigger under IRQ_QUIET, or bus error),
    /// whatever the lines' enables; cleared by writing one.
    static uint32_t raw() { return DMA->INTR; }
    static void clear_raw(uint32_t mask) { DMA->INTR = mask; }

    /// Start several channels at once (MULTI_CHAN_TRIGGER): the ones
    /// prepared, enabled and not running.
    static void trigger(uint32_t mask) { DMA->MULTI_CHAN_TRIGGER = mask; }

    /// The raw abort of several channels (CHAN_ABORT), WITHOUT erratum
    /// RP2350-E5's preparation: abort() is the verb with it.
    static void abort_raw(uint32_t mask) { DMA->CHAN_ABORT = mask; }

    /// The channels whose abort has not yet flushed its in-flight
    /// transfers: 12.6.8.3's poll, which the RP2040 did not have.
    static uint32_t abort_pending() { return DMA->CHAN_ABORT; }

    /**
     * END SEVERAL CHANNELS EARLY, 12.6.8.3's sequence with erratum
     * RP2350-E5 in it: on every channel of `mask` the EN bit is cleared
     * and CHAIN_TO is pointed at the channel itself (the erratum's
     * workaround - an abort may otherwise fire a chain, and a chained
     * channel may re-trigger the one being aborted), then CHAN_ABORT is
     * written and POLLED to zero, then the raw status and the four
     * lines' pending bits of those channels are cleared.
     *
     * THE WHOLE CHAIN AT ONCE is what this verb is for: 12.6.8.3's
     * closing advice is to abort every channel involved in a chain
     * together, and a mask is how that is said. False when the abort did
     * not flush within the bounded wait.
     *
     * The error flags are deliberately left standing: the read-modify-
     * write below masks them out of the value it stores, so a bus error
     * a caller has not read yet survives its own abort.
     */
    static bool abort(uint32_t mask) {
        for (uint8_t ch = 0; ch < dma_channel_count; ++ch) {
            if ((mask & (1u << ch)) == 0u) {
                continue;
            }
            volatile uint32_t& ctrl = channel_ctrl(ch);
            uint32_t v = ctrl;
            v &= ~(DMA_CH0_CTRL_TRIG_EN_BITS | DMA_CH0_CTRL_TRIG_CHAIN_TO_BITS | DmaError::all);
            v |= static_cast<uint32_t>(ch) << DMA_CH0_CTRL_TRIG_CHAIN_TO_LSB;
            ctrl = v;
        }
        abort_raw(mask);
        bool settled = false;
        for (uint32_t spins = 100'000u; spins != 0u; --spins) {
            if ((abort_pending() & mask) == 0u) {
                settled = true;
                break;
            }
        }
        clear_raw(mask);
        for (uint8_t line = 0; line < dma_line_count; ++line) {
            status(line) = mask;
        }
        return settled;
    }

    /// The debug fill levels of the three internal FIFOs (12.6.8.4).
    static DmaFifoLevels fifo_levels() {
        const uint32_t v = DMA->FIFO_LEVELS;
        return DmaFifoLevels{
            static_cast<uint8_t>((v & DMA_FIFO_LEVELS_RAF_LVL_BITS) >> DMA_FIFO_LEVELS_RAF_LVL_LSB),
            static_cast<uint8_t>((v & DMA_FIFO_LEVELS_WAF_LVL_BITS) >> DMA_FIFO_LEVELS_WAF_LVL_LSB),
            static_cast<uint8_t>((v & DMA_FIFO_LEVELS_TDF_LVL_BITS) >> DMA_FIFO_LEVELS_TDF_LVL_LSB)};
    }

    // ---- the security assignment, read back (12.6.6) ---------------------

    /// What level a channel's bus accesses carry, and the minimum level
    /// needed to touch its registers.
    static DmaSecurity channel_security(uint8_t ch) {
        return static_cast<DmaSecurity>(seccfg_channel(ch) &
                                        (DMA_SECCFG_CH0_S_BITS | DMA_SECCFG_CH0_P_BITS));
    }
    /// Whether that assignment is frozen until the block's next reset -
    /// which a successful write to the channel's control registers does
    /// by itself (12.6.6.1).
    static bool channel_security_locked(uint8_t ch) {
        return (seccfg_channel(ch) & DMA_SECCFG_CH0_LOCK_BITS) != 0u;
    }
    /// The level of one interrupt line (12.6.6.2): a channel above it is
    /// invisible in that line's registers and cannot assert it.
    static DmaSecurity line_security(uint8_t line) {
        return static_cast<DmaSecurity>(seccfg_line(line) &
                                        (DMA_SECCFG_IRQ0_S_BITS | DMA_SECCFG_IRQ0_P_BITS));
    }
    /// The sniffer's level: the HIGHEST channel level it can observe.
    static DmaSecurity sniffer_security() {
        const uint32_t v = DMA->SECCFG_MISC;
        return level_of((v & DMA_SECCFG_MISC_SNIFF_S_BITS) != 0u,
                        (v & DMA_SECCFG_MISC_SNIFF_P_BITS) != 0u);
    }
    /// A pacing timer's level: the minimum a channel needs to observe its
    /// request, and the minimum a bus access needs to configure it.
    static DmaSecurity timer_security(uint8_t timer) {
        const uint32_t v = DMA->SECCFG_MISC;
        const uint8_t shift = static_cast<uint8_t>(2u * timer);
        return level_of((v & (DMA_SECCFG_MISC_TIMER0_S_BITS << shift)) != 0u,
                        (v & (DMA_SECCFG_MISC_TIMER0_P_BITS << shift)) != 0u);
    }

    /// One channel's CTRL through the NON-trigger alias, by index: what a
    /// verb over a mask needs and a template parameter cannot give.
    static volatile uint32_t& channel_ctrl(uint8_t ch) {
        return reg_at(DMA_BASE, DMA_CH0_AL1_CTRL_OFFSET + 0x40u * ch);
    }
    /// INTEn / INTFn / INTSn by line: the four banks are 0x10 apart.
    static volatile uint32_t& enables(uint8_t line) {
        return reg_at(DMA_BASE, DMA_INTE0_OFFSET + 0x10u * line);
    }
    static volatile uint32_t& force_bits(uint8_t line) {
        return reg_at(DMA_BASE, DMA_INTF0_OFFSET + 0x10u * line);
    }
    static volatile uint32_t& status(uint8_t line) {
        return reg_at(DMA_BASE, DMA_INTS0_OFFSET + 0x10u * line);
    }

private:
    static constexpr DmaSecurity level_of(bool s, bool p) {
        return static_cast<DmaSecurity>((s ? 2u : 0u) | (p ? 1u : 0u));
    }
    static uint32_t seccfg_channel(uint8_t ch) {
        return reg_at(DMA_BASE, DMA_SECCFG_CH0_OFFSET + 4u * ch);
    }
    static uint32_t seccfg_line(uint8_t line) {
        return reg_at(DMA_BASE, DMA_SECCFG_IRQ0_OFFSET + 4u * line);
    }
};

/// One of the four system interrupt lines.
template <uint8_t line>
struct DmaLine {
    static_assert(line < dma_line_count, "the RP2350 DMA has four interrupt lines, 0 to 3");
    DmaLine() = delete;

    static constexpr IRQn_Type irq() {
        return static_cast<IRQn_Type>(static_cast<int>(DMA_IRQ_0_IRQn) + line);
    }

    /// The channels routed to this line (INTEn).
    static uint32_t routed() { return Dma::enables(line); }
    /// The masked status (INTSn): routed AND raised; cleared by writing one.
    static uint32_t pending() { return Dma::status(line); }
    static void clear(uint32_t mask) { Dma::status(line) = mask; }
    /// The force bits (INTFn): a channel's interrupt asserted by software.
    static void force(uint32_t mask, bool on) {
        if (on) { hw_set(Dma::force_bits(line), mask); } else { hw_clear(Dma::force_bits(line), mask); }
    }

    /// The line in the CALLING core's interrupt controller.
    static void enable() { Irq::enable(irq()); }
    static void disable() { Irq::disable(irq()); }
};

/// One channel, 0..15.
template <uint8_t ch>
class DmaChannel {
    static_assert(ch < dma_channel_count, "the RP2350 DMA has channels 0..15");

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
    /// channel (12.6.8.3), it does not end it - and BUSY stays up while
    /// it is paused.
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
    /// The count and the mode of the NEXT sequence (12.6.2.2).
    static bool set_count(uint32_t count, DmaCountMode mode = DmaCountMode::normal) {
        if (busy() || count == 0u || count > dma_count_max || !dma_count_mode_valid(mode)) {
            return false;
        }
        trans_count() = dma_count_word(count, mode);
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
        trans_count() = dma_count_word(t.count, t.mode);
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
        trans_count() = dma_count_word(t.count, t.mode);
        ctrl_trig() = dma_ctrl_word(t.config, ch, true);
        return true;
    }

    /// Start a prepared channel (MULTI_CHAN_TRIGGER's bit).
    static void trigger() { Dma::trigger(bit); }

    /// A NULL TRIGGER: zero into the trigger alias starts nothing and,
    /// under IRQ_QUIET, is what raises the channel's interrupt - the end
    /// of a control-block chain (12.6.3.3).
    static void null_trigger() { ctrl_trig() = 0; }

    /// Transfers remaining in the current sequence, live - WITHOUT the
    /// mode, which shares the register's top nibble here.
    static uint32_t count() { return trans_count() & DMA_CH0_TRANS_COUNT_COUNT_BITS; }

    /// What the count register does when it reaches zero (12.6.2.2.1).
    static DmaCountMode mode() {
        return static_cast<DmaCountMode>((trans_count() & DMA_CH0_TRANS_COUNT_MODE_BITS) >>
                                         DMA_CH0_TRANS_COUNT_MODE_LSB);
    }

    static DmaProgress progress(uint32_t programmed) {
        const uint32_t remaining = count();
        return DmaProgress{remaining, remaining > programmed ? 0u : programmed - remaining};
    }

    /// CTRL's error bits: AHB_ERROR (the read-only OR of the two) with
    /// READ_ERROR and WRITE_ERROR; a bus error halts the channel and
    /// raises its interrupt, and the channel refuses to restart until
    /// they are cleared (12.6.7.2).
    static uint32_t errors() { return ctrl() & DmaError::all; }
    static void clear_errors() { hw_set(ctrl(), DmaError::read | DmaError::write); }

    /// The raw status bit (INTR) and its clear.
    static bool raised() { return (Dma::raw() & bit) != 0u; }
    static void clear_raised() { Dma::clear_raw(bit); }

    /// Route the channel's interrupt to `line`, or take it off it.
    static void route(uint8_t line, bool on) {
        if (on) { hw_set(Dma::enables(line), bit); } else { hw_clear(Dma::enables(line), bit); }
    }
    static bool routed(uint8_t line) { return (Dma::enables(line) & bit) != 0u; }
    /// Pending on `line`: routed there and raised; cleared by writing one
    /// (which clears the raw bit too).
    static bool pending(uint8_t line) { return (Dma::status(line) & bit) != 0u; }
    static void clear_pending(uint8_t line) { Dma::status(line) = bit; }

    /**
     * End the sequence early, erratum RP2350-E5's way: the routes to all
     * four lines lifted so nothing the abort raises reaches a handler,
     * Dma::abort() (EN and the chain cleared, CHAN_ABORT written and
     * POLLED to zero - 12.6.8.3's own recovery, which the RP2040 could
     * not do), the routes restored. False when the abort did not flush
     * or BUSY is still up.
     *
     * A channel in a CHAIN wants Dma::abort() over the whole chain's mask
     * instead: this verb knows one channel, and the erratum's other half
     * is the chainee re-triggering it.
     */
    static bool abort() {
        uint32_t was_routed = 0;
        for (uint8_t line = 0; line < dma_line_count; ++line) {
            if (routed(line)) {
                was_routed |= 1u << line;
                route(line, false);
            }
        }
        const bool settled = Dma::abort(bit);
        for (uint8_t line = 0; line < dma_line_count; ++line) {
            if ((was_routed & (1u << line)) != 0u) {
                route(line, true);
            }
        }
        return settled && !busy();
    }

    /// Everything of the channel's back to nothing: aborted, disabled,
    /// off every line, the errors and the status cleared.
    static void stop() {
        (void)abort();
        for (uint8_t line = 0; line < dma_line_count; ++line) {
            route(line, false);
        }
        ctrl() = 0;
        clear_errors();
        Dma::clear_raw(bit);
    }

    /// The request credits the channel holds (DBG_CTDREQ), and their
    /// clear: THE CREDITS OUTLIVE A SEQUENCE. A peripheral keeps pulsing
    /// while the channel sits completed, and a full FIFO that overflowed
    /// meanwhile has pulsed more than it holds - the next sequence would
    /// spend those credits reading an empty FIFO. A level request like
    /// the PL011's re-pulses for every byte still waiting, so clearing
    /// the count before a new sequence loses nothing.
    static uint32_t debug_credits() {
        return reg_at(DMA_BASE, DMA_CH0_DBG_CTDREQ_OFFSET + 0x40u * ch) &
               DMA_CH0_DBG_CTDREQ_BITS;
    }
    static void clear_credits() { reg_at(DMA_BASE, DMA_CH0_DBG_CTDREQ_OFFSET + 0x40u * ch) = 0; }
    /// The value TRANS_COUNT reloads from at every trigger (DBG_TCR).
    static uint32_t debug_reload() { return reg_at(DMA_BASE, DMA_CH0_DBG_TCR_OFFSET + 0x40u * ch); }

    /// This channel's security assignment, read back (12.6.6.1). A write
    /// to the registers above LOCKS it until the block's next reset.
    static DmaSecurity security() { return Dma::channel_security(ch); }
    static bool security_locked() { return Dma::channel_security_locked(ch); }

private:
    static volatile uint32_t& word(uint8_t i) { return *(&DMA->CH0_READ_ADDR + 16u * ch + i); }
};

// ---- the pacing timers (12.6.8.1) --------------------------------------------------

/// A request every Y / X cycles of clk_sys, at most one a cycle; X = 0
/// requests nothing.
template <uint8_t n>
struct DmaTimer {
    static_assert(n < dma_timer_count, "the RP2350 DMA has four pacing timers, 0 to 3");
    DmaTimer() = delete;

    static constexpr Dreq dreq() {
        return n == 0 ? Dreq::timer0 : n == 1 ? Dreq::timer1 : n == 2 ? Dreq::timer2 : Dreq::timer3;
    }
    static void set(uint16_t x, uint16_t y) {
        reg() = (static_cast<uint32_t>(x) << DMA_TIMER0_X_LSB) | y;
    }
    static void stop() { reg() = 0; }

    static uint16_t numerator() {
        return static_cast<uint16_t>((reg() & DMA_TIMER0_X_BITS) >> DMA_TIMER0_X_LSB);
    }
    static uint16_t denominator() { return static_cast<uint16_t>(reg() & DMA_TIMER0_Y_BITS); }

    /// The minimum channel level that may observe this timer's request,
    /// and the minimum bus level that may write it (SECCFG_MISC).
    static DmaSecurity security() { return Dma::timer_security(n); }

private:
    /// Reached by ADDRESS and not as `DMA->TIMER0 + n`, because on this
    /// chip TIMER0 is also the device header's macro for the first SYSTEM
    /// timer's instance pointer, and a member named after a macro is not
    /// a member at all.
    static volatile uint32_t& reg() { return reg_at(DMA_BASE, DMA_TIMER0_OFFSET + 4u * n); }
};

// ---- the sniffer (12.6.8.2) ----------------------------------------------------------

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

    /// The HIGHEST channel level the sniffer may observe (SECCFG_MISC).
    static DmaSecurity security() { return Dma::sniffer_security(); }
};

// ---- the memory protection unit (12.6.6.3), read-only ---------------------------------

/// One region's setting as the two registers hold it: the bounds are
/// 32-byte granular (the 27 most significant bits of each address), and a
/// region matches an address between them inclusive.
struct DmaMpuRegion {
    uint32_t base = 0;
    uint32_t limit = 0;
    DmaSecurity level = DmaSecurity::nsu;
    bool enabled = false;
};

/**
 * The eight-region filter in front of both masters. READ-ONLY here, and
 * the reason is in this file's opening: every brio program on this chip
 * runs Secure and Privileged, and the reset state - a global level of NSU
 * with every region disabled - refuses it nothing. The write side is born
 * with the first program that gives a channel to another security domain.
 */
struct DmaMpu {
    DmaMpu() = delete;

    static constexpr uint8_t regions = 8;

    /// The level required where no region matches (MPU_CTRL).
    static DmaSecurity global_level() {
        const uint32_t v = DMA->MPU_CTRL;
        return static_cast<DmaSecurity>(((v & DMA_MPU_CTRL_S_BITS) != 0u ? 2u : 0u) |
                                        ((v & DMA_MPU_CTRL_P_BITS) != 0u ? 1u : 0u));
    }
    /// Whether a non-secure read of the region addresses is hidden.
    static bool hides_addresses() { return (DMA->MPU_CTRL & DMA_MPU_CTRL_NS_HIDE_ADDR_BITS) != 0u; }

    static DmaMpuRegion region(uint8_t n) {
        const uint32_t bar = reg_at(DMA_BASE, DMA_MPU_BAR0_OFFSET + 8u * n);
        const uint32_t lar = reg_at(DMA_BASE, DMA_MPU_LAR0_OFFSET + 8u * n);
        return DmaMpuRegion{bar & DMA_MPU_BAR0_ADDR_BITS, lar & DMA_MPU_LAR0_ADDR_BITS,
                            static_cast<DmaSecurity>(((lar & DMA_MPU_LAR0_S_BITS) != 0u ? 2u : 0u) |
                                                     ((lar & DMA_MPU_LAR0_P_BITS) != 0u ? 1u : 0u)),
                            (lar & DMA_MPU_LAR0_EN_BITS) != 0u};
    }
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
    static_assert(line < dma_line_count, "the RP2350 DMA has four interrupt lines, 0 to 3");
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
    /// routed to `line` and the line enabled in the calling core's
    /// interrupt controller.
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
        return begin(buffer, length, DmaStep::forward);
    }
    /// `length` copies of ONE element: the read pointer does not
    /// increment. What a full-duplex bus needs to clock a read.
    static bool start_fixed(const Elem* cell, uint32_t length) {
        return begin(cell, length, DmaStep::fixed);
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

    /// Throw the running block away (the abort with E5's preparation) and
    /// hand the channel back ready.
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
    static bool begin(const Elem* p, uint32_t length, DmaStep step) {
        if (busy_ || p == nullptr || length == 0u) {
            return false;
        }
        in_flight_ = length;
        busy_ = Channel::load(DmaTransfer{
            .read = p,
            .write = data_,
            .count = length,
            .config = {.size = size,
                       .read_step = step,
                       .write_step = DmaStep::fixed,
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
 * register, and asked how much has arrived - from TRANS_COUNT, one read,
 * nothing suspended. (The RP2040 read the count because its addresses
 * lied during a non-incrementing sequence; here they no longer do, and
 * the count is still the right reading, because a run's accounting is in
 * ITEMS and not in bytes past a base.)
 */
template <uint8_t ch, typename Elem = uint8_t, uint8_t line = 0>
class DmaRxEngine {
    static_assert(ch < dma_channel_count, "no such DMA channel for this engine");
    static_assert(line < dma_line_count, "the RP2350 DMA has four interrupt lines, 0 to 3");
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

    static bool start(Elem* buffer, uint32_t length) { return begin(buffer, length, DmaStep::forward); }
    /// `length` elements into ONE cell, thrown away (the receive side of
    /// a bus write).
    static bool start_discard(Elem* cell, uint32_t length) {
        return begin(cell, length, DmaStep::fixed);
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
    static bool begin(Elem* p, uint32_t length, DmaStep step) {
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
                       .read_step = DmaStep::fixed,
                       .write_step = step,
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

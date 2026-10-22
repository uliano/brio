/*
 * dma.hpp
 *
 * The DMA controller of the CH32X035 (RM ch. 9) as the engines a
 * transport names in its template slots, and as a copy engine for a
 * program: eight channels, each the OR of a fixed handful of peripheral
 * requests (figure 9-1), an arbiter over four software priorities with
 * the channel number deciding ties, and three flags a channel - here the
 * channel is the request: there is no multiplexer, and
 * ch32x035/dma_engine.hpp's table says which peripheral event each
 * channel answers (USART4 transmits on channel 1 and receives on 8,
 * USART2 on 7 and 6, the SPI on 3 and 2, the ADC on 1, the I2C on 6 and
 * 7). The USB host/device controller and the USB PD controller carry DMA
 * engines of their own (9.1) and touch none of these eight. A program
 * meets the engines through a driver's slots, and the copy engine
 * directly:
 *
 *     using Serial = brio::Uart<2, P, 64, 128, brio::UartFormat{}, brio::DmaTxEngine<7>, brio::DmaRxEngine<6>>;
 *     extern "C" BRIO_CH32_INTERRUPT void dma1_channel7_handler() { (void)Serial::dma_isr(); }
 *
 *     using Copy = brio::DmaCopyEngine<5>;
 *     Copy::arm();                                  // once: low priority, interrupt on
 *     (void)Copy::copy(frame, tile, 1024);          // 1024 words; busy() or isr() says when
 *
 * Who calls what. arm() and arm_ring() are setup verbs, a driver's init()
 * or a program's main(). start(), launch(), copy() and fill() program a
 * block and return at once; the block runs while the caller goes on, and
 * the caller's run or cell stays untouched and in scope until the
 * completion (complete(), take(), busy(), or the channel's vector). A
 * channel's vector calls the owning driver's dma_isr(), or the copy
 * engine's isr(), and nothing else does. Nothing here waits or masks: a
 * transport that claims a transmit engine from both its thread and its
 * completion handler masks the claim itself (claim(), below), and the
 * program's own code decides what runs where. Every engine's state is
 * static: one engine type, one channel, one binding at a time.
 *
 * What a channel is. A configuration word (CFGR: direction, circular,
 * memory-to-memory, the two increments, the two widths, the priority, the
 * three interrupt enables and EN), a count the controller decrements after
 * each item's store (CNTR, 9.2.1's three steps: the read, the store, the
 * decrement), and two addresses named after their F1 ancestry - PADDR and
 * MADDR - where DIR says which side is the source; in memory-to-memory
 * mode both are memory and the names mean nothing. 9.2.1's note makes
 * PADDR, MADDR, CNTR and CFGR's DIR, CIRC, PINC and MINC writable only
 * while EN is clear (9.3.5 and 9.3.6 repeat it for the addresses).
 *
  * Eight vectors, not consecutive: channels 1..7 are table 7-1's entries
 * 22..28 and channel 8 is entry 44. One flag register for all eight
 * (INTFR, four bits a channel, the whole word), so a handler serving two
 * channels of one transport reads it once. DMA1EN in RCC_AHBPCENR is the
 * block's one gate, closed at reset (its reset value opens the USB
 * blocks and SRAMEN), and the block has no reset line (3.4.9).
 *
 * What the silicon does that the engines are built on - each measured on
 * the CH32X035F8U6 by test_x035_dma and stated in docs/ch32x035/dma.md:
 * EN stays set when a non-circular block completes (CNTR at zero, TCIF
 * up, the channel enabled and idle - only software clears EN), so every
 * block's first store takes EN down; and 9.3.5/9.3.6 say the module
 * ignores the low address bits of a 16- or 32-bit access, so a half-word
 * transfer pointed at an odd address moves the wrong bytes and reports
 * success - every verb here that takes a whole transfer or a run refuses
 * an end that is not aligned to its width (the piecewise
 * set_peripheral()/set_memory() store what they are given).
 *
 * `DmaChannel<ch>` is the channel's whole register surface: the one-shot
 * verbs a suite or a rare user programs a block with (configure, prepare,
 * load - each validating, each opening the gate), the flags, the ISR
 * body. The engines do not go through it: they are the hot path and are
 * written as two moments (docs/design/dma.md).
 *
 *  - `arm()` configures once. The gate opened, the channel stopped, PADDR
 *    written - the peripheral's register never changes for a binding -
 *    and the configuration word computed and kept in the engine:
 *    direction, priority, the interrupt enables the binding wants. The
 *    PFIC line enabled.
 *  - `start()` writes what changes, five stores and no load: CFGR with EN
 *    clear (the word with this block's beat and increment - the disable
 *    9.2.1's note asks for before the count and the address, EN standing
 *    from the last block), CNTR, MADDR, the channel's four flags cleared
 *    in INTFCR, CFGR again with EN set. WCH's library restarts a channel
 *    with DMA_Cmd's read-modify-writes of CFGR around DMA_SetCurrDataCounter
 *    and a store of MADDR - seven bus accesses through three calls in its
 *    USART_Idle_Recv example's handlers; the engine knows the word and
 *    stores it. Nothing is validated but the run itself (its length, its
 *    alignment): the binding is the engine's type (the channel, the widest
 *    beat - checked at compile time) and arm()'s arguments.
 *
 * The beat (the width of one item moved) is the element of the run. `start()` takes a span of uint8_t,
 * uint16_t or uint32_t and the transfer's two widths follow the type, so
 * one channel moves bytes for one block and half-words for the next. The
 * engine's `Elem` is the widest beat the binding allows - the peripheral
 * register's width - and a wider span does not compile. A run whose
 * address is not aligned to its beat is refused (9.3.6).
 *
 * The claim and the programming are two verbs. A transmit engine is busy
 * from its claim to its complete(). `claim()` is that flag's test-and-set
 * and nothing else; `launch()` programs a claimed engine. A transport
 * whose completion handler starts the next block holds its mask over
 * `claim()` alone and launches outside it: a claimed channel cannot
 * complete under the programmer's feet, because it is not running.
 * `start()` is the two together, for an owner with no handler to race.
 *
 * `DmaCopyEngine<ch, Elem>` is the memory-to-memory shape: `copy(dst,
 * src, n)` and `fill(dst, cell, n)` (the source a fixed cell in memory),
 * n in elements of the beat, on any channel, with no request - 9.2.1:
 * with MEM2MEM set the channel runs as soon as it is enabled.
 *
 * The level an engine arms at is its peripheral's (ch32x035/dma_engine.hpp's
 * DmaPriority, docs/design/dma.md). The arbiter gives the bus to the
 * highest level asking, the lower channel number between equals (9.2.1),
 * and a memory-to-memory channel asks without a pause - so a receive
 * channel it outranks moves nothing until its block is over. The defaults
 * are the rule's: the receive engine at very_high, the transmit engine at
 * high, the copy at low.
 *
 * Not covered yet: the loop and ping-pong engines a block stream is
 * served by (util/block_stream.hpp's two concepts), born with their first
 * block user - the ADC, which has no driver in this stratum.
 */

#pragma once

#include <stddef.h>
#include <stdint.h>

#include <span>

#include "ch32x035/device.hpp"
#include "ch32x035/dma_engine.hpp"
#include "ch32x035/pfic.hpp"

namespace brio {

// ---- the vocabulary -------------------------------------------------------------

/// CFGR.PSIZE / MSIZE: the width of one bus access (9.3.3; code 11 is
/// reserved).
enum class DmaWidth : uint8_t { byte = 0, half = 1, word = 2 };

template <typename Elem>
constexpr DmaWidth dma_width_of() {
    static_assert(sizeof(Elem) == 1 || sizeof(Elem) == 2 || sizeof(Elem) == 4,
                  "a DMA element is one bus access wide: 1, 2 or 4 bytes");
    return sizeof(Elem) == 1 ? DmaWidth::byte : sizeof(Elem) == 2 ? DmaWidth::half : DmaWidth::word;
}

constexpr uint32_t dma_width_bytes(DmaWidth w) { return 1UL << static_cast<uint32_t>(w); }

/// RM 9.3.5/9.3.6: a 16- or 32-bit access ignores the address's low bits,
/// so an address off its width's boundary moves the wrong bytes.
constexpr bool dma_aligned(uint32_t address, DmaWidth w) {
    return (address & (dma_width_bytes(w) - 1UL)) == 0UL;
}

// CFGR.PL's vocabulary, DmaPriority, is ch32x035/dma_engine.hpp's: a
// transport names its engines' levels without this file.

/// CFGR.DIR says which side is the source: 0 reads PADDR and writes MADDR,
/// 1 the other way round (9.3.3).
enum class DmaDirection : uint8_t { peripheral_to_memory = 0, memory_to_peripheral = 1 };

/// The four bits one channel owns in INTFR and INTFCR, at 4 x (ch - 1)
/// (9.3.1, 9.3.2). TCIF, HTIF and TEIF sit at the same positions as their
/// enables in CFGR (TCIE, HTIE, TEIE), which is what lets an engine read
/// its armed set off the configuration word it keeps.
struct DmaFlag {
    static constexpr uint32_t global = 1u << 0;     ///< GIFx
    static constexpr uint32_t complete = 1u << 1;   ///< TCIFx
    static constexpr uint32_t half = 1u << 2;       ///< HTIFx
    static constexpr uint32_t error = 1u << 3;      ///< TEIFx
    static constexpr uint32_t all = global | complete | half | error;
};

struct DmaChannelConfig {
    DmaDirection direction = DmaDirection::peripheral_to_memory;
    bool circular = false;              ///< CIRC: reload and keep going
    bool memory_to_memory = false;      ///< MEM2MEM: no request, run on enable
    bool peripheral_increment = false;  ///< PINC
    bool memory_increment = true;       ///< MINC
    DmaWidth peripheral_width = DmaWidth::byte;
    DmaWidth memory_width = DmaWidth::byte;
    DmaPriority priority = DmaPriority::low;
};

/// RM 9.2.1's own rule: the memory-to-memory mode "does not support
/// cyclic mode".
constexpr bool dma_channel_config_valid(const DmaChannelConfig& c) {
    return !(c.circular && c.memory_to_memory);
}

/// One block transfer: both ends and how many data items.
struct DmaTransfer {
    volatile void* peripheral = nullptr;   ///< PADDR
    volatile void* memory = nullptr;       ///< MADDR
    uint16_t count = 0;                    ///< CNTR, in data items
    DmaChannelConfig config{};
};

constexpr bool dma_transfer_valid(const DmaTransfer& t) {
    return t.peripheral != nullptr && t.memory != nullptr && t.count != 0u &&
           dma_channel_config_valid(t.config);
}

struct DmaProgress {
    uint16_t remaining = 0;
    uint16_t done = 0;
};

// ---- the registers --------------------------------------------------------------

struct DmaChannelRegs {
    volatile uint32_t CFGR;    ///< 0x00
    volatile uint32_t CNTR;    ///< 0x04
    volatile uint32_t PADDR;   ///< 0x08
    volatile uint32_t MADDR;   ///< 0x0c
    uint32_t RESERVED0;        ///< 0x10
};

/// Table 9-2: INTFR and INTFCR, then one channel every 0x14 from 0x08 -
/// channel 8's CFGR at 0x94.
struct DmaRegs {
    volatile uint32_t INTFR;                   ///< 0x00 the flags, read-only
    volatile uint32_t INTFCR;                  ///< 0x04 write 1 to clear
    DmaChannelRegs channel[dma_channel_count]; ///< 0x08 + 0x14 x (ch - 1)
};

inline DmaRegs* dma() { return reinterpret_cast<DmaRegs*>(hb_base + 0x0000); }

inline constexpr uint32_t dma_cfgr_en      = 1UL << 0;
inline constexpr uint32_t dma_cfgr_tcie    = 1UL << 1;
inline constexpr uint32_t dma_cfgr_htie    = 1UL << 2;
inline constexpr uint32_t dma_cfgr_teie    = 1UL << 3;
inline constexpr uint32_t dma_cfgr_dir     = 1UL << 4;
inline constexpr uint32_t dma_cfgr_circ    = 1UL << 5;
inline constexpr uint32_t dma_cfgr_pinc    = 1UL << 6;
inline constexpr uint32_t dma_cfgr_minc    = 1UL << 7;
inline constexpr uint32_t dma_cfgr_mem2mem = 1UL << 14;
inline constexpr uint32_t dma_cfgr_irqs    = dma_cfgr_tcie | dma_cfgr_htie | dma_cfgr_teie;

static_assert(dma_cfgr_tcie == DmaFlag::complete && dma_cfgr_htie == DmaFlag::half &&
                  dma_cfgr_teie == DmaFlag::error,
              "an enable and its flag share a position (9.3.1, 9.3.3)");

/// A channel's vector (table 7-1): 22..28 for channels 1..7, 44 for 8.
constexpr Irq dma_channel_irq(uint8_t ch) {
    return ch == 8u ? Irq::dma1_channel8
                    : static_cast<Irq>(static_cast<uint8_t>(Irq::dma1_channel1) + (ch - 1u));
}

/// CFGR.PL in place.
constexpr uint32_t dma_cfgr_priority(DmaPriority p) { return static_cast<uint32_t>(p) << 12; }

/// CFGR for a configuration, EN and the interrupt enables clear.
constexpr uint32_t dma_cfgr_of(const DmaChannelConfig& c) {
    uint32_t v = 0;
    if (c.direction == DmaDirection::memory_to_peripheral) { v |= dma_cfgr_dir; }
    if (c.circular) { v |= dma_cfgr_circ; }
    if (c.memory_to_memory) { v |= dma_cfgr_mem2mem; }
    if (c.peripheral_increment) { v |= dma_cfgr_pinc; }
    if (c.memory_increment) { v |= dma_cfgr_minc; }
    v |= static_cast<uint32_t>(c.peripheral_width) << 8;
    v |= static_cast<uint32_t>(c.memory_width) << 10;
    v |= dma_cfgr_priority(c.priority);
    return v;
}

/// CFGR's two width fields for a beat of `T` on both sides.
template <typename T>
constexpr uint32_t dma_cfgr_beat() {
    constexpr uint32_t w = static_cast<uint32_t>(dma_width_of<T>());
    return (w << 8) | (w << 10);
}

/// The block: its gate and its flag register.
struct Dma {
    Dma() = delete;

    static void open() { rcc()->AHBPCENR |= rcc_hb_dma1; }
    static bool is_open() { return (rcc()->AHBPCENR & rcc_hb_dma1) != 0u; }
    static uint32_t flags() { return dma()->INTFR; }

    /// Whether any channel is moving data or waiting to: enabled and
    /// circular or with items left. EN stays set after a completed
    /// non-circular block (CNTR at zero), so EN alone would call a
    /// finished channel busy. One load with the gate closed, two a
    /// channel with it open.
    static bool any_working() {
        if (!is_open()) {
            return false;
        }
        // A pointer stepped over the eight, not an index the compiler
        // multiplies by the stride on a core whose multiply it spends.
        const DmaChannelRegs* c = &dma()->channel[0];
        const DmaChannelRegs* const end = c + dma_channel_count;
        do {
            const uint32_t cfgr = c->CFGR;
            if ((cfgr & dma_cfgr_en) != 0u && ((cfgr & dma_cfgr_circ) != 0u || c->CNTR != 0u)) {
                return true;
            }
        } while (++c != end);
        return false;
    }
};

/**
 * One channel, 1..8. Every configuring verb refuses while the channel is
 * enabled, because CFGR's fields, CNTR and the two addresses are
 * read-only then (RM 9.2.1's note): a store the silicon ignores would be a
 * lie the code told itself.
 */
template <uint8_t ch>
class DmaChannel {
    static_assert(ch >= 1 && ch <= dma_channel_count, "the CH32X035 DMA has channels 1..8");

public:
    DmaChannel() = delete;

    static constexpr uint8_t number = ch;
    static constexpr uint32_t flag_shift = 4u * (ch - 1u);

    static DmaChannelRegs& regs() { return dma()->channel[ch - 1u]; }
    static constexpr Irq irq() { return dma_channel_irq(ch); }

    static bool enabled() { return (regs().CFGR & dma_cfgr_en) != 0u; }

    /// Enable or disable. Disabling a running channel abandons the block
    /// where it stands; CNTR then holds what was left.
    static void enable(bool on) {
        Dma::open();
        if (on) { regs().CFGR |= dma_cfgr_en; } else { regs().CFGR &= ~dma_cfgr_en; }
    }

    /// Everything but the addresses and the count. Refused while enabled,
    /// and for a config the chapter forbids.
    static bool configure(const DmaChannelConfig& c) {
        Dma::open();
        if (enabled() || !dma_channel_config_valid(c)) {
            return false;
        }
        regs().CFGR = (regs().CFGR & dma_cfgr_irqs) | dma_cfgr_of(c);
        return true;
    }

    static bool set_count(uint16_t count) {
        if (enabled() || count == 0u) {
            return false;
        }
        regs().CNTR = count;
        return true;
    }
    /// Live while the channel runs: what is still to move.
    static uint16_t count() { return static_cast<uint16_t>(regs().CNTR); }

    static void set_peripheral(volatile void* address) {
        regs().PADDR = static_cast<uint32_t>(reinterpret_cast<uintptr_t>(address));
    }
    static void set_memory(volatile void* address) {
        regs().MADDR = static_cast<uint32_t>(reinterpret_cast<uintptr_t>(address));
    }

    /// Program the whole transfer and start it. Refused while enabled or
    /// for an invalid transfer; the flags are cleared on the way in.
    static bool load(const DmaTransfer& t) {
        if (!prepare(t)) {
            return false;
        }
        enable(true);
        return true;
    }

    /// Program the whole transfer WITHOUT starting it. Refused, besides
    /// what dma_transfer_valid() refuses, for an end that is not aligned
    /// to its width (9.3.5, 9.3.6).
    static bool prepare(const DmaTransfer& t) {
        Dma::open();
        if (enabled() || !dma_transfer_valid(t) ||
            !dma_aligned(static_cast<uint32_t>(reinterpret_cast<uintptr_t>(t.peripheral)),
                         t.config.peripheral_width) ||
            !dma_aligned(static_cast<uint32_t>(reinterpret_cast<uintptr_t>(t.memory)), t.config.memory_width)) {
            return false;
        }
        if (!configure(t.config)) {
            return false;
        }
        set_peripheral(t.peripheral);
        set_memory(t.memory);
        regs().CNTR = t.count;
        clear(DmaFlag::all);
        return true;
    }

    static uint32_t flags() { return (dma()->INTFR >> flag_shift) & DmaFlag::all; }
    static bool flag(uint32_t mask) { return (flags() & mask) != 0u; }
    static void clear(uint32_t mask) { dma()->INTFCR = (mask & DmaFlag::all) << flag_shift; }

    /// Arm the channel's interrupts (the PFIC line is the caller's).
    static void arm(uint32_t mask, bool on) {
        const uint32_t bits = mask & dma_cfgr_irqs;   // an enable's position is its flag's
        if (on) { regs().CFGR |= bits; } else { regs().CFGR &= ~bits; }
    }

    /**
     * The channel's interrupt body: read the flags that are armed and up, clear
     * exactly those, hand them back. A completion is not acted on here -
     * what it means is the owner's to decide.
     */
    [[gnu::always_inline]] static uint32_t isr() {
        const uint32_t armed = regs().CFGR & dma_cfgr_irqs;
        const uint32_t up = flags() & armed;
        if (up != 0u) {
            clear(up | DmaFlag::global);
        }
        return up;
    }

    static DmaProgress progress(uint16_t programmed) {
        const uint16_t remaining = count();
        const uint16_t done = remaining > programmed ? uint16_t{0} : static_cast<uint16_t>(programmed - remaining);
        return DmaProgress{remaining, done};
    }

    /// Stop and clear everything of the channel's: EN off, the config
    /// zeroed, the flags cleared.
    static void stop() {
        Dma::open();
        regs().CFGR = 0;
        clear(DmaFlag::all);
    }
};

// ---- the engines' common moments --------------------------------------------------

/**
 * What every engine does to its channel, in the two moments: `bind()` at
 * arm() and `restart()` at every block. Not a user's type - an engine's.
 */
template <uint8_t ch>
struct DmaEngineChannel {
    static_assert(ch >= 1 && ch <= dma_channel_count, "no such DMA channel for this engine (1..8)");

    DmaEngineChannel() = delete;

    using Channel = DmaChannel<ch>;
    static constexpr uint32_t shift = Channel::flag_shift;

    /// The binding: the gate opened (the one RCC read-modify-write an
    /// engine ever makes), the channel stopped with its flags cleared, the
    /// peripheral end written - PADDR, which nothing but the next arm()
    /// changes - and the PFIC line enabled.
    static void bind(uint32_t peripheral) {
        Dma::open();
        DmaChannelRegs& r = Channel::regs();
        r.CFGR = 0;
        dma()->INTFCR = DmaFlag::all << shift;
        r.PADDR = peripheral;
        Pfic::enable(Channel::irq());
    }

    /// One block: the configuration word with EN clear (the disable 9.2.1
    /// asks for before CNTR and MADDR, EN standing after the last block),
    /// the count, the memory end, the flags, the enable. Five stores, no
    /// load.
    [[gnu::always_inline]] static void restart(uint32_t word, uint32_t memory, uint16_t count) {
        DmaChannelRegs& r = Channel::regs();
        r.CFGR = word;
        r.CNTR = count;
        r.MADDR = memory;
        dma()->INTFCR = DmaFlag::all << shift;
        r.CFGR = word | dma_cfgr_en;
    }

    /// One block with its peripheral end too - the copy engine's, whose
    /// source changes with every block: the same five stores with PADDR
    /// between the disable and the enable. PADDR first would be dropped
    /// whenever EN still stands from the last block - 9.3.5's note, and
    /// measured on the CH32X035F8U6: a copy after a completed one read the
    /// old source. Six stores, no load.
    [[gnu::always_inline]] static void restart_from(uint32_t word, uint32_t peripheral, uint32_t memory,
                                                    uint16_t count) {
        DmaChannelRegs& r = Channel::regs();
        r.CFGR = word;
        r.PADDR = peripheral;
        r.CNTR = count;
        r.MADDR = memory;
        dma()->INTFCR = DmaFlag::all << shift;
        r.CFGR = word | dma_cfgr_en;
    }

    /// The channel's armed flags that are up in `intfr` (INTFR read once by
    /// the caller, which may serve two channels with it), cleared with GIF
    /// and handed back.
    [[gnu::always_inline]] static uint32_t service(uint32_t intfr, uint32_t armed) {
        const uint32_t up = (intfr >> shift) & armed;
        if (up != 0u) {
            dma()->INTFCR = (up | DmaFlag::global) << shift;
        }
        return up;
    }

    /// EN off, the interrupt enables off, the flags cleared; PADDR kept.
    static void halt() {
        Channel::regs().CFGR = 0;
        dma()->INTFCR = DmaFlag::all << shift;
    }
};

template <typename T, typename Elem>
inline constexpr bool dma_beat_fits = (sizeof(T) == 1 || sizeof(T) == 2 || sizeof(T) == 4) &&
                                      sizeof(T) <= sizeof(Elem);

/// A run's address and length as a block takes them: false for an empty
/// run, one over CNTR's 65535, or one off its beat's boundary.
template <typename T>
[[gnu::always_inline]] inline bool dma_run_ok(uint32_t address, size_t length) {
    return length != 0u && length <= 0xFFFFu && dma_aligned(address, dma_width_of<T>());
}

template <typename T>
[[gnu::always_inline]] inline uint32_t dma_address_of(const volatile T* p) {
    return static_cast<uint32_t>(reinterpret_cast<uintptr_t>(p));
}

// ---- the engines ---------------------------------------------------------------

/**
 * A transmit engine: one caller-owned run poured into one peripheral
 * register - the peripheral's request paces it (dma_engine.hpp's row for
 * this channel), whichever the program has enabled.
 */
template <uint8_t ch, typename Elem = uint8_t>
class DmaTxEngine {
    static_assert(ch >= 1 && ch <= dma_channel_count, "no such DMA channel for this engine (1..8)");
    static_assert(sizeof(Elem) == 1 || sizeof(Elem) == 2 || sizeof(Elem) == 4,
                  "a DMA element is one bus access wide: 1, 2 or 4 bytes");
    using Line = DmaEngineChannel<ch>;
    using Channel = DmaChannel<ch>;

public:
    DmaTxEngine() = delete;

    static constexpr bool present = true;
    static constexpr uint8_t channel = ch;
    /// The widest beat this binding moves: start() takes any narrower.
    static constexpr DmaWidth width = dma_width_of<Elem>();
    using element = Elem;

    static constexpr uint8_t flag_complete = DmaFlag::complete;
    static constexpr uint8_t flag_half = DmaFlag::half;
    static constexpr uint8_t flag_error = DmaFlag::error;

    /// The channel's ISR body folded into the engine: the armed flags that
    /// are up, cleared and handed back. Acts on nothing.
    [[gnu::always_inline]] static uint8_t service() { return service(Dma::flags()); }
    /// The same over an INTFR the caller read once for two channels.
    [[gnu::always_inline]] static uint8_t service(uint32_t intfr) {
        return static_cast<uint8_t>(Line::service(intfr, armed_));
    }
    /// INTFR, every channel's flags in one load: what service(intfr) takes,
    /// for a driver that reaches the controller only through its engines'
    /// names.
    [[gnu::always_inline]] static uint32_t block_flags() { return Dma::flags(); }

    /**
     * Bind the channel to its peripheral: `data` is the register the run is
     * poured into, `interrupts` the flags whose interrupt the binding wants
     * (flag_complete and/or flag_error) - a transport whose completion
     * another event proves arms the errors alone -, `priority` CFGR.PL,
     * `high` by default: a peripheral a transmit channel feeds waits for
     * it, or holds its last value, where a starved receive overruns
     * (DmaPriority's rule). The gate and the PFIC line are opened here,
     * once.
     */
    static void arm(volatile void* data, uint32_t interrupts = DmaFlag::complete | DmaFlag::error,
                    DmaPriority priority = DmaPriority::high) {
        armed_ = interrupts & dma_cfgr_irqs;
        word_ = dma_cfgr_dir | dma_cfgr_priority(priority) | armed_;
        busy_ = false;
        in_flight_ = 0;
        Line::bind(dma_address_of(data));
    }
    /// Both interrupts, at `priority`.
    static void arm(volatile void* data, DmaPriority priority) {
        arm(data, DmaFlag::complete | DmaFlag::error, priority);
    }

    /// The claim: true when the engine was idle and is now the caller's. A
    /// test-and-set and nothing else - the one step a transport whose
    /// completion handler starts the next block takes under its mask;
    /// launch() follows outside it.
    [[gnu::always_inline]] static bool claim() {
        if (busy_) {
            return false;
        }
        busy_ = true;
        return true;
    }
    /// Give a claim back unused.
    [[gnu::always_inline]] static void unclaim() { busy_ = false; }

    /// Program a claimed engine with the run (the caller's, and it stays
    /// put until complete() reports the block), the beat its element's
    /// width. False - and the claim given back - for an empty run, one over
    /// 65535 elements, or one off its beat's boundary (9.3.6).
    template <typename T>
    static bool launch(std::span<const T> run) {
        static_assert(dma_beat_fits<T, Elem>, "a beat wider than this binding's widest (its Elem)");
        return begin<T>(dma_cfgr_minc, dma_address_of(run.data()), run.size());
    }

    /// claim() and launch() in one, for an owner with no handler to race:
    /// false on a busy engine as for a bad run.
    template <typename T>
    static bool start(std::span<const T> run) {
        return claim() && launch(run);
    }
    template <typename T>
    static bool start(std::span<T> run) {
        return start(std::span<const T>(run));
    }

    /// `length` copies of one element: the memory pointer does not
    /// increment. What a full-duplex bus needs to clock a read. Claims as
    /// start() does.
    template <typename T>
    static bool start_fixed(const T* cell, uint16_t length) {
        static_assert(dma_beat_fits<T, Elem>, "a beat wider than this binding's widest (its Elem)");
        return claim() && begin<T>(0u, dma_address_of(cell), length);
    }

    /// The block ended: how many elements it carried, so the owner can
    /// release exactly that much of its ring. No register is touched: EN
    /// stands after a completed block and the next start() clears it.
    static uint16_t complete() {
        const uint16_t n = in_flight_;
        busy_ = false;
        in_flight_ = 0;
        return n;
    }

    static bool busy() { return busy_; }
    static uint16_t in_flight() { return busy_ ? in_flight_ : 0u; }
    static DmaProgress progress() { return Channel::progress(in_flight_); }

    /// Throw the running block away - the channel halted (EN off; a
    /// transfer error has already cleared it, a time-out has not) - and
    /// count the fault. The binding stands: the next start() needs no arm().
    static bool abandon() {
        faults_ = faults_ + 1u;
        Line::halt();
        busy_ = false;
        in_flight_ = 0;
        return true;
    }

    static uint32_t faults() { return faults_; }
    static void clear_faults() { faults_ = 0; }

    static void stop() {
        Line::halt();
        busy_ = false;
        in_flight_ = 0;
    }

private:
    /// The block on a claimed engine; a refused run gives the claim back.
    template <typename T>
    [[gnu::always_inline]] static bool begin(uint32_t minc, uint32_t address, size_t length) {
        if (!dma_run_ok<T>(address, length)) {
            busy_ = false;
            return false;
        }
        in_flight_ = static_cast<uint16_t>(length);
        Line::restart(word_ | dma_cfgr_beat<T>() | minc, address, static_cast<uint16_t>(length));
        return true;
    }

    static inline uint32_t word_ = 0;
    static inline uint32_t armed_ = 0;
    static inline volatile uint32_t faults_ = 0;   ///< bumped from a vector, read by the thread
    static inline uint16_t in_flight_ = 0;
    static inline volatile bool busy_ = false;
};

/**
 * A receive engine: one caller-owned run filled from one peripheral
 * register, and asked how much has arrived - or, in its CIRCULAR SHAPE, a
 * caller-owned ring filled for ever.
 *
 * The circular shape is CFGR.CIRC (9.2.1's third item: at a count of zero
 * the channel reloads CNTR and its two addresses and goes on with no CPU
 * in the path): arm_ring() is handed the ring's whole storage and binds it
 * for good - PADDR, MADDR, the count every lap reloads, the word with
 * CIRC, MINC, the beat, the lap's completion and the transfer error, and
 * with `half_mark` the half lap's flag - and starts it. Nothing re-arms it
 * afterwards. The producer index of that ring is the channel's own count,
 * and this engine is util/ring.hpp's RingCounter for it: remaining() is
 * CNTR, one load (9.2.1 orders a transfer as the read, the store and then
 * the decrement, so the count never counts an element whose store has not
 * been made), and laps() the completions lap() counted in the channel's
 * handler - lagging the count by a handler's latency, never leading it.
 * util/ring.hpp's HardwareRing is the consumer half. A circular channel
 * never stops on its own: idle() after a transfer error (the hardware
 * clears EN, 9.2.1) is what says it has.
 */
template <uint8_t ch, typename Elem = uint8_t>
class DmaRxEngine {
    static_assert(ch >= 1 && ch <= dma_channel_count, "no such DMA channel for this engine (1..8)");
    static_assert(sizeof(Elem) == 1 || sizeof(Elem) == 2 || sizeof(Elem) == 4,
                  "a DMA element is one bus access wide: 1, 2 or 4 bytes");
    using Line = DmaEngineChannel<ch>;
    using Channel = DmaChannel<ch>;

public:
    DmaRxEngine() = delete;

    static constexpr bool present = true;
    static constexpr uint8_t channel = ch;
    /// The widest beat this binding moves: start() takes any narrower.
    static constexpr DmaWidth width = dma_width_of<Elem>();
    using element = Elem;

    static constexpr uint8_t flag_complete = DmaFlag::complete;
    static constexpr uint8_t flag_half = DmaFlag::half;
    static constexpr uint8_t flag_error = DmaFlag::error;

    [[gnu::always_inline]] static uint8_t service() { return service(Dma::flags()); }
    [[gnu::always_inline]] static uint8_t service(uint32_t intfr) {
        return static_cast<uint8_t>(Line::service(intfr, armed_));
    }
    [[gnu::always_inline]] static uint32_t block_flags() { return Dma::flags(); }

    /// Bind the channel to its peripheral (DmaTxEngine::arm()'s terms),
    /// `very_high` by default: a peripheral a receive channel empties
    /// overruns when the channel is starved (DmaPriority's rule).
    static void arm(volatile void* data, uint32_t interrupts = DmaFlag::complete | DmaFlag::error,
                    DmaPriority priority = DmaPriority::very_high) {
        armed_ = interrupts & dma_cfgr_irqs;
        word_ = dma_cfgr_priority(priority) | armed_;
        capacity_ = 0;
        taken_ = 0;
        Line::bind(dma_address_of(data));
    }
    static void arm(volatile void* data, DmaPriority priority) {
        arm(data, DmaFlag::complete | DmaFlag::error, priority);
    }

    /// Not running: stopped, never started, or halted by a transfer error
    /// (9.2.1: the hardware clears EN). One CFGR read.
    static bool idle() { return !Channel::enabled(); }

    /// Fill the run, the beat its element's width. False for an empty run,
    /// one over 65535 elements, or one off its beat's boundary.
    template <typename T>
    static bool start(std::span<T> run) {
        static_assert(dma_beat_fits<T, Elem>, "a beat wider than this binding's widest (its Elem)");
        return begin<T>(dma_cfgr_minc, dma_address_of(run.data()), run.size());
    }

    /// `length` elements into one cell, thrown away (the receive side of a
    /// bus write).
    template <typename T>
    static bool start_discard(T* cell, uint16_t length) {
        static_assert(dma_beat_fits<T, Elem>, "a beat wider than this binding's widest (its Elem)");
        return begin<T>(0u, dma_address_of(cell), length);
    }

    /// How many elements have arrived since the last take(): one CNTR read,
    /// nothing suspended, nothing refused.
    static uint16_t take() {
        if (capacity_ == 0u) {
            return 0;
        }
        const uint16_t remaining = Channel::count();
        const uint16_t filled = remaining > capacity_ ? capacity_
                                                      : static_cast<uint16_t>(capacity_ - remaining);
        if (filled <= taken_) {
            return 0;
        }
        const uint16_t fresh = static_cast<uint16_t>(filled - taken_);
        taken_ = filled;
        return fresh;
    }

    static bool full() { return capacity_ != 0u && taken_ >= capacity_; }
    static uint16_t capacity() { return capacity_; }
    static uint16_t taken() { return taken_; }

    /// Throw the running block away - the channel halted - and count the
    /// fault. The binding stands.
    static bool abandon() {
        faults_ = faults_ + 1u;
        Line::halt();
        capacity_ = 0;
        taken_ = 0;
        return true;
    }

    static uint32_t faults() { return faults_; }
    static void clear_faults() { faults_ = 0; }

    static void stop() {
        Line::halt();
        capacity_ = 0;
        taken_ = 0;
    }

    /**
     * The circular binding, which is also its start (the class header):
     * `storage` the caller's whole ring, its element the beat, its length
     * the count every lap reloads. False, and nothing started, for a
     * storage off its beat's boundary. Calling it again restarts the ring
     * at its first element with laps() at zero - the moment a HardwareRing
     * over the same storage is clear()ed. `very_high` by default, the
     * one-shot shape's level.
     */
    template <typename T, size_t N>
    static bool arm_ring(volatile void* data, T (&storage)[N], bool half_mark = false,
                         DmaPriority priority = DmaPriority::very_high) {
        static_assert(dma_beat_fits<T, Elem>, "a beat wider than this binding's widest (its Elem)");
        static_assert(N >= 2u && N <= 0xFFFFu,
                      "brio DmaRxEngine: a ring of 2..65535 items - CNTR is sixteen bits (9.3.4)");
        const uint32_t m = dma_address_of(&storage[0]);
        if (!dma_aligned(m, dma_width_of<T>())) {
            return false;
        }
        armed_ = DmaFlag::complete | DmaFlag::error | (half_mark ? DmaFlag::half : 0u);
        word_ = dma_cfgr_priority(priority) | armed_;
        capacity_ = 0;
        taken_ = 0;
        laps_ = 0;
        Line::bind(dma_address_of(data));
        Line::restart(word_ | dma_cfgr_beat<T>() | dma_cfgr_minc | dma_cfgr_circ, m,
                      static_cast<uint16_t>(N));
        return true;
    }

    /// CNTR, live: the elements still to land in the current lap.
    [[gnu::always_inline]] static uint32_t remaining() { return Channel::count(); }
    /// The completions lap() counted since arm_ring(): the ring's laps.
    [[gnu::always_inline]] static uint32_t laps() { return laps_; }
    /// The circular shape's completion, counted - from the channel's
    /// handler, after service() reported it.
    [[gnu::always_inline]] static void lap() { laps_ = laps_ + 1u; }

private:
    template <typename T>
    [[gnu::always_inline]] static bool begin(uint32_t minc, uint32_t address, size_t length) {
        if (!dma_run_ok<T>(address, length)) {
            return false;
        }
        capacity_ = static_cast<uint16_t>(length);
        taken_ = 0;
        Line::restart(word_ | dma_cfgr_beat<T>() | minc, address, static_cast<uint16_t>(length));
        return true;
    }

    static inline uint32_t word_ = 0;
    static inline uint32_t armed_ = 0;
    static inline volatile uint32_t faults_ = 0;   ///< bumped from a vector, read by the thread
    static inline volatile uint32_t laps_ = 0;
    static inline uint16_t capacity_ = 0;
    static inline uint16_t taken_ = 0;
};

/**
 * Memory to memory: `copy()` a run, or `fill()` one from a fixed cell, on
 * any channel - 9.2.1: with MEM2MEM set the channel needs no request and
 * runs from its enable, so the channel number says only where it sits in
 * the arbitration (ties go to the lower number) and which vector reports
 * it. Neither circular mode (the chapter refuses it with MEM2MEM) nor a
 * half-transfer interrupt is used.
 *
 * The source is the PADDR side (DIR clear: read PADDR, write MADDR), so a
 * copy is PINC and MINC set and a fill is MINC alone. The engine is BUSY
 * from a block's start to its end, which the program learns either from
 * the channel's vector - arm(..., true), the app binding isr() - or by
 * polling busy(), which reads the flags while a block runs. `abandon()`
 * stops a running block where it stands: EN cleared, the count holding
 * what was left.
 */
template <uint8_t ch, typename Elem = uint32_t>
class DmaCopyEngine {
    static_assert(ch >= 1 && ch <= dma_channel_count, "no such DMA channel for this engine (1..8)");
    static_assert(sizeof(Elem) == 1 || sizeof(Elem) == 2 || sizeof(Elem) == 4,
                  "a DMA element is one bus access wide: 1, 2 or 4 bytes");
    using Line = DmaEngineChannel<ch>;
    using Channel = DmaChannel<ch>;

public:
    DmaCopyEngine() = delete;

    static constexpr bool present = true;
    static constexpr uint8_t channel = ch;
    static constexpr DmaWidth width = dma_width_of<Elem>();
    using element = Elem;

    /// The configuration, once: the gate, the channel stopped, the
    /// priority, the interrupt (complete and error) wanted or not. With no
    /// interrupt the line stays disabled and busy() is the witness. `low`
    /// by default: a copy asks for the bus without a pause, and a
    /// request-paced channel it outranked would move nothing until its
    /// block ended (DmaPriority's rule).
    static void arm(DmaPriority priority = DmaPriority::low, bool interrupt = true) {
        armed_ = interrupt ? (DmaFlag::complete | DmaFlag::error) : 0u;
        word_ = dma_cfgr_mem2mem | dma_cfgr_minc | dma_cfgr_priority(priority) | armed_;
        busy_ = false;
        Line::bind(0u);
        if (!interrupt) {
            Pfic::disable(Channel::irq());
        }
    }

    /// Copy `n` elements from `src` to `dst`, the beat the element's width
    /// (T: uint8_t, uint16_t or uint32_t, no wider than Elem). False while a
    /// block runs, and for n of zero or over CNTR's 65535, or an end off its
    /// beat's boundary (9.3.5, 9.3.6).
    template <typename T>
    static bool copy(T* dst, const T* src, uint32_t n) {
        static_assert(dma_beat_fits<T, Elem>, "a beat wider than this binding's widest (its Elem)");
        return begin<T>(dma_cfgr_pinc, dma_address_of(src), dma_address_of(dst), n);
    }

    /// Fill `n` elements at `dst` with copies of `*cell` - a cell in memory,
    /// the caller's, which stays put until the block ends (the controller
    /// reads an address). False as copy() is.
    template <typename T>
    static bool fill(T* dst, const T* cell, uint32_t n) {
        static_assert(dma_beat_fits<T, Elem>, "a beat wider than this binding's widest (its Elem)");
        return begin<T>(0u, dma_address_of(cell), dma_address_of(dst), n);
    }

    /// A block started and not yet seen to end. Polled, it reads the
    /// channel's flags while a block runs and frees the engine at the
    /// completion or the error it finds; under the interrupt, isr() has
    /// done that already.
    static bool busy() {
        if (!busy_) {
            return false;
        }
        const uint32_t f = (Dma::flags() >> Line::shift) & (DmaFlag::complete | DmaFlag::error);
        if (f == 0u) {
            return true;
        }
        if ((f & DmaFlag::error) != 0u) {
            faults_ = faults_ + 1u;
        }
        busy_ = false;
        return false;
    }

    /// Stop a running block where it stands - EN cleared, CNTR holding what
    /// was left, the flags cleared - and free the engine. False when no
    /// block was running.
    static bool abandon() {
        if (!busy_) {
            return false;
        }
        Line::halt();
        busy_ = false;
        return true;
    }

    /// The channel's interrupt body: the armed flags that are up, cleared and
    /// handed back; a completion or an error frees the engine (an error
    /// counted). Non-zero when the block ended.
    [[gnu::always_inline]] static uint8_t isr() {
        const uint32_t up = Line::service(Dma::flags(), armed_);
        if (up != 0u) {
            if ((up & DmaFlag::error) != 0u) {
                faults_ = faults_ + 1u;
            }
            busy_ = false;
        }
        return static_cast<uint8_t>(up);
    }

    /// Blocks ended by a transfer error.
    static uint32_t faults() { return faults_; }

private:
    template <typename T>
    [[gnu::always_inline]] static bool begin(uint32_t pinc, uint32_t source, uint32_t destination, uint32_t n) {
        if (busy_ || !dma_run_ok<T>(destination, n) || !dma_aligned(source, dma_width_of<T>())) {
            return false;
        }
        busy_ = true;
        Line::restart_from(word_ | dma_cfgr_beat<T>() | pinc, source, destination, static_cast<uint16_t>(n));
        return true;
    }

    static inline uint32_t word_ = 0;
    static inline uint32_t armed_ = 0;
    static inline volatile uint32_t faults_ = 0;   ///< bumped from a vector, read by the thread
    static inline volatile bool busy_ = false;
};

} // namespace brio

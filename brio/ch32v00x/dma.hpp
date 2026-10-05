/*
 * dma.hpp
 *
 * The DMA controller of the CH32V00x (RM ch. 8): seven channels, each
 * wired to a fixed handful of peripheral requests, an arbiter over four
 * software priorities with the channel index deciding ties, and three
 * flags per channel - the STM32F1's DMA1 under WCH's names, with no
 * request multiplexer: HERE THE CHANNEL IS THE REQUEST. RM table 8-2
 * says which peripheral event each channel answers, and an engine is
 * named by channel number with that table in hand (USART1 transmits on
 * channel 4 and receives on 5, SPI1 on 3 and 2, the ADC on 1, I2C1 on 6
 * and 7, TIM1's update on 5 and TIM2's on 2).
 *
 * WHAT A CHANNEL IS. A configuration word (CFGR: direction, circular,
 * memory-to-memory, the two increments, the two widths, the priority,
 * the three interrupt enables and EN), a count that the controller
 * decrements as it moves items (CNTR), and two addresses named after
 * the registers' F1 ancestry - PADDR and MADDR - where DIR says which
 * side is the source: in memory-to-memory mode both are memory and the
 * names mean nothing. 8.2.1's note makes PADDR, MADDR, CNTR and CFGR's
 * DIR, CIRC, PINC and MINC writable only while EN is clear.
 *
 * THE VECTORS ARE ONE PER CHANNEL (Irq::dma1_channel1..7), and the
 * FLAGS OF ALL SEVEN ARE ONE REGISTER (INTFR, four bits a channel), so a
 * handler serving two channels of one transport reads it once. DMA1EN in
 * RCC_HBPCENR is the block's one gate, CLOSED at reset (the register's
 * reset value opens the SRAM alone).
 *
 * TWO THINGS THE SILICON DOES that the engines are built on. EN STAYS
 * SET when a non-circular block completes (CNTR at zero, TCIF up, the
 * channel enabled and idle - only software clears EN), and 8.3.4's note
 * says a channel whose count is zero moves nothing enabled or not. And
 * A READ FROM A HOLE IN THE MAP COMPLETES AS A NORMAL BLOCK (zeros
 * arrive, TEIF stays down): the transfer-error path is written and
 * armed on every engine, but no address the bench could name reaches
 * it. And 8.3.5/8.3.6 say the module IGNORES the low address bits of a
 * 16- or 32-bit access, so a half-word transfer pointed at an odd
 * address moves the wrong bytes and reports success - every verb here
 * that takes a whole transfer or a run refuses an end that is not
 * aligned to its width (the piecewise set_peripheral()/set_memory()
 * store what they are given).
 *
 * `DmaChannel<ch>` is the channel's whole register surface: the
 * one-shot verbs a suite or a rare user programs a block with
 * (configure, prepare, load - each validating, each opening the gate),
 * the flags, the ISR body. THE ENGINES do not go through it: they are
 * the hot path and are written as TWO MOMENTS.
 *
 *  - `arm()` CONFIGURES ONCE. The gate opened, the channel stopped,
 *    PADDR written - the peripheral's register never changes for a
 *    binding - and the configuration word computed and KEPT IN THE
 *    ENGINE: direction, priority, the interrupt enables the binding
 *    wants. The PFIC line enabled.
 *  - `start()` WRITES WHAT CHANGES, five stores and no load: CFGR with
 *    EN clear (the configuration word with this block's beat and
 *    increment - the disable 8.2.1 asks for before the count and the
 *    address, and EN may still be standing from the last block), CNTR,
 *    MADDR, the channel's four flags cleared in INTFCR, CFGR again with
 *    EN set. The WCH examples restart a channel with a disable and an
 *    enable as read-modify-writes of CFGR around the count and the
 *    address; the engine knows the word and stores it. Nothing is
 *    validated but the run itself (its length, its alignment): the
 *    binding is the engine's type (the channel, the widest beat - checked
 *    at compile time) and arm()'s arguments.
 *
 * THE BEAT IS THE ELEMENT OF THE RUN. `start()` takes a span of uint8_t,
 * uint16_t or uint32_t and the transfer's two widths follow the type, so
 * one channel moves bytes for one block and half-words for the next (an
 * SPI's 8- and 16-bit frames). The engine's `Elem` is the WIDEST beat
 * the binding allows - the peripheral register's width - and a wider
 * span does not compile. A run whose address is not aligned to its beat
 * is REFUSED (8.3.6), and its caller falls back to whatever moves it
 * otherwise.
 *
 * THE CLAIM AND THE PROGRAMMING ARE TWO VERBS. A transmit engine is
 * BUSY from its claim to its complete(). `claim()` is that flag's
 * test-and-set and nothing else; `launch()` programs a claimed engine.
 * A transport whose completion handler starts the next block holds its
 * mask over `claim()` alone and launches outside it: a claimed channel
 * cannot complete under the programmer's feet, because it is not
 * running. `start()` is the two together, for an owner with no handler
 * to race.
 *
 * `DmaCopyEngine<ch, Elem>` is the memory-to-memory shape: `copy(dst,
 * src, n)` and `fill(dst, cell, n)` (the source a fixed cell in memory),
 * n in elements of the beat, on any channel, with no request - 8.2.1:
 * with MEM2MEM set the channel runs as soon as it is enabled.
 *
 * NOT COVERED YET: the loop and ping-pong engines a block stream is
 * served by (util/block_stream.hpp's two concepts: an ADC sampled into
 * caller-owned halves, a table played for ever) and the circular
 * receive whose producer index is the hardware's count - the first
 * born with their first block user, the second with the ring that
 * reads its producer index from outside.
 */

#pragma once

#include <stddef.h>
#include <stdint.h>

#include <span>

#include "ch32v00x/device.hpp"
#include "ch32v00x/pfic.hpp"

namespace brio {

// ---- the vocabulary -------------------------------------------------------------

/// CFGR.PSIZE / MSIZE: the width of one bus access.
enum class DmaWidth : uint8_t { byte = 0, half = 1, word = 2 };

template <typename Elem>
constexpr DmaWidth dma_width_of() {
    static_assert(sizeof(Elem) == 1 || sizeof(Elem) == 2 || sizeof(Elem) == 4,
                  "a DMA element is one bus access wide: 1, 2 or 4 bytes");
    return sizeof(Elem) == 1 ? DmaWidth::byte : sizeof(Elem) == 2 ? DmaWidth::half : DmaWidth::word;
}

constexpr uint32_t dma_width_bytes(DmaWidth w) { return 1UL << static_cast<uint32_t>(w); }

/// RM 8.3.5/8.3.6: a 16- or 32-bit access IGNORES the address's low
/// bits, so an address off its width's boundary moves the wrong bytes.
constexpr bool dma_aligned(uint32_t address, DmaWidth w) {
    return (address & (dma_width_bytes(w) - 1UL)) == 0UL;
}

/// CFGR.PL: the software half of the arbitration; ties go to the lower
/// channel index (RM 8.2.1).
enum class DmaPriority : uint8_t { low = 0, medium = 1, high = 2, very_high = 3 };

/// CFGR.DIR says WHICH SIDE IS THE SOURCE: 0 reads PADDR and writes
/// MADDR, 1 the other way round.
enum class DmaDirection : uint8_t { peripheral_to_memory = 0, memory_to_peripheral = 1 };

/// The four bits one channel owns in INTFR and INTFCR, at 4 x (ch - 1).
/// TCIF, HTIF and TEIF sit at the same positions as their enables in
/// CFGR (TCIE, HTIE, TEIE), which is what lets an engine read its armed
/// set off the configuration word it keeps.
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

/// RM 8.2.1's own rule: circular mode is not for memory-to-memory.
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

struct DmaRegs {
    volatile uint32_t INTFR;         ///< 0x00 the flags, read-only
    volatile uint32_t INTFCR;        ///< 0x04 write 1 to clear
    DmaChannelRegs channel[7];       ///< 0x08 + 0x14 x (ch - 1)
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
inline constexpr uint32_t rcc_hb_dma1      = 1UL << 0;

static_assert(dma_cfgr_tcie == DmaFlag::complete && dma_cfgr_htie == DmaFlag::half &&
                  dma_cfgr_teie == DmaFlag::error,
              "an enable and its flag share a position (8.3.1, 8.3.3)");

inline constexpr uint8_t dma_channel_count = 7;

constexpr Irq dma_channel_irq(uint8_t ch) {
    return static_cast<Irq>(static_cast<uint8_t>(Irq::dma1_channel1) + (ch - 1u));
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

    static void open() { rcc()->HBPCENR |= rcc_hb_dma1; }
    static uint32_t flags() { return dma()->INTFR; }
};

/**
 * One channel, 1..7. Every configuring verb refuses while the channel
 * is enabled, because CFGR's fields, CNTR and the two addresses are
 * read-only then (RM 8.2.1's note): a store the silicon ignores would be
 * a lie the code told itself.
 */
template <uint8_t ch>
class DmaChannel {
    static_assert(ch >= 1 && ch <= dma_channel_count, "the CH32V00x DMA has channels 1..7");

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

    /// Everything but the addresses and the count. Refused while
    /// enabled, and for a config the chapter forbids.
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
    /// to its width (8.3.5, 8.3.6).
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
     * The channel's ISR body: read the flags that are ARMED and up, clear
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
 * What every engine does to its channel, in the two moments: `bind()`
 * at arm() and `restart()` at every block. Not a user's type - an
 * engine's.
 */
template <uint8_t ch>
struct DmaEngineChannel {
    static_assert(ch >= 1 && ch <= dma_channel_count, "no such DMA channel for this engine");

    DmaEngineChannel() = delete;

    using Channel = DmaChannel<ch>;
    static constexpr uint32_t shift = Channel::flag_shift;

    /// The binding: the gate opened (the one RCC read-modify-write an
    /// engine ever makes), the channel stopped with its flags cleared,
    /// the peripheral end written - PADDR, which nothing but the next
    /// arm() changes - and the PFIC line enabled.
    static void bind(uint32_t peripheral) {
        Dma::open();
        DmaChannelRegs& r = Channel::regs();
        r.CFGR = 0;
        dma()->INTFCR = DmaFlag::all << shift;
        r.PADDR = peripheral;
        Pfic::enable(Channel::irq());
    }

    /// One block: the configuration word with EN clear (the disable
    /// 8.2.1 asks for before CNTR and MADDR, EN standing after the last
    /// block), the count, the memory end, the flags, the enable. Five
    /// stores, no load.
    [[gnu::always_inline]] static void restart(uint32_t word, uint32_t memory, uint16_t count) {
        DmaChannelRegs& r = Channel::regs();
        r.CFGR = word;
        r.CNTR = count;
        r.MADDR = memory;
        dma()->INTFCR = DmaFlag::all << shift;
        r.CFGR = word | dma_cfgr_en;
    }

    /// The channel's armed flags that are up in `intfr` (INTFR read once
    /// by the caller, which may serve two channels with it), cleared with
    /// GIF and handed back.
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
 * register - the peripheral's request paces it (table 8-2's rows for
 * this channel), whichever the program has enabled.
 */
template <uint8_t ch, typename Elem = uint8_t>
class DmaTxEngine {
    static_assert(ch >= 1 && ch <= dma_channel_count, "no such DMA channel for this engine");
    static_assert(sizeof(Elem) == 1 || sizeof(Elem) == 2 || sizeof(Elem) == 4,
                  "a DMA element is one bus access wide: 1, 2 or 4 bytes");
    using Line = DmaEngineChannel<ch>;
    using Channel = DmaChannel<ch>;

public:
    DmaTxEngine() = delete;

    static constexpr bool present = true;
    static constexpr uint8_t channel = ch;
    /// The WIDEST beat this binding moves: start() takes any narrower.
    static constexpr DmaWidth width = dma_width_of<Elem>();
    using element = Elem;

    static constexpr uint8_t flag_complete = DmaFlag::complete;
    static constexpr uint8_t flag_half = DmaFlag::half;
    static constexpr uint8_t flag_error = DmaFlag::error;

    /// The channel's ISR body folded into the engine: the armed flags
    /// that are up, cleared and handed back. Acts on nothing.
    [[gnu::always_inline]] static uint8_t service() { return service(Dma::flags()); }
    /// The same over an INTFR the caller read once for two channels.
    [[gnu::always_inline]] static uint8_t service(uint32_t intfr) {
        return static_cast<uint8_t>(Line::service(intfr, armed_));
    }
    /// INTFR, every channel's flags in one load: what service(intfr)
    /// takes, for a driver that reaches the controller only through its
    /// engines' names.
    [[gnu::always_inline]] static uint32_t block_flags() { return Dma::flags(); }

    /**
     * Bind the channel to its peripheral: `data` is the register the run
     * is poured into, `interrupts` the flags whose interrupt the binding
     * wants (flag_complete and/or flag_error) - a transport whose
     * completion another event proves arms the errors alone -, `priority`
     * CFGR.PL. The gate and the PFIC line are opened here, once.
     */
    static void arm(volatile void* data, uint32_t interrupts = DmaFlag::complete | DmaFlag::error,
                    DmaPriority priority = DmaPriority::low) {
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

    /// THE CLAIM: true when the engine was idle and is now the caller's.
    /// A test-and-set and nothing else - the one step a transport whose
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

    /// Program a CLAIMED engine with the run (the caller's, and it stays
    /// put until complete() reports the block), the beat its element's
    /// width. False - and the claim given back - for an empty run, one
    /// over 65535 elements, or one off its beat's boundary (8.3.6).
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

    /// `length` copies of ONE element: the memory pointer does not
    /// increment. What a full-duplex bus needs to clock a read. Claims
    /// as start() does.
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
        ++faults_;
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
    static inline uint32_t faults_ = 0;
    static inline uint16_t in_flight_ = 0;
    static inline volatile bool busy_ = false;
};

/**
 * A receive engine: one caller-owned run filled from one peripheral
 * register, and asked how much has arrived - or, in its CIRCULAR SHAPE,
 * a caller-owned ring filled for ever.
 *
 * THE CIRCULAR SHAPE is CFGR.CIRC (8.2.1: at a count of zero the channel
 * reloads CNTR and its two addresses and goes on with no CPU in the
 * path): arm_ring() is handed the ring's whole storage and binds it for
 * good - PADDR, MADDR, the count every lap reloads, the word with CIRC,
 * MINC, the beat, the lap's completion and the transfer error, and with
 * `half_mark` the half lap's flag - and starts it. Nothing re-arms it
 * afterwards. The producer index of that ring is the channel's own
 * count, and this engine is util/ring.hpp's RingCounter for it:
 * remaining() is CNTR, one load (8.2.1 orders a transfer as the read, the
 * store and then the decrement, so the count never counts an element
 * whose store has not been made), and laps() the completions lap()
 * counted in the channel's handler - lagging the count by a handler's
 * latency, never leading it. util/ring.hpp's HardwareRing is the
 * consumer half. A circular channel never stops on its own: idle()
 * after a transfer error (the hardware clears EN) is what says it has.
 */
template <uint8_t ch, typename Elem = uint8_t>
class DmaRxEngine {
    static_assert(ch >= 1 && ch <= dma_channel_count, "no such DMA channel for this engine");
    static_assert(sizeof(Elem) == 1 || sizeof(Elem) == 2 || sizeof(Elem) == 4,
                  "a DMA element is one bus access wide: 1, 2 or 4 bytes");
    using Line = DmaEngineChannel<ch>;
    using Channel = DmaChannel<ch>;

public:
    DmaRxEngine() = delete;

    static constexpr bool present = true;
    static constexpr uint8_t channel = ch;
    /// The WIDEST beat this binding moves: start() takes any narrower.
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

    /// Bind the channel to its peripheral (DmaTxEngine::arm()'s terms).
    static void arm(volatile void* data, uint32_t interrupts = DmaFlag::complete | DmaFlag::error,
                    DmaPriority priority = DmaPriority::low) {
        armed_ = interrupts & dma_cfgr_irqs;
        word_ = dma_cfgr_priority(priority) | armed_;
        capacity_ = 0;
        taken_ = 0;
        Line::bind(dma_address_of(data));
    }
    static void arm(volatile void* data, DmaPriority priority) {
        arm(data, DmaFlag::complete | DmaFlag::error, priority);
    }

    /// Not running: stopped, never started, or halted by a transfer
    /// error (8.2.1: the hardware clears EN). One CFGR read.
    static bool idle() { return !Channel::enabled(); }

    /// Fill the run, the beat its element's width. False for an empty
    /// run, one over 65535 elements, or one off its beat's boundary.
    template <typename T>
    static bool start(std::span<T> run) {
        static_assert(dma_beat_fits<T, Elem>, "a beat wider than this binding's widest (its Elem)");
        return begin<T>(dma_cfgr_minc, dma_address_of(run.data()), run.size());
    }

    /// `length` elements into ONE cell, thrown away (the receive side of
    /// a bus write).
    template <typename T>
    static bool start_discard(T* cell, uint16_t length) {
        static_assert(dma_beat_fits<T, Elem>, "a beat wider than this binding's widest (its Elem)");
        return begin<T>(0u, dma_address_of(cell), length);
    }

    /// How many elements have arrived since the last take(): one CNTR
    /// read, nothing suspended, nothing refused.
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
        ++faults_;
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
     * THE CIRCULAR BINDING, which is also its start (the class header):
     * `storage` the caller's whole ring, its element the beat, its length
     * the count every lap reloads. False, and nothing started, for a
     * storage off its beat's boundary. Calling it again restarts the
     * ring at its first element with laps() at zero - the moment a
     * HardwareRing over the same storage is clear()ed.
     */
    template <typename T, size_t N>
    static bool arm_ring(volatile void* data, T (&storage)[N], bool half_mark = false,
                         DmaPriority priority = DmaPriority::low) {
        static_assert(dma_beat_fits<T, Elem>, "a beat wider than this binding's widest (its Elem)");
        static_assert(N >= 2u && N <= 0xFFFFu,
                      "brio DmaRxEngine: a ring of 2..65535 items - CNTR is sixteen bits (8.3.2)");
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
    static inline uint32_t faults_ = 0;
    static inline volatile uint32_t laps_ = 0;
    static inline uint16_t capacity_ = 0;
    static inline uint16_t taken_ = 0;
};

/**
 * Memory to memory: `copy()` a run, or `fill()` one from a fixed cell,
 * on any channel - 8.2.1: with MEM2MEM set the channel needs no request
 * and runs from its enable, so the channel number says only where it
 * sits in the arbitration (ties go to the lower number) and which
 * vector reports it. Neither circular mode (the chapter refuses it with
 * MEM2MEM) nor a half-transfer interrupt is used.
 *
 * The source is the PADDR side (DIR clear: read PADDR, write MADDR), so
 * a copy is PINC and MINC set and a fill is MINC alone. The engine is
 * BUSY from a block's start to its end, which the program learns either
 * from the channel's vector - arm(..., true), the app binding isr() -
 * or by polling busy(), which reads the flags while a block runs.
 * `abandon()` stops a running block where it stands: EN cleared, the
 * count holding what was left.
 */
template <uint8_t ch, typename Elem = uint32_t>
class DmaCopyEngine {
    static_assert(ch >= 1 && ch <= dma_channel_count, "no such DMA channel for this engine");
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
    /// priority, the interrupt (complete and error) wanted or not. With
    /// no interrupt the line stays disabled and busy() is the witness.
    static void arm(DmaPriority priority = DmaPriority::low, bool interrupt = true) {
        armed_ = interrupt ? (DmaFlag::complete | DmaFlag::error) : 0u;
        word_ = dma_cfgr_mem2mem | dma_cfgr_minc | dma_cfgr_priority(priority) | armed_;
        busy_ = false;
        Line::bind(0u);
        if (!interrupt) {
            Pfic::disable(Channel::irq());
        }
    }

    /// Copy `n` elements from `src` to `dst`, the beat the element's
    /// width (T: uint8_t, uint16_t or uint32_t, no wider than Elem).
    /// False while a block runs, and for n of zero or over CNTR's 65535,
    /// or an end off its beat's boundary (8.3.5, 8.3.6).
    template <typename T>
    static bool copy(T* dst, const T* src, uint32_t n) {
        static_assert(dma_beat_fits<T, Elem>, "a beat wider than this binding's widest (its Elem)");
        return begin<T>(dma_cfgr_pinc, dma_address_of(src), dma_address_of(dst), n);
    }

    /// Fill `n` elements at `dst` with copies of `*cell` - a cell IN
    /// MEMORY, the caller's, which stays put until the block ends (the
    /// controller reads an address). False as copy() is.
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
            ++faults_;
        }
        busy_ = false;
        return false;
    }

    /// Stop a running block where it stands - EN cleared, CNTR holding
    /// what was left, the flags cleared - and free the engine. False when
    /// no block was running.
    static bool abandon() {
        if (!busy_) {
            return false;
        }
        Line::halt();
        busy_ = false;
        return true;
    }

    /// The channel's ISR body: the armed flags that are up, cleared and
    /// handed back; a completion or an error frees the engine (an error
    /// counted). Non-zero when the block ended.
    [[gnu::always_inline]] static uint8_t isr() {
        const uint32_t up = Line::service(Dma::flags(), armed_);
        if (up != 0u) {
            if ((up & DmaFlag::error) != 0u) {
                ++faults_;
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
        Channel::regs().PADDR = source;
        Line::restart(word_ | dma_cfgr_beat<T>() | pinc, destination, static_cast<uint16_t>(n));
        return true;
    }

    static inline uint32_t word_ = 0;
    static inline uint32_t armed_ = 0;
    static inline uint32_t faults_ = 0;
    static inline volatile bool busy_ = false;
};

} // namespace brio

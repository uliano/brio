/*
 * dma.hpp
 *
 * The DMA controller of the CH32V203 (RM ch. 11): EIGHT channels, each
 * wired to a fixed handful of peripheral requests, an arbiter over four
 * software priorities with the channel index deciding ties, and four
 * flag bits a channel - the STM32F1's DMA1 under WCH's names, with one
 * channel more than the ancestor and than the CH32V00x's seven. On the
 * CH32V303 the same block is DMA1 with SEVEN channels and a second
 * controller of eleven beside it (device::dma1_channel_count,
 * dma2_channel_count); this file drives DMA1 on every part, and DMA2 is
 * not reached by it.
 *
 * THE CHANNEL IS THE REQUEST. There is no request multiplexer: table
 * 11-5 (our class) and table 11-6 (the other one) wire each peripheral
 * event to ONE channel, and an engine is named by channel number with
 * that table in hand. The table itself is ch32vx03/dma_engine.hpp's,
 * because a transport must be able to refuse an engine on the wrong
 * channel without including this file; `DmaRequestOf<r>::channel` is
 * how a program names the request instead of the number.
 *
 * WHAT A CHANNEL IS. A configuration word (CFGR: direction, circular,
 * memory-to-memory, the two increments, the two widths, the priority,
 * the three interrupt enables and EN), a count the controller
 * decrements as it moves items (CNTR, read-only while enabled), and two
 * addresses named after the registers' F1 ancestry - PADDR and MADDR -
 * where DIR says which side is the source: in memory-to-memory mode
 * both are memory and the names mean nothing.
 *
 * THE ADDRESSES ARE ALIGNED BY THE SILICON, SILENTLY. 11.3.5 and 11.3.6
 * say the module IGNORES the low address bits of a 16- or 32-bit
 * access, so a half-word transfer pointed at an odd address moves the
 * WRONG BYTES and reports success. Every verb here refuses an address
 * that is not aligned to its own width rather than let the hardware
 * round it: a transfer that quietly reads elsewhere is worse than one
 * that does not start.
 *
 * THE VECTORS ARE ONE PER CHANNEL, and the eighth is on the tail of the
 * table this DEVICE CLASS has - 62 here, 67 on the CH32V203RB (the crt
 * states the per-class truth, and device.hpp's Irq enum carries both).
 * So a channel's ISR body reads only its own flags and a handler needs
 * no "which channel" question. DMA1EN in RCC_HBPCENR is the block's one
 * gate and it is CLOSED at reset, so `Dma::open()` is the first thing
 * every channel verb does.
 *
 * NO DMA-FED TRANSPORT SLEEPS ON THIS FAMILY. In the Sleep of RM 2.4 no
 * bus master but the core gets a cycle: a memory-to-memory block
 * started right before an idle() moves the handful of items already in
 * the pipeline and then nothing until the core wakes, while the timers
 * and the core's counter count the whole sleep (measured -
 * docs/ch32vx03/dma.md, and it is the same starvation that kills the
 * USB controller in Sleep). SO A CHANNEL COUNTS ITSELF: the EN
 * transition is one bus master entering and leaving
 * (ch32vx03/bus_activity.hpp), the kernel's idle path does not sleep
 * while the count stands, and a sleep site refuses to arm over it -
 * a program with an engine running holds itself awake without having
 * to know that it does. `Dma::any_enabled()` is the same question
 * asked of the registers instead, for a caller that wants the
 * silicon's own answer.
 *
 * THE ENGINES. `DmaTxEngine<ch, Elem>` and `DmaRxEngine<ch, Elem>` are
 * the CH32V00x stratum's: a transmit engine pours a caller-owned run
 * into one peripheral register and reports how many items the block
 * carried when it completes; a receive engine fills a caller-owned run
 * from one register and answers how many items have arrived since it
 * was last asked (one CNTR read - nothing suspended, nothing refused).
 * ch32vx03/usart.hpp's two engine slots are built on them, and a driver
 * reaches its engines only through their published names (start,
 * service, flag_complete, flag_error) so that a driver with an empty
 * slot never includes this file. TWO ENGINES OF ONE TRANSPORT NAME TWO
 * CHANNELS: a channel moves data one way, and the table gives each
 * direction its own.
 *
 * ONE THING THE SILICON DOES that the engines are built on: EN STAYS
 * SET when a non-circular block completes (CNTR at zero, TCIF up, the
 * channel enabled and idle - the F1 lineage: only software clears EN),
 * so every verb that programs a channel refuses while EN is up and the
 * owner disables before the next load. A TRANSFER ERROR is the one
 * thing that clears EN by itself (11.3.3).
 *
 * THE BLOCK ENGINES beside them are util/block_stream.hpp's two
 * concepts: `DmaLoopEngine<ch, Elem>` is a BlockPlayer, one
 * caller-owned table poured into a peripheral for ever on THE
 * CONTROLLER'S OWN CIRCULAR MODE, with the lap interrupt doing nothing
 * but count; `DmaPingPongEngine<ch, Elem>` is a BlockSource, two
 * caller-owned buffers filled in turn - and it does NOT use circular
 * mode, which is the one place this file departs from what the
 * controller offers. The reason is the contract's and not the API's: a
 * circular channel never stops, so "skip rather than tear" could only
 * be decided after the edge, with the controller already writing the
 * buffer the caller holds (measured on the STM32G0, and again here -
 * docs/ch32vx03/adc.md). So the source stops itself at every block and
 * the handler re-arms the other buffer, which is the SAM C21's shape
 * and the STM32G0's; the four engine names are the same words in all
 * three strata (docs/design/block-stream.md).
 */

#pragma once

#include <stdint.h>

#include "ch32vx03/bus_activity.hpp"
#include "ch32vx03/clock.hpp"
#include "ch32vx03/device.hpp"
#include "ch32vx03/dma_engine.hpp"
#include "ch32vx03/pfic.hpp"

namespace brio {

// ---- the vocabulary -------------------------------------------------------------

/// CFGR.PSIZE / MSIZE: the width of one bus access (11.3.3).
enum class DmaWidth : uint8_t { byte = 0, half = 1, word = 2 };

template <typename Elem>
constexpr DmaWidth dma_width_of() {
    static_assert(sizeof(Elem) == 1 || sizeof(Elem) == 2 || sizeof(Elem) == 4,
                  "a DMA element is one bus access wide: 1, 2 or 4 bytes");
    return sizeof(Elem) == 1 ? DmaWidth::byte : sizeof(Elem) == 2 ? DmaWidth::half : DmaWidth::word;
}

/// The bytes one access moves, and the address bits it must not carry.
constexpr uint8_t dma_width_bytes(DmaWidth w) {
    return w == DmaWidth::byte ? 1u : w == DmaWidth::half ? 2u : 4u;
}
constexpr uint32_t dma_width_mask(DmaWidth w) { return dma_width_bytes(w) - 1u; }

/// CFGR.PL: the software half of the arbitration; ties go to the lower
/// channel index (11.2.1).
enum class DmaPriority : uint8_t { low = 0, medium = 1, high = 2, very_high = 3 };

/// CFGR.DIR says WHICH SIDE IS THE SOURCE: 0 reads PADDR and writes
/// MADDR, 1 the other way round.
enum class DmaDirection : uint8_t { peripheral_to_memory = 0, memory_to_peripheral = 1 };

/// The four bits one channel owns in INTFR and INTFCR, at 4 x (ch - 1).
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

/// 11.2.1's own rule: circular mode is not for memory-to-memory, which
/// has no request to reload against.
constexpr bool dma_channel_config_valid(const DmaChannelConfig& c) {
    return !(c.circular && c.memory_to_memory);
}

/// One block transfer: both ends and how many data items. The count is
/// in ITEMS of the respective width, and 11.1 caps it at 65535.
struct DmaTransfer {
    volatile void* peripheral = nullptr;   ///< PADDR
    volatile void* memory = nullptr;       ///< MADDR
    uint16_t count = 0;                    ///< CNTR, in data items
    DmaChannelConfig config{};
};

/// The shape of a transfer, at compile time where it is a constant:
/// both ends named, something to move, a legal configuration.
constexpr bool dma_transfer_valid(const DmaTransfer& t) {
    return t.peripheral != nullptr && t.memory != nullptr && t.count != 0u &&
           dma_channel_config_valid(t.config);
}

/// The other half of a transfer's validity, which only an address can
/// answer: each end aligned to ITS OWN width (11.3.5, 11.3.6 - the
/// module rounds a misaligned address down in silence).
inline bool dma_transfer_aligned(const DmaTransfer& t) {
    const uint32_t p = static_cast<uint32_t>(reinterpret_cast<uintptr_t>(t.peripheral));
    const uint32_t m = static_cast<uint32_t>(reinterpret_cast<uintptr_t>(t.memory));
    return (p & dma_width_mask(t.config.peripheral_width)) == 0u &&
           (m & dma_width_mask(t.config.memory_width)) == 0u;
}

struct DmaProgress {
    uint16_t remaining = 0;
    uint16_t done = 0;
};

// ---- the registers --------------------------------------------------------------

/// One channel's four registers and the word the map leaves between
/// them: table 11-7's stride is twenty bytes, not sixteen.
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
    DmaChannelRegs channel[8];       ///< 0x08 + 20 x (ch - 1)
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
inline constexpr uint32_t dma_cfgr_psize_shift = 8;
inline constexpr uint32_t dma_cfgr_msize_shift = 10;
inline constexpr uint32_t dma_cfgr_pl_shift    = 12;
inline constexpr uint32_t dma_cfgr_mem2mem = 1UL << 14;

/// DMA1's channels on THIS part: eight on every CH32V203, whose two
/// classes the register notes of 11.3.1 to 11.3.6 name among the five
/// that have channel 8, and seven on the CH32V303, whose class they do
/// not (device::dma1_channel_count).
inline constexpr uint8_t dma_channel_count = device::dma1_channel_count;

/// The line a channel reports on. Seven of them are consecutive from
/// the table's entry 27; the eighth, where there is one, sits on the
/// class's own tail, which device.hpp's Irq table places.
constexpr Irq dma_channel_irq(uint8_t ch) {
    return ch == 8u ? Irq::dma1_channel8
                    : static_cast<Irq>(static_cast<uint8_t>(Irq::dma1_channel1) + (ch - 1u));
}

/**
 * The block: its gate, its two flag registers, and the one question the
 * power model asks it.
 */
struct Dma {
    Dma() = delete;

    static void open() { Rcc::enable(Bus::hb, rcc_hb_dma1); }
    static bool opened() { return Rcc::enabled(Bus::hb, rcc_hb_dma1); }

    /**
     * Every channel back to the chapter's own reset values, the gate
     * left open.
     *
     * THIS BLOCK HAS NO RESET LINE: RCC_AHBRSTR (3.4.11) reserves bits
     * [11:0] and names only the Ethernet MAC, the DVP and the USB OTG
     * core, so the pulse every PB1 and PB2 peripheral of this stratum
     * is put back with does not exist here and software does the work.
     */
    static void stop_all() {
        open();
        for (uint8_t ch = 0; ch < dma_channel_count; ++ch) {
            const bool was = (dma()->channel[ch].CFGR & dma_cfgr_en) != 0u;
            dma()->channel[ch].CFGR = 0;
            if (was) {
                BusActivity::left();
            }
            dma()->channel[ch].CNTR = 0;
            dma()->channel[ch].PADDR = 0;
            dma()->channel[ch].MADDR = 0;
        }
        dma()->INTFCR = 0xFFFFFFFFUL;
    }

    static uint32_t flags() { return dma()->INTFR; }
    static void clear(uint32_t mask) { dma()->INTFCR = mask; }

    /**
     * Is ANY channel enabled? The question the sleep site of this family
     * has to ask before it arms a mode: in Sleep the bus matrix serves
     * the core alone, so a channel that is up is a transfer that will
     * stall for the whole sleep. Answers false with the gate closed,
     * which is a block that cannot have a channel running.
     */
    static bool any_enabled() {
        if (!opened()) {
            return false;
        }
        for (uint8_t ch = 0; ch < dma_channel_count; ++ch) {
            if ((dma()->channel[ch].CFGR & dma_cfgr_en) != 0u) {
                return true;
            }
        }
        return false;
    }
};

/**
 * One channel, 1..8. Every configuring verb refuses while the channel
 * is enabled, because CFGR's fields, CNTR and the two addresses are
 * read-only then (11.2.1's note and each register's own): a store the
 * silicon ignores would be a lie the code told itself.
 */
template <uint8_t ch>
class DmaChannel {
    static_assert(ch >= 1 && ch <= dma_channel_count,
                  "DMA1 has channels 1..8 on the CH32V203 and 1..7 on the CH32V303 "
                  "(device::dma1_channel_count)");

public:
    DmaChannel() = delete;

    static constexpr uint8_t number = ch;
    static constexpr uint32_t flag_shift = 4u * (ch - 1u);

    static DmaChannelRegs& regs() { return dma()->channel[ch - 1u]; }
    static constexpr Irq irq() { return dma_channel_irq(ch); }

    static bool enabled() { return (regs().CFGR & dma_cfgr_en) != 0u; }

    /// Enable or disable. Disabling a running channel abandons the block
    /// where it stands; CNTR then holds what was left.
    ///
    /// AND THIS IS WHERE THE FAMILY'S BUS ACTIVITY IS COUNTED. EN is the
    /// bit that makes this channel a bus master, and a master that is up
    /// forbids a sleep of any depth here (ch32vx03/bus_activity.hpp), so
    /// the TRANSITION is counted in the one verb every engine and every
    /// task goes through - never the enable itself, which would count an
    /// idempotent store twice.
    static void enable(bool on) {
        Dma::open();
        const bool was = enabled();
        if (on) {
            regs().CFGR |= dma_cfgr_en;
        } else {
            regs().CFGR &= ~dma_cfgr_en;
        }
        if (on != was) {
            if (on) {
                BusActivity::entered();
            } else {
                BusActivity::left();
            }
        }
    }

    /// Everything but the addresses and the count. Refused while
    /// enabled, and for the configuration the chapter forbids.
    static bool configure(const DmaChannelConfig& c) {
        Dma::open();
        if (enabled() || !dma_channel_config_valid(c)) {
            return false;
        }
        uint32_t v = regs().CFGR & (dma_cfgr_tcie | dma_cfgr_htie | dma_cfgr_teie);
        if (c.direction == DmaDirection::memory_to_peripheral) { v |= dma_cfgr_dir; }
        if (c.circular) { v |= dma_cfgr_circ; }
        if (c.memory_to_memory) { v |= dma_cfgr_mem2mem; }
        if (c.peripheral_increment) { v |= dma_cfgr_pinc; }
        if (c.memory_increment) { v |= dma_cfgr_minc; }
        v |= static_cast<uint32_t>(c.peripheral_width) << dma_cfgr_psize_shift;
        v |= static_cast<uint32_t>(c.memory_width) << dma_cfgr_msize_shift;
        v |= static_cast<uint32_t>(c.priority) << dma_cfgr_pl_shift;
        regs().CFGR = v;
        return true;
    }

    /// The configuration as the register holds it, which is what a test
    /// compares against what it asked for.
    static DmaChannelConfig configuration() {
        const uint32_t v = regs().CFGR;
        DmaChannelConfig c{};
        c.direction = (v & dma_cfgr_dir) != 0u ? DmaDirection::memory_to_peripheral
                                               : DmaDirection::peripheral_to_memory;
        c.circular = (v & dma_cfgr_circ) != 0u;
        c.memory_to_memory = (v & dma_cfgr_mem2mem) != 0u;
        c.peripheral_increment = (v & dma_cfgr_pinc) != 0u;
        c.memory_increment = (v & dma_cfgr_minc) != 0u;
        c.peripheral_width = static_cast<DmaWidth>((v >> dma_cfgr_psize_shift) & 3u);
        c.memory_width = static_cast<DmaWidth>((v >> dma_cfgr_msize_shift) & 3u);
        c.priority = static_cast<DmaPriority>((v >> dma_cfgr_pl_shift) & 3u);
        return c;
    }

    static bool set_count(uint16_t count) {
        if (enabled() || count == 0u) {
            return false;
        }
        regs().CNTR = count;
        return true;
    }

    /// Live while the channel runs: what is STILL TO MOVE. Zero with the
    /// channel still enabled is a completed block (see the file header).
    static uint16_t remaining() { return static_cast<uint16_t>(regs().CNTR); }

    /// Each end, refused unaligned to the width the channel is
    /// configured for (11.3.5, 11.3.6) and refused while enabled.
    static bool set_peripheral(volatile void* address) {
        if (enabled() ||
            (static_cast<uint32_t>(reinterpret_cast<uintptr_t>(address)) &
             dma_width_mask(configuration().peripheral_width)) != 0u) {
            return false;
        }
        regs().PADDR = static_cast<uint32_t>(reinterpret_cast<uintptr_t>(address));
        return true;
    }
    static bool set_memory(volatile void* address) {
        if (enabled() ||
            (static_cast<uint32_t>(reinterpret_cast<uintptr_t>(address)) &
             dma_width_mask(configuration().memory_width)) != 0u) {
            return false;
        }
        regs().MADDR = static_cast<uint32_t>(reinterpret_cast<uintptr_t>(address));
        return true;
    }

    /// Program the whole transfer WITHOUT starting it. The flags are
    /// cleared on the way in, so a poller cannot see the last block's.
    static bool prepare(const DmaTransfer& t) {
        Dma::open();
        if (enabled() || !dma_transfer_valid(t) || !dma_transfer_aligned(t)) {
            return false;
        }
        if (!configure(t.config)) {
            return false;
        }
        regs().PADDR = static_cast<uint32_t>(reinterpret_cast<uintptr_t>(t.peripheral));
        regs().MADDR = static_cast<uint32_t>(reinterpret_cast<uintptr_t>(t.memory));
        regs().CNTR = t.count;
        clear(DmaFlag::all);
        return true;
    }

    /// Program the whole transfer and start it.
    static bool load(const DmaTransfer& t) {
        if (!prepare(t)) {
            return false;
        }
        enable(true);
        return true;
    }

    /// A channel already prepared, started again with the same
    /// programme: what a circular stream's owner does after an abort,
    /// and what makes "prepare then trigger" two verbs and not one.
    static bool trigger() {
        if (enabled() || remaining() == 0u) {
            return false;
        }
        enable(true);
        return true;
    }

    static uint32_t flags() { return (dma()->INTFR >> flag_shift) & DmaFlag::all; }
    static bool flag(uint32_t mask) { return (flags() & mask) != 0u; }
    static void clear(uint32_t mask) { dma()->INTFCR = (mask & DmaFlag::all) << flag_shift; }

    /// Arm the channel's interrupts (the PFIC line is the caller's).
    static void arm(uint32_t mask, bool on) {
        uint32_t bits = 0;
        if ((mask & DmaFlag::complete) != 0u) { bits |= dma_cfgr_tcie; }
        if ((mask & DmaFlag::half) != 0u) { bits |= dma_cfgr_htie; }
        if ((mask & DmaFlag::error) != 0u) { bits |= dma_cfgr_teie; }
        if (on) {
            regs().CFGR |= bits;
        } else {
            regs().CFGR &= ~bits;
        }
    }

    /// Which of the three this channel's interrupts are armed for.
    static uint32_t armed() {
        const uint32_t cfg = regs().CFGR;
        uint32_t mask = 0;
        if ((cfg & dma_cfgr_tcie) != 0u) { mask |= DmaFlag::complete; }
        if ((cfg & dma_cfgr_htie) != 0u) { mask |= DmaFlag::half; }
        if ((cfg & dma_cfgr_teie) != 0u) { mask |= DmaFlag::error; }
        return mask;
    }

    /**
     * The channel's ISR body: read the flags that are ARMED and up, clear
     * exactly those with the global bit, hand them back. A completion is
     * not acted on here - what it means is the owner's to decide.
     */
    [[gnu::always_inline]] static uint32_t isr() {
        const uint32_t up = flags() & armed();
        if (up != 0u) {
            clear(up | DmaFlag::global);
        }
        return up;
    }

    static DmaProgress progress(uint16_t programmed) {
        const uint16_t left = remaining();
        const uint16_t done =
            left > programmed ? uint16_t{0} : static_cast<uint16_t>(programmed - left);
        return DmaProgress{left, done};
    }

    /// Stop and clear everything of the channel's: EN off, the config
    /// zeroed, the flags cleared.
    static void stop() {
        Dma::open();
        const bool was = enabled();
        regs().CFGR = 0;
        if (was) {
            BusActivity::left();
        }
        clear(DmaFlag::all);
    }
};

// ---- the engines ---------------------------------------------------------------

/**
 * A transmit engine: one caller-owned run poured into one peripheral
 * register. The channel IS the request (table 11-5), so the channel
 * number is the whole of the engine's identity.
 */
template <uint8_t ch, typename Elem = uint8_t>
class DmaTxEngine {
    static_assert(ch >= 1 && ch <= dma_channel_count, "no such DMA channel for this engine");
    static_assert(sizeof(Elem) == 1 || sizeof(Elem) == 2 || sizeof(Elem) == 4,
                  "a DMA element is one bus access wide: 1, 2 or 4 bytes");
    using Channel = DmaChannel<ch>;

public:
    DmaTxEngine() = delete;

    static constexpr bool present = true;
    static constexpr uint8_t channel = ch;
    static constexpr DmaWidth width = dma_width_of<Elem>();
    using element = Elem;

    static constexpr uint8_t flag_complete = DmaFlag::complete;
    static constexpr uint8_t flag_half = DmaFlag::half;
    static constexpr uint8_t flag_error = DmaFlag::error;

    /// The channel's ISR body folded into the engine: the armed flags
    /// that are up, cleared and handed back. Acts on nothing.
    [[gnu::always_inline]] static uint8_t service() {
        return static_cast<uint8_t>(Channel::isr());
    }

    /// Claim the channel for this peripheral: `data` is the register the
    /// run is poured into. The PFIC line is enabled here.
    static void arm(volatile void* data, DmaPriority priority = DmaPriority::low) {
        data_ = data;
        priority_ = priority;
        claim();
        Pfic::enable(Channel::irq());
    }

    /// Start moving `length` elements from `buffer` (the caller's, and
    /// it stays put until complete() reports the block).
    static bool start(const Elem* buffer, uint16_t length) {
        if (busy_ || buffer == nullptr || length == 0u) {
            return false;
        }
        in_flight_ = length;
        busy_ = Channel::load(run(const_cast<Elem*>(buffer), length, true));
        if (!busy_) {
            in_flight_ = 0;
        }
        return busy_;
    }

    /// `length` copies of ONE element: the memory pointer does not
    /// increment. What a full-duplex bus needs to clock a read.
    static bool start_fixed(const Elem* cell, uint16_t length) {
        if (busy_ || cell == nullptr || length == 0u) {
            return false;
        }
        in_flight_ = length;
        busy_ = Channel::load(run(const_cast<Elem*>(cell), length, false));
        if (!busy_) {
            in_flight_ = 0;
        }
        return busy_;
    }

    /// The block ended: how many elements it carried, so the owner can
    /// release exactly that much of its ring. The channel is disabled
    /// here because the silicon does not do it: EN stays set after a
    /// completed block, and load() refuses an enabled channel.
    static uint16_t complete() {
        const uint16_t n = in_flight_;
        Channel::enable(false);
        busy_ = false;
        in_flight_ = 0;
        return n;
    }

    static bool busy() { return busy_; }
    static uint16_t in_flight() { return busy_ ? in_flight_ : uint16_t{0}; }
    static DmaProgress progress() { return Channel::progress(in_flight_); }

    /// Throw the running block away and hand the channel back ready.
    static bool abandon() {
        ++faults_;
        busy_ = false;
        in_flight_ = 0;
        claim();
        return true;
    }

    /// Start the run again from the beginning - what an owner does when
    /// a request was armed after the channel and the first datum never
    /// came. Nothing is in flight afterwards unless it succeeds.
    static bool kick(const Elem* buffer, uint16_t length) {
        Channel::enable(false);
        busy_ = false;
        in_flight_ = 0;
        return start(buffer, length);
    }

    static uint32_t faults() { return faults_; }
    static void clear_faults() { faults_ = 0; }

    static void stop() {
        Channel::stop();
        busy_ = false;
        in_flight_ = 0;
    }

private:
    static DmaTransfer run(Elem* buffer, uint16_t length, bool increment) {
        return DmaTransfer{
            .peripheral = data_,
            .memory = buffer,
            .count = length,
            .config = {.direction = DmaDirection::memory_to_peripheral,
                       .circular = false,
                       .memory_to_memory = false,
                       .peripheral_increment = false,
                       .memory_increment = increment,
                       .peripheral_width = width,
                       .memory_width = width,
                       .priority = priority_},
        };
    }

    static void claim() {
        Channel::stop();
        Channel::arm(DmaFlag::complete | DmaFlag::error, true);
    }

    static inline volatile void* data_ = nullptr;
    static inline DmaPriority priority_ = DmaPriority::low;
    static inline uint16_t in_flight_ = 0;
    static inline uint32_t faults_ = 0;
    static inline bool busy_ = false;
};

/**
 * A receive engine: one caller-owned run filled from one peripheral
 * register, and asked how much has arrived.
 */
template <uint8_t ch, typename Elem = uint8_t>
class DmaRxEngine {
    static_assert(ch >= 1 && ch <= dma_channel_count, "no such DMA channel for this engine");
    static_assert(sizeof(Elem) == 1 || sizeof(Elem) == 2 || sizeof(Elem) == 4,
                  "a DMA element is one bus access wide: 1, 2 or 4 bytes");
    using Channel = DmaChannel<ch>;

public:
    DmaRxEngine() = delete;

    static constexpr bool present = true;
    static constexpr uint8_t channel = ch;
    static constexpr DmaWidth width = dma_width_of<Elem>();
    using element = Elem;

    static constexpr uint8_t flag_complete = DmaFlag::complete;
    static constexpr uint8_t flag_half = DmaFlag::half;
    static constexpr uint8_t flag_error = DmaFlag::error;

    [[gnu::always_inline]] static uint8_t service() {
        return static_cast<uint8_t>(Channel::isr());
    }

    static void arm(volatile void* data, DmaPriority priority = DmaPriority::low) {
        data_ = data;
        priority_ = priority;
        claim();
        Pfic::enable(Channel::irq());
    }

    static bool idle() { return !Channel::enabled(); }

    static bool start(Elem* buffer, uint16_t length) {
        if (buffer == nullptr || length == 0u) {
            return false;
        }
        Channel::enable(false);
        capacity_ = length;
        taken_ = 0;
        return Channel::load(run(buffer, length, true));
    }

    /// `length` elements into ONE cell, thrown away (the receive side of
    /// a bus write).
    static bool start_discard(Elem* cell, uint16_t length) {
        if (cell == nullptr || length == 0u) {
            return false;
        }
        Channel::enable(false);
        capacity_ = length;
        taken_ = 0;
        return Channel::load(run(cell, length, false));
    }

    /// How many elements have arrived since the last take(): one CNTR
    /// read, nothing suspended, nothing refused.
    static uint16_t take() {
        if (capacity_ == 0u) {
            return 0;
        }
        const uint16_t left = Channel::remaining();
        const uint16_t filled =
            left > capacity_ ? capacity_ : static_cast<uint16_t>(capacity_ - left);
        if (filled <= taken_) {
            return 0;
        }
        const uint16_t fresh = static_cast<uint16_t>(filled - taken_);
        taken_ = filled;
        return fresh;
    }

    /// What the run has collected so far WITHOUT taking it: the same
    /// arithmetic, the bookkeeping left alone.
    static uint16_t harvest() {
        if (capacity_ == 0u) {
            return 0;
        }
        const uint16_t left = Channel::remaining();
        return left > capacity_ ? capacity_ : static_cast<uint16_t>(capacity_ - left);
    }

    static bool full() { return capacity_ != 0u && taken_ >= capacity_; }
    static uint16_t capacity() { return capacity_; }
    static uint16_t taken() { return taken_; }

    /// Throw the standing run away: what an owner does to a receive
    /// that outlived its sender, so the next start() begins clean.
    static bool abandon() {
        ++faults_;
        capacity_ = 0;
        taken_ = 0;
        claim();
        return true;
    }

    /// The same run started again from its beginning.
    static bool kick(Elem* buffer, uint16_t length) {
        Channel::enable(false);
        capacity_ = 0;
        taken_ = 0;
        return start(buffer, length);
    }

    static uint32_t faults() { return faults_; }
    static void clear_faults() { faults_ = 0; }

    static void stop() {
        Channel::stop();
        capacity_ = 0;
        taken_ = 0;
    }

private:
    static DmaTransfer run(Elem* buffer, uint16_t length, bool increment) {
        return DmaTransfer{
            .peripheral = data_,
            .memory = buffer,
            .count = length,
            .config = {.direction = DmaDirection::peripheral_to_memory,
                       .circular = false,
                       .memory_to_memory = false,
                       .peripheral_increment = false,
                       .memory_increment = increment,
                       .peripheral_width = width,
                       .memory_width = width,
                       .priority = priority_},
        };
    }

    static void claim() {
        Channel::stop();
        Channel::arm(DmaFlag::complete | DmaFlag::error, true);
    }

    static inline volatile void* data_ = nullptr;
    static inline DmaPriority priority_ = DmaPriority::low;
    static inline uint16_t capacity_ = 0;
    static inline uint16_t taken_ = 0;
    static inline uint32_t faults_ = 0;
};

// ---- the block engines ----------------------------------------------------------

/**
 * DmaLoopEngine<ch, Elem> - "play one caller-owned table into a
 * peripheral, for ever": util/block_stream.hpp's BlockPlayer.
 *
 * THIS ONE RIDES THE CONTROLLER'S CIRCULAR MODE (CFGR.CIRC, 11.2.1):
 * CNTR reloads itself at the end of every lap and the channel never
 * stops, so there is no re-arm window and the completion interrupt does
 * nothing but count. A player wants exactly that - the peripheral's own
 * request paces it, the table does not change, and `laps()` moving is
 * the one fact that says the stream is alive.
 *
 * The table is the caller's and must outlive the stream. Nothing is
 * published per lap: an owner that wants a lap as an event arms its own
 * TimeEvent (design/block-stream.md).
 */
template <uint8_t ch, typename Elem = uint16_t>
class DmaLoopEngine {
    static_assert(ch >= 1 && ch <= dma_channel_count, "no such DMA channel for this engine");
    static_assert(sizeof(Elem) == 1 || sizeof(Elem) == 2 || sizeof(Elem) == 4,
                  "a DMA element is one bus access wide: 1, 2 or 4 bytes");
    using Channel = DmaChannel<ch>;

public:
    DmaLoopEngine() = delete;

    static constexpr bool present = true;
    static constexpr uint8_t channel = ch;
    static constexpr DmaWidth width = dma_width_of<Elem>();
    using element = Elem;

    static constexpr uint8_t flag_complete = DmaFlag::complete;
    static constexpr uint8_t flag_half = DmaFlag::half;
    static constexpr uint8_t flag_error = DmaFlag::error;

    /// The channel's ISR body folded into the engine: the armed flags
    /// that are up, cleared, handed back. Acts on nothing.
    [[gnu::always_inline]] static uint8_t service() {
        return static_cast<uint8_t>(Channel::isr());
    }

    /// Claim the channel for this peripheral; `data` is the register the
    /// table is poured into. The PFIC line is enabled here.
    static void arm(volatile void* data, DmaPriority priority = DmaPriority::low) {
        data_ = data;
        priority_ = priority;
        claim();
        Pfic::enable(Channel::irq());
    }

    /// Begin playing `length` elements of `table`, over and over. The
    /// buffer is the caller's and must outlive the stream.
    static bool start(const Elem* table, uint16_t length) {
        if (table == nullptr || length == 0u) {
            return false;
        }
        table_ = table;
        length_ = length;
        laps_ = 0;
        faults_ = 0;
        running_ = Channel::load(DmaTransfer{
            .peripheral = data_,
            .memory = const_cast<Elem*>(table),
            .count = length,
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

    /// A lap ended - called from the handler on the completion flag.
    /// The controller has already reloaded: there is nothing to do but
    /// count, which is the whole gain of the circular mode.
    static void lap() {
        if (running_) {
            laps_ = laps_ + 1u;
        }
    }
    /// A transfer error - called from the handler on the error flag.
    /// 11.3.3: the silicon drops EN by itself there.
    static void fail() {
        faults_ = faults_ + 1u;
        running_ = false;
    }

    static uint32_t laps() { return laps_; }
    static uint32_t faults() { return faults_; }
    static void clear_faults() { faults_ = 0; }
    static bool running() { return running_; }
    static uint16_t length() { return length_; }

    /// How far into the current lap the controller has got - one
    /// register read, never refused.
    static DmaProgress progress() { return Channel::progress(length_); }

    /// Start the same table again from its beginning - what an owner
    /// does when the request was armed after the channel and the first
    /// datum never came.
    static bool kick() {
        Channel::enable(false);
        running_ = false;
        return start(table_, length_);
    }

    static void stop() {
        Channel::stop();
        running_ = false;
        length_ = 0;
    }

private:
    static void claim() {
        Channel::stop();
        Channel::arm(DmaFlag::complete | DmaFlag::error, true);
    }

    static inline volatile void* data_ = nullptr;
    static inline const Elem* table_ = nullptr;
    static inline DmaPriority priority_ = DmaPriority::low;
    static inline uint16_t length_ = 0;
    // Handler-written, loop-read.
    static inline volatile uint32_t laps_ = 0;
    static inline volatile uint32_t faults_ = 0;
    static inline volatile bool running_ = false;
};

/**
 * DmaPingPongEngine<ch, Elem> - "fill one caller-owned buffer while the
 * caller drains the other": util/block_stream.hpp's BlockSource.
 *
 * WHY IT IS *NOT* CIRCULAR, on a controller that has a circular mode
 * and a half-transfer flag that would seem to make two halves free. The
 * contract's rule is SKIP RATHER THAN TEAR: a block handed over is
 * valid until the caller releases it, and a source that cannot keep
 * that promise must drop a lap instead of writing into a buffer
 * somebody is reading. A circular channel never stops, so the decision
 * could only be taken AFTER the edge - by which time the controller is
 * already filling the half the caller holds, with nothing between the
 * flag and the first store but the handler's own latency. That was
 * measured on the STM32G0 (0 to 6 elements had landed before a handler
 * whose whole body was disable-and-read could act) and it is measured
 * again on this silicon in test_vx03_adc's letter d.
 *
 * So each block is a PLAIN transfer: the completion handler hands the
 * full buffer over and launches the other one, and when the caller
 * holds both the engine counts an overrun, stalls, and is restarted by
 * `release()`. The gap between one block and the next is the re-arm,
 * which the accounting does not hide: a sample lost there is a sample
 * the peripheral never delivered, and `overruns()` counts the laps the
 * engine chose not to take.
 *
 * Both buffers are the caller's, both hold `length` elements, and both
 * must outlive the stream. They are `volatile` because the controller
 * writes them and the compiler sees nothing.
 */
template <uint8_t ch, typename Elem = uint16_t>
class DmaPingPongEngine {
    static_assert(ch >= 1 && ch <= dma_channel_count, "no such DMA channel for this engine");
    static_assert(sizeof(Elem) == 1 || sizeof(Elem) == 2 || sizeof(Elem) == 4,
                  "a DMA element is one bus access wide: 1, 2 or 4 bytes");
    using Channel = DmaChannel<ch>;

public:
    DmaPingPongEngine() = delete;

    static constexpr bool present = true;
    static constexpr uint8_t channel = ch;
    static constexpr DmaWidth width = dma_width_of<Elem>();
    using element = Elem;

    static constexpr uint8_t flag_complete = DmaFlag::complete;
    static constexpr uint8_t flag_half = DmaFlag::half;
    static constexpr uint8_t flag_error = DmaFlag::error;

    [[gnu::always_inline]] static uint8_t service() {
        return static_cast<uint8_t>(Channel::isr());
    }

    /// Claim the channel; `data` is the peripheral's data register.
    static void arm(volatile void* data, DmaPriority priority = DmaPriority::low) {
        data_ = data;
        priority_ = priority;
        claim();
        Pfic::enable(Channel::irq());
    }

    /// Begin streaming into `first`, with `second` as the buffer the
    /// next block will use.
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
     * completion flag. Hands the buffer to the caller and starts the
     * next block in the other one, or counts an overrun and stalls.
     *
     * Returns the elements the finished block carried, or zero when
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
    static volatile Elem* ready() { return pending_ != 0u ? buffer_[drain_] : nullptr; }
    /// How many elements it holds - always the whole block, because a
    /// buffer is handed over only when it is full.
    static uint16_t ready_length() { return pending_ != 0u ? length_ : uint16_t{0}; }

    /**
     * Hand the ready buffer back. Restarts a stalled stream, which is
     * the one place this verb does more than bookkeeping - and why it
     * holds the interrupt guard: complete() runs in the DMA handler and
     * touches the same three counters.
     */
    static bool release() {
        InterruptGuard guard;
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
    /// Times the engine found both buffers held by the caller.
    static uint32_t overruns() { return overruns_; }
    static bool stalled() { return stalled_; }
    static bool running() { return running_; }
    static uint16_t length() { return length_; }
    /// Buffers filled and not yet released: 0, 1, or 2 (2 = stalled).
    static uint8_t pending() { return pending_; }

    /// How far into the CURRENT block the controller has got.
    static DmaProgress progress() { return Channel::progress(length_); }

    /**
     * Throw away a block the silicon has stopped running and start a
     * fresh one in the same buffer. The partly filled buffer is NOT
     * handed over: a torn block is what this engine exists not to
     * produce, so the whole thing is discarded and counted.
     *
     * A STALLED STREAM IS NOT A DEAD ONE and this refuses it without
     * counting anything: while the engine waits for a release there is
     * no block in flight. release() is the verb for that state.
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
    /// EN STAYS SET WHEN A BLOCK COMPLETES on this controller (the file
    /// header: only software clears it), and every configuring verb
    /// refuses an enabled channel - so the re-arm drops it first. On the
    /// first launch there is nothing to drop.
    static bool launch() {
        if (buffer_[fill_] == nullptr || length_ == 0u) {
            return false;
        }
        Channel::enable(false);
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
        Channel::arm(DmaFlag::complete | DmaFlag::error, true);
    }

    static inline volatile void* data_ = nullptr;
    static inline volatile Elem* buffer_[2] = {nullptr, nullptr};
    static inline DmaPriority priority_ = DmaPriority::low;
    static inline uint16_t length_ = 0;
    // Handler-written and loop-read: volatile for the ticker's reason.
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

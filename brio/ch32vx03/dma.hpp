/*
 * dma.hpp
 *
 * The DMA controllers of the CH32V203 and the CH32V303 (RM ch. 11): the
 * STM32F1's DMA under WCH's names - channels each wired to a fixed handful
 * of peripheral requests, an arbiter over four software priorities with
 * the channel index deciding ties, and four flag bits a channel. How many
 * of them a part has is its DEVICE CLASS's: ONE controller of EIGHT
 * channels on every CH32V203 (one more than the F1 ancestor and than the
 * CH32V00x's seven), and on the CH32V303 (CH32V30x_D8) TWO - DMA1 with
 * SEVEN channels and DMA2 with eleven (device::dma_controller_count,
 * dma1_channel_count, dma2_channel_count). Every type here names its
 * CONTROLLER FIRST - `Dma<1|2>`, `DmaChannel<controller, n>`, the engines
 * `<controller, n, Elem>` - because a channel number alone names two
 * channels on the CH32V303.
 *
 * THE CHANNEL IS THE REQUEST. There is no request multiplexer: the tables
 * of 11.2.3 wire each peripheral event to ONE channel of ONE controller,
 * and an engine is named by that slot with the table in hand. The table
 * itself is ch32vx03/dma_engine.hpp's, because a transport must be able
 * to refuse an engine on the wrong slot without including this file;
 * `DmaRequestOf<r>` is how a program names the request instead of the
 * numbers. And the rows of one channel are an OR: every peripheral wired
 * to it whose DMA bit is set drives whatever transfer the channel holds -
 * measured, a receiver left running served a timer's transfer sixteen
 * items at once - so stopping a channel withdraws no request, and a
 * peripheral that is done with a channel is turned off.
 *
 * WHAT A CHANNEL IS. A configuration word (CFGR: direction, circular,
 * memory-to-memory, the two increments, the two widths, the priority,
 * the three interrupt enables and EN), a count the controller
 * decrements as it moves items (CNTR, read-only while enabled), and two
 * addresses named after the registers' F1 ancestry - PADDR and MADDR -
 * where DIR says which side is the source: in memory-to-memory mode
 * both are memory and the names mean nothing.
 *
 * DMA2'S LAST FOUR CHANNELS SIT ELSEWHERE. Channels 1..7 of either
 * controller are twenty bytes apart from offset 0x08 of the block (table
 * 11-7's stride, a reserved word after each channel), and so is DMA1's
 * eighth; DMA2's channels 8..11 follow its seventh at SIXTEEN bytes apart
 * from offset 0x90 (table 11-8, 11.3.7..11.3.10), and their four flags a
 * channel live in a register pair of their own, DMA2_EXTEM_INTFR and
 * DMA2_EXTEM_INTFCR at 0xD0 (11.3.11, 11.3.12). A channel knows which
 * pair holds its flags; nothing above it has to.
 *
 * THE ADDRESSES ARE ALIGNED BY THE SILICON, SILENTLY. 11.3.5 and 11.3.6
 * say the module IGNORES the low address bits of a 16- or 32-bit
 * access, so a half-word transfer pointed at an odd address moves the
 * WRONG BYTES and reports success. Every verb here refuses an address
 * that is not aligned to its own width rather than let the hardware
 * round it: a transfer that quietly reads elsewhere is worse than one
 * that does not start.
 *
 * AND ON THE CH32V303 DMA1 IS REFUSED A SPAN THAT CROSSES 64 KB. 11.2.3's
 * note (1) gives the CH32V30x_D8 a DMA1 whose accesses must not cross a
 * 64-kilobyte boundary on lots whose penultimate sixth digit is zero
 * ("the DMA source address + the number of transmission data ... can only
 * be in the 0-64K, or 64K-128K area"), and a program cannot read its lot
 * number - so every transfer DMA1 is handed on that class is checked, end
 * by end, and one that would cross is refused. The CH32V303VCT6 of the
 * reference suite is such a lot, measured: with the refusal bypassed, the
 * upper half of a span across 0x0801 0000 was read from the bottom of the
 * page it started in - a wrap, reported as a completed block. DMA2 has "no
 * restriction" (note (3)), and neither has a CH32V203, whose class the
 * note does not name. The piecewise verbs (configure, set_count,
 * set_peripheral, set_memory) never see a transfer whole and do not ask
 * the rule; the verbs that take a DmaTransfer do.
 *
 * THE VECTORS ARE ONE PER CHANNEL: DMA1's seven consecutive from the
 * table's entry 27 and its eighth, where there is one, on the tail of the
 * table this DEVICE CLASS has (62, 67 on the CH32V203RB); DMA2's first
 * five at 72..76 and its other six at 98..103 (the crt states the
 * per-class truth, and device.hpp's Irq enum carries it). So a channel's
 * ISR body reads only its own flags and a handler needs no "which
 * channel" question. DMA1EN and DMA2EN in RCC_HBPCENR are the blocks'
 * gates and both are CLOSED at reset, so `Dma<c>::open()` is the first
 * thing every channel verb does.
 *
 * NO DMA-FED TRANSPORT SLEEPS ON THIS FAMILY. In the Sleep of RM 2.4 no
 * bus master but the core gets a cycle: a memory-to-memory block
 * started right before an idle() moves the handful of items already in
 * the pipeline and then nothing until the core wakes, while the timers
 * and the core's counter count the whole sleep (measured on both parts -
 * docs/ch32vx03/dma.md, and it is the same starvation that kills the
 * USB controller in Sleep). SO A CHANNEL COUNTS ITSELF, on either
 * controller: setting EN takes one count of a bus master and clearing it
 * gives the count back (ch32vx03/bus_activity.hpp), the controller's
 * LEDGER (`Dma<c>::count`) remembering which channel holds one, the
 * kernel's idle path does not sleep while the count stands, and a sleep
 * site refuses to arm over it - a program with an engine running holds
 * itself awake without having to know that it does. `Dma<c>::any_enabled()`
 * is the same question asked of the registers instead, of EVERY controller
 * the part has, for a caller that wants the silicon's own answer.
 *
 * THE ENGINES. `DmaTxEngine<c, ch, Elem>` and `DmaRxEngine<c, ch, Elem>`
 * pour caller-owned runs into one peripheral register and fill them from
 * one: the transmit engine reports how many items a block carried when it
 * completes, the receive engine answers how many have arrived since it was
 * last asked (one CNTR read - nothing suspended, nothing refused) or, in
 * its CIRCULAR shape, fills a ring for ever on the controller's cycle mode,
 * its count being the ring's producer index (util/ring.hpp's
 * HardwareRing); the beat of a block is its run's element, `Elem` the
 * widest the binding takes. `DmaCopyEngine<c, ch>` is memory to memory, a
 * copy or a fill.
 * ch32vx03/usart.hpp's, spi.hpp's and i2c.hpp's engine slots are built on
 * the first two, and a driver reaches its engines only through their
 * published names (start, claim, launch, service, complete, flag_complete,
 * flag_error, controller, channel) so that a driver with an empty slot
 * never includes this file. TWO ENGINES OF ONE TRANSPORT NAME TWO SLOTS: a
 * channel moves data one way, and the table gives each direction its own.
 *
 * EVERY ENGINE STANDS ON `DmaBinding<c, ch>`, which is where this
 * controller's two moments are kept apart: what is constant for a binding
 * written once when the engine is armed, and per block the four stores the
 * vendor's own restart is made of. Its header says which, and why.
 *
 * ONE THING THE SILICON DOES that the engines are built on: EN STAYS
 * SET when a non-circular block completes (CNTR at zero, TCIF up, the
 * channel enabled and idle - the F1 lineage: only software clears EN),
 * so every verb that programs a channel refuses while EN is up, an engine
 * stops its channel at the block's end, and a channel that may still be
 * standing is stopped before it is programmed again. A TRANSFER ERROR is
 * the one thing that clears EN by itself (11.3.3).
 *
 * THE BLOCK ENGINES beside them are util/block_stream.hpp's two
 * concepts: `DmaLoopEngine<c, ch, Elem>` is a BlockPlayer, one
 * caller-owned table poured into a peripheral for ever on THE
 * CONTROLLER'S OWN CIRCULAR MODE, with the lap interrupt doing nothing
 * but count; `DmaPingPongEngine<c, ch, Elem>` is a BlockSource, two
 * caller-owned buffers filled in turn - and it does NOT use circular
 * mode, which is the one place this file departs from what the
 * controller offers. The reason is the contract's and not the API's: a
 * circular channel never stops, so "skip rather than tear" could only
 * be decided after the edge, with the controller already writing the
 * buffer the caller holds (measured on the STM32G0, and again here -
 * docs/ch32vx03/adc.md). So the source stops itself at every block and
 * the handler re-arms the other buffer, the STM32G0's shape; the four
 * engine names are the same words in both strata
 * (docs/design/block-stream.md).
 */

#pragma once

#include <stddef.h>
#include <stdint.h>

#include <span>
#include <type_traits>

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
/// them: table 11-7's stride is twenty bytes, not sixteen. DMA2's channels
/// 8..11 are packed at sixteen (table 11-8), so for those the reserved
/// word is the next channel's CFGR - it is declared and never touched.
struct DmaChannelRegs {
    volatile uint32_t CFGR;    ///< 0x00
    volatile uint32_t CNTR;    ///< 0x04
    volatile uint32_t PADDR;   ///< 0x08
    volatile uint32_t MADDR;   ///< 0x0c
    uint32_t RESERVED0;        ///< 0x10
};

/// A controller's block: the flag pair and the channels at twenty bytes.
/// DMA1 fills all eight slots on a CH32V203 and seven on a CH32V303; DMA2
/// uses the first seven and keeps its last four elsewhere.
struct DmaRegs {
    volatile uint32_t INTFR;         ///< 0x00 the flags, read-only
    volatile uint32_t INTFCR;        ///< 0x04 write 1 to clear
    DmaChannelRegs channel[8];       ///< 0x08 + 20 x (ch - 1)
};

/// DMA2's extended flag pair, channels 8..11 at 4 x (ch - 8) (11.3.11,
/// 11.3.12): the CH32V30x_D8's alone.
struct Dma2ExtendRegs {
    volatile uint32_t INTFR;         ///< 0x4d0 DMA2_EXTEM_INTFR, read-only
    volatile uint32_t INTFCR;        ///< 0x4d4 DMA2_EXTEM_INTFCR, write 1 to clear
};

/// The two blocks: DMA1 at the head of the HB bus, DMA2 a kilobyte above
/// (device.hpp's dma2_base).
constexpr uint32_t dma_base_for(uint8_t c) { return c == 2u ? dma2_base : hb_base + 0x0000; }
inline DmaRegs* dma_regs(uint8_t c) { return reinterpret_cast<DmaRegs*>(dma_base_for(c)); }
inline Dma2ExtendRegs* dma2_extend() {
    return reinterpret_cast<Dma2ExtendRegs*>(dma2_base + 0xD0);
}

/// Where one channel's registers start: 0x08 + 20 x (ch - 1) from its
/// block, but DMA2's 8..11 at 0x90 + 16 x (ch - 8).
constexpr uint32_t dma_channel_address(uint8_t c, uint8_t ch) {
    return (c == 2u && ch >= 8u) ? dma2_base + 0x90u + 16u * (ch - 8u)
                                 : dma_base_for(c) + 0x08u + 20u * (ch - 1u);
}

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

/// How many channels controller `c` has on THIS part: DMA1 eight on every
/// CH32V203, whose two classes the register notes of 11.3.1 to 11.3.6 name
/// among the five that have channel 8, and seven on the CH32V303, whose
/// class they do not; DMA2 eleven on the CH32V303 and none anywhere else
/// (11.3.7..11.3.12's notes name the CH32V30x_D8 and no CH32V20x).
constexpr uint8_t dma_channels_of(uint8_t c) {
    return c == 1u   ? device::dma1_channel_count
           : c == 2u ? (device::dma_controller_count >= 2u ? device::dma2_channel_count : 0u)
                     : 0u;
}

/// Whether channel `ch` of controller `c` exists on this part.
constexpr bool dma_channel_exists(uint8_t c, uint8_t ch) {
    return ch >= 1u && ch <= dma_channels_of(c);
}

/// The line a channel reports on. DMA1's first seven are consecutive from
/// the table's entry 27 and its eighth, where there is one, sits on the
/// class's own tail; DMA2's are 72..76 and 98..103 - all of which
/// device.hpp's Irq table places.
constexpr Irq dma_channel_irq(uint8_t c, uint8_t ch) {
    if (c == 2u) {
        return ch <= 5u ? static_cast<Irq>(static_cast<uint8_t>(Irq::dma2_channel1) + (ch - 1u))
                        : static_cast<Irq>(static_cast<uint8_t>(Irq::dma2_channel6) + (ch - 6u));
    }
    return ch == 8u ? Irq::dma1_channel8
                    : static_cast<Irq>(static_cast<uint8_t>(Irq::dma1_channel1) + (ch - 1u));
}

/// 11.2.3's note (1): DMA1 of the CH32V30x_D8 must not cross a 64 KB
/// boundary on lots whose penultimate sixth digit is zero - which a
/// program cannot read, so the rule is kept on every lot of the class.
inline constexpr bool dma1_bounded_to_64k = device::device_class == DeviceClass::v30x_d8;

/// Whether one end of a transfer crosses a 64-kilobyte boundary: its first
/// and last access in different 64 KB pages. An end that does not
/// increment touches one address and crosses nothing.
constexpr bool dma_span_crosses_64k(uint32_t address, uint16_t count, DmaWidth w,
                                    bool increment) {
    if (!increment || count == 0u) {
        return false;
    }
    const uint32_t last = address + static_cast<uint32_t>(count - 1u) * dma_width_bytes(w);
    return (address >> 16) != (last >> 16);
}

/// The transfer as a whole, both ends.
inline bool dma_transfer_crosses_64k(const DmaTransfer& t) {
    const uint32_t p = static_cast<uint32_t>(reinterpret_cast<uintptr_t>(t.peripheral));
    const uint32_t m = static_cast<uint32_t>(reinterpret_cast<uintptr_t>(t.memory));
    return dma_span_crosses_64k(p, t.count, t.config.peripheral_width,
                                t.config.peripheral_increment) ||
           dma_span_crosses_64k(m, t.count, t.config.memory_width, t.config.memory_increment);
}

/// Which channels of controller `c` have EN set, bit ch - 1 for channel
/// ch - zero for a controller this part has not got, or whose gate is
/// closed. A plain function over the addresses, so that a question about
/// both controllers never names a controller type this part refuses.
inline uint16_t dma_enabled_channels(uint8_t c) {
    const uint8_t count = dma_channels_of(c);
    if (count == 0u || !Rcc::enabled(Bus::hb, c == 1u ? rcc_hb_dma1 : rcc_hb_dma2)) {
        return 0;
    }
    uint16_t mask = 0;
    for (uint8_t ch = 1; ch <= count; ++ch) {
        const DmaChannelRegs& r =
            *reinterpret_cast<const DmaChannelRegs*>(dma_channel_address(c, ch));
        if ((r.CFGR & dma_cfgr_en) != 0u) {
            mask = static_cast<uint16_t>(mask | (1u << (ch - 1u)));
        }
    }
    return mask;
}

/**
 * One controller: its gate, its flag registers, and the one question the
 * power model asks.
 */
template <uint8_t c>
struct Dma {
    static_assert(c == 1u || c == 2u,
                  "brio DMA: this family's controllers are DMA1 and DMA2 (RM 11.2.3)");
    static_assert(c <= device::dma_controller_count,
                  "brio DMA: this part has ONE DMA controller - DMA2 and its eleven channels are "
                  "the CH32V303's (device::dma_controller_count, RM 11.2.3)");

    Dma() = delete;

    static constexpr uint8_t controller = c;
    static constexpr uint8_t channel_count = dma_channels_of(c);
    static constexpr uint32_t gate = c == 1u ? rcc_hb_dma1 : rcc_hb_dma2;
    /// Whether this controller has the extended flag pair (DMA2 alone).
    static constexpr bool has_extended_flags = (c == 2u);

    static DmaRegs& regs() { return *dma_regs(c); }

    static void open() { Rcc::enable(Bus::hb, gate); }
    static bool opened() { return Rcc::enabled(Bus::hb, gate); }

    /**
     * Every channel of this controller back to the chapter's own reset
     * values, the gate left open.
     *
     * THESE BLOCKS HAVE NO RESET LINE: RCC_AHBRSTR (3.4.11) reserves bits
     * [11:0] and names only the Ethernet MAC, the DVP and the USB OTG
     * core, so the pulse every PB1 and PB2 peripheral of this stratum
     * is put back with does not exist here and software does the work.
     */
    static void stop_all() {
        open();
        for (uint8_t ch = 1; ch <= channel_count; ++ch) {
            DmaChannelRegs& r = *reinterpret_cast<DmaChannelRegs*>(dma_channel_address(c, ch));
            r.CFGR = 0;
            uncount(ch);
            r.CNTR = 0;
            r.PADDR = 0;
            r.MADDR = 0;
        }
        regs().INTFCR = 0xFFFFFFFFUL;
        if constexpr (has_extended_flags) {
            dma2_extend()->INTFCR = 0xFFFFUL;
        }
    }

    /**
     * THE BUS-MASTER LEDGER, one byte a channel: whether channel `ch`
     * holds one count in ch32vx03/bus_activity.hpp. A channel takes its
     * count when a verb sets EN and gives it back when a verb clears it,
     * and the ledger - not the register - is what says which: the one
     * thing that clears EN behind software's back is a TRANSFER ERROR
     * (11.3.3), and a count read off the register there would never be
     * given back. Each channel's byte is touched by that channel's own
     * users alone, so no two channels share a read-modify-write.
     */
    [[gnu::always_inline]] static void count(uint8_t ch) {
        if (counted_[ch - 1u] == 0u) {
            counted_[ch - 1u] = 1;
            BusActivity::entered();
        }
    }
    [[gnu::always_inline]] static void uncount(uint8_t ch) {
        if (counted_[ch - 1u] != 0u) {
            counted_[ch - 1u] = 0;
            BusActivity::left();
        }
    }
    static bool counted(uint8_t ch) { return counted_[ch - 1u] != 0u; }

    /// INTFR whole: channels 1..8 of DMA1, 1..7 of DMA2.
    static uint32_t flags() { return regs().INTFR; }
    static void clear(uint32_t mask) { regs().INTFCR = mask; }

    /// DMA2_EXTEM_INTFR whole: channels 8..11 at 4 x (ch - 8).
    static uint32_t extended_flags() {
        static_assert(has_extended_flags,
                      "brio DMA: the extended flag pair is DMA2's (RM 11.3.11) - DMA1's channels "
                      "all report in INTFR");
        return dma2_extend()->INTFR;
    }
    static void clear_extended(uint32_t mask) {
        static_assert(has_extended_flags,
                      "brio DMA: the extended flag pair is DMA2's (RM 11.3.12)");
        dma2_extend()->INTFCR = mask;
    }

    /// Which of THIS controller's channels have EN set, bit ch - 1 for
    /// channel ch. Zero with the gate closed.
    static uint16_t enabled_channels() { return dma_enabled_channels(c); }

    /**
     * Is ANY channel enabled, on ANY controller this part has? The question
     * the sleep site of this family asks before it arms a mode: in Sleep the
     * bus matrix serves the core alone, so a channel that is up is a
     * transfer that will stall for the whole sleep. It is asked of every
     * controller whichever one it is spelled on, because the count
     * ch32vx03/bus_activity.hpp keeps is fed by the channels of both and the
     * silicon's answer has to mean the same thing. A closed gate answers
     * false: a block that cannot have a channel running.
     */
    static bool any_enabled() {
        return dma_enabled_channels(1) != 0u ||
               (device::dma_controller_count >= 2u && dma_enabled_channels(2) != 0u);
    }

private:
    static inline uint8_t counted_[channel_count] = {};
};

/**
 * One channel of one controller: n = 1..8 on DMA1 of a CH32V203, 1..7 on
 * DMA1 of a CH32V303, 1..11 on its DMA2. Every configuring verb refuses
 * while the channel is enabled, because CFGR's fields, CNTR and the two
 * addresses are read-only then (11.2.1's note and each register's own): a
 * store the silicon ignores would be a lie the code told itself.
 */
template <uint8_t c, uint8_t ch>
class DmaChannel {
    static_assert(c == 1u || c == 2u,
                  "brio DMA: this family's controllers are DMA1 and DMA2 (RM 11.2.3)");
    static_assert(c <= device::dma_controller_count,
                  "brio DMA: this part has ONE DMA controller - DMA2 and its eleven channels are "
                  "the CH32V303's (device::dma_controller_count, RM 11.2.3)");
    static_assert(dma_channel_exists(c, ch),
                  "brio DMA: no such channel - DMA1 has channels 1..8 on the CH32V203 and 1..7 on "
                  "the CH32V303, DMA2 has 1..11 (device::dma1_channel_count, dma2_channel_count)");

public:
    DmaChannel() = delete;

    using Controller = Dma<c>;

    static constexpr uint8_t controller = c;
    static constexpr uint8_t number = ch;
    static constexpr DmaSlot slot{c, ch};
    /// DMA2's channels 8..11 report in the extended flag pair.
    static constexpr bool extended = (c == 2u && ch >= 8u);
    static constexpr uint32_t flag_shift = extended ? 4u * (ch - 8u) : 4u * (ch - 1u);
    /// Whether this channel is held to 11.2.3's 64 KB rule.
    static constexpr bool bounded_to_64k = (c == 1u) && dma1_bounded_to_64k;

    static DmaChannelRegs& regs() {
        if constexpr (extended) {
            return *reinterpret_cast<DmaChannelRegs*>(dma_channel_address(c, ch));
        } else {
            return Controller::regs().channel[ch - 1u];
        }
    }
    static constexpr Irq irq() { return dma_channel_irq(c, ch); }

    static bool enabled() { return (regs().CFGR & dma_cfgr_en) != 0u; }

    /// Enable or disable. Disabling a running channel abandons the block
    /// where it stands; CNTR then holds what was left.
    ///
    /// AND THIS IS WHERE THE FAMILY'S BUS ACTIVITY IS COUNTED. EN is the
    /// bit that makes this channel a bus master, and a master that is up
    /// forbids a sleep of any depth here (ch32vx03/bus_activity.hpp), so
    /// every verb that sets or clears EN keeps the controller's ledger
    /// (`Dma<c>::count`): the TRANSITION is counted, never the store,
    /// which would count an idempotent enable twice.
    static void enable(bool on) {
        Controller::open();
        if (on) {
            regs().CFGR |= dma_cfgr_en;
            Controller::count(ch);
        } else {
            regs().CFGR &= ~dma_cfgr_en;
            Controller::uncount(ch);
        }
    }

    /// Everything but the addresses and the count. Refused while
    /// enabled, and for the configuration the chapter forbids.
    static bool configure(const DmaChannelConfig& cfg) {
        Controller::open();
        if (enabled() || !dma_channel_config_valid(cfg)) {
            return false;
        }
        uint32_t v = regs().CFGR & (dma_cfgr_tcie | dma_cfgr_htie | dma_cfgr_teie);
        if (cfg.direction == DmaDirection::memory_to_peripheral) { v |= dma_cfgr_dir; }
        if (cfg.circular) { v |= dma_cfgr_circ; }
        if (cfg.memory_to_memory) { v |= dma_cfgr_mem2mem; }
        if (cfg.peripheral_increment) { v |= dma_cfgr_pinc; }
        if (cfg.memory_increment) { v |= dma_cfgr_minc; }
        v |= static_cast<uint32_t>(cfg.peripheral_width) << dma_cfgr_psize_shift;
        v |= static_cast<uint32_t>(cfg.memory_width) << dma_cfgr_msize_shift;
        v |= static_cast<uint32_t>(cfg.priority) << dma_cfgr_pl_shift;
        regs().CFGR = v;
        return true;
    }

    /// The configuration as the register holds it, which is what a test
    /// compares against what it asked for.
    static DmaChannelConfig configuration() {
        const uint32_t v = regs().CFGR;
        DmaChannelConfig cfg{};
        cfg.direction = (v & dma_cfgr_dir) != 0u ? DmaDirection::memory_to_peripheral
                                                 : DmaDirection::peripheral_to_memory;
        cfg.circular = (v & dma_cfgr_circ) != 0u;
        cfg.memory_to_memory = (v & dma_cfgr_mem2mem) != 0u;
        cfg.peripheral_increment = (v & dma_cfgr_pinc) != 0u;
        cfg.memory_increment = (v & dma_cfgr_minc) != 0u;
        cfg.peripheral_width = static_cast<DmaWidth>((v >> dma_cfgr_psize_shift) & 3u);
        cfg.memory_width = static_cast<DmaWidth>((v >> dma_cfgr_msize_shift) & 3u);
        cfg.priority = static_cast<DmaPriority>((v >> dma_cfgr_pl_shift) & 3u);
        return cfg;
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

    /// Whether this channel would take the transfer: its shape, its
    /// alignment, and - on the CH32V303's DMA1 - the 64 KB rule.
    static bool accepts(const DmaTransfer& t) {
        if (!dma_transfer_valid(t) || !dma_transfer_aligned(t)) {
            return false;
        }
        if constexpr (bounded_to_64k) {
            if (dma_transfer_crosses_64k(t)) {
                return false;
            }
        }
        return true;
    }

    /// Program the whole transfer WITHOUT starting it. The flags are
    /// cleared on the way in, so a poller cannot see the last block's.
    static bool prepare(const DmaTransfer& t) {
        Controller::open();
        if (enabled() || !dma_transfer_valid(t) || !dma_transfer_aligned(t)) {
            return false;
        }
        if constexpr (bounded_to_64k) {
            if (dma_transfer_crosses_64k(t)) {
                return false;
            }
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

    static uint32_t flags() {
        if constexpr (extended) {
            return (dma2_extend()->INTFR >> flag_shift) & DmaFlag::all;
        } else {
            return (Controller::regs().INTFR >> flag_shift) & DmaFlag::all;
        }
    }
    static bool flag(uint32_t mask) { return (flags() & mask) != 0u; }
    static void clear(uint32_t mask) {
        if constexpr (extended) {
            dma2_extend()->INTFCR = (mask & DmaFlag::all) << flag_shift;
        } else {
            Controller::regs().INTFCR = (mask & DmaFlag::all) << flag_shift;
        }
    }

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
        Controller::open();
        regs().CFGR = 0;
        Controller::uncount(ch);
        clear(DmaFlag::all);
    }
};

// ---- the binding: what every engine is built on --------------------------------

/// CFGR's two width fields set equal, PSIZE = MSIZE: every engine's
/// transfer (table 11-1's widening and truncation are the channel verbs').
constexpr uint32_t dma_cfgr_widths(DmaWidth w) {
    return (static_cast<uint32_t>(w) << dma_cfgr_psize_shift) |
           (static_cast<uint32_t>(w) << dma_cfgr_msize_shift);
}

/// CFGR's three interrupt enables for a mask in DmaFlag's terms.
constexpr uint32_t dma_cfgr_interrupts(uint32_t mask) {
    return ((mask & DmaFlag::complete) != 0u ? dma_cfgr_tcie : 0u) |
           ((mask & DmaFlag::half) != 0u ? dma_cfgr_htie : 0u) |
           ((mask & DmaFlag::error) != 0u ? dma_cfgr_teie : 0u);
}

constexpr uint32_t dma_cfgr_priority(DmaPriority p) {
    return static_cast<uint32_t>(p) << dma_cfgr_pl_shift;
}

inline uint32_t dma_address(const volatile void* p) {
    return static_cast<uint32_t>(reinterpret_cast<uintptr_t>(p));
}

/**
 * DmaBinding<c, ch> - one channel HELD BY AN ENGINE, and the stores every
 * block of every engine in this file is started and ended with.
 *
 * TWO MOMENTS. What is constant for a binding - the gate, the peripheral's
 * address, the direction, the increments, the priority, the interrupt
 * enables - is written ONCE, when the engine is armed: `bind()` opens the
 * gate, stops the channel, clears its flags and stores PADDR, and the
 * engine keeps the rest of CFGR as a word of its own, because the register
 * cannot keep it - EN lives in the same word, and every field of the word
 * is read-only while EN is set (11.2.1's note). What changes is written per
 * block, and on this controller that is FOUR STORES: MADDR, CNTR, the
 * channel's four flags in INTFCR, and CFGR written WHOLE with EN - never a
 * read-modify-write, the engine knowing the word. A block ends with one
 * store more, the same word without EN. The vendor's own restart is the
 * same four registers (the EVT's SPI_LCD, `SPI1_Read_DMA`), each of its
 * enables a read-modify-write.
 *
 * ONE STORE SETS THE FIELDS AND EN TOGETHER. 11.2.1 makes the fields
 * read-only while EN is SET; the store that sets it lands on a channel
 * whose EN is clear, so the widths, the increments and the enable are
 * taken in one access - measured: a word copy, a byte copy and a fill
 * back to back, each starting with a different width in that one store,
 * every byte where it belongs (docs/ch32vx03/dma.md).
 *
 * The gate is opened in `bind()` and nowhere on the block path: nothing in
 * this stratum closes it again, and a program that does - a clock suite's
 * gate walk - arms its engines again after it.
 *
 * Nothing here validates. The engines check what a run can get wrong
 * (null, empty, longer than CNTR, misaligned for its width, across 64 KB
 * on the CH32V303's DMA1) before they call `go()`.
 */
template <uint8_t c, uint8_t ch>
struct DmaBinding {
    using Channel = DmaChannel<c, ch>;   // its static_asserts check c and ch

    DmaBinding() = delete;

    static constexpr uint32_t flag_bits = DmaFlag::all << Channel::flag_shift;

    static volatile uint32_t& clear_register() {
        if constexpr (Channel::extended) {
            return dma2_extend()->INTFCR;
        } else {
            return dma_regs(c)->INTFCR;
        }
    }
    static uint32_t flags() {
        if constexpr (Channel::extended) {
            return (dma2_extend()->INTFR >> Channel::flag_shift) & DmaFlag::all;
        } else {
            return (dma_regs(c)->INTFR >> Channel::flag_shift) & DmaFlag::all;
        }
    }

    /// The binding: the gate opened, the channel stopped and its flags
    /// cleared, PADDR written - once, at an engine's arm().
    static void bind(volatile void* peripheral) {
        Dma<c>::open();
        halt(0);
        clear_register() = flag_bits;
        Channel::regs().PADDR = dma_address(peripheral);
    }

    /// A block on a channel whose EN is CLEAR: MADDR, CNTR, the flags,
    /// CFGR whole with EN. The ledger's count is taken after the enable.
    [[gnu::always_inline]] static void go(uint32_t memory, uint16_t count, uint32_t cfgr) {
        DmaChannelRegs& r = Channel::regs();
        r.MADDR = memory;
        r.CNTR = count;
        clear_register() = flag_bits;
        r.CFGR = cfgr | dma_cfgr_en;
        Dma<c>::count(ch);
    }

    /// The same on a channel whose EN may still stand - a completed block
    /// leaves it set (the file header) - with one store first to drop it.
    /// The ledger's count stays taken across the two.
    [[gnu::always_inline]] static void rearm(uint32_t memory, uint16_t count, uint32_t cfgr) {
        Channel::regs().CFGR = cfgr;
        go(memory, count, cfgr);
    }

    /// A block's end: the word without EN, and the count given back.
    [[gnu::always_inline]] static void halt(uint32_t cfgr) {
        Channel::regs().CFGR = cfgr;
        Dma<c>::uncount(ch);
    }

    /// The ISR body: the ARMED flags that are up, cleared with the global
    /// bit and handed back - one load and one store, the armed mask being
    /// the engine's own word and not a second read of CFGR.
    [[gnu::always_inline]] static uint32_t service(uint32_t armed) {
        uint32_t up;
        if constexpr (Channel::extended) {
            up = (dma2_extend()->INTFR >> Channel::flag_shift) & armed;
        } else {
            up = (dma_regs(c)->INTFR >> Channel::flag_shift) & armed;
        }
        if (up != 0u) {
            clear_register() = (up | DmaFlag::global) << Channel::flag_shift;
        }
        return up;
    }

    /// Whether a run of `count` items of width `w` from `memory` would
    /// cross 64 KB where this channel is held to 11.2.3's rule.
    static bool crosses(uint32_t memory, uint32_t count, DmaWidth w, bool increment) {
        if constexpr (Channel::bounded_to_64k) {
            return dma_span_crosses_64k(memory, static_cast<uint16_t>(count), w, increment);
        } else {
            (void)memory;
            (void)count;
            (void)w;
            (void)increment;
            return false;
        }
    }
};

// ---- the engines ---------------------------------------------------------------

/**
 * A transmit engine: caller-owned runs poured into one peripheral register.
 * The channel IS the request (11.2.3's tables), so the slot - the
 * controller and its channel - is the whole of the engine's identity.
 *
 * THE BEAT IS THE ELEMENT OF THE RUN. `start()` takes a span of bytes, of
 * half-words or - where `Elem` allows - of words, and the width of the
 * block is the span's: PSIZE = MSIZE = sizeof(T), a 16-bit frame one
 * 16-bit access to the data register. `Elem` is the WIDEST beat the binding
 * takes, the data register's width (16 bits by default, every serial and
 * bus data register of this family; 32 for a timer's 32-bit compare or a
 * memory cell), and a wider span is a compile error. A half-word run whose
 * address is odd is REFUSED at run time - 11.3.6's silent rounding would
 * move the wrong bytes - and the caller falls back to whatever it does
 * without the engine.
 *
 * THE CLAIM AND THE PROGRAMMING ARE TWO VERBS. `claim()` is the busy flag's
 * test-and-set and nothing else; `launch()` programs a claimed engine.
 * A transport whose completion handler starts the next block holds its
 * mask over `claim()` alone - a claimed channel cannot complete under the
 * loader, there being no block in flight to complete - and programs
 * unmasked. `start()` is the two together, for an owner with no handler to
 * race.
 */
template <uint8_t c, uint8_t ch, typename Elem = uint16_t>
class DmaTxEngine {
    static_assert(c == 1u || c == 2u,
                  "brio DMA: this family's controllers are DMA1 and DMA2 (RM 11.2.3)");
    static_assert(c <= device::dma_controller_count,
                  "brio DMA: this part has ONE DMA controller - DMA2 and its eleven channels are "
                  "the CH32V303's (device::dma_controller_count, RM 11.2.3)");
    static_assert(dma_channel_exists(c, ch),
                  "brio DMA: no such DMA channel for this engine - DMA1 has channels 1..8 on the "
                  "CH32V203 and 1..7 on the CH32V303, DMA2 has 1..11");
    static_assert(sizeof(Elem) == 1 || sizeof(Elem) == 2 || sizeof(Elem) == 4,
                  "a DMA element is one bus access wide: 1, 2 or 4 bytes");
    using Channel = DmaChannel<c, ch>;
    using B = DmaBinding<c, ch>;

public:
    DmaTxEngine() = delete;

    static constexpr bool present = true;
    static constexpr uint8_t controller = c;
    static constexpr uint8_t channel = ch;
    static constexpr DmaSlot slot{c, ch};
    /// The widest beat this binding takes.
    static constexpr DmaWidth width = dma_width_of<Elem>();
    using element = Elem;

    static constexpr uint8_t flag_complete = DmaFlag::complete;
    static constexpr uint8_t flag_half = DmaFlag::half;
    static constexpr uint8_t flag_error = DmaFlag::error;

    /**
     * Bind the channel to this peripheral: `data` is the register the runs
     * are poured into. Everything constant is written here (DmaBinding).
     * `interrupts` is which flags raise the channel's line - the
     * completion and the error by default; a transport whose completion is
     * proved by ANOTHER channel arms the error alone and takes no interrupt
     * per block. The PFIC line is enabled when anything is armed. False,
     * and nothing armed, when `data` is not aligned to `Elem`.
     */
    static bool arm(volatile void* data, uint8_t interrupts,
                    DmaPriority priority = DmaPriority::low) {
        stop();
        if ((dma_address(data) & dma_width_mask(width)) != 0u) {
            return false;
        }
        B::bind(data);
        armed_ = static_cast<uint8_t>(interrupts & (DmaFlag::complete | DmaFlag::half |
                                                    DmaFlag::error));
        cfg_ = dma_cfgr_dir | dma_cfgr_priority(priority) | dma_cfgr_interrupts(armed_);
        if (armed_ != 0u) {
            Pfic::enable(Channel::irq());
        } else {
            Pfic::disable(Channel::irq());
        }
        return true;
    }
    /// The completion and the error armed.
    static bool arm(volatile void* data, DmaPriority priority = DmaPriority::low) {
        return arm(data, static_cast<uint8_t>(DmaFlag::complete | DmaFlag::error), priority);
    }

    /// The channel's ISR body: the armed flags that are up, cleared and
    /// handed back. Acts on nothing.
    [[gnu::always_inline]] static uint8_t service() {
        return static_cast<uint8_t>(B::service(armed_));
    }

    /// Take the engine for a block: false when one is in flight. The test
    /// and the set are the whole of it - what a caller racing this
    /// engine's completion handler masks.
    static bool claim() {
        if (busy_) {
            return false;
        }
        busy_ = true;
        return true;
    }
    /// Give a claim back unused.
    static void unclaim() { busy_ = false; }

    /// Program and start a block on a CLAIMED engine. False - and the
    /// claim given back - for an empty run, one longer than CNTR counts, a
    /// misaligned one, or one across 64 KB on the CH32V303's DMA1.
    static bool launch(std::span<const uint8_t> run) { return go(run.data(), run.size(), true); }
    static bool launch(std::span<const uint16_t> run)
        requires(sizeof(Elem) >= 2)
    {
        return go(run.data(), run.size(), true);
    }
    static bool launch(std::span<const uint32_t> run)
        requires(sizeof(Elem) >= 4)
    {
        return go(run.data(), run.size(), true);
    }

    /// claim() and launch() in one: start moving the run (the caller's,
    /// and it stays put until complete() reports the block).
    static bool start(std::span<const uint8_t> run) { return claim() && launch(run); }
    static bool start(std::span<const uint16_t> run)
        requires(sizeof(Elem) >= 2)
    {
        return claim() && launch(run);
    }
    static bool start(std::span<const uint32_t> run)
        requires(sizeof(Elem) >= 4)
    {
        return claim() && launch(run);
    }
    template <typename T>
    static bool start(const T* buffer, uint16_t length) {
        return start(std::span<const T>(buffer, length));
    }

    /// `length` copies of ONE element: the memory pointer does not
    /// increment. What a full-duplex bus needs to clock a read.
    template <typename T>
    static bool start_fixed(const T* cell, uint16_t length) {
        static_assert(sizeof(T) <= sizeof(Elem), "brio DMA: a beat wider than this binding");
        return claim() && go(cell, length, false);
    }

    /// The block ended: how many elements it carried, so the owner can
    /// release exactly that much of its ring. The channel is disabled here
    /// because the silicon does not do it: EN stays set after a completed
    /// block, and the next block's store would be dropped.
    static uint16_t complete() {
        const uint16_t n = in_flight_;
        B::halt(cfg_);
        in_flight_ = 0;
        busy_ = false;
        return n;
    }

    static bool busy() { return busy_; }
    static uint16_t in_flight() { return busy_ ? in_flight_ : uint16_t{0}; }
    static DmaProgress progress() { return Channel::progress(in_flight_); }

    /// Throw the running block away and hand the channel back ready.
    static bool abandon() {
        ++faults_;
        B::halt(cfg_);
        busy_ = false;
        in_flight_ = 0;
        return true;
    }

    /// Start a run again from its beginning - what an owner does when a
    /// request was armed after the channel and the first datum never came.
    /// Nothing is in flight afterwards unless it succeeds.
    template <typename T>
    static bool kick(const T* buffer, uint16_t length) {
        B::halt(cfg_);
        busy_ = false;
        in_flight_ = 0;
        return start(buffer, length);
    }

    static uint32_t faults() { return faults_; }
    static void clear_faults() { faults_ = 0; }

    /// The channel stopped and its flags cleared; the binding (PADDR and
    /// the engine's word) stays, so start() works again without arm().
    static void stop() {
        B::halt(0);
        B::clear_register() = B::flag_bits;
        busy_ = false;
        in_flight_ = 0;
    }

private:
    template <typename T>
    static bool go(const T* buffer, size_t length, bool increment) {
        const uint32_t m = dma_address(buffer);
        if (buffer == nullptr || length == 0u || length > 0xFFFFu ||
            (m & (sizeof(T) - 1u)) != 0u ||
            B::crosses(m, static_cast<uint32_t>(length), dma_width_of<T>(), increment)) {
            busy_ = false;
            return false;
        }
        in_flight_ = static_cast<uint16_t>(length);
        B::go(m, static_cast<uint16_t>(length),
              cfg_ | dma_cfgr_widths(dma_width_of<T>()) | (increment ? dma_cfgr_minc : 0u));
        return true;
    }

    static inline uint32_t cfg_ = 0;
    static inline uint8_t armed_ = 0;
    static inline uint16_t in_flight_ = 0;
    static inline uint32_t faults_ = 0;
    // Set in the thread, cleared in the completion handler.
    static inline volatile bool busy_ = false;
};

/**
 * A receive engine, in two shapes on one binding.
 *
 * THE ONE-SHOT SHAPE fills a caller-owned run and stops: a bounded block,
 * the receive half of a bus transaction (an SPI host's data phase, an I2C
 * read). A block completes only when its run FILLS, which on a quiet line
 * may be never, so the owner may also ASK what has arrived since it last
 * asked - take(), one CNTR read, nothing suspended and nothing refused. The
 * beat is the run's element, as on the transmit side; a new run always
 * takes the channel, whatever the last one left standing.
 *
 * THE CIRCULAR SHAPE fills a RING for ever, on the controller's own cycle
 * mode (CFGR.CIRC, 11.2.1): when the count reaches zero the controller
 * reloads CNTR AND BOTH INTERNAL ADDRESSES and goes on, with no CPU in the
 * path. The circular arm() is handed the ring's whole storage and keeps it
 * - the address, the length every lap reloads, and the word with CIRC,
 * MINC, the beat, the priority and the two interrupts the ring needs, the
 * lap and the error - and start() runs it from the storage's first element
 * with the binding's five stores. Nothing re-arms it afterwards, so nothing
 * is lost between runs: there are none. The producer index of that ring is
 * no variable anybody stores - it is the channel's own count - and this
 * engine IS util/ring.hpp's RingCounter for it:
 *
 *   remaining()  CNTR, one load: the items still to be written in this
 *                lap. 11.2.1 orders a transfer as the read, the store, and
 *                THEN the decrement, so the count never counts an item
 *                whose store has not been made;
 *   laps()       the laps completed since start(): the transfer-complete
 *                flag, raised at every wrap, turned into a count by lap()
 *                in the channel's handler - so it lags the count by a
 *                handler's latency and never leads it.
 *
 * util/ring.hpp's HardwareRing<storage, DmaRxEngine<c, ch, Elem>> is the
 * consumer half, and a lap the consumer did not keep up with is that view's
 * accounting (its overruns()), because only the consumer knows where its
 * tail is. The half-transfer flag is armed on request (`half_mark`): the
 * view reads the count whenever it looks and the wrap is the one edge the
 * lap count needs, but a byte transport's ring wants the lap's half and
 * full marks as the edge of a stream that never falls silent. A
 * circular channel never stops on its own - EN stands, and with it the
 * bus-master count the channel holds (a program with a ring running does
 * not sleep on this family, the file header) - so the one way it stops by
 * itself is a transfer error, which idle() reports.
 */
template <uint8_t c, uint8_t ch, typename Elem = uint16_t>
class DmaRxEngine {
    static_assert(c == 1u || c == 2u,
                  "brio DMA: this family's controllers are DMA1 and DMA2 (RM 11.2.3)");
    static_assert(c <= device::dma_controller_count,
                  "brio DMA: this part has ONE DMA controller - DMA2 and its eleven channels are "
                  "the CH32V303's (device::dma_controller_count, RM 11.2.3)");
    static_assert(dma_channel_exists(c, ch),
                  "brio DMA: no such DMA channel for this engine - DMA1 has channels 1..8 on the "
                  "CH32V203 and 1..7 on the CH32V303, DMA2 has 1..11");
    static_assert(sizeof(Elem) == 1 || sizeof(Elem) == 2 || sizeof(Elem) == 4,
                  "a DMA element is one bus access wide: 1, 2 or 4 bytes");
    using Channel = DmaChannel<c, ch>;
    using B = DmaBinding<c, ch>;

public:
    DmaRxEngine() = delete;

    static constexpr bool present = true;
    static constexpr uint8_t controller = c;
    static constexpr uint8_t channel = ch;
    static constexpr DmaSlot slot{c, ch};
    static constexpr DmaWidth width = dma_width_of<Elem>();
    using element = Elem;
    /// Whether this channel is held to 11.2.3's 64 KB rule (the CH32V303's
    /// DMA1): what an owner placing a ring's storage asks, so that no
    /// placement can straddle a 64 KB page and arm() never refuses it.
    static constexpr bool bounded_to_64k = Channel::bounded_to_64k;

    static constexpr uint8_t flag_complete = DmaFlag::complete;
    static constexpr uint8_t flag_half = DmaFlag::half;
    static constexpr uint8_t flag_error = DmaFlag::error;

    [[gnu::always_inline]] static uint8_t service() {
        return static_cast<uint8_t>(B::service(armed_));
    }

    /// Bind the channel to this peripheral's register for ONE-SHOT runs;
    /// DmaTxEngine::arm()'s rules. False, and nothing armed, when `data` is
    /// not aligned to Elem. A ring an earlier circular arm() bound stays
    /// bound - its word is its own - and start() with no run runs it.
    static bool arm(volatile void* data, uint8_t interrupts,
                    DmaPriority priority = DmaPriority::low) {
        stop();
        if ((dma_address(data) & dma_width_mask(width)) != 0u) {
            return false;
        }
        B::bind(data);
        armed_ = static_cast<uint8_t>(interrupts & (DmaFlag::complete | DmaFlag::half |
                                                    DmaFlag::error));
        cfg_ = dma_cfgr_priority(priority) | dma_cfgr_interrupts(armed_);
        if (armed_ != 0u) {
            Pfic::enable(Channel::irq());
        } else {
            Pfic::disable(Channel::irq());
        }
        return true;
    }
    /// The completion and the error armed.
    static bool arm(volatile void* data, DmaPriority priority = DmaPriority::low) {
        return arm(data, static_cast<uint8_t>(DmaFlag::complete | DmaFlag::error), priority);
    }

    /**
     * Bind the channel to this peripheral's register AND to a ring's whole
     * storage, for good: the CIRCULAR shape. `storage` is the caller's
     * array - the consumer view's own, named in its type - and its element
     * is the beat; its length, the count every lap reloads, is a compile-
     * time fact and is checked here (CNTR counts 65535 at most). The lap and
     * the error are armed, with `half_mark` the half lap too (service()
     * hands it back; lap() is the completion's alone), and the PFIC line
     * enabled; the channel comes out
     * stopped, and start() runs the ring. False, and nothing armed, when
     * `data` is not aligned to Elem, the storage not aligned to its element,
     * or - on the CH32V303's DMA1 - the storage across a 64 KB boundary.
     * The one-shot verbs stay usable on the same binding: a start(run)
     * replaces the ring until the next start().
     */
    /// The same at the low priority, the half lap's mark asked for or
    /// not: what a transport that names no DmaPriority spells.
    template <typename T, size_t N>
    static bool arm(volatile void* data, T (&storage)[N], bool half_mark) {
        return arm(data, storage, DmaPriority::low, half_mark);
    }
    template <typename T, size_t N>
    static bool arm(volatile void* data, T (&storage)[N],
                    DmaPriority priority = DmaPriority::low, bool half_mark = false) {
        static_assert(sizeof(T) <= sizeof(Elem),
                      "brio DmaRxEngine: a ring wider than this binding's beat - name the engine "
                      "with the wider element if the register gives it");
        static_assert(!std::is_const_v<T> && !std::is_volatile_v<T>,
                      "brio DmaRxEngine: the ring is plain storage the channel writes");
        static_assert(N >= 2u && N <= 0xFFFFu,
                      "brio DmaRxEngine: a ring of 2..65535 items - CNTR is the count every lap "
                      "reloads, and it is sixteen bits (11.3.4)");
        ring_length_ = 0;
        if (!arm(data,
                 static_cast<uint8_t>(DmaFlag::complete | DmaFlag::error |
                                      (half_mark ? DmaFlag::half : 0u)),
                 priority)) {
            return false;
        }
        const uint32_t m = dma_address(&storage[0]);
        if ((m & (sizeof(T) - 1u)) != 0u ||
            B::crosses(m, static_cast<uint32_t>(N), dma_width_of<T>(), true)) {
            Pfic::disable(Channel::irq());
            armed_ = 0;
            cfg_ = 0;
            return false;
        }
        ring_ = m;
        ring_length_ = static_cast<uint16_t>(N);
        ring_cfg_ = cfg_ | dma_cfgr_circ | dma_cfgr_minc | dma_cfgr_widths(dma_width_of<T>());
        laps_ = 0;
        return true;
    }

    /**
     * (Re)start the RING from its storage's first element with the lap
     * count at zero: the binding's re-arm, five stores. Once after the
     * circular arm(), and again only after the channel has stopped - a
     * transfer error, or stop() - and then the consumer's view is cleared
     * with it, its positions being counted from this element and this lap.
     * False on a binding that has no ring.
     */
    static bool start() {
        if (ring_length_ == 0u) {
            return false;
        }
        capacity_ = 0;
        taken_ = 0;
        laps_ = 0;
        B::rearm(ring_, ring_length_, ring_cfg_);
        return true;
    }

    // -- the ring's counter (util/ring.hpp's RingCounter) ---------------------

    /// The items still to be written in the current lap: CNTR, ONE load,
    /// never refused - live while the channel runs (11.3.4).
    [[gnu::always_inline]] static uint32_t remaining() {
        return static_cast<uint16_t>(Channel::regs().CNTR);
    }

    /// The laps completed since start(): lap()'s count. A volatile word, so
    /// a consumer polling it sees the handler's store.
    [[gnu::always_inline]] static uint32_t laps() { return laps_; }

    /// The completion flag's verb ON A RING - called from the channel's
    /// handler when service() reports it. A lap, counted; the controller
    /// has already reloaded itself and nothing is re-armed. (A one-shot
    /// block's completion is complete(), which stops the channel.) Always
    /// inline: an ISR body's verb, so the handler stays free of a call.
    [[gnu::always_inline]] static void lap() { laps_ = laps_ + 1u; }

    /// The channel is not running - the silicon asked, because a transfer
    /// error drops EN by itself.
    static bool idle() { return !Channel::enabled(); }

    /// Fill a run, the width the span's. False for an empty run, one longer
    /// than CNTR counts, a misaligned one or one across 64 KB on the
    /// CH32V303's DMA1 - the channel then left as it was.
    static bool start(std::span<uint8_t> run) { return go(run.data(), run.size(), true); }
    static bool start(std::span<uint16_t> run)
        requires(sizeof(Elem) >= 2)
    {
        return go(run.data(), run.size(), true);
    }
    static bool start(std::span<uint32_t> run)
        requires(sizeof(Elem) >= 4)
    {
        return go(run.data(), run.size(), true);
    }
    template <typename T>
    static bool start(T* buffer, uint16_t length) {
        return start(std::span<T>(buffer, length));
    }

    /// `length` elements into ONE cell, thrown away (the receive side of
    /// a bus write).
    template <typename T>
    static bool start_discard(T* cell, uint16_t length) {
        static_assert(sizeof(T) <= sizeof(Elem), "brio DMA: a beat wider than this binding");
        return go(cell, length, false);
    }

    /// The block ended: the channel disabled - EN stays set after a
    /// completed block, and a channel left enabled holds the bus-master
    /// count that keeps the core awake. What it collected stays readable
    /// through take() and harvest().
    static void complete() { B::halt(cfg_); }

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
        B::halt(cfg_);
        capacity_ = 0;
        taken_ = 0;
        return true;
    }

    /// The same run started again from its beginning.
    template <typename T>
    static bool kick(T* buffer, uint16_t length) {
        B::halt(cfg_);
        capacity_ = 0;
        taken_ = 0;
        return start(buffer, length);
    }

    static uint32_t faults() { return faults_; }
    static void clear_faults() { faults_ = 0; }

    /// The channel stopped and its flags cleared; the binding stays.
    static void stop() {
        B::halt(0);
        B::clear_register() = B::flag_bits;
        capacity_ = 0;
        taken_ = 0;
    }

private:
    template <typename T>
    static bool go(T* buffer, size_t length, bool increment) {
        const uint32_t m = dma_address(buffer);
        if (buffer == nullptr || length == 0u || length > 0xFFFFu ||
            (m & (sizeof(T) - 1u)) != 0u ||
            B::crosses(m, static_cast<uint32_t>(length), dma_width_of<T>(), increment)) {
            return false;
        }
        capacity_ = static_cast<uint16_t>(length);
        taken_ = 0;
        B::rearm(m, static_cast<uint16_t>(length),
                 cfg_ | dma_cfgr_widths(dma_width_of<T>()) | (increment ? dma_cfgr_minc : 0u));
        return true;
    }

    static inline uint32_t cfg_ = 0;
    static inline uint8_t armed_ = 0;
    static inline uint16_t capacity_ = 0;
    static inline uint16_t taken_ = 0;
    static inline uint32_t faults_ = 0;
    // The ring the circular arm() bound: its address, its length (zero:
    // none) and its word.
    static inline uint32_t ring_ = 0;
    static inline uint16_t ring_length_ = 0;
    static inline uint32_t ring_cfg_ = 0;
    // Handler-written, consumer-read.
    static inline volatile uint32_t laps_ = 0;
};

/**
 * DmaCopyEngine<c, ch> - memory to memory: `copy()` and `fill()` on one
 * channel in the controller's MEMORY-TO-MEMORY MODE (CFGR.MEM2MEM, 11.2.1:
 * no request, the block runs on the enable), the source on the PADDR side
 * (DIR clear) and the destination on MADDR's.
 *
 * THE BEAT IS T. `n` counts elements of T - a byte, a half-word or a word,
 * PSIZE = MSIZE = sizeof(T) - and this controller moves any of the three in
 * six cycles (docs/ch32vx03/dma.md), so a word is four times the bytes a
 * cycle of a byte: a caller with an aligned run hands it over as words. An
 * address not aligned to T is refused (11.3.5 and 11.3.6 round it down in
 * silence), and so is a run longer than CNTR's 65535 or, on the CH32V303's
 * DMA1, one across 64 KB.
 *
 * THE FILL'S CELL IS THE CALLER'S, in memory: the controller reads an
 * ADDRESS each beat, with the source increment off, so the cell must stay
 * put until the block is over - as the buffers of a copy must.
 *
 * Completion by the channel's interrupt (`service()` the ISR body) or, armed
 * without one, by `busy()` asking the flag. One owner: the busy flag is
 * tested and set without a mask.
 */
template <uint8_t c, uint8_t ch>
class DmaCopyEngine {
    static_assert(c == 1u || c == 2u,
                  "brio DMA: this family's controllers are DMA1 and DMA2 (RM 11.2.3)");
    static_assert(c <= device::dma_controller_count,
                  "brio DMA: this part has ONE DMA controller - DMA2 and its eleven channels are "
                  "the CH32V303's (device::dma_controller_count, RM 11.2.3)");
    static_assert(dma_channel_exists(c, ch),
                  "brio DMA: no such DMA channel for this engine - DMA1 has channels 1..8 on the "
                  "CH32V203 and 1..7 on the CH32V303, DMA2 has 1..11");
    using Channel = DmaChannel<c, ch>;
    using B = DmaBinding<c, ch>;

public:
    DmaCopyEngine() = delete;

    static constexpr bool present = true;
    static constexpr uint8_t controller = c;
    static constexpr uint8_t channel = ch;
    static constexpr DmaSlot slot{c, ch};

    static constexpr uint8_t flag_complete = DmaFlag::complete;
    static constexpr uint8_t flag_error = DmaFlag::error;

    /// Bind the channel: the gate, the mode, the priority, and whether the
    /// completion raises the channel's line (`interrupt`) or is asked for
    /// by busy().
    static void arm(DmaPriority priority = DmaPriority::low, bool interrupt = true) {
        stop();
        B::bind(nullptr);
        interrupt_ = interrupt;
        cfg_ = dma_cfgr_mem2mem | dma_cfgr_minc | dma_cfgr_priority(priority) |
               (interrupt ? dma_cfgr_interrupts(DmaFlag::complete | DmaFlag::error) : 0u);
        if (interrupt) {
            Pfic::enable(Channel::irq());
        } else {
            Pfic::disable(Channel::irq());
        }
    }

    /// Copy `n` elements from `src` to `dst`, one beat an element. False -
    /// nothing started - while a block is in flight and for a run the
    /// header refuses.
    template <typename T>
    static bool copy(T* dst, const T* src, uint32_t n) {
        static_assert(sizeof(T) == 1 || sizeof(T) == 2 || sizeof(T) == 4,
                      "brio DMA: a beat is a byte, a half-word or a word");
        return run(dma_address(dst), dma_address(src), n, dma_width_of<T>(), dma_cfgr_pinc);
    }

    /// Write `n` copies of `*cell` from `dst` on - the cell read each beat,
    /// so it stays put until the block is over.
    template <typename T>
    static bool fill(T* dst, const T* cell, uint32_t n) {
        static_assert(sizeof(T) == 1 || sizeof(T) == 2 || sizeof(T) == 4,
                      "brio DMA: a beat is a byte, a half-word or a word");
        return run(dma_address(dst), dma_address(cell), n, dma_width_of<T>(), 0u);
    }

    /// A block in flight. Armed without the interrupt, this is also where
    /// the completion is found: the flag asked, the channel stopped.
    static bool busy() {
        if (busy_ && !interrupt_) {
            const uint32_t f = B::flags();
            if ((f & (DmaFlag::complete | DmaFlag::error)) != 0u) {
                if ((f & DmaFlag::error) != 0u) {
                    ++faults_;
                }
                finish();
            }
        }
        return busy_;
    }

    /// Stop the block where it stands - CNTR then holds what was left -
    /// and free the engine. False when nothing was in flight.
    static bool abandon() {
        if (!busy_) {
            return false;
        }
        finish();
        return true;
    }

    /// The channel's ISR body: the block's end acted on - the channel
    /// stopped, the engine free again - and the flags handed back.
    [[gnu::always_inline]] static uint8_t service() {
        const uint32_t up = B::service(DmaFlag::complete | DmaFlag::error);
        if ((up & DmaFlag::error) != 0u) {
            ++faults_;
        }
        if (up != 0u) {
            finish();
        }
        return static_cast<uint8_t>(up);
    }

    static uint32_t faults() { return faults_; }
    static void clear_faults() { faults_ = 0; }

    static void stop() {
        B::halt(0);
        B::clear_register() = B::flag_bits;
        busy_ = false;
    }

private:
    /// One block of `n` beats of width `w` from `source` (incremented when
    /// `pinc` says so) to `dst`.
    static bool run(uint32_t dst, uint32_t source, uint32_t n, DmaWidth w, uint32_t pinc) {
        const uint32_t mask = dma_width_mask(w);
        if (busy_ || dst == 0u || source == 0u || n == 0u || n > 0xFFFFu ||
            ((dst | source) & mask) != 0u || B::crosses(dst, n, w, true) ||
            (pinc != 0u && B::crosses(source, n, w, true))) {
            return false;
        }
        busy_ = true;
        Channel::regs().PADDR = source;
        B::go(dst, static_cast<uint16_t>(n), cfg_ | pinc | dma_cfgr_widths(w));
        return true;
    }

    static void finish() {
        B::halt(cfg_);
        busy_ = false;
    }

    static inline uint32_t cfg_ = 0;
    static inline uint32_t faults_ = 0;
    static inline bool interrupt_ = true;
    // Set by the owner, cleared by the completion.
    static inline volatile bool busy_ = false;
};

// ---- the block engines ----------------------------------------------------------

/**
 * DmaLoopEngine<c, ch, Elem> - "play one caller-owned table into a
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
template <uint8_t c, uint8_t ch, typename Elem = uint16_t>
class DmaLoopEngine {
    static_assert(c == 1u || c == 2u,
                  "brio DMA: this family's controllers are DMA1 and DMA2 (RM 11.2.3)");
    static_assert(c <= device::dma_controller_count,
                  "brio DMA: this part has ONE DMA controller - DMA2 and its eleven channels are "
                  "the CH32V303's (device::dma_controller_count, RM 11.2.3)");
    static_assert(dma_channel_exists(c, ch),
                  "brio DMA: no such DMA channel for this engine - DMA1 has channels 1..8 on the "
                  "CH32V203 and 1..7 on the CH32V303, DMA2 has 1..11");
    static_assert(sizeof(Elem) == 1 || sizeof(Elem) == 2 || sizeof(Elem) == 4,
                  "a DMA element is one bus access wide: 1, 2 or 4 bytes");
    using Channel = DmaChannel<c, ch>;
    using B = DmaBinding<c, ch>;

public:
    DmaLoopEngine() = delete;

    static constexpr bool present = true;
    static constexpr uint8_t controller = c;
    static constexpr uint8_t channel = ch;
    static constexpr DmaSlot slot{c, ch};
    static constexpr DmaWidth width = dma_width_of<Elem>();
    using element = Elem;

    static constexpr uint8_t flag_complete = DmaFlag::complete;
    static constexpr uint8_t flag_half = DmaFlag::half;
    static constexpr uint8_t flag_error = DmaFlag::error;

    /// The channel's ISR body folded into the engine: the armed flags
    /// that are up, cleared, handed back. Acts on nothing.
    [[gnu::always_inline]] static uint8_t service() {
        return static_cast<uint8_t>(B::service(DmaFlag::complete | DmaFlag::error));
    }

    /// Bind the channel to this peripheral; `data` is the register the
    /// table is poured into. The PFIC line is enabled here. False, and
    /// nothing armed, when `data` is not aligned to Elem.
    static bool arm(volatile void* data, DmaPriority priority = DmaPriority::low) {
        stop();
        if ((dma_address(data) & dma_width_mask(width)) != 0u) {
            return false;
        }
        B::bind(data);
        cfg_ = dma_cfgr_dir | dma_cfgr_circ | dma_cfgr_minc | dma_cfgr_widths(width) |
               dma_cfgr_priority(priority) |
               dma_cfgr_interrupts(DmaFlag::complete | DmaFlag::error);
        Pfic::enable(Channel::irq());
        return true;
    }

    /// Begin playing `length` elements of `table`, over and over. The
    /// buffer is the caller's and must outlive the stream. Refused while
    /// the stream runs, and for a table this channel cannot take.
    static bool start(const Elem* table, uint16_t length) {
        const uint32_t m = dma_address(table);
        if (running_ || table == nullptr || length == 0u || (m & dma_width_mask(width)) != 0u ||
            B::crosses(m, length, width, true)) {
            return false;
        }
        table_ = table;
        length_ = length;
        laps_ = 0;
        faults_ = 0;
        B::rearm(m, length, cfg_);
        running_ = true;
        return true;
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
        B::halt(cfg_);
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
        B::halt(cfg_);
        running_ = false;
        return start(table_, length_);
    }

    static void stop() {
        B::halt(0);
        B::clear_register() = B::flag_bits;
        running_ = false;
        length_ = 0;
    }

private:
    static inline uint32_t cfg_ = 0;
    static inline const Elem* table_ = nullptr;
    static inline uint16_t length_ = 0;
    // Handler-written, loop-read.
    static inline volatile uint32_t laps_ = 0;
    static inline volatile uint32_t faults_ = 0;
    static inline volatile bool running_ = false;
};

/**
 * DmaPingPongEngine<c, ch, Elem> - "fill one caller-owned buffer while the
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
 * full buffer over and starts the other one - five stores, the binding's
 * re-arm - and when the caller holds both the engine counts an overrun,
 * stalls, and is restarted by `release()`. The gap between one block and
 * the next is the re-arm, which the accounting does not hide: a sample
 * lost there is a sample the peripheral never delivered, and `overruns()`
 * counts the laps the engine chose not to take.
 *
 * Both buffers are the caller's, both hold `length` elements, and both
 * must outlive the stream. They are `volatile` because the controller
 * writes them and the compiler sees nothing.
 */
template <uint8_t c, uint8_t ch, typename Elem = uint16_t>
class DmaPingPongEngine {
    static_assert(c == 1u || c == 2u,
                  "brio DMA: this family's controllers are DMA1 and DMA2 (RM 11.2.3)");
    static_assert(c <= device::dma_controller_count,
                  "brio DMA: this part has ONE DMA controller - DMA2 and its eleven channels are "
                  "the CH32V303's (device::dma_controller_count, RM 11.2.3)");
    static_assert(dma_channel_exists(c, ch),
                  "brio DMA: no such DMA channel for this engine - DMA1 has channels 1..8 on the "
                  "CH32V203 and 1..7 on the CH32V303, DMA2 has 1..11");
    static_assert(sizeof(Elem) == 1 || sizeof(Elem) == 2 || sizeof(Elem) == 4,
                  "a DMA element is one bus access wide: 1, 2 or 4 bytes");
    using Channel = DmaChannel<c, ch>;
    using B = DmaBinding<c, ch>;

public:
    DmaPingPongEngine() = delete;

    static constexpr bool present = true;
    static constexpr uint8_t controller = c;
    static constexpr uint8_t channel = ch;
    static constexpr DmaSlot slot{c, ch};
    static constexpr DmaWidth width = dma_width_of<Elem>();
    using element = Elem;

    static constexpr uint8_t flag_complete = DmaFlag::complete;
    static constexpr uint8_t flag_half = DmaFlag::half;
    static constexpr uint8_t flag_error = DmaFlag::error;

    [[gnu::always_inline]] static uint8_t service() {
        return static_cast<uint8_t>(B::service(DmaFlag::complete | DmaFlag::error));
    }

    /// Bind the channel; `data` is the peripheral's data register. False,
    /// and nothing armed, when it is not aligned to Elem.
    static bool arm(volatile void* data, DmaPriority priority = DmaPriority::low) {
        stop();
        if ((dma_address(data) & dma_width_mask(width)) != 0u) {
            return false;
        }
        B::bind(data);
        cfg_ = dma_cfgr_minc | dma_cfgr_widths(width) | dma_cfgr_priority(priority) |
               dma_cfgr_interrupts(DmaFlag::complete | DmaFlag::error);
        Pfic::enable(Channel::irq());
        return true;
    }

    /// Begin streaming into `first`, with `second` as the buffer the
    /// next block will use. Both are checked here, once: their alignment,
    /// and on the CH32V303's DMA1 the 64 KB rule.
    static bool start(volatile Elem* first, volatile Elem* second, uint16_t length) {
        if (first == nullptr || second == nullptr || first == second || length == 0u ||
            !fits(first, length) || !fits(second, length)) {
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
            // not released. Skip the lap rather than write into it - and
            // stop the channel, whose EN a completed block leaves set.
            overruns_ = overruns_ + 1u;
            stalled_ = true;
            running_ = false;
            B::halt(cfg_);
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
        B::halt(cfg_);
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
        return launch();
    }

    static uint32_t faults() { return faults_; }
    static void clear_faults() { faults_ = 0; }

    static void stop() {
        B::halt(0);
        B::clear_register() = B::flag_bits;
        running_ = false;
        stalled_ = false;
        pending_ = 0;
        length_ = 0;
    }

private:
    static bool fits(volatile Elem* buffer, uint16_t length) {
        const uint32_t m = dma_address(buffer);
        return (m & dma_width_mask(width)) == 0u && !B::crosses(m, length, width, true);
    }

    /// EN STAYS SET WHEN A BLOCK COMPLETES on this controller (the file
    /// header: only software clears it), and the four stores are dropped
    /// while it is - so the re-arm is the binding's, which drops it first.
    static bool launch() {
        if (buffer_[fill_] == nullptr || length_ == 0u) {
            return false;
        }
        B::rearm(dma_address(buffer_[fill_]), length_, cfg_);
        running_ = true;
        return true;
    }

    static inline uint32_t cfg_ = 0;
    static inline volatile Elem* buffer_[2] = {nullptr, nullptr};
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

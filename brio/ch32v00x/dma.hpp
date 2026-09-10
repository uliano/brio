/*
 * dma.hpp
 *
 * The DMA controller of the CH32V00x (RM ch. 8): seven channels, each
 * wired to a fixed handful of peripheral requests, an arbiter over four
 * software priorities with the channel index deciding ties, and three
 * flags per channel - the STM32F1's DMA1, which is where the STM32G0
 * stratum's design of a channel and its engines comes from, minus that
 * family's request multiplexer: HERE THE CHANNEL IS THE REQUEST. RM
 * table 8-2 says which peripheral event each channel answers, and an
 * engine is named by channel number with that table in hand (USART1
 * transmits on channel 4 and receives on 5, SPI1 on 3 and 2, the ADC
 * on 1, I2C1 on 6 and 7).
 *
 * WHAT A CHANNEL IS. A configuration word (CFGR: direction, circular,
 * memory-to-memory, the two increments, the two widths, the priority,
 * the three interrupt enables and EN), a count that the controller
 * decrements as it moves items (CNTR, read-only while enabled), and
 * two addresses named after the registers' F1 ancestry - PADDR and
 * MADDR - where DIR says which side is the source: in memory-to-memory
 * mode both are memory and the names mean nothing.
 *
 * THE VECTORS ARE ONE PER CHANNEL (Irq::dma1_channel1..7), so a
 * channel's ISR body reads only its own flags and a handler needs no
 * "which channel" question. DMA1EN in RCC_HBPCENR is the block's one
 * gate, and it is CLOSED at reset (the register's reset value opens
 * the SRAM alone), so `Dma::open()` is the first thing every channel
 * verb does.
 *
 * THE ENGINES. `DmaTxEngine<ch, Elem>` and `DmaRxEngine<ch, Elem>` are
 * the STM32G0 stratum's, minus the request id: a transmit engine pours
 * a caller-owned run into one peripheral register and reports how many
 * items the block carried when it completes; a receive engine fills a
 * caller-owned run from one register and answers how many items have
 * arrived since it was last asked (one CNTR read - nothing suspended,
 * nothing refused). ch32v00x/usart.hpp's two engine slots are built on
 * them, and a driver reaches its engines only through their own
 * published names (start, service, flag_complete, flag_error) so that
 * a driver with an empty slot never includes this file.
 *
 * TWO THINGS THE SILICON DOES that the engines are built on. EN STAYS
 * SET when a non-circular block completes (CNTR at zero, TCIF up, the
 * channel enabled and idle - the F1 lineage: only software clears EN),
 * so every verb that programs a channel refuses while EN is up and the
 * owner disables before the next load. And A READ FROM A HOLE IN THE
 * MAP COMPLETES AS A NORMAL BLOCK (zeros arrive, TEIF stays down): the
 * transfer-error path is written and armed on every engine, but no
 * address the bench could name reaches it.
 *
 * NOT COVERED YET: the loop and ping-pong engines the STM32G0 stratum
 * keeps for a block stream (an ADC sampled into caller-owned halves) -
 * born with the ADC chapter; the timer-triggered channels, born with
 * the timers.
 */

#pragma once

#include <stdint.h>

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

/// CFGR.PL: the software half of the arbitration; ties go to the lower
/// channel index (RM 8.2.1).
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
inline constexpr uint32_t rcc_hb_dma1      = 1UL << 0;

inline constexpr uint8_t dma_channel_count = 7;

constexpr Irq dma_channel_irq(uint8_t ch) {
    return static_cast<Irq>(static_cast<uint8_t>(Irq::dma1_channel1) + (ch - 1u));
}

/// The block: its gate and its two flag registers.
struct Dma {
    Dma() = delete;

    static void open() { rcc()->HBPCENR |= rcc_hb_dma1; }
    static uint32_t flags() { return dma()->INTFR; }
};

/**
 * One channel, 1..7. Every configuring verb refuses while the channel
 * is enabled, because CFGR's fields, CNTR and the two addresses are
 * read-only then (RM 8.3.3): a store the silicon ignores would be a
 * lie the code told itself.
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
        uint32_t v = regs().CFGR & (dma_cfgr_tcie | dma_cfgr_htie | dma_cfgr_teie);
        if (c.direction == DmaDirection::memory_to_peripheral) { v |= dma_cfgr_dir; }
        if (c.circular) { v |= dma_cfgr_circ; }
        if (c.memory_to_memory) { v |= dma_cfgr_mem2mem; }
        if (c.peripheral_increment) { v |= dma_cfgr_pinc; }
        if (c.memory_increment) { v |= dma_cfgr_minc; }
        v |= static_cast<uint32_t>(c.peripheral_width) << 8;
        v |= static_cast<uint32_t>(c.memory_width) << 10;
        v |= static_cast<uint32_t>(c.priority) << 12;
        regs().CFGR = v;
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

    /// Program the whole transfer WITHOUT starting it.
    static bool prepare(const DmaTransfer& t) {
        Dma::open();
        if (enabled() || !dma_transfer_valid(t)) {
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
        uint32_t bits = 0;
        if ((mask & DmaFlag::complete) != 0u) { bits |= dma_cfgr_tcie; }
        if ((mask & DmaFlag::half) != 0u) { bits |= dma_cfgr_htie; }
        if ((mask & DmaFlag::error) != 0u) { bits |= dma_cfgr_teie; }
        if (on) { regs().CFGR |= bits; } else { regs().CFGR &= ~bits; }
    }

    /**
     * The channel's ISR body: read the flags that are ARMED and up, clear
     * exactly those, hand them back. A completion is not acted on here -
     * what it means is the owner's to decide.
     */
    [[gnu::always_inline]] static uint32_t isr() {
        const uint32_t cfg = regs().CFGR;
        uint32_t armed = 0;
        if ((cfg & dma_cfgr_tcie) != 0u) { armed |= DmaFlag::complete; }
        if ((cfg & dma_cfgr_htie) != 0u) { armed |= DmaFlag::half; }
        if ((cfg & dma_cfgr_teie) != 0u) { armed |= DmaFlag::error; }
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

// ---- the engines ---------------------------------------------------------------

/**
 * A transmit engine: one caller-owned run poured into one peripheral
 * register. The STM32G0 stratum's, minus the request id - the channel
 * IS the request here (RM table 8-2).
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
        busy_ = Channel::load(DmaTransfer{
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
        });
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
        busy_ = Channel::load(DmaTransfer{
            .peripheral = data_,
            .memory = const_cast<Elem*>(cell),
            .count = length,
            .config = {.direction = DmaDirection::memory_to_peripheral,
                       .circular = false,
                       .memory_to_memory = false,
                       .peripheral_increment = false,
                       .memory_increment = false,
                       .peripheral_width = width,
                       .memory_width = width,
                       .priority = priority_},
        });
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
    static uint16_t in_flight() { return busy_ ? in_flight_ : 0u; }
    static DmaProgress progress() { return Channel::progress(in_flight_); }

    /// Throw the running block away and hand the channel back ready.
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

    /// `length` elements into ONE cell, thrown away (the receive side of
    /// a bus write).
    static bool start_discard(Elem* cell, uint16_t length) {
        if (cell == nullptr || length == 0u) {
            return false;
        }
        Channel::enable(false);
        capacity_ = length;
        taken_ = 0;
        return Channel::load(DmaTransfer{
            .peripheral = data_,
            .memory = cell,
            .count = length,
            .config = {.direction = DmaDirection::peripheral_to_memory,
                       .circular = false,
                       .memory_to_memory = false,
                       .peripheral_increment = false,
                       .memory_increment = false,
                       .peripheral_width = width,
                       .memory_width = width,
                       .priority = priority_},
        });
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
        Channel::arm(DmaFlag::complete | DmaFlag::error, true);
    }

    static inline volatile void* data_ = nullptr;
    static inline DmaPriority priority_ = DmaPriority::low;
    static inline uint16_t capacity_ = 0;
    static inline uint16_t taken_ = 0;
    static inline uint32_t faults_ = 0;
};

} // namespace brio

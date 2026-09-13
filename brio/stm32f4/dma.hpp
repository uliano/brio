/*
 * dma.hpp
 *
 * The DMA controllers of the STM32F4 (RM0090 ch. 10, RM0390 ch. 9,
 * RM0383 ch. 9 - one chapter, three manuals, the same IP), in the two
 * strata every brio driver has:
 *
 *   Dma<n>                the BLOCK: its AHB1 gate, its reset, the two
 *                         interrupt-status registers every stream reports
 *                         into and the two write-one-to-clear registers
 *                         beside them, and the one thing that separates
 *                         the two controllers - only DMA2 can move memory
 *                         to memory;
 *   DmaStream<n, s>       the RESOURCE: one stream's six registers
 *                         (SxCR/SxNDTR/SxPAR/SxM0AR/SxM1AR/SxFCR), the
 *                         enable discipline of 10.3.17, the FIFO with the
 *                         burst rules of table 49, the double buffer with
 *                         its current-target readback, the flow
 *                         controller, all five flags and one ISR body;
 *   DmaTxEngine / DmaRxEngine
 *                         the two TASKS a byte transport wants: drain a
 *                         run into a peripheral, fill a run from one -
 *                         the slots stm32f4/usart.hpp's Uart declares.
 *
 * A STREAM IS NOT A CHANNEL, and the difference is the whole shape of
 * this controller. On the STM32G0 a channel is the unit and a
 * multiplexer points it at any request line. Here the unit is the
 * STREAM - eight per controller, each with its own FIFO, its own
 * priority and its OWN VECTOR - and a stream chooses between exactly
 * EIGHT request lines with CHSEL (10.3.3). Which peripheral sits on
 * which (stream, channel) cell is a fixed wiring table per part class,
 * so a peripheral cannot be served by any stream: it can be served by
 * the one or two the table gives it. That is why the engines here name
 * a stream AND a channel where the G0's name a channel and a request
 * id, and why the check that the pair really carries the peripheral is
 * a compile-time one (stm32f4/usart.hpp's static_assert over
 * usart_dma_placement_valid()).
 *
 * THE DRIVER OWNS THE FABRIC AND NOT THE REQUEST VOCABULARY - the same
 * division the STM32G0 stratum draws, and for the same reason. This
 * file takes a plain channel number and knows nothing about USARTs; a
 * peripheral publishes its own cells of tables 43 and 44 (RM0390's 28
 * and 29, RM0383's 27 and 28) and the reserve keys them on the
 * device-select define, because no device header of this pack carries a
 * request mapping and the tables differ by part. The serial slice is in
 * stm32f4/device_tables.hpp; the next chapter's slice joins it there.
 *
 * FOUR FACTS OF THIS CONTROLLER shape everything below.
 *
 *  - EN IS NOT A SWITCH, IT IS A REQUEST. 10.3.17 step 1 and 10.3.14:
 *    a write of 0 to EN takes effect only when the current transfer has
 *    finished AND, on a peripheral-to-memory or memory-to-memory stream,
 *    when the FIFO has been flushed into the destination - so EN reads
 *    back 1 until then, and reconfiguring a stream before that read
 *    comes down writes into registers the silicon ignores. abort() is
 *    that wait, spelled once; every configuring verb refuses while EN
 *    reads 1 rather than store into a protected field. And the SET side
 *    has a slack of its own, measured rather than documented anywhere:
 *    EN reads back 0 in the load that follows its store and 1 in the
 *    next one, so enable() polls its answer instead of trusting one
 *    read.
 *
 *  - THE FIFO IS A CONFIGURATION, NOT A BUFFER THE CALLER SEES. Four
 *    words per stream, a threshold, and table 49's list of which
 *    (memory burst, threshold, memory width) triples the silicon
 *    accepts - the rest raise FEIF AT THE ENABLE and disable the stream
 *    (10.3.18). dma_stream_config_valid() is table 49 as arithmetic
 *    (the threshold in bytes must be a whole number of memory bursts,
 *    and a burst must fit the FIFO), so the forbidden triples are
 *    refused before the enable rather than reported after it.
 *
 *  - MEMORY-TO-MEMORY IS DMA2's ALONE, and no register says so. DMA1's
 *    AHB peripheral port is not connected to the bus matrix (figures 33
 *    and 34, note 1), so a DMA1 stream cannot READ a memory through the
 *    port that would be its source. Refused at compile time where the
 *    controller is a template argument and at run time where the config
 *    is a value.
 *
 *  - A REQUEST IS A LEVEL SERVED ON ENABLE. 10.3.2's handshake is the
 *    G0's: the peripheral drives its request, the controller
 *    acknowledges, the peripheral releases. A stream enabled while its
 *    peripheral's request already stands is served at once, so there is
 *    no software-trigger register on this controller and no verb here
 *    to kick a stalled first beat - measured, because a wrong answer is
 *    a transmitter that never starts.
 *
 * ERRATA. Neither ES0206 (F427/F437/F429/F439) nor ES0298 (F446) has a
 * DMA section: their "DMA" items are the DAC's request behaviour, that
 * chapter's business. Two items do reach this file:
 *  - ES0206 2.2.7 / ES0298 2.2.7, delay after an RCC peripheral clock
 *    enabling: the workaround is a dummy read of the enable register
 *    after the write, and stm32f4/clock.hpp's Rcc::ahb1_clock() already
 *    does exactly that for every caller.
 *  - ES0287 2.2.11 (the F411 alone), DMA2 data corruption when AHB and
 *    APB2 requests are concurrent: a transfer that reaches QUADSPI, the
 *    FSMC or a GPIO register THROUGH THE PERIPHERAL PORT can be
 *    performed several times. The workaround is to put such a
 *    destination on the MEMORY port instead - which this driver can
 *    express, because DIR names which side is the source and the two
 *    ends are separate arguments, but cannot choose: only the
 *    application knows what its addresses are. Stated as an obligation
 *    on the caller.
 */

#pragma once

#include <stdint.h>

#include "stm32f4xx.h"

#include "stm32f4/clock.hpp"
#include "stm32f4/device_tables.hpp"
#include "stm32f4/nvic.hpp"

namespace brio {

// ---- vocabulary ---------------------------------------------------------------

/// SxCR.PSIZE / SxCR.MSIZE (10.5.5): the width of one bus access. The
/// code is not the width - `dma_width_bytes()` is.
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
 * THE ELEMENT TYPE IS THE ACCESS WIDTH. One `sizeof` decides PSIZE,
 * MSIZE and the address arithmetic together, so they cannot disagree.
 *
 * Only the three widths the silicon has are element types: anything else
 * is a compile error at the engine that named it, which is where a
 * programmer can read it.
 */
template <typename Elem>
constexpr DmaWidth dma_width_of() {
    static_assert(sizeof(Elem) == 1 || sizeof(Elem) == 2 || sizeof(Elem) == 4,
                  "a DMA element is one bus access wide: 1, 2 or 4 bytes "
                  "(SxCR.PSIZE/MSIZE have no other code)");
    return sizeof(Elem) == 1   ? DmaWidth::byte
           : sizeof(Elem) == 2 ? DmaWidth::half
                               : DmaWidth::word;
}

/// SxCR.PL (10.5.5): the software half of the arbitration. The hardware
/// half is the stream INDEX and cannot be configured: on equal levels
/// the lower-numbered stream wins (10.3.4).
enum class DmaPriority : uint8_t {
    low = 0,
    medium = 1,
    high = 2,
    very_high = 3,
};

/**
 * SxCR.DIR (10.5.5, table 45). DIR names WHICH SIDE IS THE SOURCE, and
 * the register names of the two ends are fixed whatever it says: SxPAR
 * is the "peripheral" port and SxM0AR the "memory" port even in
 * memory-to-memory, where both are memories and the names mean nothing
 * but which AHB master port the access goes out of. Which is exactly
 * what ES0287 2.2.11's workaround needs a caller to be able to choose.
 */
enum class DmaDirection : uint8_t {
    peripheral_to_memory = 0,   ///< SxPAR is read, SxM0AR written
    memory_to_peripheral = 1,   ///< SxM0AR is read, SxPAR written
    memory_to_memory = 2,       ///< SxPAR is read, SxM0AR written - DMA2 only
};

/// SxCR.MBURST / SxCR.PBURST (10.3.11): how many beats one AHB burst
/// carries. A burst is indivisible - the bus matrix does not degrant the
/// controller inside one - which is what makes packing coherent.
enum class DmaBurst : uint8_t {
    single = 0,
    incr4 = 1,
    incr8 = 2,
    incr16 = 3,
};

constexpr uint8_t dma_burst_beats(DmaBurst b) {
    switch (b) {
        case DmaBurst::single: return 1;
        case DmaBurst::incr4: return 4;
        case DmaBurst::incr8: return 8;
        default: return 16;
    }
}

/// SxFCR.FTH (10.5.10): how full the four-word FIFO gets before it is
/// drained. Unused in direct mode.
enum class DmaFifoThreshold : uint8_t {
    quarter = 0,
    half = 1,
    three_quarters = 2,
    full = 3,
};

/// The threshold in BYTES - the FIFO is four words, so the quarters are
/// 4, 8, 12 and 16 bytes. Table 49's arithmetic is written in these.
constexpr uint8_t dma_fifo_threshold_bytes(DmaFifoThreshold t) {
    return static_cast<uint8_t>(4u * (static_cast<uint8_t>(t) + 1u));
}

/// SxFCR.FS (10.5.10), read-only and meaningless in direct mode.
enum class DmaFifoStatus : uint8_t {
    below_quarter = 0,
    quarter_to_half = 1,
    half_to_three_quarters = 2,
    three_quarters_to_full = 3,
    empty = 4,
    full = 5,
};

/**
 * The five bits one stream owns in DMA_LISR/DMA_HISR and their clears in
 * DMA_LIFCR/DMA_HIFCR (10.5.1 .. 10.5.4). A stream's group is SIX bits
 * wide with bit 1 reserved, and the four groups of a register sit at 0,
 * 6, 16 and 22 - which is why the shift is a table and not a
 * multiplication.
 */
struct DmaFlag {
    DmaFlag() = delete;
    static constexpr uint32_t fifo_error = 1u << 0;         ///< FEIFx
    static constexpr uint32_t direct_mode_error = 1u << 2;  ///< DMEIFx
    static constexpr uint32_t transfer_error = 1u << 3;     ///< TEIFx
    static constexpr uint32_t half = 1u << 4;               ///< HTIFx
    static constexpr uint32_t complete = 1u << 5;           ///< TCIFx
    /// Everything a stream can raise. There is no global bit on this
    /// controller (the STM32G0's GIFx has no counterpart), so the mask is
    /// simply all five.
    static constexpr uint32_t all =
        fifo_error | direct_mode_error | transfer_error | half | complete;
    /// The three that mean something went wrong. TEIF and a FEIF raised
    /// by an illegal burst/threshold pair disable the stream in hardware;
    /// DMEIF and an overrun FEIF do not (10.3.18).
    static constexpr uint32_t errors = fifo_error | direct_mode_error | transfer_error;
};

/**
 * Everything about a stream that is not an address or a length: SxCR's
 * configuration fields and SxFCR's two. EVERY ONE OF THEM IS READ-ONLY
 * WHILE EN IS SET (10.5.5), so configure() refuses rather than store
 * into a register the silicon ignores.
 */
struct DmaStreamConfig {
    uint8_t channel = 0;                  ///< CHSEL: which of the eight request lines
    DmaDirection direction = DmaDirection::peripheral_to_memory;
    bool circular = false;                ///< CIRC: reload and keep going
    bool double_buffer = false;           ///< DBM: swap M0AR/M1AR at every end (forces CIRC)
    bool current_target_is_m1 = false;    ///< CT's initial value, writable only with EN clear
    bool peripheral_flow_control = false; ///< PFCTRL: the peripheral says when it is done
    bool peripheral_increment = false;    ///< PINC
    bool memory_increment = true;         ///< MINC
    bool peripheral_increment_fixed4 = false;  ///< PINCOS: step 4 whatever PSIZE says
    DmaWidth peripheral_width = DmaWidth::byte;   ///< PSIZE
    DmaWidth memory_width = DmaWidth::byte;       ///< MSIZE
    DmaBurst peripheral_burst = DmaBurst::single; ///< PBURST
    DmaBurst memory_burst = DmaBurst::single;     ///< MBURST
    DmaPriority priority = DmaPriority::low;      ///< PL
    /// SxFCR.DMDIS inverted: direct mode is the reset state and the
    /// FIFO's threshold is unused in it.
    bool use_fifo = false;
    DmaFifoThreshold fifo_threshold = DmaFifoThreshold::half;
};

/**
 * Table 49 as arithmetic, plus the note under it. A memory burst carries
 * `beats x MSIZE` bytes and the FIFO drains at the threshold, so THE
 * THRESHOLD MUST BE A WHOLE NUMBER OF BURSTS - and a burst must fit in
 * the four-word FIFO at all. Those two conditions reproduce every cell
 * of the table, "forbidden" ones included, which is why the table is not
 * copied here as data.
 *
 * The second clause is the note's own: with (PBURST x PSIZE) equal to the
 * whole FIFO, a 3/4 threshold leaves too little room and underruns or
 * overruns for ever.
 */
constexpr bool dma_fifo_burst_valid(DmaFifoThreshold threshold, DmaBurst memory_burst,
                                    DmaWidth memory_width, DmaBurst peripheral_burst,
                                    DmaWidth peripheral_width) {
    const uint32_t fifo_bytes = 4u * dma_fifo_words;
    const uint32_t level = dma_fifo_threshold_bytes(threshold);
    const uint32_t mchunk =
        static_cast<uint32_t>(dma_burst_beats(memory_burst)) * dma_width_bytes(memory_width);
    if (memory_burst != DmaBurst::single) {
        if (mchunk > fifo_bytes || (level % mchunk) != 0u) {
            return false;
        }
    }
    const uint32_t pchunk = static_cast<uint32_t>(dma_burst_beats(peripheral_burst)) *
                            dma_width_bytes(peripheral_width);
    if (peripheral_burst != DmaBurst::single) {
        if (pchunk > fifo_bytes) {
            return false;
        }
        if (pchunk == fifo_bytes && threshold == DmaFifoThreshold::three_quarters) {
            return false;
        }
    }
    return true;
}

/**
 * Every rule of table 50 and of the paragraphs it summarizes, as one
 * predicate. `controller` is needed because memory-to-memory is a
 * property of the wiring and not of the register.
 */
constexpr bool dma_stream_config_valid(const DmaStreamConfig& c, uint8_t controller) {
    if (c.channel >= dma_channels) {
        return false;
    }
    const bool mem2mem = c.direction == DmaDirection::memory_to_memory;
    if (mem2mem) {
        // Figures 33/34 note 1: DMA1's peripheral port does not reach the
        // bus matrix. 10.3.6: circular and direct modes are not allowed,
        // and 10.3.9's table 46 forbids the double buffer with it.
        if (!dma_memory_to_memory_capable(controller)) {
            return false;
        }
        if (c.circular || c.double_buffer || !c.use_fifo || c.peripheral_flow_control) {
            return false;
        }
    }
    // 10.3.15: "The Circular mode is forbidden in the peripheral flow
    // controller mode" - and the double buffer forces circular.
    if (c.peripheral_flow_control && (c.circular || c.double_buffer)) {
        return false;
    }
    if (!c.use_fifo) {
        // 10.3.12, direct mode: the two widths are equal and defined by
        // PSIZE (MSIZE is forced by hardware), and neither port may burst.
        // Refused rather than silently corrected, so what the caller wrote
        // is what the stream does.
        if (c.memory_width != c.peripheral_width) {
            return false;
        }
        if (c.memory_burst != DmaBurst::single || c.peripheral_burst != DmaBurst::single) {
            return false;
        }
        return true;
    }
    return dma_fifo_burst_valid(c.fifo_threshold, c.memory_burst, c.memory_width,
                                c.peripheral_burst, c.peripheral_width);
}

/// One block transfer: both ends, the double buffer's second memory, and
/// how many data items. NDT counts items of the PERIPHERAL-port width
/// (10.3.10), whichever side the peripheral port is.
struct DmaTransfer {
    volatile void* peripheral = nullptr;   ///< SxPAR
    volatile void* memory = nullptr;       ///< SxM0AR
    volatile void* memory1 = nullptr;      ///< SxM1AR, the double buffer's other half
    uint16_t count = 0;                    ///< SxNDTR, in peripheral-width items
    DmaStreamConfig config{};
};

/// The alignment 10.3.6 demands: an address is aligned on the width of
/// the accesses made through it.
constexpr bool dma_address_aligned(uint32_t address, DmaWidth w) {
    return (address & (dma_width_bytes(w) - 1u)) == 0u;
}

/**
 * Table 48 and the circular-mode note of 10.3.8, as one predicate over a
 * whole transfer.
 *
 *  - a null end, or a zero count where the DMA is the flow controller,
 *    moves nothing (10.5.6: "If the value of this register is zero, no
 *    transaction can be served even if the stream is enabled");
 *  - both addresses are aligned on the width of the accesses through
 *    them - in direct mode both ports use PSIZE;
 *  - table 48: with PSIZE below MSIZE the last memory access would be
 *    incomplete unless NDT is a multiple of MSIZE/PSIZE;
 *  - 10.3.8: under CIRC with a memory burst, NDT x PSIZE must be a whole
 *    number of memory bursts, and NDT a whole number of peripheral burst
 *    beats - "if this formula is not respected, the DMA behavior and data
 *    integrity are not guaranteed";
 *  - a double buffer needs its second memory.
 */
constexpr bool dma_transfer_valid(const DmaTransfer& t, uint8_t controller) {
    if (!dma_stream_config_valid(t.config, controller)) {
        return false;
    }
    if (t.peripheral == nullptr || t.memory == nullptr) {
        return false;
    }
    if (t.config.double_buffer && t.memory1 == nullptr) {
        return false;
    }
    if (t.count == 0u && !t.config.peripheral_flow_control) {
        return false;
    }
    const DmaWidth pw = t.config.peripheral_width;
    const DmaWidth mw = t.config.use_fifo ? t.config.memory_width : t.config.peripheral_width;
    if (!dma_address_aligned(static_cast<uint32_t>(reinterpret_cast<uintptr_t>(t.peripheral)), pw)) {
        return false;
    }
    if (!dma_address_aligned(static_cast<uint32_t>(reinterpret_cast<uintptr_t>(t.memory)), mw)) {
        return false;
    }
    if (t.memory1 != nullptr &&
        !dma_address_aligned(static_cast<uint32_t>(reinterpret_cast<uintptr_t>(t.memory1)), mw)) {
        return false;
    }
    const uint32_t pb = dma_width_bytes(pw);
    const uint32_t mb = dma_width_bytes(mw);
    if (mb > pb && (t.count % (mb / pb)) != 0u) {
        return false;   // table 48
    }
    if (t.config.circular || t.config.double_buffer) {
        const uint32_t chunk =
            static_cast<uint32_t>(dma_burst_beats(t.config.memory_burst)) * mb;
        if (((static_cast<uint32_t>(t.count) * pb) % chunk) != 0u) {
            return false;
        }
        if ((t.count % dma_burst_beats(t.config.peripheral_burst)) != 0u) {
            return false;
        }
    }
    return true;
}

/// How far a running stream has got. The reading costs nothing and is
/// never refused: SxNDTR is a live register the controller decrements
/// and software may read at any time (10.5.6).
struct DmaProgress {
    uint16_t remaining = 0;   ///< data items still to move
    uint16_t done = 0;        ///< data items moved in the current block
};

/// What NDTR holds once a peripheral-flow-controlled stream is enabled -
/// "whatever the value written, it is forced by hardware to 0xFFFF"
/// (10.3.15), and the count transferred is that minus what remains.
inline constexpr uint16_t dma_flow_control_count = 0xFFFFu;

// ---- the block ----------------------------------------------------------------

/**
 * Dma<n>: one controller. Four registers of its own - LISR, HISR and
 * their write-one-to-clear twins - a clock, a reset; everything else
 * belongs to a stream.
 *
 * The gate is opened by the STREAM's configuring verbs as well, the way
 * stm32f4/pin.hpp's port clock is: a caller cannot forget it and a
 * program that never configures a stream never pays for it. init() is
 * for the case where the block is wanted before any stream is - reading
 * the flag banks, for instance, which does need the clock.
 */
template <uint8_t n>
struct Dma {
    static_assert(dma_present(n),
                  "brio Dma: this device has no such DMA controller (the device header "
                  "declares no DMAn_BASE for it; this family has DMA1 and DMA2)");

    Dma() = delete;

    static constexpr uint8_t instance = n;
    static constexpr uint8_t streams = dma_streams;
    /// Only DMA2's peripheral port reaches the bus matrix (10.3.1).
    static constexpr bool memory_to_memory = dma_memory_to_memory_capable(n);

    static DMA_TypeDef& regs() { return *reinterpret_cast<DMA_TypeDef*>(dma_base(n)); }

    /// The controller's AHB1 clock. Rcc::ahb1_clock() reads the register
    /// back after the store, which is ES0206 2.2.7's workaround.
    static void bus_clock(bool on) { Rcc::ahb1_clock(dma_clock_mask(n), on); }
    static bool bus_clock() { return Rcc::ahb1_clock(dma_clock_mask(n)); }

    /// Open the gate and leave every register at its reset value.
    static void init() {
        bus_clock(true);
        reset();
    }

    /// Every register of THIS CONTROLLER back to its reset value, through
    /// the RCC's reset line. The other controller is untouched.
    static void reset() {
        bus_clock(true);
        Rcc::ahb1_reset(dma_reset_mask(n));
    }

    /// The four groups of DMA_LISR (streams 0..3) and DMA_HISR (4..7).
    static uint32_t low_flags() { return regs().LISR; }
    static uint32_t high_flags() { return regs().HISR; }

    /// Where stream `s`'s six-bit group starts in its register. The four
    /// groups do not tile: bit 1 of each is reserved and the upper half
    /// starts at 16, so this is a table (10.5.1).
    static constexpr uint32_t flag_shift(uint8_t s) {
        constexpr uint32_t shifts[4] = {0u, 6u, 16u, 22u};
        return shifts[s & 3u];
    }

    /// Stream `s`'s five flags, shifted down to DmaFlag's positions.
    static uint32_t stream_flags(uint8_t s) {
        const uint32_t bank = s < 4u ? regs().LISR : regs().HISR;
        return (bank >> flag_shift(s)) & DmaFlag::all;
    }

    /// Clear some of stream `s`'s flags. The clear registers are
    /// write-only and every bit is independent, so this is one store and
    /// never a read-modify-write - which is what makes it safe to call
    /// from a handler while another stream's owner does the same.
    static void clear(uint8_t s, uint32_t mask) {
        const uint32_t bits = (mask & DmaFlag::all) << flag_shift(s);
        if (s < 4u) {
            regs().LIFCR = bits;
        } else {
            regs().HIFCR = bits;
        }
    }

    /// The NVIC line stream `s` reports on - one vector per stream on
    /// this family, shared with nothing.
    static constexpr IRQn_Type irq(uint8_t s) { return dma_stream_irq(n, s); }
};

// ---- the stream ---------------------------------------------------------------

/**
 * DmaStream<n, s>: one stream of controller n, numbered as the silicon
 * numbers it (0..7).
 *
 * THE ENABLE DISCIPLINE IS THE CHAPTER'S, and it is stricter than the
 * STM32G0's. 10.3.17 step 1: clear EN, READ IT BACK until it is 0 -
 * "writing this bit to 0 is not immediately effective" - and clear every
 * flag the previous block left before setting EN again, or the enable
 * raises an interrupt at once. load() is that sequence, and configure()
 * refuses outright while EN reads 1, because SxCR's fields, SxFCR's two,
 * SxPAR, SxNDTR and (outside the double buffer) both memory addresses are
 * write-protected then.
 *
 * SUSPEND AND RESUME IS SUPPORTED HERE, unlike on the G0's controller,
 * and it is the caller's to drive: 10.3.14 says to read SxNDTR after the
 * disable has come down, adjust the addresses, write the remainder and
 * enable again. progress() and the address setters are what that needs;
 * the driver offers no verb that pretends to do it, because only the
 * caller knows how its addresses advance.
 */
template <uint8_t n, uint8_t s>
class DmaStream {
public:
    static_assert(dma_present(n), "brio DmaStream: this device has no such DMA controller");
    static_assert(dma_stream_present(n, s),
                  "brio DmaStream: a controller of this family has EIGHT streams, "
                  "numbered 0..7");

    DmaStream() = delete;

    using Block = Dma<n>;
    static constexpr uint8_t controller = n;
    static constexpr uint8_t index = s;
    /// Only a DMA2 stream can move memory to memory (10.3.1).
    static constexpr bool memory_to_memory_capable = dma_memory_to_memory_capable(n);

    static DMA_Stream_TypeDef& regs() {
        return *reinterpret_cast<DMA_Stream_TypeDef*>(dma_stream_base(n, s));
    }

    static constexpr IRQn_Type irq() { return dma_stream_irq(n, s); }

    // -- enable ------------------------------------------------------------

    static bool enabled() { return (regs().CR & DMA_SxCR_EN) != 0u; }

    /**
     * Set EN in a write of its own (10.3.17 step 10, after everything
     * else). Returns whether the silicon TOOK the enable: it clears EN
     * again, and raises FEIF, on a FIFO threshold the memory burst does
     * not divide (10.3.18), which is the failure this answer is for.
     *
     * AND THE STORE IS NOT VISIBLE TO THE LOAD THAT FOLLOWS IT.
     * Measured (docs/stm32f4/dma.md): EN reads back 0 in the load right
     * after the store and 1 in the one after that - one read of slack,
     * and EN's alone, because a store into SxNDTR is visible to the very
     * next load. A read-back taken at once therefore reports a perfectly
     * good stream as a refused one, so the answer is polled over a few
     * reads.
     *
     * A COMPLETION IS AS GOOD AN ANSWER AS A STANDING EN, for a second
     * measured reason: a memory-to-memory block of a few hundred bytes
     * can be over before the poll ends (256 bytes take under two
     * microseconds at 180 MHz).
     */
    static bool enable() {
        regs().CR = regs().CR | DMA_SxCR_EN;
        for (uint8_t read = 0; read < 4u; ++read) {
            if (enabled() || flag(DmaFlag::complete)) {
                return true;
            }
        }
        return false;
    }

    /**
     * Clear EN AND WAIT FOR IT TO READ BACK 0 - the one verb that makes
     * 10.3.14 usable. The wait is not a formality: on a
     * peripheral-to-memory or memory-to-memory stream the controller
     * finishes the current transfer and FLUSHES THE FIFO into the
     * destination first, and only then does EN come down (and TCIF go
     * up, which is why a caller that reads flags after this sees a
     * completion that is really an abort).
     *
     * Bounded, because a stream whose peripheral has stopped answering
     * must not hang the program: false means EN was still set after
     * `spins` reads, and the stream is then not safe to reconfigure.
     */
    static bool abort(uint32_t spins = 200'000u) {
        regs().CR = regs().CR & ~DMA_SxCR_EN;
        while (enabled()) {
            if (spins-- == 0u) {
                return false;
            }
        }
        return true;
    }

    // -- configuration -----------------------------------------------------

    /**
     * Write SxCR's configuration fields and SxFCR. REFUSED while the
     * stream is enabled (every field is read-only then, 10.5.5) and
     * refused for a combination table 50 or table 49 forbids - nothing
     * is written in either case.
     *
     * The interrupt enables are NOT touched: arm() owns them, so a
     * re-configuration cannot disarm a handler by omission. They live in
     * two registers (FEIE in SxFCR, the other four in SxCR), which is why
     * both are read-modify-written here.
     */
    static bool configure(const DmaStreamConfig& c) {
        if (enabled() || !dma_stream_config_valid(c, n)) {
            return false;
        }
        Block::bus_clock(true);
        const uint32_t armed_cr = regs().CR & (DMA_SxCR_TCIE | DMA_SxCR_HTIE | DMA_SxCR_TEIE |
                                               DMA_SxCR_DMEIE);
        const uint32_t armed_fcr = regs().FCR & DMA_SxFCR_FEIE;

        uint32_t fcr = armed_fcr;
        if (c.use_fifo) {
            fcr |= DMA_SxFCR_DMDIS;
            fcr |= static_cast<uint32_t>(c.fifo_threshold) << DMA_SxFCR_FTH_Pos;
        }
        regs().FCR = fcr;

        uint32_t cr = armed_cr;
        cr |= static_cast<uint32_t>(c.channel) << DMA_SxCR_CHSEL_Pos;
        cr |= static_cast<uint32_t>(c.memory_burst) << DMA_SxCR_MBURST_Pos;
        cr |= static_cast<uint32_t>(c.peripheral_burst) << DMA_SxCR_PBURST_Pos;
        if (c.current_target_is_m1) {
            cr |= DMA_SxCR_CT;
        }
        if (c.double_buffer) {
            cr |= DMA_SxCR_DBM;
        }
        cr |= static_cast<uint32_t>(c.priority) << DMA_SxCR_PL_Pos;
        if (c.peripheral_increment_fixed4) {
            cr |= DMA_SxCR_PINCOS;
        }
        cr |= static_cast<uint32_t>(c.memory_width) << DMA_SxCR_MSIZE_Pos;
        cr |= static_cast<uint32_t>(c.peripheral_width) << DMA_SxCR_PSIZE_Pos;
        if (c.memory_increment) {
            cr |= DMA_SxCR_MINC;
        }
        if (c.peripheral_increment) {
            cr |= DMA_SxCR_PINC;
        }
        if (c.circular) {
            cr |= DMA_SxCR_CIRC;
        }
        cr |= static_cast<uint32_t>(c.direction) << DMA_SxCR_DIR_Pos;
        if (c.peripheral_flow_control) {
            cr |= DMA_SxCR_PFCTRL;
        }
        regs().CR = cr;
        return true;
    }

    static uint32_t control() { return regs().CR; }
    static uint32_t fifo_control() { return regs().FCR; }

    /// The mode bits a caller may want to ASK about rather than remember
    /// - and what a test says "this stream really is circular" with,
    /// without spelling a register bit.
    static uint8_t channel() {
        return static_cast<uint8_t>((regs().CR & DMA_SxCR_CHSEL_Msk) >> DMA_SxCR_CHSEL_Pos);
    }
    static bool circular() { return (regs().CR & DMA_SxCR_CIRC) != 0u; }
    static bool double_buffer() { return (regs().CR & DMA_SxCR_DBM) != 0u; }
    static bool flow_controlled() { return (regs().CR & DMA_SxCR_PFCTRL) != 0u; }
    /// SxFCR.DMDIS inverted: true while the four-word FIFO is bypassed.
    static bool direct_mode() { return (regs().FCR & DMA_SxFCR_DMDIS) == 0u; }
    static DmaDirection direction() {
        return static_cast<DmaDirection>((regs().CR & DMA_SxCR_DIR_Msk) >> DMA_SxCR_DIR_Pos);
    }
    static DmaFifoStatus fifo_status() {
        return static_cast<DmaFifoStatus>((regs().FCR & DMA_SxFCR_FS_Msk) >> DMA_SxFCR_FS_Pos);
    }

    /// SxNDTR, writable only with the stream disabled (10.5.6).
    static bool set_count(uint16_t count) {
        if (enabled()) {
            return false;
        }
        regs().NDTR = count;
        return true;
    }
    static uint16_t count() { return static_cast<uint16_t>(regs().NDTR); }

    static bool set_peripheral(volatile void* address) {
        if (enabled()) {
            return false;
        }
        regs().PAR = static_cast<uint32_t>(reinterpret_cast<uintptr_t>(address));
        return true;
    }
    static bool set_memory(volatile void* address) {
        if (enabled()) {
            return false;
        }
        regs().M0AR = static_cast<uint32_t>(reinterpret_cast<uintptr_t>(address));
        return true;
    }
    static bool set_memory1(volatile void* address) {
        if (enabled()) {
            return false;
        }
        regs().M1AR = static_cast<uint32_t>(reinterpret_cast<uintptr_t>(address));
        return true;
    }

    // -- the double buffer -------------------------------------------------

    /// CT (10.5.5): which memory the running stream is using now. Written
    /// by hardware at every swap; readable at any time, and the only way
    /// to know which half is the caller's.
    static bool current_target_is_m1() { return (regs().CR & DMA_SxCR_CT) != 0u; }

    /**
     * Replace the half the stream is NOT using, WITHOUT stopping it -
     * the one write into an address register that a running stream
     * allows (10.3.9). The rule is exact and its penalty is loud: with
     * CT = 0 only M1AR may be written and with CT = 1 only M0AR, and the
     * wrong one "sets an error flag (TEIF) and the stream is
     * automatically disabled". So this verb takes the address for the
     * IDLE half, reads CT and writes whichever register that is - and
     * refuses on a stream that is not in double-buffer mode, where both
     * registers are simply protected.
     *
     * The chapter's own advice about WHEN: as soon as TCIF is asserted,
     * because the target has just changed and the window is widest.
     */
    static bool set_idle_buffer(volatile void* address) {
        if (!double_buffer()) {
            return false;
        }
        const uint32_t v = static_cast<uint32_t>(reinterpret_cast<uintptr_t>(address));
        if (current_target_is_m1()) {
            regs().M0AR = v;
        } else {
            regs().M1AR = v;
        }
        return true;
    }

    // -- loading -----------------------------------------------------------

    /**
     * The whole configuration procedure of 10.3.17 up to but NOT
     * including the enable: stop the stream, wait for EN to come down,
     * clear every flag the last block left, then the two or three
     * addresses, the count, SxFCR and SxCR.
     *
     * Separate from load() because a stream whose peripheral must be
     * armed first (the peripheral's DMA request bit set only once
     * everything is in place) wants exactly this.
     */
    static bool prepare(const DmaTransfer& t) {
        if (!dma_transfer_valid(t, n)) {
            return false;
        }
        Block::bus_clock(true);
        if (!abort()) {
            return false;
        }
        Block::clear(s, DmaFlag::all);
        if (!set_peripheral(t.peripheral) || !set_memory(t.memory)) {
            return false;
        }
        if (t.memory1 != nullptr && !set_memory1(t.memory1)) {
            return false;
        }
        if (!set_count(t.count)) {
            return false;
        }
        return configure(t.config);
    }

    /// prepare() and then the enable - the sequence that leaves the
    /// stream RUNNING. False when anything in the transfer was refused or
    /// when EN did not stay up.
    static bool load(const DmaTransfer& t) { return prepare(t) && enable(); }

    /// Start a stream that has been prepared. A separate verb because on
    /// this controller the enable is a step of its own and a peripheral
    /// may have to be armed between the two.
    static bool trigger() { return enable(); }

    // -- flags and interrupts ----------------------------------------------

    /// This stream's five flags, at DmaFlag's positions.
    static uint32_t flags() { return Block::stream_flags(s); }
    static bool flag(uint32_t mask) { return (flags() & mask) != 0u; }
    static void clear(uint32_t mask) { Block::clear(s, mask); }

    /**
     * The five interrupt enables. FOUR OF THEM ARE IN SxCR AND ONE IN
     * SxFCR (FEIE), which is the only reason this is not one store.
     * Read-modify-write under a guard: a handler may be touching SxCR
     * through load().
     *
     * 10.4's note is the caller's to respect and load() does respect it:
     * clear a flag before arming its enable, or the enable raises an
     * interrupt at once.
     */
    static void arm(uint32_t mask, bool on) {
        uint32_t cr_bits = 0;
        if ((mask & DmaFlag::complete) != 0u) {
            cr_bits |= DMA_SxCR_TCIE;
        }
        if ((mask & DmaFlag::half) != 0u) {
            cr_bits |= DMA_SxCR_HTIE;
        }
        if ((mask & DmaFlag::transfer_error) != 0u) {
            cr_bits |= DMA_SxCR_TEIE;
        }
        if ((mask & DmaFlag::direct_mode_error) != 0u) {
            cr_bits |= DMA_SxCR_DMEIE;
        }
        InterruptGuard guard;
        if (cr_bits != 0u) {
            regs().CR = on ? (regs().CR | cr_bits) : (regs().CR & ~cr_bits);
        }
        if ((mask & DmaFlag::fifo_error) != 0u) {
            regs().FCR = on ? (regs().FCR | DMA_SxFCR_FEIE) : (regs().FCR & ~DMA_SxFCR_FEIE);
        }
    }

    static uint32_t armed() {
        const uint32_t cr = regs().CR;
        uint32_t mask = 0;
        if ((cr & DMA_SxCR_TCIE) != 0u) {
            mask |= DmaFlag::complete;
        }
        if ((cr & DMA_SxCR_HTIE) != 0u) {
            mask |= DmaFlag::half;
        }
        if ((cr & DMA_SxCR_TEIE) != 0u) {
            mask |= DmaFlag::transfer_error;
        }
        if ((cr & DMA_SxCR_DMEIE) != 0u) {
            mask |= DmaFlag::direct_mode_error;
        }
        if ((regs().FCR & DMA_SxFCR_FEIE) != 0u) {
            mask |= DmaFlag::fifo_error;
        }
        return mask;
    }

    /**
     * The stream's ISR BODY: read this stream's own flags, keep the
     * ARMED ones, clear exactly those, hand them back.
     *
     * One vector per stream on this family, so a body never has to ask
     * "was it me" - but only armed flags are reported and cleared, for
     * the G0's reason: HTIF is set by hardware whether or not HTIE is on,
     * and a body that swallowed it would consume a flag its owner polls
     * for.
     */
    [[gnu::always_inline]] static uint32_t isr() {
        const uint32_t pending = flags() & armed();
        if (pending != 0u) {
            clear(pending);
        }
        return pending;
    }

    // -- progress ----------------------------------------------------------

    /**
     * Where the current block has got to. `programmed` is the count the
     * caller wrote - or dma_flow_control_count on a peripheral-flow-
     * controlled stream, where the silicon forced NDTR to 0xFFFF and
     * 10.3.15's formula is exactly this subtraction.
     */
    static DmaProgress progress(uint16_t programmed) {
        const uint16_t remaining = count();
        return DmaProgress{remaining, static_cast<uint16_t>(remaining > programmed
                                                                ? 0u
                                                                : programmed - remaining)};
    }

    /// Stop the stream and put its flags and interrupt enables back.
    /// After this the stream is safe to hand to another owner.
    static void stop() {
        Block::bus_clock(true);
        (void)abort();
        arm(DmaFlag::all, false);
        clear(DmaFlag::all);
    }
};

// ---- the byte-transport engines ------------------------------------------------

/**
 * DmaTxEngine<n, s, ch, Elem> - "drain a run of memory into a
 * peripheral's data register".
 *
 * The engine owns a stream and nothing else: the peripheral's data
 * address is handed in at arm() time by whoever owns the peripheral, so
 * this type knows nothing about USARTs and would serve an SPI or a DAC
 * unchanged. The CHANNEL, though, is a template argument and not an
 * arm() one - on this controller the (stream, channel) pair IS the
 * peripheral's wiring, so it belongs where the type is written and where
 * a static_assert can check it (stm32f4/usart.hpp does exactly that).
 *
 * `Elem` IS THE ACCESS WIDTH (dma_width_of): the default byte engine is
 * what a USART wants; a converter's data register wants uint16_t and
 * nothing else in the engine changes.
 *
 * DIRECT MODE, NOT THE FIFO, and the reason is the contract above it. A
 * byte transport hands over runs whose length it does not choose, into a
 * register one byte wide; the FIFO would buy burst efficiency at the
 * price of table 49's constraints on every run length. 10.3.12's direct
 * mode is what "an immediate and single transfer after each DMA request"
 * means, and it is what these two engines use.
 *
 * THERE IS NO VERB TO KICK A STALLED FIRST BEAT, and the absence is a
 * fact: 10.3.2's handshake is level-driven, so a stream enabled with TXE
 * already standing moves the first byte at once.
 */
template <uint8_t n, uint8_t s, uint8_t ch, typename Elem = uint8_t>
class DmaTxEngine {
    using Stream = DmaStream<n, s>;

public:
    static_assert(ch < dma_channels, "brio DmaTxEngine: CHSEL selects one of eight channels");
    static_assert(sizeof(Elem) == 1 || sizeof(Elem) == 2 || sizeof(Elem) == 4,
                  "brio DmaTxEngine: the element type is the bus access - 1, 2 or 4 bytes");

    DmaTxEngine() = delete;

    static constexpr bool present = true;
    static constexpr uint8_t controller = n;
    static constexpr uint8_t stream = s;
    static constexpr uint8_t channel = ch;
    static constexpr DmaWidth width = dma_width_of<Elem>();
    using element = Elem;

    /// The bits service() reports, published BY THE ENGINE so a driver
    /// that owns one never has to name DmaFlag - or include this file's
    /// stream types - to read the answer.
    static constexpr uint8_t flag_complete = DmaFlag::complete;
    static constexpr uint8_t flag_half = DmaFlag::half;
    static constexpr uint8_t flag_error = DmaFlag::transfer_error;
    static constexpr uint8_t flag_fifo_error = DmaFlag::fifo_error;

    /**
     * The stream's ISR BODY folded into the engine: read this stream's
     * own armed flags, clear exactly those, hand them back. One vector
     * per stream here, so it answers for itself alone.
     *
     * IT ACTS ON NOTHING. What a completion or an error MEANS is the
     * owner's to decide - complete(), abandon() are the verbs - because
     * only the owner can see its peripheral's state.
     */
    [[gnu::always_inline]] static uint8_t service() {
        return static_cast<uint8_t>(Stream::isr());
    }

    /// Claim the stream for this peripheral: `data` is the register the
    /// run is poured into.
    static void arm(volatile void* data, DmaPriority priority = DmaPriority::low) {
        data_ = data;
        priority_ = priority;
        claim();
        Nvic::enable(Stream::irq());
    }

    /// Start moving `length` elements from `buffer`. The buffer is the
    /// CALLER'S and must stay put until complete() reports the block.
    static bool start(const Elem* buffer, uint16_t length) {
        if (busy_ || buffer == nullptr || length == 0u) {
            return false;
        }
        in_flight_ = length;
        busy_ = true;
        if (!Stream::load(transfer(const_cast<Elem*>(buffer), length, true))) {
            busy_ = false;
            in_flight_ = 0;
            return false;
        }
        return true;
    }

    /**
     * The same block from ONE CELL, sent `length` times: the memory
     * pointer does not increment. What a full-duplex bus needs for the
     * transmit side of a READ - the clock has to run, so something must
     * be shifted out, and the dummy is one element of the caller's that
     * stays put for the whole block.
     *
     * A SIBLING VERB and not a defaulted argument to start(): a defaulted
     * argument moves the images of every caller that does not pass it,
     * where a sibling leaves them byte-identical.
     */
    static bool start_fixed(const Elem* cell, uint16_t length) {
        if (busy_ || cell == nullptr || length == 0u) {
            return false;
        }
        in_flight_ = length;
        busy_ = true;
        if (!Stream::load(transfer(const_cast<Elem*>(cell), length, false))) {
            busy_ = false;
            in_flight_ = 0;
            return false;
        }
        return true;
    }

    /// The block ended - called from the stream's handler when its
    /// completion flag is up. Returns the elements the block carried, so
    /// the owner can release exactly that much of its ring.
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
    static DmaProgress progress() { return Stream::progress(in_flight_); }

    /**
     * Throw away a block the silicon has stopped running and free the
     * stream. THE CALLER DECIDES, because only the peripheral's owner can
     * read the flags that make "dead" a fact rather than a timeout. What
     * is lost is the untransmitted tail, and it is counted rather than
     * papered over.
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
        Stream::stop();
        busy_ = false;
        in_flight_ = 0;
    }

private:
    static DmaTransfer transfer(Elem* buffer, uint16_t length, bool increment) {
        return DmaTransfer{
            .peripheral = data_,
            .memory = buffer,
            .memory1 = nullptr,
            .count = length,
            .config = {.channel = ch,
                       .direction = DmaDirection::memory_to_peripheral,
                       .circular = false,
                       .double_buffer = false,
                       .current_target_is_m1 = false,
                       .peripheral_flow_control = false,
                       .peripheral_increment = false,
                       .memory_increment = increment,
                       .peripheral_increment_fixed4 = false,
                       .peripheral_width = width,
                       .memory_width = width,
                       .peripheral_burst = DmaBurst::single,
                       .memory_burst = DmaBurst::single,
                       .priority = priority_,
                       .use_fifo = false,
                       .fifo_threshold = DmaFifoThreshold::half},
        };
    }

    /// Take the stream from whatever state it is in and arm the two flags
    /// that matter. arm() and abandon() are the same act with a different
    /// reason.
    static void claim() {
        Stream::stop();
        Stream::arm(DmaFlag::complete | DmaFlag::transfer_error, true);
    }

    static inline volatile void* data_ = nullptr;
    static inline DmaPriority priority_ = DmaPriority::low;
    static inline uint16_t in_flight_ = 0;
    static inline uint32_t faults_ = 0;
    static inline volatile bool busy_ = false;
};

/**
 * DmaRxEngine<n, s, ch, Elem> - "fill a run of memory from a
 * peripheral's data register".
 *
 * The asymmetry that shapes it: a receive block completes only when the
 * buffer FILLS, which on an idle line may be never. So the owner does
 * not wait for a completion - it ASKS, with take(), which reports what
 * has arrived since the last question.
 *
 * AND ON THIS SILICON ASKING IS FREE. SxNDTR is a live counter the
 * controller decrements and software may read at any time (10.5.6), so
 * take() is one register read and a subtraction: nothing is suspended,
 * no write-back has to be judged. The PACING is still the owner's (a
 * kernel TimeEvent every few ticks), because the latency of asking late
 * is the owner's to choose; what is gone is the cost of asking.
 */
template <uint8_t n, uint8_t s, uint8_t ch, typename Elem = uint8_t>
class DmaRxEngine {
    using Stream = DmaStream<n, s>;

public:
    static_assert(ch < dma_channels, "brio DmaRxEngine: CHSEL selects one of eight channels");
    static_assert(sizeof(Elem) == 1 || sizeof(Elem) == 2 || sizeof(Elem) == 4,
                  "brio DmaRxEngine: the element type is the bus access - 1, 2 or 4 bytes");

    DmaRxEngine() = delete;

    static constexpr bool present = true;
    static constexpr uint8_t controller = n;
    static constexpr uint8_t stream = s;
    static constexpr uint8_t channel = ch;
    static constexpr DmaWidth width = dma_width_of<Elem>();
    using element = Elem;

    static constexpr uint8_t flag_complete = DmaFlag::complete;
    static constexpr uint8_t flag_half = DmaFlag::half;
    static constexpr uint8_t flag_error = DmaFlag::transfer_error;
    static constexpr uint8_t flag_fifo_error = DmaFlag::fifo_error;

    [[gnu::always_inline]] static uint8_t service() {
        return static_cast<uint8_t>(Stream::isr());
    }

    static void arm(volatile void* data, DmaPriority priority = DmaPriority::low) {
        data_ = data;
        priority_ = priority;
        claim();
        Nvic::enable(Stream::irq());
    }

    /// True while the stream is not running a block at all - it filled
    /// up, it errored, or it was never started. The owner hands it a new
    /// run then, whatever the arithmetic says.
    static bool idle() { return !Stream::enabled(); }

    /// Point the stream at a run of free memory and start filling it.
    static bool start(Elem* buffer, uint16_t length) {
        if (buffer == nullptr || length == 0u) {
            return false;
        }
        capacity_ = length;
        taken_ = 0;
        return Stream::load(transfer(buffer, length, true));
    }

    /**
     * `length` elements into ONE CELL, thrown away: the memory pointer
     * does not increment. What a full-duplex bus needs for the receive
     * side of a WRITE - every frame clocked out brings one back, and a
     * receiver left unread would overrun.
     */
    static bool start_discard(Elem* cell, uint16_t length) {
        if (cell == nullptr || length == 0u) {
            return false;
        }
        capacity_ = length;
        taken_ = 0;
        return Stream::load(transfer(cell, length, false));
    }

    /// How many elements have arrived since the last take(). One NDTR
    /// read; nothing is suspended and nothing can be refused, so the
    /// return is a plain number and not an optional.
    static uint16_t take() {
        if (capacity_ == 0u) {
            return 0;
        }
        const uint16_t remaining = Stream::count();
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
        Stream::stop();
        capacity_ = 0;
        taken_ = 0;
    }

private:
    static DmaTransfer transfer(Elem* buffer, uint16_t length, bool increment) {
        return DmaTransfer{
            .peripheral = data_,
            .memory = buffer,
            .memory1 = nullptr,
            .count = length,
            .config = {.channel = ch,
                       .direction = DmaDirection::peripheral_to_memory,
                       .circular = false,
                       .double_buffer = false,
                       .current_target_is_m1 = false,
                       .peripheral_flow_control = false,
                       .peripheral_increment = false,
                       .memory_increment = increment,
                       .peripheral_increment_fixed4 = false,
                       .peripheral_width = width,
                       .memory_width = width,
                       .peripheral_burst = DmaBurst::single,
                       .memory_burst = DmaBurst::single,
                       .priority = priority_,
                       .use_fifo = false,
                       .fifo_threshold = DmaFifoThreshold::half},
        };
    }

    static void claim() {
        Stream::stop();
        Stream::arm(DmaFlag::complete | DmaFlag::transfer_error, true);
    }

    static inline volatile void* data_ = nullptr;
    static inline DmaPriority priority_ = DmaPriority::low;
    static inline uint16_t capacity_ = 0;
    static inline uint16_t taken_ = 0;
    static inline uint32_t faults_ = 0;
};

} // namespace brio

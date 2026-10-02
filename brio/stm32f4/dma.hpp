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
 *                         the two TASKS a transport wants: drain a run
 *                         into a peripheral, fill a run from one - the
 *                         slots stm32f4/usart.hpp's Uart and the bus
 *                         hosts declare, bound once and started with four
 *                         stores a block, the beat the run's own element;
 *                         and the receive engine's second shape, a ring
 *                         filled lap after lap in circular mode, whose
 *                         producer index is SxNDTR itself;
 *   DmaCopyEngine         the TASK memory wants: copy and fill, memory to
 *                         memory on a DMA2 stream, the beat and the bursts
 *                         chosen per block from its alignment.
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
 *    next one - and on the STM32F446 and the STM32F411 that load can
 *    wedge the stream - so nothing in this file reads SxCR after an EN
 *    store: enable() and the engines' starts store and move on.
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

#include <stddef.h>
#include <stdint.h>

#include <span>
#include <type_traits>

#include "stm32f4xx.h"

#include "stm32f4/clock.hpp"
#include "stm32f4/device_tables.hpp"
#include "stm32f4/dma_engine.hpp"
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

/**
 * SxCR's CONFIGURATION as one word: every field of `c` at its place, and
 * neither the four interrupt enables nor EN. configure() writes it beside
 * the enables it keeps; an engine computes its own once, at arm(), and a
 * block start never builds it again.
 */
constexpr uint32_t dma_control_word(const DmaStreamConfig& c) {
    uint32_t cr = static_cast<uint32_t>(c.channel) << DMA_SxCR_CHSEL_Pos;
    cr |= static_cast<uint32_t>(c.memory_burst) << DMA_SxCR_MBURST_Pos;
    cr |= static_cast<uint32_t>(c.peripheral_burst) << DMA_SxCR_PBURST_Pos;
    cr |= c.current_target_is_m1 ? DMA_SxCR_CT : 0u;
    cr |= c.double_buffer ? DMA_SxCR_DBM : 0u;
    cr |= static_cast<uint32_t>(c.priority) << DMA_SxCR_PL_Pos;
    cr |= c.peripheral_increment_fixed4 ? DMA_SxCR_PINCOS : 0u;
    cr |= static_cast<uint32_t>(c.memory_width) << DMA_SxCR_MSIZE_Pos;
    cr |= static_cast<uint32_t>(c.peripheral_width) << DMA_SxCR_PSIZE_Pos;
    cr |= c.memory_increment ? DMA_SxCR_MINC : 0u;
    cr |= c.peripheral_increment ? DMA_SxCR_PINC : 0u;
    cr |= c.circular ? DMA_SxCR_CIRC : 0u;
    cr |= static_cast<uint32_t>(c.direction) << DMA_SxCR_DIR_Pos;
    cr |= c.peripheral_flow_control ? DMA_SxCR_PFCTRL : 0u;
    return cr;
}

/// SxFCR's two configuration fields, DMDIS and FTH (FEIE is an enable and
/// not configuration; FS is read-only).
constexpr uint32_t dma_fifo_word(const DmaStreamConfig& c) {
    return c.use_fifo ? (DMA_SxFCR_DMDIS |
                         static_cast<uint32_t>(c.fifo_threshold) << DMA_SxFCR_FTH_Pos)
                      : 0u;
}

/// PSIZE and MSIZE both at `w` - a beat the two ports agree on, which is
/// what direct mode demands (10.3.12) and what an engine's start writes.
constexpr uint32_t dma_beat_word(DmaWidth w) {
    return static_cast<uint32_t>(w) << DMA_SxCR_PSIZE_Pos |
           static_cast<uint32_t>(w) << DMA_SxCR_MSIZE_Pos;
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
     * else) - AND DO NOT READ SxCR BACK. Two measured facts decide the
     * shape (docs/stm32f4/dma.md). The store is not visible to the load
     * that follows it: EN reads 0 in the load right after and 1 in the
     * one after that, so a read-back taken at once would report a good
     * stream as a refused one. And on the STM32F446 and the STM32F411 A LOAD OF SxCR IN THE
     * CYCLES AFTER THE STORE WEDGES THE STREAM every other time - sixteen
     * bytes into the FIFO and nothing drains, EN standing, no flag, until
     * the controller is reset - which is what a version of this verb that
     * polled EN did, and what ST's library, which never reads back, does
     * not. The one refusal the silicon can make at this point - a FIFO
     * threshold the memory burst does not divide, 10.3.18 - is caught by
     * configure() before the store, so there is nothing left to ask here;
     * a caller that wants to know whether the stream is running asks
     * enabled() or the flags LATER, as a handler or a poll would anyway.
     */
    static bool enable() {
        regs().CR = regs().CR | DMA_SxCR_EN;
        return true;
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
        regs().FCR = armed_fcr | dma_fifo_word(c);
        regs().CR = armed_cr | dma_control_word(c);
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

    /// Where this stream's group sits in its flag register: a constant,
    /// so the two verbs below are a load and a shift, and a store.
    static constexpr uint32_t flag_shift = Block::flag_shift(s);

    /// This stream's five flags, at DmaFlag's positions.
    [[gnu::always_inline]] static uint32_t flags() {
        if constexpr (s < 4u) {
            return (Block::regs().LISR >> flag_shift) & DmaFlag::all;
        } else {
            return (Block::regs().HISR >> flag_shift) & DmaFlag::all;
        }
    }
    static bool flag(uint32_t mask) { return (flags() & mask) != 0u; }
    /// One store into the write-one-to-clear twin - never a
    /// read-modify-write, so it is safe from any context.
    [[gnu::always_inline]] static void clear(uint32_t mask) {
        if constexpr (s < 4u) {
            Block::regs().LIFCR = (mask & DmaFlag::all) << flag_shift;
        } else {
            Block::regs().HIFCR = (mask & DmaFlag::all) << flag_shift;
        }
    }

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

// ---- the engines ------------------------------------------------------------------

/// The DmaFlag bits an engine armed with `i` hands back from service().
constexpr uint32_t dma_interrupt_flags(DmaInterrupts i) {
    return i == DmaInterrupts::completion    ? (DmaFlag::complete | DmaFlag::transfer_error)
           : i == DmaInterrupts::errors_only ? DmaFlag::transfer_error
                                             : 0u;
}

/// The SxCR enables behind them (TCIE, TEIE - both in SxCR, 10.5.5).
constexpr uint32_t dma_interrupt_enables(DmaInterrupts i) {
    return i == DmaInterrupts::completion    ? (DMA_SxCR_TCIE | DMA_SxCR_TEIE)
           : i == DmaInterrupts::errors_only ? DMA_SxCR_TEIE
                                             : 0u;
}

/**
 * DmaTxEngine<n, s, ch, Elem> - "drain a run of memory into a
 * peripheral's data register".
 *
 * The engine owns a stream and nothing else: the peripheral's data
 * address is handed in at arm() by whoever owns the peripheral, so this
 * type knows nothing about USARTs and serves an SPI or a DAC unchanged.
 * The CHANNEL is a template argument and not an arm() one - on this
 * controller the (stream, channel) pair IS the peripheral's wiring, so it
 * belongs where a static_assert can check it (stm32f4/usart.hpp does).
 *
 * TWO MOMENTS, and 10.3.17's procedure says which step is which. arm() is
 * the BINDING and runs once: the gate opened, the stream stopped, step 2
 * (SxPAR, the peripheral's register), step 8 (SxFCR: direct mode) and
 * steps 5, 7 and 9 (SxCR: CHSEL, the priority, the direction, the
 * interrupt enables) - the configuration word kept in the engine, because
 * a block needs it back. A block start is steps 1, 3, 4 and 10 and nothing
 * else: the flags cleared (one store into LIFCR/HIFCR), SxM0AR, SxNDTR,
 * EN. SxCR's configuration is COMPARED with the word the block needs and
 * stored only when it differs - the beat and the memory increment are the
 * block's own, so a run of bytes after a run of bytes stores nothing.
 * The vendor's library stores five registers a block and reads SxCR back
 * three times (HAL_DMA_Start_IT); this start stores four and reads SxCR
 * once.
 *
 * A BLOCK START NEVER WAITS. 10.3.17's first step - clear EN and wait
 * for it to read 0 - is for a stream somebody else may be running; an
 * engine's own stream between two blocks is already stopped, because EN
 * is cleared by hardware at the end of a non-circular block (10.3.13) and
 * at a transfer error (10.5.5), and abandon() and stop() wait for it
 * themselves. So start() reads SxCR ONCE, BEFORE anything is stored, and
 * REFUSES if EN still stands - it never polls, which is what lets a
 * transport hold its mask over the claim alone - and it reads nothing
 * after the EN store: on the STM32F446 and the STM32F411 a load of SxCR in
 * the cycles after that store wedges the stream (docs/stm32f4/dma.md).
 *
 * THE BEAT IS THE ELEMENT OF THE RUN. `Elem` is the WIDEST beat this
 * binding allows - the width of the register the stream is pointed at -
 * and start() is overloaded on every narrower one: a byte run of an SPI
 * in 8-bit frames and a half-word run of the same SPI in 16-bit frames go
 * through one engine, the width from the span's element type. 10.3.6: an
 * address is aligned on the width of the accesses through it, so a run
 * whose address is not is REFUSED - a 16-bit span built over a byte
 * buffer can be misaligned, and the caller then takes its pump.
 *
 * THE CLAIM IS ITS OWN VERB. start() = claim() + the block start. A
 * transport whose two contexts can race for the stream (a print in the
 * loop, the completion handler starting the ring's next run) masks
 * claim() - a test-and-set, a dozen cycles - and calls start_claimed()
 * with the mask off: a claimed stream cannot complete under the loader's
 * feet, because it is not running.
 *
 * DIRECT MODE, NOT THE FIFO. 10.3.12's direct mode is "an immediate and
 * single transfer after each DMA request" with the one datum preloaded,
 * which is what a peripheral register a beat wide wants; the FIFO would
 * buy bursts at the price of table 49's constraints on every run length.
 *
 * THERE IS NO VERB TO KICK A STALLED FIRST BEAT, and the absence is a
 * fact: 10.3.2's handshake is level-driven, so a stream enabled with TXE
 * already standing moves the first datum at once.
 */
template <uint8_t n, uint8_t s, uint8_t ch, typename Elem = uint8_t>
class DmaTxEngine {
    using Stream = DmaStream<n, s>;

public:
    static_assert(ch < dma_channels, "brio DmaTxEngine: CHSEL selects one of eight channels");
    static_assert(sizeof(Elem) == 1 || sizeof(Elem) == 2 || sizeof(Elem) == 4,
                  "brio DmaTxEngine: the element type is the widest bus access - 1, 2 or 4 bytes");

    DmaTxEngine() = delete;

    static constexpr bool present = true;
    static constexpr uint8_t controller = n;
    static constexpr uint8_t stream = s;
    static constexpr uint8_t channel = ch;
    /// The WIDEST beat this binding takes; start() takes every narrower one.
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
     * The stream's ISR BODY: this stream's flags, those the binding armed,
     * cleared and handed back - one load of the flag bank and one store
     * into its clear register, SxCR not read at all (the armed set is the
     * engine's own, from arm()).
     *
     * IT ACTS ON NOTHING. What a completion or an error MEANS is the
     * owner's to decide - complete(), abandon() are the verbs - because
     * only the owner can see its peripheral's state.
     */
    [[gnu::always_inline]] static uint8_t service() {
        const uint32_t pending = Stream::flags() & st_.irq_flags;
        if (pending != 0u) {
            Stream::clear(pending);
        }
        return static_cast<uint8_t>(pending);
    }

    /// THE BINDING: `data` is the register the runs are poured into,
    /// `irqs` what the stream interrupts for. Opens the controller's gate,
    /// stops the stream, writes everything a block does not change.
    static void arm(volatile void* data, DmaPriority priority = DmaPriority::low,
                    DmaInterrupts irqs = DmaInterrupts::completion) {
        st_.data = data;
        st_.control = dma_control_word(DmaStreamConfig{
                       .channel = ch,
                       .direction = DmaDirection::memory_to_peripheral,
                       .memory_increment = false,   // the block's own, at its start
                       .priority = priority}) |
                   dma_interrupt_enables(irqs);
        st_.irq_flags = static_cast<uint8_t>(dma_interrupt_flags(irqs));
        bind();
        if (irqs == DmaInterrupts::none) {
            Nvic::disable(Stream::irq());
        } else {
            Nvic::enable(Stream::irq());
        }
    }

    /// THE CLAIM: take the stream for one block, or answer false because a
    /// block holds it. A test-and-set and nothing else - the one step a
    /// caller with two racing contexts masks.
    [[gnu::always_inline]] static bool claim() {
        if (st_.busy) {
            return false;
        }
        st_.busy = true;
        return true;
    }

    /// Start moving `run` - claim() and the block start. The buffer is the
    /// CALLER'S and must stay put until complete() reports the block.
    /// False, and nothing started, when a block holds the stream, the run
    /// is empty or longer than SxNDTR counts, its address is not aligned
    /// on its beat, or the stream had not stopped.
    [[gnu::always_inline]] static bool start(std::span<const uint8_t> run) {
        return claim() && launch(run.data(), run.size(), DMA_SxCR_MINC);
    }
    [[gnu::always_inline]] static bool start(std::span<const uint16_t> run)
        requires(sizeof(Elem) >= 2)
    {
        return claim() && launch(run.data(), run.size(), DMA_SxCR_MINC);
    }
    [[gnu::always_inline]] static bool start(std::span<const uint32_t> run)
        requires(sizeof(Elem) >= 4)
    {
        return claim() && launch(run.data(), run.size(), DMA_SxCR_MINC);
    }

    /// The block start on a stream the caller has already claim()ed; the
    /// claim is given back if the start is refused.
    [[gnu::always_inline]] static bool start_claimed(std::span<const uint8_t> run) {
        return launch(run.data(), run.size(), DMA_SxCR_MINC);
    }
    [[gnu::always_inline]] static bool start_claimed(std::span<const uint16_t> run)
        requires(sizeof(Elem) >= 2)
    {
        return launch(run.data(), run.size(), DMA_SxCR_MINC);
    }
    [[gnu::always_inline]] static bool start_claimed(std::span<const uint32_t> run)
        requires(sizeof(Elem) >= 4)
    {
        return launch(run.data(), run.size(), DMA_SxCR_MINC);
    }

    /**
     * The same block from ONE CELL, sent `length` times: the memory
     * pointer does not increment. What a full-duplex bus needs for the
     * transmit side of a READ - the clock has to run, so something must
     * be shifted out, and the dummy is one element of the caller's that
     * stays put for the whole block. The cell's type is the beat.
     */
    [[gnu::always_inline]] static bool start_fixed(const uint8_t* cell, uint16_t length) {
        return claim() && launch(cell, length, 0u);
    }
    [[gnu::always_inline]] static bool start_fixed(const uint16_t* cell, uint16_t length)
        requires(sizeof(Elem) >= 2)
    {
        return claim() && launch(cell, length, 0u);
    }
    [[gnu::always_inline]] static bool start_fixed(const uint32_t* cell, uint16_t length)
        requires(sizeof(Elem) >= 4)
    {
        return claim() && launch(cell, length, 0u);
    }

    /// The block ended - called from the stream's handler when its
    /// completion flag is up, or by an owner that has proof of the end
    /// from elsewhere. Returns the elements the block carried, so the owner
    /// can release exactly that much of its ring.
    static uint16_t complete() {
        if (!st_.busy) {
            return 0;
        }
        st_.busy = false;
        const uint16_t moved = st_.in_flight;
        st_.in_flight = 0;
        return moved;
    }

    static bool busy() { return st_.busy; }
    static uint16_t in_flight() { return st_.busy ? st_.in_flight : 0u; }
    static DmaProgress progress() { return Stream::progress(st_.in_flight); }

    /**
     * Throw away a block the silicon has stopped running and free the
     * stream, the binding written back. THE CALLER DECIDES, because only
     * the peripheral's owner can read the flags that make "dead" a fact
     * rather than a timeout. What is lost is the untransmitted tail, and
     * it is counted rather than papered over.
     */
    static bool abandon() {
        if (!st_.busy) {
            return false;
        }
        ++st_.faults;
        bind();
        return true;
    }

    static uint32_t faults() { return st_.faults; }
    static void clear_faults() { st_.faults = 0; }

    /// The stream stopped (EN waited down) and its flags and enables
    /// cleared. The binding survives: the next start() writes the
    /// configuration word back, enables included.
    static void stop() {
        Stream::stop();
        st_.busy = false;
        st_.in_flight = 0;
    }

private:
    template <typename T>
    [[gnu::always_inline]] static bool launch(const T* first, size_t count, uint32_t increment) {
        const uint32_t address = static_cast<uint32_t>(reinterpret_cast<uintptr_t>(first));
        if (first == nullptr || count == 0u || count > dma_max_items ||
            (address & (sizeof(T) - 1u)) != 0u) {
            st_.busy = false;
            return false;
        }
        DMA_Stream_TypeDef& r = Stream::regs();
        const uint32_t want = st_.control | dma_beat_word(dma_width_of<T>()) | increment;
        const uint32_t now = r.CR;   // the one read of SxCR, before every store
        if ((now & DMA_SxCR_EN) != 0u) {
            st_.busy = false;
            return false;
        }
        if (now != want) {
            r.CR = want;
        }
        Stream::clear(DmaFlag::all);
        r.M0AR = address;
        r.NDTR = static_cast<uint32_t>(count);
        st_.in_flight = static_cast<uint16_t>(count);
        r.CR = want | DMA_SxCR_EN;   // and nothing reads SxCR after this
        return true;
    }

    /// The binding written: the stream stopped (the gate opened, EN waited
    /// down, enables and flags cleared), then SxPAR, SxFCR and SxCR's
    /// configuration. arm() and abandon() are the same act.
    static void bind() {
        Stream::stop();
        DMA_Stream_TypeDef& r = Stream::regs();
        r.PAR = static_cast<uint32_t>(reinterpret_cast<uintptr_t>(st_.data));
        r.FCR = 0u;   // direct mode, FEIE clear
        r.CR = st_.control;
        st_.busy = false;
        st_.in_flight = 0;
    }

    /// The engine's state in ONE object, so a block start loads one base
    /// address and reaches every field by an offset from it - where
    /// separate statics cost a literal-pool load from the flash apiece.
    struct State {
        uint32_t control;          ///< SxCR's binding word, EN and the beat clear
        volatile void* data;       ///< SxPAR
        uint32_t faults;
        uint16_t in_flight;
        uint8_t irq_flags;         ///< what service() reports
        volatile bool busy;
    };
    static inline State st_{};
};

/**
 * DmaRxEngine<n, s, ch, Elem> - "fill memory from a peripheral's data
 * register", in TWO SHAPES.
 *
 * THE ONE-SHOT SHAPE is the transmit engine's two moments and its
 * one-read start, mirrored: arm(data, priority, interrupts) is the
 * binding, a block start is the flag clear, SxM0AR, SxNDTR and EN, SxCR
 * compared and not rebuilt, refused while EN stands. It is what a
 * BOUNDED block wants - the data phase of an I2C or SPI read, a block of
 * conversions. A receive block completes only when the buffer FILLS,
 * which on an idle line may be never, so its owner does not wait for a
 * completion: it ASKS, with take(), what has arrived since the last
 * question - one SxNDTR read and a subtraction, because SxNDTR is a live
 * counter software may read at any time (10.5.6).
 *
 * THE CIRCULAR SHAPE is what an UNBOUNDED stream wants - a serial line -
 * and on this controller it costs nothing to run: under CIRC "the number
 * of data items to be transferred is automatically reloaded ... and the
 * DMA requests continue to be served" (10.3.8), the memory pointer back
 * at SxM0AR, so a stream pointed at a whole ring never stops and never
 * needs a start again. arm(data, ring, priority) is therefore the
 * binding AND the only start there is: SxPAR, the ring's first element
 * in SxM0AR, its length in SxNDTR, SxCR with CIRC and the completion
 * enable, EN. From then on the engine is the PRODUCER of util/ring.hpp's
 * HardwareRing - a RingCounter:
 *
 *  - remaining() is SxNDTR, read live: the elements still to land in the
 *    current lap. In direct mode a datum loaded from the peripheral is
 *    "immediately drained and stored into the destination" (10.3.6), and
 *    the post-decrement of SxNDTR is the last of a transfer's three
 *    operations (10.3.2) - so the count says only what memory already
 *    holds;
 *  - laps() counts the completions the stream's handler has reported
 *    through service_ring() - the circular shape's ISR body - since the
 *    binding: every completion is the end of a lap (TCIF rises as
 *    SxNDTR reaches zero, before the reload - measured, two completions
 *    for two laps, docs/stm32f4/dma.md), so the count lags the counter by
 *    the handler's latency and never leads it. The half-transfer flag is
 *    not armed: a lap is one interrupt.
 *
 * What the circular shape cannot do is stop for a consumer that has not
 * caught up - the stream writes over what was not read - and the view
 * counts that as its overrun. take(), full(), capacity() and taken() are
 * the one-shot shape's and answer nothing useful in the other.
 */
template <uint8_t n, uint8_t s, uint8_t ch, typename Elem = uint8_t>
class DmaRxEngine {
    using Stream = DmaStream<n, s>;

public:
    static_assert(ch < dma_channels, "brio DmaRxEngine: CHSEL selects one of eight channels");
    static_assert(sizeof(Elem) == 1 || sizeof(Elem) == 2 || sizeof(Elem) == 4,
                  "brio DmaRxEngine: the element type is the widest bus access - 1, 2 or 4 bytes");

    DmaRxEngine() = delete;

    static constexpr bool present = true;
    static constexpr uint8_t controller = n;
    static constexpr uint8_t stream = s;
    static constexpr uint8_t channel = ch;
    /// The WIDEST beat this binding takes; start() takes every narrower one.
    static constexpr DmaWidth width = dma_width_of<Elem>();
    using element = Elem;

    static constexpr uint8_t flag_complete = DmaFlag::complete;
    static constexpr uint8_t flag_half = DmaFlag::half;
    static constexpr uint8_t flag_error = DmaFlag::transfer_error;
    static constexpr uint8_t flag_fifo_error = DmaFlag::fifo_error;

    /// The one-shot shape's ISR BODY, the transmit engine's: this stream's
    /// armed flags, cleared and handed back, acting on nothing.
    [[gnu::always_inline]] static uint8_t service() {
        const uint32_t pending = Stream::flags() & st_.irq_flags;
        if (pending != 0u) {
            Stream::clear(pending);
        }
        return static_cast<uint8_t>(pending);
    }

    /// The circular shape's ISR BODY: service(), and a reported completion
    /// COUNTED as the lap it is - the count laps() hands util/ring.hpp's
    /// HardwareRing as half of the producer index. Its own verb, so the
    /// one-shot shape's owners (the bus hosts, a block of conversions) pay
    /// nothing for a count they have no use for; whoever bound the ring
    /// binds the vector to this one.
    [[gnu::always_inline]] static uint8_t service_ring() {
        const uint32_t pending = Stream::flags() & st_.irq_flags;
        if (pending != 0u) {
            Stream::clear(pending);
            if ((pending & DmaFlag::complete) != 0u) {
                st_.laps = st_.laps + 1u;
            }
        }
        return static_cast<uint8_t>(pending);
    }

    /// THE ONE-SHOT BINDING: `data` is the register the blocks are filled
    /// from, `irqs` what the stream interrupts for. Opens the controller's
    /// gate, stops the stream, writes everything a block does not change.
    static void arm(volatile void* data, DmaPriority priority = DmaPriority::low,
                    DmaInterrupts irqs = DmaInterrupts::completion) {
        st_.data = data;
        st_.control = dma_control_word(DmaStreamConfig{
                       .channel = ch,
                       .direction = DmaDirection::peripheral_to_memory,
                       .memory_increment = false,   // the block's own, at its start
                       .priority = priority}) |
                   dma_interrupt_enables(irqs);
        st_.irq_flags = static_cast<uint8_t>(dma_interrupt_flags(irqs));
        bind();
        if (irqs == DmaInterrupts::none) {
            Nvic::disable(Stream::irq());
        } else {
            Nvic::enable(Stream::irq());
        }
    }

    /**
     * THE CIRCULAR BINDING, which is also the stream's only start: `ring`
     * is the caller's WHOLE storage, filled from its first element lap
     * after lap for as long as the binding stands, `data` the register it
     * is filled from. The completion and the transfer error interrupt -
     * a completion is a lap, counted by service_ring(), which the stream's
     * vector must call - and the half-transfer flag is left unarmed.
     *
     * False, and nothing touched, for a ring that is empty, longer than
     * SxNDTR counts (65535) or not aligned on its element (10.3.6). A
     * stream that is running is stopped first (the binding's wait), so
     * calling it again restarts the ring at its first element with laps()
     * at zero - the moment util/ring.hpp's HardwareRing over the same
     * storage is clear()ed.
     *
     * NOTHING READS SxCR AFTER THE EN STORE, the one-shot start's rule
     * (the wedge, docs/stm32f4/dma.md).
     */
    static bool arm(volatile void* data, std::span<uint8_t> ring,
                    DmaPriority priority = DmaPriority::low) {
        return arm_ring(data, ring.data(), ring.size(), priority);
    }
    static bool arm(volatile void* data, std::span<uint16_t> ring,
                    DmaPriority priority = DmaPriority::low)
        requires(sizeof(Elem) >= 2)
    {
        return arm_ring(data, ring.data(), ring.size(), priority);
    }
    static bool arm(volatile void* data, std::span<uint32_t> ring,
                    DmaPriority priority = DmaPriority::low)
        requires(sizeof(Elem) >= 4)
    {
        return arm_ring(data, ring.data(), ring.size(), priority);
    }

    /// SxNDTR, live (10.5.6): the elements still to land in the current
    /// lap of the circular shape, or the current block of the one-shot
    /// one. One load of a 32-bit register; nothing is suspended.
    [[gnu::always_inline]] static uint32_t remaining() {
        return static_cast<uint16_t>(Stream::regs().NDTR);
    }

    /// The completions service_ring() has counted since the binding: the
    /// laps of the circular shape. Written by the handler and read from
    /// the consumer's context, so the load is a volatile one.
    [[gnu::always_inline]] static uint32_t laps() { return st_.laps; }

    /// True while the stream is not running a block at all - it filled
    /// up, it errored, or it was never started. The owner hands it a new
    /// run then, whatever the arithmetic says. A read of SxCR: not in the
    /// cycles right after an EN store (the wedge).
    static bool idle() { return !Stream::enabled(); }

    /// Point the stream at a run of free memory and start filling it. The
    /// refusals are the transmit engine's.
    [[gnu::always_inline]] static bool start(std::span<uint8_t> run) {
        return launch(run.data(), run.size(), DMA_SxCR_MINC);
    }
    [[gnu::always_inline]] static bool start(std::span<uint16_t> run)
        requires(sizeof(Elem) >= 2)
    {
        return launch(run.data(), run.size(), DMA_SxCR_MINC);
    }
    [[gnu::always_inline]] static bool start(std::span<uint32_t> run)
        requires(sizeof(Elem) >= 4)
    {
        return launch(run.data(), run.size(), DMA_SxCR_MINC);
    }

    /**
     * `length` elements into ONE CELL, thrown away: the memory pointer
     * does not increment. What a full-duplex bus needs for the receive
     * side of a WRITE - every frame clocked out brings one back, and a
     * receiver left unread would overrun. The cell's type is the beat.
     */
    [[gnu::always_inline]] static bool start_discard(uint8_t* cell, uint16_t length) {
        return launch(cell, length, 0u);
    }
    [[gnu::always_inline]] static bool start_discard(uint16_t* cell, uint16_t length)
        requires(sizeof(Elem) >= 2)
    {
        return launch(cell, length, 0u);
    }
    [[gnu::always_inline]] static bool start_discard(uint32_t* cell, uint16_t length)
        requires(sizeof(Elem) >= 4)
    {
        return launch(cell, length, 0u);
    }

    /// How many elements have arrived since the last take(). One NDTR
    /// read; nothing is suspended and nothing can be refused, so the
    /// return is a plain number and not an optional.
    static uint16_t take() {
        if (st_.capacity == 0u) {
            return 0;
        }
        const uint16_t remaining = Stream::count();
        const uint16_t filled = remaining > st_.capacity
                                    ? st_.capacity
                                    : static_cast<uint16_t>(st_.capacity - remaining);
        if (filled <= st_.taken) {
            return 0;
        }
        const uint16_t fresh = static_cast<uint16_t>(filled - st_.taken);
        st_.taken = filled;
        return fresh;
    }

    /// True once the run is full: the owner must hand over a new one or
    /// the peripheral piles its own losses up.
    static bool full() { return st_.capacity != 0u && st_.taken >= st_.capacity; }
    static uint16_t capacity() { return st_.capacity; }
    static uint16_t taken() { return st_.taken; }

    static bool abandon() {
        ++st_.faults;
        bind();
        return true;
    }

    static uint32_t faults() { return st_.faults; }
    static void clear_faults() { st_.faults = 0; }

    static void stop() {
        Stream::stop();
        st_.capacity = 0;
        st_.taken = 0;
    }

private:
    template <typename T>
    [[gnu::always_inline]] static bool launch(T* first, size_t count, uint32_t increment) {
        const uint32_t address = static_cast<uint32_t>(reinterpret_cast<uintptr_t>(first));
        if (first == nullptr || count == 0u || count > dma_max_items ||
            (address & (sizeof(T) - 1u)) != 0u) {
            return false;
        }
        DMA_Stream_TypeDef& r = Stream::regs();
        const uint32_t want = st_.control | dma_beat_word(dma_width_of<T>()) | increment;
        const uint32_t now = r.CR;   // the one read of SxCR, before every store
        if ((now & DMA_SxCR_EN) != 0u) {
            return false;
        }
        if (now != want) {
            r.CR = want;
        }
        Stream::clear(DmaFlag::all);
        r.M0AR = address;
        r.NDTR = static_cast<uint32_t>(count);
        st_.capacity = static_cast<uint16_t>(count);
        st_.taken = 0;
        r.CR = want | DMA_SxCR_EN;   // and nothing reads SxCR after this
        return true;
    }

    /// The circular binding: the one-shot binding's steps (10.3.17's 1, 2,
    /// 8), then SxM0AR and SxNDTR (3, 4) once for good, SxCR with CIRC,
    /// MINC, the beat and the two enables (5, 7, 9), and EN (10) in a
    /// store of its own. CIRC is not kept in the binding word: a one-shot
    /// start after a transfer error stopped the ring is an ordinary block.
    template <typename T>
    static bool arm_ring(volatile void* data, T* first, size_t count, DmaPriority priority) {
        const uint32_t address = static_cast<uint32_t>(reinterpret_cast<uintptr_t>(first));
        if (first == nullptr || count == 0u || count > dma_max_items ||
            (address & (sizeof(T) - 1u)) != 0u) {
            return false;
        }
        st_.data = data;
        st_.control = dma_control_word(DmaStreamConfig{
                          .channel = ch,
                          .direction = DmaDirection::peripheral_to_memory,
                          .memory_increment = false,
                          .priority = priority}) |
                      dma_interrupt_enables(DmaInterrupts::completion);
        st_.irq_flags = static_cast<uint8_t>(dma_interrupt_flags(DmaInterrupts::completion));
        bind();   // stopped, flags and laps cleared, SxPAR, SxFCR
        DMA_Stream_TypeDef& r = Stream::regs();
        const uint32_t word =
            st_.control | dma_beat_word(dma_width_of<T>()) | DMA_SxCR_MINC | DMA_SxCR_CIRC;
        r.M0AR = address;
        r.NDTR = static_cast<uint32_t>(count);
        r.CR = word;
        Nvic::enable(Stream::irq());
        r.CR = word | DMA_SxCR_EN;   // and nothing reads SxCR after this
        return true;
    }

    static void bind() {
        Stream::stop();
        DMA_Stream_TypeDef& r = Stream::regs();
        r.PAR = static_cast<uint32_t>(reinterpret_cast<uintptr_t>(st_.data));
        r.FCR = 0u;   // direct mode, FEIE clear
        r.CR = st_.control;
        st_.capacity = 0;
        st_.taken = 0;
        st_.laps = 0u;
    }

    /// One object, for the transmit engine's reason.
    struct State {
        uint32_t control;          ///< SxCR's binding word, EN and the beat clear
        volatile void* data;       ///< SxPAR
        uint32_t faults;
        /// Completions service_ring() counted since the binding: written
        /// by the handler, read by the consumer's HardwareRing.
        volatile uint32_t laps;
        uint16_t capacity;
        uint16_t taken;
        uint8_t irq_flags;         ///< what service() reports
    };
    static inline State st_{};
};

/**
 * DmaCopyEngine<n, s> - copy and fill, memory to memory, on one stream of
 * DMA2.
 *
 * MEMORY TO MEMORY NEEDS NO REQUEST AND IS DMA2's ALONE (10.3.6): the
 * stream "immediately starts to fill the FIFO" on EN, and DMA1's
 * peripheral port - the SOURCE of such a block - reaches no memory. So
 * the controller is refused at compile time, and the stream's channel is
 * nobody's.
 *
 * THE ELEMENT IS THE BEAT. copy() and fill() count ELEMENTS of a type one
 * bus access wide - uint8_t, uint16_t or uint32_t - and the stream moves
 * them as beats of that width at both ports, so a block of pixels moves
 * as half-words and a block of words as words; an address not aligned on
 * its element is refused (10.3.6). The surface is the one every family's
 * copy engine has.
 *
 * THE FIFO, BECAUSE DIRECT MODE IS FORBIDDEN HERE (10.3.12), and it buys
 * the BURST: sixteen bytes a burst on each port - a word INCR4, a
 * half-word INCR8 or a byte INCR16, the full-threshold row of table 49
 * that every beat has - which the measured ladder (docs/stm32f4/dma.md)
 * puts at a third more than single beats. A port bursts only from a
 * sixteen-byte boundary: 10.3.11 forbids a burst across a 1 KB boundary
 * (an AHB error the DMA's registers do not report), and a sixteen-byte
 * burst from a sixteen-byte boundary cannot cross one.
 *
 * FILL is a copy whose source does not move: the stream reads the
 * caller's CELL - one element in memory, the caller's to keep in place
 * until busy() answers false - at every beat, single, because an
 * incrementing burst has no meaning on a fixed address.
 *
 * TWO MOMENTS, as the transport engines: arm() writes SxFCR (the FIFO, a
 * full threshold) and keeps SxCR's configuration word; a block is the
 * flag clear, SxPAR, SxM0AR, SxNDTR, then SxCR - its beat and bursts are
 * the block's, so it is stored whole - and EN. NOTHING READS SxCR: the
 * completion is read in the FLAG BANK (busy(), one load of LISR/HISR,
 * harmless at any distance from the enable), and a block start needs no
 * EN read because busy() saw the last block end - TCIF or TEIF, each of
 * which clears EN (10.3.13, 10.5.5) - before it lets another start.
 */
template <uint8_t n, uint8_t s>
class DmaCopyEngine {
    static_assert(dma_memory_to_memory_capable(n),
                  "brio DmaCopyEngine: memory to memory is DMA2's alone - DMA1's peripheral "
                  "port, the source of such a block, is not connected to the bus matrix "
                  "(RM0090 10.3.6, figures 33 and 34 note 1)");
    using Stream = DmaStream<n, s>;

    template <typename T>
    static constexpr bool beat =
        std::is_same_v<T, uint8_t> || std::is_same_v<T, uint16_t> || std::is_same_v<T, uint32_t>;

public:
    DmaCopyEngine() = delete;

    static constexpr uint8_t controller = n;
    static constexpr uint8_t stream = s;

    /// THE BINDING: the gate, the stream stopped, the FIFO at a full
    /// threshold, the configuration word kept. The completion is polled
    /// (busy()), so the stream's interrupt stays off.
    static void arm(DmaPriority priority = DmaPriority::low) {
        st_.control = dma_control_word(DmaStreamConfig{
            .direction = DmaDirection::memory_to_memory,
            .memory_increment = true,
            .priority = priority});
        Stream::stop();
        Stream::regs().FCR = dma_fifo_word(DmaStreamConfig{
            .use_fifo = true, .fifo_threshold = DmaFifoThreshold::full});
        Stream::regs().CR = st_.control;
        Nvic::disable(Stream::irq());
        st_.busy = false;
    }

    /// `count` elements from `src` to `dst`, memcpy's shape (no overlap),
    /// the element the beat. False, and nothing started, while a block runs,
    /// for an empty block or one past SxNDTR's 65535, or for an address
    /// not aligned on its element.
    template <typename T>
        requires beat<T>
    static bool copy(T* dst, const T* src, uint32_t count) {
        return launch(dst, src, count, DMA_SxCR_PINC);
    }

    /// `count` copies of the element at `cell` from `dst` on - a clear, a
    /// rectangle of pixels. The cell is read by the stream at every beat:
    /// it stays put until busy() answers false.
    template <typename T>
        requires beat<T>
    static bool fill(T* dst, const T* cell, uint32_t count) {
        return launch(dst, cell, count, 0u);
    }

    /// True while the last block runs. Reads the stream's flags (one load
    /// of LISR/HISR) and retires the block when they say it ended; a
    /// transfer error is counted in faults().
    static bool busy() {
        if (!st_.busy) {
            return false;
        }
        const uint32_t f = Stream::flags();
        if ((f & (DmaFlag::complete | DmaFlag::transfer_error)) == 0u) {
            return true;
        }
        if ((f & DmaFlag::transfer_error) != 0u) {
            ++st_.faults;
        }
        st_.busy = false;
        return false;
    }

    /// Stop a running block where it is - abort()'s wait, the FIFO
    /// flushed into the destination first (10.3.14) - and free the engine.
    /// The block is lost and counted in faults(); false when none ran.
    static bool abandon() {
        if (!busy()) {
            return false;
        }
        Stream::stop();
        Stream::regs().CR = st_.control;
        ++st_.faults;
        st_.busy = false;
        return true;
    }

    /// Blocks a transfer error ended (a bus error at either port - an
    /// address no DMA master reaches, 10.3.18) or abandon() threw away.
    static uint32_t faults() { return st_.faults; }

private:
    template <typename T>
    static bool launch(T* dst, const volatile T* src, uint32_t count, uint32_t source_increment) {
        const uint32_t d = static_cast<uint32_t>(reinterpret_cast<uintptr_t>(dst));
        const uint32_t from = static_cast<uint32_t>(reinterpret_cast<uintptr_t>(src));
        if (count == 0u || count > dma_max_items || ((d | from) & (sizeof(T) - 1u)) != 0u ||
            busy()) {
            return false;
        }
        // Sixteen bytes a burst at every beat: INCR4 words, INCR8
        // half-words, INCR16 bytes.
        constexpr DmaWidth w = dma_width_of<T>();
        constexpr uint32_t burst = sizeof(T) == 4u ? 1u : sizeof(T) == 2u ? 2u : 3u;
        uint32_t cr = st_.control | source_increment | dma_beat_word(w);
        if (source_increment != 0u && (from & 15u) == 0u) {
            cr |= burst << DMA_SxCR_PBURST_Pos;
        }
        if ((d & 15u) == 0u) {
            cr |= burst << DMA_SxCR_MBURST_Pos;
        }
        st_.busy = true;
        DMA_Stream_TypeDef& r = Stream::regs();
        Stream::clear(DmaFlag::all);
        r.PAR = from;
        r.M0AR = d;
        r.NDTR = count;
        r.CR = cr;
        r.CR = cr | DMA_SxCR_EN;
        return true;
    }

    /// One object, for the transmit engine's reason.
    struct State {
        uint32_t control;          ///< SxCR's binding word, the block's fields clear
        uint32_t faults;
        volatile bool busy;
    };
    static inline State st_{};
};

} // namespace brio

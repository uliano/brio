/*
 * dma_engine.hpp
 *
 * The "no DMA engine" tag of every optional engine slot in this stratum
 * - the Uart's two and the bus hosts' pairs - and the three things a
 * slot's owner names without the controller: the level an engine arms
 * at, what it is armed to interrupt for, and the longest block. The
 * STM32G0's file, for the same reasons.
 *
 * IT IS A TAG, NOT A BASE CLASS: `present` is the only thing a task asks
 * about, and it asks with `if constexpr`, so every engine branch - the
 * pump, the completion path and the state they need - disappears from a
 * driver that does not name one: an image whose tasks all take the
 * default carries no engine code at all.
 *
 * IT LIVES IN A FILE OF ITS OWN AND NOT IN A dma.hpp, because a driver
 * with such a slot must not include the DMA controller, or every program
 * with a console would carry it. The engines themselves - over this
 * family's stream-and-FIFO DMA (RM0090 ch. 10) - are stm32f4/dma.hpp's,
 * which a program includes when it fills a slot.
 */

#pragma once

#include <stdint.h>

namespace brio {

struct NoDmaEngine {
    NoDmaEngine() = delete;
    static constexpr bool present = false;
};

/**
 * SxCR.PL (RM0090 10.5.5): the software half of the arbitration. The
 * hardware half is the stream INDEX and cannot be configured: between
 * equal levels the lower-numbered stream wins (10.3.4).
 *
 * THE LEVEL IS THE PERIPHERAL'S (docs/design/dma.md). An engine whose
 * peripheral OVERRUNS when its stream is starved - a receive ring, an SPI
 * host's receive - arms at `very_high`; one whose peripheral waits - a
 * transmit, a paced output, an I2C host under its stretched clock - at
 * `high`; a memory-to-memory copy at `low`.
 *
 * THIS ARBITER'S OWN FACTS bound what a copy can do. Each controller
 * arbitrates its eight streams by these levels separately for each of its
 * two AHB ports, the memory port and the peripheral port (10.3.4), and
 * the two controllers meet only in the bus matrix, which serves its
 * masters round robin (RM0090 2.1.10). Measured on the STM32F446RE, a
 * USART's receive ring on DMA2 at 11.25 Mbaud - a frame every 160 cycles:
 * one or two copies ranked above the ring, back to back, never starved it;
 * three or four did, and it lost a third to a half of what came; ranked
 * very_high it lost nothing beside four (docs/stm32f4/dma.md). A copy on
 * DMA2 never outranks a DMA1 stream at all - it only shares the memory -
 * and four of them left a ring on DMA1 whole.
 *
 * The engines' defaults are those three by direction (stm32f4/dma.hpp),
 * and every driver arms its engines at its own level, with its chapter's
 * reason beside the call. It lives here, beside the tag, because a
 * transport names its engines' levels and must not include the controller
 * to do it.
 */
enum class DmaPriority : uint8_t {
    low = 0,
    medium = 1,
    high = 2,
    very_high = 3,
};

/**
 * What an engine's stream raises an interrupt for, chosen ONCE at arm().
 *
 *  - `completion`: the end of every block and a transfer error - what a
 *    transport whose next block waits on this one's end wants;
 *  - `errors_only`: a transfer error alone, for an engine whose end is
 *    PROVED BY ANOTHER. The SPI host's transmit stream is the case: every
 *    frame the receive stream takes was clocked out first, so the receive
 *    block's completion is the transaction's one interrupt and the
 *    transmit stream's would only say what is already known;
 *  - `none`: nothing at all - a caller that polls the flags.
 */
enum class DmaInterrupts : uint8_t { completion, errors_only, none };

/// The most items one block carries: SxNDTR is sixteen bits (RM0090
/// 10.5.6). Here and not in dma.hpp, because a driver with an engine slot
/// cuts its runs to it without including the controller.
inline constexpr uint32_t dma_max_items = 0xFFFFu;

} // namespace brio

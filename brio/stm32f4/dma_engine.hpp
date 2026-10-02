/*
 * dma_engine.hpp
 *
 * The "no DMA engine" tag of every optional engine slot in this stratum
 * - the Uart's two and the bus hosts' pairs - and the three things a
 * slot's owner names without the controller: an engine's priority, what
 * it is armed to interrupt for, and the longest block. The STM32G0's file, for the same
 * reasons.
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

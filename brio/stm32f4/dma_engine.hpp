/*
 * dma_engine.hpp
 *
 * The "no DMA engine" tag of every optional engine slot in this stratum
 * - stm32f4/usart.hpp's two today - and nothing else: the STM32G0's
 * file, for the same reasons.
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
 * family's stream-and-FIFO DMA (RM0090 ch. 10) - are the DMA chapter's;
 * until they exist a slot holds this tag and nothing else, and a task
 * says so where it checks its slots.
 */

#pragma once

namespace brio {

struct NoDmaEngine {
    NoDmaEngine() = delete;
    static constexpr bool present = false;
};

} // namespace brio
